function summaries = run_v01(configPath)
%RUN_V01 Generate deterministic AWR2944P flat-sea V0.1 HDF5 cases.
%   The reference pose is a vertical PCB with a horizontal forward
%   boresight. Positive mounting pitch points the boresight downward.

if nargin < 1 || strlength(string(configPath)) == 0
    scriptDir = fileparts(mfilename("fullpath"));
    configPath = fullfile(scriptDir, "..", "configs", "baseline_1m.json");
end

configPath = string(configPath);
configDir = fileparts(configPath);
config = jsondecode(fileread(configPath));
outputDir = resolvePath(configDir, string(config.output.directory));
radarCfgPath = resolvePath(configDir, string(config.radar.cfg_path));
radarMetadata = parseRadarCfg(radarCfgPath);
if ~isfolder(outputDir)
    mkdir(outputDir);
end
copyfile(configPath, fullfile(outputDir, "run_config.json"));
copyfile(radarCfgPath, fullfile(outputDir, "radar_profile.cfg"));

pitches = double(config.installation.mounting_pitch_sweep_deg(:));
summaries = repmat(struct( ...
    "mounting_pitch_deg", 0, ...
    "boresight_intersection_m", NaN, ...
    "three_db_near_m", NaN, ...
    "three_db_far_m", NaN, ...
    "six_db_near_m", NaN, ...
    "six_db_far_m", NaN, ...
    "peak_grid_range_m", NaN, ...
    "total_relative_power_linear", NaN, ...
    "total_relative_power_db", NaN, ...
    "peak_relative_power_linear", NaN, ...
    "peak_relative_power_db", NaN, ...
    "mainlobe_power_fraction", NaN), numel(pitches), 1);

for index = 1:numel(pitches)
    pitchDeg = pitches(index);
    result = simulateFlatSea(config, pitchDeg, radarMetadata);
    stem = pitchFilename(pitchDeg);
    writeCaseHdf5(result, fullfile(outputDir, stem + ".h5"));

    halfWidth = double(config.antenna.elevation_3db_half_width_deg);
    summaries(index).mounting_pitch_deg = pitchDeg;
    summaries(index).boresight_intersection_m = surfaceIntersection( ...
        double(config.installation.height_m), pitchDeg);
    summaries(index).three_db_near_m = surfaceIntersection( ...
        double(config.installation.height_m), pitchDeg + halfWidth);
    summaries(index).three_db_far_m = surfaceIntersection( ...
        double(config.installation.height_m), pitchDeg - halfWidth);
    sixDbHalfWidth = double(config.antenna.elevation_6db_half_width_deg);
    summaries(index).six_db_near_m = surfaceIntersection( ...
        double(config.installation.height_m), pitchDeg + sixDbHalfWidth);
    summaries(index).six_db_far_m = surfaceIntersection( ...
        double(config.installation.height_m), pitchDeg - sixDbHalfWidth);
    [~, peakIndex] = max(result.relative_power_linear, [], "all", "linear");
    summaries(index).peak_grid_range_m = result.horizontal_range_m(peakIndex);
    totalPower = sum(result.relative_power_linear, "all");
    peakPower = max(result.relative_power_linear, [], "all");
    mainlobePower = sum(result.relative_power_linear(result.one_way_gain_db >= -3), "all");
    summaries(index).total_relative_power_linear = totalPower;
    summaries(index).total_relative_power_db = 10 * log10(totalPower);
    summaries(index).peak_relative_power_linear = peakPower;
    summaries(index).peak_relative_power_db = 10 * log10(peakPower);
    summaries(index).mainlobe_power_fraction = mainlobePower / totalPower;
end

summaryJson = jsonencode(summaries, PrettyPrint=true);
summaryFile = fopen(fullfile(outputDir, "summary.json"), "w", "n", "UTF-8");
cleanup = onCleanup(@() fclose(summaryFile));
fprintf(summaryFile, "%s", summaryJson);
fprintf("Generated %d MATLAB V0.1 cases in %s\n", numel(pitches), outputDir);
end


function result = simulateFlatSea(config, pitchDeg, radarMetadata)
heightM = double(config.installation.height_m);
grid = config.grid;
antenna = config.antenna;
ranges = double(grid.range_min_m):double(grid.range_step_m):double(grid.range_max_m);
azimuths = double(grid.azimuth_min_deg):double(grid.azimuth_step_deg):double(grid.azimuth_max_deg);
[vesselAzimuthDeg, groundRangeM] = meshgrid(azimuths, ranges);

xM = groundRangeM .* sind(vesselAzimuthDeg);
yM = groundRangeM .* cosd(vesselAzimuthDeg);
zRelativeM = -heightM .* ones(size(xM));
radarXM = xM;
radarYM = yM .* cosd(pitchDeg) - zRelativeM .* sind(pitchDeg);
radarZM = yM .* sind(pitchDeg) + zRelativeM .* cosd(pitchDeg);
radarHorizontalM = hypot(radarXM, radarYM);
radarAzimuthDeg = atan2d(radarXM, radarYM);
radarElevationDeg = atan2d(radarZM, radarHorizontalM);

azimuthGainDb = patternLoss( ...
    radarAzimuthDeg, ...
    double(antenna.azimuth_3db_half_width_deg), ...
    double(antenna.azimuth_6db_half_width_deg), ...
    double(antenna.gain_floor_db));
elevationGainDb = patternLoss( ...
    radarElevationDeg, ...
    double(antenna.elevation_3db_half_width_deg), ...
    double(antenna.elevation_6db_half_width_deg), ...
    double(antenna.gain_floor_db));
oneWayGainDb = max(azimuthGainDb + elevationGainDb, double(antenna.gain_floor_db));
twoWayGainLinear = 10 .^ ((2 .* oneWayGainDb) ./ 10);
slantRangeM = hypot(groundRangeM, heightM);
cellAreaM2 = groundRangeM .* double(grid.range_step_m) .* deg2rad(double(grid.azimuth_step_deg));
relativePowerLinear = cellAreaM2 .* twoWayGainLinear ./ (slantRangeM .^ 4);
relativePowerDb = 10 .* log10(max(relativePowerLinear ./ max(relativePowerLinear, [], "all"), realmin));

result = struct( ...
    "schema_version", "awr2944p-flat-sea-v0.1", ...
    "power_model", "unit_sigma0_pattern_r4", ...
    "height_m", heightM, ...
    "mounting_pitch_deg", pitchDeg, ...
    "radar_metadata", radarMetadata, ...
    "x_m", xM, ...
    "y_m", yM, ...
    "z_m", zeros(size(xM)), ...
    "horizontal_range_m", groundRangeM, ...
    "slant_range_m", slantRangeM, ...
    "azimuth_deg", radarAzimuthDeg, ...
    "elevation_deg", radarElevationDeg, ...
    "grazing_angle_deg", atan2d(heightM, groundRangeM), ...
    "cell_area_m2", cellAreaM2, ...
    "one_way_gain_db", oneWayGainDb, ...
    "two_way_gain_linear", twoWayGainLinear, ...
    "relative_power_linear", relativePowerLinear, ...
    "relative_power_db", relativePowerDb);
end


function lossDb = patternLoss(angleDeg, halfWidth3Db, halfWidth6Db, floorDb)
lossDb = interp1( ...
    [0, halfWidth3Db, halfWidth6Db, 90], ...
    [0, -3, -6, floorDb], ...
    abs(angleDeg), "linear", floorDb);
end


function writeCaseHdf5(result, outputPath)
if isfile(outputPath)
    delete(outputPath);
end
writeScalar(outputPath, "/installation/height_m", result.height_m);
writeScalar(outputPath, "/installation/mounting_pitch_deg", result.mounting_pitch_deg);
h5writeatt(outputPath, "/", "schema_version", result.schema_version);
h5writeatt(outputPath, "/", "power_model", result.power_model);
h5writeatt(outputPath, "/", "producer", "matlab");
writeRadarMetadata(outputPath, result.radar_metadata);

truthNames = ["x_m", "y_m", "z_m", "horizontal_range_m", "slant_range_m", ...
    "azimuth_deg", "elevation_deg", "grazing_angle_deg", "cell_area_m2"];
antennaNames = ["one_way_gain_db", "two_way_gain_linear"];
processedNames = ["relative_power_linear", "relative_power_db"];
writeGroup(outputPath, "/truth/", truthNames, result);
writeGroup(outputPath, "/antenna/", antennaNames, result);
writeGroup(outputPath, "/processed/", processedNames, result);
end


function writeScalar(outputPath, datasetPath, value)
h5create(outputPath, datasetPath, 1, Datatype="double");
h5write(outputPath, datasetPath, double(value));
end


function writeGroup(outputPath, prefix, names, result)
for index = 1:numel(names)
    name = names(index);
    value = double(result.(name));
    datasetPath = prefix + name;
    h5create(outputPath, datasetPath, size(value), Datatype="double", ...
        ChunkSize=min(size(value), [128, 128]), Deflate=5, Shuffle=true);
    h5write(outputPath, datasetPath, value);
end
end


function writeRadarMetadata(outputPath, metadata)
names = string(fieldnames(metadata));
for index = 1:numel(names)
    name = names(index);
    value = metadata.(name);
    if name == "chirp_indices" || name == "chirp_tx_masks"
        h5create(outputPath, "/radar/" + name, numel(value), Datatype="int64");
        h5write(outputPath, "/radar/" + name, int64(value));
    elseif isnumeric(value) && isscalar(value)
        writeScalar(outputPath, "/radar/" + name, value);
    end
end
for index = 1:numel(names)
    name = names(index);
    value = metadata.(name);
    if ischar(value) || (isstring(value) && isscalar(value))
        h5writeatt(outputPath, "/radar", name, string(value));
    end
end
end


function metadata = parseRadarCfg(cfgPath)
metadata = struct("cfg_path", string(cfgPath));
chirpIndices = [];
chirpMasks = [];
lines = splitlines(string(fileread(cfgPath)));
for lineIndex = 1:numel(lines)
    line = strtrim(lines(lineIndex));
    if strlength(line) == 0 || startsWith(line, "%")
        continue;
    end
    tokens = split(line);
    command = tokens(1);
    if command == "channelCfg" && numel(tokens) >= 3
        rxMask = str2double(tokens(2));
        txMask = str2double(tokens(3));
        metadata.rx_mask = rxMask;
        metadata.tx_mask = txMask;
        metadata.num_rx = sum(bitget(uint32(rxMask), 1:32));
        metadata.num_tx = sum(bitget(uint32(txMask), 1:32));
    elseif command == "profileCfg" && numel(tokens) >= 12
        metadata.start_freq_ghz = str2double(tokens(3));
        metadata.idle_time_us = str2double(tokens(4));
        metadata.adc_start_time_us = str2double(tokens(5));
        metadata.ramp_end_time_us = str2double(tokens(6));
        metadata.freq_slope_mhz_per_us = str2double(tokens(9));
        metadata.num_adc_samples = str2double(tokens(11));
        metadata.sample_rate_ksps = str2double(tokens(12));
    elseif command == "chirpCfg" && numel(tokens) >= 9
        chirpStart = str2double(tokens(2));
        chirpEnd = str2double(tokens(3));
        txMask = str2double(tokens(9));
        chirpIndices = [chirpIndices, chirpStart:chirpEnd]; %#ok<AGROW>
        chirpMasks = [chirpMasks, repmat(txMask, 1, chirpEnd - chirpStart + 1)]; %#ok<AGROW>
    elseif command == "frameCfg" && numel(tokens) >= 7
        chirpStart = str2double(tokens(2));
        chirpEnd = str2double(tokens(3));
        loops = str2double(tokens(4));
        if numel(tokens) >= 8
            framePeriodIndex = 7;
        else
            framePeriodIndex = 6;
        end
        metadata.frame_chirp_start = chirpStart;
        metadata.frame_chirp_end = chirpEnd;
        metadata.num_loops = loops;
        metadata.frame_period_ms = str2double(tokens(framePeriodIndex));
        metadata.num_chirps_per_loop = chirpEnd - chirpStart + 1;
        metadata.num_chirps_per_frame = (chirpEnd - chirpStart + 1) * loops;
    end
end
metadata.chirp_indices = chirpIndices;
metadata.chirp_tx_masks = chirpMasks;
end


function distanceM = surfaceIntersection(heightM, depressionDeg)
if depressionDeg <= 0
    distanceM = NaN;
else
    distanceM = heightM / tand(depressionDeg);
end
end


function stem = pitchFilename(pitchDeg)
if pitchDeg < 0
    stem = "pitch_m" + replace(sprintf("%04.1f", abs(pitchDeg)), ".", "p") + "_deg";
else
    stem = "pitch_" + replace(sprintf("%04.1f", pitchDeg), ".", "p") + "_deg";
end
end


function resolvedPath = resolvePath(baseDir, candidatePath)
pathObject = java.io.File(char(candidatePath));
if pathObject.isAbsolute()
    resolvedPath = string(pathObject.getCanonicalPath());
else
    resolvedPath = string(java.io.File(char(fullfile(baseDir, candidatePath))).getCanonicalPath());
end
end
