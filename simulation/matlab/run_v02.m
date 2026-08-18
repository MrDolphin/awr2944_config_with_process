function summaries = run_v02(configPath, runId, selectedCaseIds, selectedSeeds)
%RUN_V02 Sample target-controlled sea states 0--3 with Radar Toolbox.
%   Output is dynamic sea-surface truth only. It is not ADC/IQ, clutter
%   power, Range-Doppler, AoA, CFAR, or a point cloud.

if nargin < 1 || isempty(configPath) || strlength(string(configPath)) == 0
    scriptDir = fileparts(mfilename("fullpath"));
    configPath = fullfile(scriptDir, "..", "configs", "sea_states_0_to_3.json");
end
if nargin < 2 || strlength(string(runId)) == 0
    runId = defaultRunId();
end
if nargin < 3
    selectedCaseIds = strings(0, 1);
else
    selectedCaseIds = string(selectedCaseIds(:));
end

configPath = string(configPath);
configDir = fileparts(configPath);
config = jsondecode(fileread(configPath));
if nargin < 4 || isempty(selectedSeeds)
    selectedSeeds = double(config.sea_surface.random_seeds(:));
else
    selectedSeeds = double(selectedSeeds(:));
end
cases = config.sea_states;
validateConfiguration(cases, selectedCaseIds, selectedSeeds);
resultsRoot = resolvePath(configDir, string(config.output.directory));
radarCfgPath = resolvePath(configDir, string(config.radar.cfg_path));
if ~isfile(radarCfgPath)
    error("run_v02:MissingRadarCfg", "Radar CFG does not exist: %s", radarCfgPath);
end
outputDir = createRunDirectory(resultsRoot, "matlab", string(runId));
dataDir = fullfile(outputDir, "data");
copyfile(configPath, fullfile(outputDir, "run_config.json"));
copyfile(radarCfgPath, fullfile(outputDir, "radar_profile.cfg"));

summaries = repmat(struct( ...
    "case_id", "", ...
    "sea_state", 0, ...
    "random_seed", 0, ...
    "target_hs_m", 0, ...
    "raw_hs_m", 0, ...
    "amplitude_scale_factor", 0, ...
    "achieved_hs_m", 0, ...
    "wind_speed_mps", 0, ...
    "wind_direction_deg", 0, ...
    "fetch_m", 0), 0, 1);

for caseIndex = 1:numel(cases)
    caseConfig = cases(caseIndex);
    caseId = string(caseConfig.case_id);
    if ~isempty(selectedCaseIds) && ~any(selectedCaseIds == caseId)
        continue;
    end
    seaState = double(caseConfig.sea_state);
    targetHsM = double(caseConfig.target_hs_m);
    validateSeaState(caseId, seaState, targetHsM);
    for seedIndex = 1:numel(selectedSeeds)
        seed = selectedSeeds(seedIndex);
        sampled = sampleSurface(config, seaState, targetHsM, seed);
        stem = caseId + "_seed" + sprintf("%03d", seed);
        writeRawHdf5(sampled, caseId, seaState, targetHsM, seed, config, ...
            fullfile(dataDir, stem + ".h5"));
        summary = struct( ...
            "case_id", caseId, ...
            "sea_state", seaState, ...
            "random_seed", seed, ...
            "target_hs_m", targetHsM, ...
            "raw_hs_m", sampled.raw_hs_m, ...
            "amplitude_scale_factor", sampled.amplitude_scale_factor, ...
            "achieved_hs_m", sampled.achieved_hs_m, ...
            "wind_speed_mps", sampled.wind_speed_mps, ...
            "wind_direction_deg", double(config.sea_surface.wind_direction_deg), ...
            "fetch_m", double(config.sea_surface.fetch_m));
        summaries(end + 1, 1) = summary; %#ok<AGROW>
    end
end

if isempty(summaries)
    error("run_v02:NoCases", "No sea-state cases matched the selection.");
end
writeText(fullfile(outputDir, "summary.json"), ...
    jsonencode(summaries, PrettyPrint=true));
writeText(fullfile(outputDir, "validation.md"), ...
    "# Validation" + newline + newline + ...
    "- [x] MATLAB sea-height generation completed" + newline + ...
    "- [x] Height cubes normalized to target Hs" + newline + ...
    "- [ ] Python truth analysis completed" + newline + ...
    "- [ ] Manual figure review recorded" + newline);
fprintf("Generated %d MATLAB V0.2 raw sea cases in %s\n", numel(summaries), outputDir);
end


function validateConfiguration(cases, selectedCaseIds, selectedSeeds)
allCaseIds = strings(numel(cases), 1);
for index = 1:numel(cases)
    caseConfig = cases(index);
    allCaseIds(index) = string(caseConfig.case_id);
    validateSeaState(allCaseIds(index), double(caseConfig.sea_state), ...
        double(caseConfig.target_hs_m));
end
if numel(unique(allCaseIds)) ~= numel(allCaseIds)
    error("run_v02:DuplicateCaseId", "Sea-state case IDs must be unique.");
end
if ~isempty(selectedCaseIds) && ~all(ismember(selectedCaseIds, allCaseIds))
    error("run_v02:UnknownCase", "selectedCaseIds contains an unknown case ID.");
end
if isempty(selectedSeeds) || any(~isfinite(selectedSeeds)) || ...
        any(selectedSeeds < 0) || any(selectedSeeds ~= floor(selectedSeeds))
    error("run_v02:InvalidSeed", "Seeds must be nonnegative finite integers.");
end
end


function sampled = sampleSurface(config, seaState, targetHsM, seed)
grid = config.grid;
timeConfig = config.time;
xM = double(grid.x_min_m):double(grid.spacing_m):double(grid.x_max_m);
yM = double(grid.y_min_m):double(grid.spacing_m):double(grid.y_max_m);
timeS = double(timeConfig.start_s):double(timeConfig.step_s):double(timeConfig.stop_s);
[xGrid, yGrid] = meshgrid(xM, yM);
rawHeightM = zeros(numel(yM), numel(xM), numel(timeS));

if seaState == 0
    windSpeedMps = 0;
else
    [~, ~, windSpeedMps] = searoughness(seaState);
    rng(seed, "twister");
    scene = radarScenario(IsEarthCentered=false);
    spectrum = seaSpectrum(Resolution=double(config.sea_surface.spectrum_resolution_m));
    boundary = [xM(1), xM(end); yM(1), yM(end)];
    surface = seaSurface(scene, ...
        Boundary=boundary, ...
        WindSpeed=windSpeedMps, ...
        WindDirection=double(config.sea_surface.wind_direction_deg), ...
        Fetch=double(config.sea_surface.fetch_m), ...
        SpectralModel=spectrum);
    queryPoints = [xGrid(:)'; yGrid(:)'];
    for timeIndex = 1:numel(timeS)
        values = height(surface, queryPoints, timeS(timeIndex));
        rawHeightM(:, :, timeIndex) = reshape(values, size(xGrid));
    end
end

centeredHeightM = rawHeightM - mean(rawHeightM, [1, 2]);
rawHsM = 4 * std(centeredHeightM, 0, "all");
if targetHsM == 0
    normalizedHeightM = zeros(size(centeredHeightM));
    scaleFactor = 0;
elseif rawHsM <= eps
    error("run_v02:FlatRandomSurface", ...
        "Sea state %d produced no height variation for seed %d.", seaState, seed);
else
    scaleFactor = targetHsM / rawHsM;
    normalizedHeightM = centeredHeightM .* scaleFactor;
end
achievedHsM = 4 * std(normalizedHeightM, 0, "all");

sampled = struct( ...
    "x_m", xM, ...
    "y_m", yM, ...
    "time_s", timeS, ...
    "height_m", normalizedHeightM, ...
    "raw_hs_m", rawHsM, ...
    "amplitude_scale_factor", scaleFactor, ...
    "achieved_hs_m", achievedHsM, ...
    "wind_speed_mps", windSpeedMps);
end


function writeRawHdf5(sampled, caseId, seaState, targetHsM, seed, config, outputPath)
writeScalar(outputPath, "/case/sea_state", seaState);
writeScalar(outputPath, "/case/target_hs_m", targetHsM);
writeScalar(outputPath, "/case/raw_hs_m", sampled.raw_hs_m);
writeScalar(outputPath, "/case/amplitude_scale_factor", sampled.amplitude_scale_factor);
writeScalar(outputPath, "/case/random_seed", seed);
writeScalar(outputPath, "/case/wind_speed_mps", sampled.wind_speed_mps);
writeScalar(outputPath, "/case/wind_direction_deg", ...
    double(config.sea_surface.wind_direction_deg));
writeScalar(outputPath, "/case/fetch_m", double(config.sea_surface.fetch_m));
writeScalar(outputPath, "/installation/height_m", double(config.installation.height_m));
writeScalar(outputPath, "/installation/mounting_pitch_deg", ...
    double(config.installation.mounting_pitch_deg));
writeVector(outputPath, "/axes/x_m", sampled.x_m);
writeVector(outputPath, "/axes/y_m", sampled.y_m);
writeVector(outputPath, "/axes/time_s", sampled.time_s);

% Store x-by-y-by-time. h5py observes the reversed HDF5 dimensions as
% time-by-y-by-x, which is the public Python contract.
storedHeightM = permute(sampled.height_m, [2, 1, 3]);
h5create(outputPath, "/truth/height_m", size(storedHeightM), ...
    Datatype="double", ChunkSize=min(size(storedHeightM), [64, 64, 4]), ...
    Deflate=5, Shuffle=true);
h5write(outputPath, "/truth/height_m", storedHeightM);
h5writeatt(outputPath, "/", "schema_version", "awr2944p-dynamic-sea-v0.2");
h5writeatt(outputPath, "/", "producer", "matlab");
h5writeatt(outputPath, "/", "case_id", caseId);
end


function writeScalar(outputPath, datasetPath, value)
h5create(outputPath, datasetPath, 1, Datatype="double");
h5write(outputPath, datasetPath, double(value));
end


function writeVector(outputPath, datasetPath, value)
value = double(value(:));
h5create(outputPath, datasetPath, numel(value), Datatype="double");
h5write(outputPath, datasetPath, value);
end


function validateSeaState(caseId, seaState, targetHsM)
if seaState < 0 || seaState > 3 || seaState ~= floor(seaState)
    error("run_v02:SeaStateLimit", "%s sea_state must be an integer from 0 to 3.", caseId);
end
if targetHsM < 0 || targetHsM > 1.25
    error("run_v02:SeaStateLimit", "%s target Hs must be between 0 and 1.25 m.", caseId);
end
upperBounds = [0, 0.1, 0.5, 1.25];
classified = find(targetHsM <= upperBounds, 1, "first") - 1;
if classified ~= seaState
    error("run_v02:SeaStateMismatch", ...
        "%s target Hs classifies as state %d, not state %d.", ...
        caseId, classified, seaState);
end
end


function runId = defaultRunId()
timestamp = string(datetime("now", "TimeZone", "UTC", ...
    "Format", "yyyyMMdd'T'HHmmss_SSSSSS'Z'"));
uuid = erase(string(java.util.UUID.randomUUID), "-");
runId = timestamp + "_" + extractBefore(uuid, 9);
end


function outputDir = createRunDirectory(resultsRoot, producer, runId)
identifierPattern = "^[A-Za-z0-9][A-Za-z0-9_.-]*$";
if isempty(regexp(char(runId), identifierPattern, "once"))
    error("run_v02:InvalidRunId", ...
        "runId may contain only letters, digits, dot, dash or underscore.");
end
outputDir = fullfile(resultsRoot, producer, runId);
if isfolder(outputDir) || isfile(outputDir)
    error("run_v02:RunExists", "Run directory already exists: %s", outputDir);
end
mkdir(outputDir);
mkdir(fullfile(outputDir, "data"));
mkdir(fullfile(outputDir, "figures"));
createdUtc = string(datetime("now", "TimeZone", "UTC", ...
    "Format", "yyyy-MM-dd'T'HH:mm:ss.SSSXXX"));
environment = struct( ...
    "stage_id", "v02_dynamic_sea_truth", ...
    "producer", producer, ...
    "run_id", runId, ...
    "created_utc", createdUtc, ...
    "matlab_version", string(version));
[gitStatus, gitCommit] = system("git rev-parse HEAD");
environment.git_commit = "";
if gitStatus == 0
    environment.git_commit = strtrim(string(gitCommit));
end
products = ver;
environment.matlab_products = string({products.Name});
writeText(fullfile(outputDir, "environment.json"), ...
    jsonencode(environment, PrettyPrint=true));
writeText(fullfile(outputDir, "design_snapshot.md"), ...
    "# v02_dynamic_sea_truth run " + runId + newline + newline + ...
    "- Producer: `matlab`" + newline + ...
    "- Created UTC: `" + createdUtc + "`" + newline + ...
    "- Amplitude control: sampled spectral shape scaled to target Hs." + newline + ...
    "- Scope excludes IQ, clutter power, AoA and CFAR." + newline);
writeText(fullfile(outputDir, "validation.md"), ...
    "# Validation" + newline + newline + ...
    "- [ ] Generation completed" + newline + ...
    "- [ ] Python truth analysis completed" + newline + ...
    "- [ ] Manual review recorded" + newline);
end


function writeText(path, content)
fileId = fopen(path, "w", "n", "UTF-8");
if fileId < 0
    error("run_v02:FileOpen", "Unable to open %s", path);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s", content);
end


function resolvedPath = resolvePath(baseDir, candidatePath)
pathObject = java.io.File(char(candidatePath));
if pathObject.isAbsolute()
    resolvedPath = string(pathObject.getCanonicalPath());
else
    resolvedPath = string(java.io.File(char(fullfile(baseDir, candidatePath))).getCanonicalPath());
end
end
