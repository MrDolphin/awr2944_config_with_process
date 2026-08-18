function tests = test_run_v01
tests = functiontests(localfunctions);
end


function testFiveDegreeBoresightAndHdf5Contract(testCase)
testDir = string(tempname);
mkdir(testDir);
cleanup = onCleanup(@() rmdir(testDir, "s"));
repoRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
radarCfg = fullfile(repoRoot, "Config", "profile_3d_3Azim_1ElevTx_awr2944P.cfg");
outputDir = fullfile(testDir, "output");
config = struct( ...
    "radar", struct("cfg_path", radarCfg), ...
    "installation", struct("height_m", 1, "mounting_pitch_sweep_deg", 5), ...
    "grid", struct( ...
        "range_min_m", 5, "range_max_m", 30, "range_step_m", 0.01, ...
        "azimuth_min_deg", 0, "azimuth_max_deg", 0, "azimuth_step_deg", 1), ...
    "antenna", struct( ...
        "azimuth_3db_half_width_deg", 30, "azimuth_6db_half_width_deg", 45, ...
        "elevation_3db_half_width_deg", 3, "elevation_6db_half_width_deg", 5, ...
        "gain_floor_db", -30), ...
    "output", struct("directory", outputDir));
configPath = fullfile(testDir, "test_config.json");
fileId = fopen(configPath, "w", "n", "UTF-8");
fileCleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s", jsonencode(config, PrettyPrint=true));
clear fileCleanup;

summaries = run_v01(configPath, "test_baseline");
verifyEqual(testCase, summaries.boresight_intersection_m, 1 / tand(5), AbsTol=1e-10);
verifyEqual(testCase, summaries.six_db_near_m, 1 / tand(10), AbsTol=1e-10);
verifyTrue(testCase, isnan(summaries.six_db_far_m));
runDir = fullfile(outputDir, "matlab", "test_baseline");
casePath = fullfile(runDir, "data", "pitch_05p0_deg.h5");
verifyTrue(testCase, isfile(casePath));
verifyTrue(testCase, isfile(fullfile(runDir, "design_snapshot.md")));
verifyTrue(testCase, isfile(fullfile(runDir, "environment.json")));
verifyTrue(testCase, isfile(fullfile(runDir, "validation.md")));
verifyEqual(testCase, h5readatt(casePath, "/", "schema_version"), "awr2944p-flat-sea-v0.1");
verifyEqual(testCase, h5read(casePath, "/radar/num_adc_samples"), 656);
verifyEqual(testCase, h5read(casePath, "/radar/frame_num_adc_samples"), 656);
verifyEqual(testCase, h5read(casePath, "/radar/frame_period_ms"), 100);
txMasks = h5read(casePath, "/radar/chirp_tx_masks");
verifyEqual(testCase, txMasks(:), int64([1; 4; 8; 2]));
elevation = h5read(casePath, "/truth/elevation_deg");
range = h5read(casePath, "/truth/horizontal_range_m");
[~, centerIndex] = min(abs(elevation), [], "all", "linear");
verifyEqual(testCase, range(centerIndex), 1 / tand(5), AbsTol=0.011);
verifyError(testCase, @() run_v01(configPath, "test_baseline"), ...
    "run_v01:RunExists");
end
