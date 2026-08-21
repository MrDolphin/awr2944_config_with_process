function summaries = run_v02_direction_matrix()
%RUN_V02_DIRECTION_MATRIX Generate the V0.2 direction-semantics matrix.
%   Uses ss2_normal and ss3_upper, seed 101, and MATLAB WindDirection
%   values 0/90/180 deg. Each direction receives an isolated config
%   snapshot and run directory; the production sea-state config is never
%   modified. MATLAB seaSurface accepts only 0..180 deg; the 270 deg
%   (westward) case requires a custom spectrum or coordinate reflection.

scriptDir = fileparts(mfilename("fullpath"));
repoDir = string(java.io.File(char(fullfile(scriptDir, "..", ".."))).getCanonicalPath());
baseConfigPath = fullfile(repoDir, "simulation", "configs", ...
    "sea_states_0_to_3.json");
cfg = jsondecode(fileread(baseConfigPath));
resultsRoot = fullfile(repoDir, "simulation", "stages", ...
    "v02_dynamic_sea_truth", "results");
radarCfgPath = string(java.io.File(char(fullfile(repoDir, ...
    "simulation", "configs", string(cfg.radar.cfg_path)))).getCanonicalPath());

directions = [0, 90, 180];
caseIds = ["ss2_normal", "ss3_upper"];
seed = 101;
summaries = cell(numel(directions), 1);

for index = 1:numel(directions)
    direction = directions(index);
    directionCfg = cfg;
    directionCfg.sea_surface.wind_direction_deg = direction;
    directionCfg.output.directory = resultsRoot;
    directionCfg.radar.cfg_path = radarCfgPath;
    tempConfig = fullfile(tempdir, sprintf( ...
        "sea_states_direction_%03d.json", direction));
    writeJson(tempConfig, directionCfg);
    runId = sprintf("v02b_dir%03d_seed101", direction);
    summaries{index} = run_v02(tempConfig, runId, caseIds, seed);
end

fprintf("Generated V0.2 direction matrix for WindDirection=[0 90 180] deg.\n");
end


function writeJson(path, value)
fileId = fopen(path, "w", "n", "UTF-8");
if fileId < 0
    error("run_v02_direction_matrix:FileOpen", ...
        "Unable to open temporary config: %s", path);
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, "%s", jsonencode(value, PrettyPrint=true));
end
