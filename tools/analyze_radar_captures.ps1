<#
.SYNOPSIS
  Analyze locally synchronized DCA1000 captures without entering file paths.

.DESCRIPTION
  Scans timestamped capture directories, discovers the single BIN, metadata
  JSON and capture-time CFG, and writes PC-side results below pc_analysis.
  Exact capture_config.cfg snapshots are preferred; legacy recovered CFG files
  are accepted only when an exact snapshot is absent.

.EXAMPLE
  .\tools\analyze_radar_captures.ps1

.EXAMPLE
  .\tools\analyze_radar_captures.ps1 -CaptureRoot D:\radar_runs -Recursive

.EXAMPLE
  .\tools\analyze_radar_captures.ps1 `
    -RunFolder "awr2944p_shore400m_v0\20260912_133429"
#>

[CmdletBinding()]
param(
    [string]$CaptureRoot = "D:\radar_runs\awr2944p",
    [string]$RunsRoot = "D:\radar_runs",
    [string]$RunFolder = "",
    [string]$RunId = "",
    # Zero means: derive the range display limit from capture_config.cfg's
    # range CFAR FOV; older CFGs without it retain the 15-m fallback.
    [ValidateRange(0.0, 10000.0)]
    [double]$MaxRangeM = 0.0,
    [string]$PythonCommand = "python",
    [switch]$Recursive,
    [switch]$Force,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$rangeAnalyzer = Join-Path $PSScriptRoot "analyze_adc_range.py"
$postAnalyzer = Join-Path $PSScriptRoot "post_capture_analysis.py"

function Assert-SafeRunId([string]$Value) {
    if ($Value -notmatch '^\d{8}_\d{6}$') {
        throw "RunId must look like YYYYMMDD_HHMMSS, got: $Value"
    }
}

function Get-UniqueCaptureFile([string]$RunDirectory, [string]$Pattern, [string]$Label) {
    $matches = @(Get-ChildItem -LiteralPath $RunDirectory -File -Filter $Pattern | Sort-Object Name)
    if ($matches.Count -eq 0) {
        return $null
    }
    if ($matches.Count -gt 1) {
        throw "Multiple $Label files found in ${RunDirectory}: $($matches.Name -join ', ')"
    }
    return $matches[0]
}

function Get-CaptureCfg([string]$RunDirectory) {
    $exact = Get-UniqueCaptureFile $RunDirectory "capture_config.cfg" "exact CFG"
    if ($exact) {
        return $exact
    }
    return Get-UniqueCaptureFile $RunDirectory "capture_config_recovered.cfg" "recovered CFG"
}

function Resolve-MaxRangeM([System.IO.FileInfo]$CfgFile, [double]$RequestedMaxRangeM) {
    if ($RequestedMaxRangeM -gt 0) {
        return $RequestedMaxRangeM
    }
    foreach ($line in Get-Content -LiteralPath $CfgFile.FullName) {
        if ($line -match '^\s*cfarFovCfg\s+-1\s+0\s+0\s+([0-9]+(?:\.[0-9]+)?)\s*$') {
            return [double]$matches[1]
        }
    }
    return 15.0
}

function Invoke-Checked([string]$FilePath, [string[]]$Arguments) {
    Write-Host "[RUN] $FilePath $($Arguments -join ' ')" -ForegroundColor Cyan
    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $FilePath"
    }
}

function Analyze-OneRun([System.IO.DirectoryInfo]$RunDirectory) {
    $binFile = Get-UniqueCaptureFile $RunDirectory.FullName "adc_data_*.bin" "ADC BIN"
    $metadataFile = Get-UniqueCaptureFile $RunDirectory.FullName "adc_data_*.json" "capture metadata"
    $cfgFile = Get-CaptureCfg $RunDirectory.FullName

    if (-not $binFile -or -not $metadataFile -or -not $cfgFile) {
        Write-Host "[SKIP] Incomplete capture inputs: $($RunDirectory.FullName)" -ForegroundColor Yellow
        return
    }
    $effectiveMaxRangeM = Resolve-MaxRangeM $cfgFile $MaxRangeM

    $outputRoot = Join-Path $RunDirectory.FullName "pc_analysis"
    $rangeOutput = Join-Path $outputRoot "range_analysis"
    $reportOutput = Join-Path $outputRoot "quality_report"
    $dashboard = Join-Path $rangeOutput "diagnostic_dashboard.png"
    $postReport = Join-Path $reportOutput "post_capture_analysis.json"

    Write-Host "[PLAN] BIN: $($binFile.FullName)" -ForegroundColor Cyan
    Write-Host "[PLAN] metadata: $($metadataFile.FullName)" -ForegroundColor Cyan
    Write-Host "[PLAN] CFG: $($cfgFile.FullName)" -ForegroundColor Cyan
    Write-Host "[PLAN] analysis max range: $effectiveMaxRangeM m" -ForegroundColor Cyan
    Write-Host "[PLAN] output: $outputRoot" -ForegroundColor Cyan
    Write-Host "[PLAN] quality report output: $reportOutput" -ForegroundColor Cyan

    if ($DryRun) {
        return
    }
    if (-not $Force -and (Test-Path -LiteralPath $dashboard) -and (Test-Path -LiteralPath $postReport)) {
        Write-Host "[SKIP] PC analysis already complete: $outputRoot" -ForegroundColor DarkGray
        return
    }

    New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null
    Invoke-Checked $PythonCommand @(
        $rangeAnalyzer,
        "--bin", $binFile.FullName,
        "--cfg", $cfgFile.FullName,
        "--output-dir", $rangeOutput,
        "--max-range-m", ([string]::Format([Globalization.CultureInfo]::InvariantCulture, "{0}", $effectiveMaxRangeM)),
        "--remove-mean"
    )
    Invoke-Checked $PythonCommand @(
        $postAnalyzer,
        "--bin", $binFile.FullName,
        "--cfg", $cfgFile.FullName,
        "--metadata", $metadataFile.FullName,
        "--range-analysis-dir", $rangeOutput,
        "--output-dir", $reportOutput
    )
    Write-Host "[DONE] PC analysis: $outputRoot" -ForegroundColor Green
}

if ($RunFolder) {
    if ($RunId) {
        throw "RunFolder and RunId cannot be used together."
    }
    $runFolderPath = if ([System.IO.Path]::IsPathRooted($RunFolder)) {
        $RunFolder
    }
    else {
        Join-Path $RunsRoot $RunFolder
    }
    if (-not (Test-Path -LiteralPath $runFolderPath -PathType Container)) {
        throw "Run folder not found: $runFolderPath"
    }
    $selectedRun = Get-Item -LiteralPath $runFolderPath
    Assert-SafeRunId $selectedRun.Name
    $selectedRuns = @($selectedRun)
}
else {
    if (-not (Test-Path -LiteralPath $CaptureRoot -PathType Container)) {
        throw "Capture root not found: $CaptureRoot"
    }
    if ($Recursive) {
        $captureDirectories = @(Get-ChildItem -LiteralPath $CaptureRoot -Directory -Recurse |
            Where-Object { $_.Name -match '^\d{8}_\d{6}$' } |
            Sort-Object FullName)
    }
    else {
        $captureDirectories = @(Get-ChildItem -LiteralPath $CaptureRoot -Directory |
            Where-Object { $_.Name -match '^\d{8}_\d{6}$' } |
            Sort-Object Name)
    }

    if ($RunId) {
    Assert-SafeRunId $RunId
        $selectedRuns = @($captureDirectories | Where-Object { $_.Name -eq $RunId })
    }
    else {
        $selectedRuns = $captureDirectories
    }
}

if ($selectedRuns.Count -eq 0) {
    throw "No timestamped capture directories found below: $CaptureRoot"
}

Push-Location $projectRoot
try {
    foreach ($selectedRun in $selectedRuns) {
        Analyze-OneRun $selectedRun
    }
}
finally {
    Pop-Location
}
