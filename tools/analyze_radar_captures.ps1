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
  .\tools\analyze_radar_captures.ps1 -CaptureRoot D:\radar_runs\awr2944p_lane4_ab
#>

[CmdletBinding()]
param(
    [string]$CaptureRoot = "D:\radar_runs\awr2944p",
    [string]$RunId = "",
    [double]$MaxRangeM = 15.0,
    [string]$PythonCommand = "python",
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

    $outputRoot = Join-Path $RunDirectory.FullName "pc_analysis"
    $rangeOutput = Join-Path $outputRoot "range_analysis"
    $dashboard = Join-Path $rangeOutput "diagnostic_dashboard.png"
    $postReport = Join-Path $outputRoot "post_capture_analysis.json"

    Write-Host "[PLAN] BIN: $($binFile.FullName)" -ForegroundColor Cyan
    Write-Host "[PLAN] metadata: $($metadataFile.FullName)" -ForegroundColor Cyan
    Write-Host "[PLAN] CFG: $($cfgFile.FullName)" -ForegroundColor Cyan
    Write-Host "[PLAN] output: $outputRoot" -ForegroundColor Cyan

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
        "--max-range-m", ([string]::Format([Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxRangeM)),
        "--remove-mean"
    )
    Invoke-Checked $PythonCommand @(
        $postAnalyzer,
        "--bin", $binFile.FullName,
        "--cfg", $cfgFile.FullName,
        "--metadata", $metadataFile.FullName,
        "--range-analysis-dir", $rangeOutput,
        "--output-dir", $outputRoot
    )
    Write-Host "[DONE] PC analysis: $outputRoot" -ForegroundColor Green
}

if (-not (Test-Path -LiteralPath $CaptureRoot -PathType Container)) {
    throw "Capture root not found: $CaptureRoot"
}

if ($RunId) {
    Assert-SafeRunId $RunId
    $selectedRuns = @(Get-Item -LiteralPath (Join-Path $CaptureRoot $RunId))
}
else {
    $selectedRuns = @(Get-ChildItem -LiteralPath $CaptureRoot -Directory |
        Where-Object { $_.Name -match '^\d{8}_\d{6}$' } |
        Sort-Object Name)
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
