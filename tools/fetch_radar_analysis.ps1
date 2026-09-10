<#
.SYNOPSIS
  Copy the latest (or named) Raspberry Pi radar-analysis artifacts to Windows.

.DESCRIPTION
  The script copies PNG, JSON and Markdown analysis evidence but intentionally
  does not copy the raw ADC BIN unless -IncludeBin is supplied.  Configure SSH
  public-key login first, so both SSH discovery and SCP transfer are passwordless.

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -RunId 20260910_154213 -IncludeBin

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -OpenDashboard
#>

[CmdletBinding()]
param(
    [string]$PiHost = "172.20.10.10",
    [string]$PiUser = "pi",
    [string]$RemoteCaptureRoot = "/home/pi/radar_runs/awr2944p",
    [string]$LocalCaptureRoot = "D:\radar_runs\awr2944p",
    [string]$RunId = "",
    [switch]$IncludeBin,
    [switch]$OpenDashboard
)

$ErrorActionPreference = "Stop"

function Invoke-Checked([string]$FilePath, [string[]]$Arguments) {
    Write-Host "[RUN] $FilePath $($Arguments -join ' ')" -ForegroundColor Cyan
    $output = & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $FilePath"
    }
    return $output
}

function Assert-SafeRunId([string]$Value) {
    if ($Value -notmatch '^\d{8}_\d{6}$') {
        throw "RunId must look like YYYYMMDD_HHMMSS, got: $Value"
    }
}

$remoteTarget = "${PiUser}@${PiHost}"
if (-not $RunId) {
    $findCommand = "find '$RemoteCaptureRoot' -mindepth 1 -maxdepth 1 -type d -name '[0-9]*_[0-9]*' -printf '%f\n' | sort | tail -n 1"
    $RunId = (Invoke-Checked "ssh" @($remoteTarget, $findCommand) | Select-Object -Last 1).Trim()
}
Assert-SafeRunId $RunId

$remoteRun = "$RemoteCaptureRoot/$RunId"
$localRun = Join-Path $LocalCaptureRoot $RunId
New-Item -ItemType Directory -Force -Path $localRun | Out-Null

# One recursive SCP session for all generated visual and numerical products.
Invoke-Checked "scp" @("-r", "${remoteTarget}:$remoteRun/range_analysis", $localRun) | Out-Null

# The capture-level report and metadata live next to range_analysis.
$topLevelFiles = @("output_analysis.md", "post_capture_analysis.json")
foreach ($name in $topLevelFiles) {
    Invoke-Checked "scp" @("${remoteTarget}:$remoteRun/$name", $localRun) | Out-Null
}

$metadataPath = (Invoke-Checked "ssh" @($remoteTarget, "find '$remoteRun' -maxdepth 1 -type f -name 'adc_data_*.json' -printf '%f\n' | sort | head -n 1") | Select-Object -Last 1).Trim()
if ($metadataPath) {
    Invoke-Checked "scp" @("${remoteTarget}:$remoteRun/$metadataPath", $localRun) | Out-Null
}

if ($IncludeBin) {
    $binPath = (Invoke-Checked "ssh" @($remoteTarget, "find '$remoteRun' -maxdepth 1 -type f -name '*.bin' -printf '%f\n' | sort | head -n 1") | Select-Object -Last 1).Trim()
    if (-not $binPath) {
        throw "No raw ADC BIN found in remote run: $remoteRun"
    }
    Invoke-Checked "scp" @("${remoteTarget}:$remoteRun/$binPath", $localRun) | Out-Null
}

Write-Host "[DONE] Run: $RunId" -ForegroundColor Green
Write-Host "[DONE] Local analysis: $(Join-Path $localRun 'range_analysis')" -ForegroundColor Green
$dashboardPath = Join-Path $localRun 'range_analysis\diagnostic_dashboard.png'
Write-Host "[OPEN] $dashboardPath" -ForegroundColor Green
if ($OpenDashboard -and (Test-Path -LiteralPath $dashboardPath)) {
    Start-Process -FilePath $dashboardPath
}
