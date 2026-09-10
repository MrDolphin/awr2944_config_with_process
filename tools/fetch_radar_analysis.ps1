<#
.SYNOPSIS
  Copy Raspberry Pi radar-analysis artifacts to Windows.

.DESCRIPTION
  With no selection switch, the script copies the latest capture's analysis.
  Use -SyncMissing to copy every run directory present on the Pi but absent
  beneath the local capture root. Analysis artifacts are always copied; raw
  ADC BIN files remain opt-in through -IncludeBin to avoid accidental bulk
  transfers of large recordings.

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -SyncMissing -IncludeBin -OpenDashboard

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -RunId 20260910_154213 -IncludeBin
#>

[CmdletBinding()]
param(
    [string]$PiHost = "172.20.10.10",
    [string]$PiUser = "pi",
    [string]$RemoteCaptureRoot = "/home/pi/radar_runs/awr2944p",
    [string]$LocalCaptureRoot = "D:\radar_runs\awr2944p",
    [string]$RunId = "",
    [switch]$SyncMissing,
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

function Get-RemoteRunIds([string]$Target) {
    $findCommand = "find '$RemoteCaptureRoot' -mindepth 1 -maxdepth 1 -type d -name '[0-9]*_[0-9]*' -printf '%f\n' | sort"
    return @(Invoke-Checked "ssh" @($Target, $findCommand) |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_ })
}

function Copy-RemoteFileIfPresent([string]$Target, [string]$RemoteRun, [string]$LocalRun, [string]$NamePattern) {
    $findCommand = "find '$RemoteRun' -maxdepth 1 -type f -name '$NamePattern' -printf '%f\n' | sort | head -n 1"
    $fileProbe = @(Invoke-Checked "ssh" @($Target, $findCommand))
    $fileName = if ($fileProbe) { $fileProbe[-1].Trim() } else { "" }
    if ($fileName) {
        Invoke-Checked "scp" @("${Target}:$RemoteRun/$fileName", $LocalRun) | Out-Null
    }
    return $fileName
}

function Copy-OneRun([string]$Target, [string]$SelectedRunId) {
    Assert-SafeRunId $SelectedRunId
    $remoteRun = "$RemoteCaptureRoot/$SelectedRunId"
    $localRun = Join-Path $LocalCaptureRoot $SelectedRunId
    New-Item -ItemType Directory -Force -Path $localRun | Out-Null

    # Older captures can predate the analysis pipeline. They remain usable as
    # raw-data/metadata archives, so do not abort a batch when this directory
    # is absent.
    $remoteAnalysis = "$remoteRun/range_analysis"
    $rangeAnalysisProbe = @(Invoke-Checked "ssh" @($Target, "if [ -d '$remoteAnalysis' ]; then printf yes; fi"))
    $hasRangeAnalysis = if ($rangeAnalysisProbe) { $rangeAnalysisProbe[-1].Trim() } else { "" }
    if ($hasRangeAnalysis -eq "yes") {
        Invoke-Checked "scp" @("-r", "${Target}:$remoteAnalysis", $localRun) | Out-Null
    }
    else {
        Write-Host "[SKIP] No range_analysis directory for old run: $SelectedRunId" -ForegroundColor Yellow
    }

    # These top-level files can be absent in older runs, so copy them only when present.
    Copy-RemoteFileIfPresent $Target $remoteRun $localRun "output_analysis.md" | Out-Null
    Copy-RemoteFileIfPresent $Target $remoteRun $localRun "post_capture_analysis.json" | Out-Null
    Copy-RemoteFileIfPresent $Target $remoteRun $localRun "adc_data_*.json" | Out-Null

    if ($IncludeBin) {
        $binPath = Copy-RemoteFileIfPresent $Target $remoteRun $localRun "adc_data_*.bin"
        if (-not $binPath) {
            throw "No raw ADC BIN found in remote run: $remoteRun"
        }
    }

    $dashboardPath = Join-Path $localRun 'range_analysis\diagnostic_dashboard.png'
    Write-Host "[DONE] Run: $SelectedRunId" -ForegroundColor Green
    Write-Host "[DONE] Local analysis: $(Join-Path $localRun 'range_analysis')" -ForegroundColor Green
    Write-Host "[OPEN] $dashboardPath" -ForegroundColor Green
    return $dashboardPath
}

if ($RunId -and $SyncMissing) {
    throw "Use either -RunId or -SyncMissing, not both."
}

$remoteTarget = "${PiUser}@${PiHost}"
$remoteRunIds = Get-RemoteRunIds $remoteTarget
if (-not $remoteRunIds) {
    throw "No capture directories found below $RemoteCaptureRoot"
}

if ($RunId) {
    Assert-SafeRunId $RunId
    $selectedRunIds = @($RunId)
}
elseif ($SyncMissing) {
    $selectedRunIds = @(
        $remoteRunIds | Where-Object {
            -not (Test-Path -LiteralPath (Join-Path $LocalCaptureRoot $_))
        }
    )
    if (-not $selectedRunIds) {
        Write-Host "[DONE] No Pi capture directories are missing from $LocalCaptureRoot" -ForegroundColor Green
        exit 0
    }
}
else {
    $selectedRunIds = @($remoteRunIds | Select-Object -Last 1)
}

$lastDashboard = ""
foreach ($selectedRunId in $selectedRunIds) {
    $lastDashboard = Copy-OneRun $remoteTarget $selectedRunId
}

if ($OpenDashboard -and $lastDashboard -and (Test-Path -LiteralPath $lastDashboard)) {
    Start-Process -FilePath $lastDashboard
}
