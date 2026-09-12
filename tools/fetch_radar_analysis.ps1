<#
.SYNOPSIS
  Copy Raspberry Pi radar-analysis artifacts to Windows.

.DESCRIPTION
  By default, inventory timestamped captures below /home/pi/radar_runs across
  all capture families (for example awr2944p_shore350m_v0), newest first.
  A capture is transferred only when its complete relative run directory does
  not yet exist locally. Existing local run directories are not probed again.
  Analysis artifacts are included; raw ADC BIN files remain opt-in through
  -IncludeBin to avoid accidental bulk transfers of large recordings. Select a
  remembered network name with -NetworkMode instead of entering Raspberry Pi
  IP addresses.

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -NetworkMode lab -IncludeBin -OpenDashboard

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -IncludeBin

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -NetworkMode hotspot -IncludeBin

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -NetworkMode phone -IncludeBin

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -NetworkMode phone -AnalyzeOnPc

.EXAMPLE
  .\tools\fetch_radar_analysis.ps1 -NetworkMode lab `
    -RemoteCaptureRoot /home/pi/radar_runs/awr2944p_shore200m_v0 `
    -LocalCaptureRoot D:\radar_runs\awr2944p_shore200m_v0 `
    -AnalyzeOnPc -MaxRangeM 180
#>

[CmdletBinding()]
param(
    [ValidateSet("lab", "hotspot", "phone")]
    [string]$NetworkMode = "lab",
    [string]$PiUser = "pi",
    # Leave both CaptureRoot arguments empty for the default all-family sync.
    # Specify both to retain the former one-capture-family behavior.
    [string]$RemoteCaptureRoot = "",
    [string]$RemoteProjectRoot = "/home/pi/awr2944_config_with_process_github",
    [string]$LocalCaptureRoot = "",
    [string]$RemoteRunsRoot = "/home/pi/radar_runs",
    [string]$LocalRunsRoot = "D:\radar_runs",
    [string]$RunId = "",
    # Retained as a no-op compatibility switch; incremental sync is now the default.
    [switch]$SyncMissing,
    [switch]$IncludeBin,
    [switch]$AnalyzeOnPc,
    # Zero defers to each capture's exact CFG range CFAR FOV on the PC.
    [ValidateRange(0.0, 10000.0)]
    [double]$MaxRangeM = 0.0,
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

function Assert-SafeRelativePath([string]$Value) {
    if ([string]::IsNullOrWhiteSpace($Value) -or
        [System.IO.Path]::IsPathRooted($Value) -or
        $Value -match '(^|[\\/])\.\.([\\/]|$)' -or
        $Value -notmatch '^[A-Za-z0-9._/-]+$') {
        throw "Unsafe relative path returned by remote host: $Value"
    }
}

function Get-RemoteRunPaths([string]$Target, [string]$Root, [bool]$AllFamilies) {
    $findCommand = if ($AllFamilies) {
        "find '$Root' -mindepth 2 -maxdepth 2 -type d -name '[0-9]*_[0-9]*' -printf '%P\n'"
    }
    else {
        "find '$Root' -mindepth 1 -maxdepth 1 -type d -name '[0-9]*_[0-9]*' -printf '%f\n'"
    }
    $paths = @(Invoke-Checked "ssh" @($Target, $findCommand) |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_ })
    # Sort on Windows by the final YYYYMMDD_HHMMSS component. Keeping this
    # operation local avoids remote shell/awk quoting ambiguity across SSH.
    return @($paths | Sort-Object { ($_ -split '/')[-1] } -Descending)
}

function Get-LocalRunPathSet([string]$Root, [bool]$AllFamilies) {
    $paths = @{}
    if (-not (Test-Path -LiteralPath $Root -PathType Container)) {
        return $paths
    }

    $resolvedRoot = (Resolve-Path -LiteralPath $Root).Path.TrimEnd('\', '/')
    $directories = if ($AllFamilies) {
        @(Get-ChildItem -LiteralPath $resolvedRoot -Directory -Recurse |
            Where-Object { $_.Name -match '^\d{8}_\d{6}$' })
    }
    else {
        @(Get-ChildItem -LiteralPath $resolvedRoot -Directory |
            Where-Object { $_.Name -match '^\d{8}_\d{6}$' })
    }

    foreach ($directory in $directories) {
        $relativePath = $directory.FullName.Substring($resolvedRoot.Length).TrimStart('\', '/')
        if (-not [string]::IsNullOrWhiteSpace($relativePath)) {
            $paths[$relativePath.Replace('\', '/')] = $true
        }
    }
    return $paths
}

function Copy-RemoteFileIfMissing([string]$Target, [string]$RemoteRun, [string]$LocalRun, [string]$NamePattern) {
    $findCommand = "find '$RemoteRun' -maxdepth 1 -type f -name '$NamePattern' -printf '%f\n' | sort | head -n 1"
    $fileProbe = @(Invoke-Checked "ssh" @($Target, $findCommand))
    $fileName = if ($fileProbe) { $fileProbe[-1].Trim() } else { "" }
    if ($fileName) {
        $localPath = Join-Path $LocalRun $fileName
        if (Test-Path -LiteralPath $localPath) {
            Write-Host "[SKIP] Already exists: $localPath" -ForegroundColor DarkGray
        }
        else {
            Invoke-Checked "scp" @("${Target}:$RemoteRun/$fileName", $LocalRun) | Out-Null
        }
    }
    return $fileName
}

function Copy-RemoteTreeMissing([string]$Target, [string]$RemoteRoot, [string]$LocalRoot) {
    New-Item -ItemType Directory -Force -Path $LocalRoot | Out-Null
    $findCommand = "find '$RemoteRoot' -type f -printf '%P\n' | sort"
    $relativePaths = @(Invoke-Checked "ssh" @($Target, $findCommand) |
        ForEach-Object { $_.Trim() } |
        Where-Object { $_ })
    foreach ($relativePath in $relativePaths) {
        Assert-SafeRelativePath $relativePath
        $windowsRelativePath = $relativePath.Replace('/', [System.IO.Path]::DirectorySeparatorChar)
        $localPath = Join-Path $LocalRoot $windowsRelativePath
        if (Test-Path -LiteralPath $localPath) {
            Write-Host "[SKIP] Already exists: $localPath" -ForegroundColor DarkGray
            continue
        }
        $localParent = Split-Path -Parent $localPath
        New-Item -ItemType Directory -Force -Path $localParent | Out-Null
        Invoke-Checked "scp" @("${Target}:$RemoteRoot/$relativePath", $localPath) | Out-Null
    }
}

function Copy-LegacyCaptureCfg([string]$Target, [string]$RemoteRun, [string]$LocalRun) {
    $destination = Join-Path $LocalRun "capture_config_recovered.cfg"
    if (Test-Path -LiteralPath $destination) {
        Write-Host "[SKIP] Already exists: $destination" -ForegroundColor DarkGray
        return
    }

    $metadataFile = Get-ChildItem -LiteralPath $LocalRun -Filter "adc_data_*.json" -File |
        Sort-Object Name |
        Select-Object -First 1
    if (-not $metadataFile) {
        Write-Host "[SKIP] Cannot recover CFG for legacy run without metadata: $RemoteRun" -ForegroundColor Yellow
        return
    }

    try {
        $metadata = Get-Content -LiteralPath $metadataFile.FullName -Raw | ConvertFrom-Json
    }
    catch {
        Write-Host "[SKIP] Cannot parse legacy metadata: $($metadataFile.FullName)" -ForegroundColor Yellow
        return
    }

    $cfgReference = $metadata.radar_cfg.cfg_path
    if ([string]::IsNullOrWhiteSpace($cfgReference)) {
        Write-Host "[SKIP] Legacy metadata has no radar_cfg.cfg_path: $($metadataFile.FullName)" -ForegroundColor Yellow
        return
    }
    Assert-SafeRelativePath $cfgReference

    $remoteCfg = "$RemoteProjectRoot/$cfgReference"
    $cfgProbe = @(Invoke-Checked "ssh" @($Target, "if [ -f '$remoteCfg' ]; then printf yes; fi"))
    $hasCfg = if ($cfgProbe) { $cfgProbe[-1].Trim() } else { "" }
    if ($hasCfg -ne "yes") {
        Write-Host "[SKIP] Current Pi CFG no longer exists: $remoteCfg" -ForegroundColor Yellow
        return
    }

    Invoke-Checked "scp" @("${Target}:$remoteCfg", $destination) | Out-Null
    Write-Host "[WARN] Recovered current Pi CFG for legacy run: $destination" -ForegroundColor Yellow
    Write-Host "[WARN] No capture-time CFG hash exists; this is not an exact historical snapshot." -ForegroundColor Yellow
}

function Copy-OneRun([string]$Target, [string]$SelectedRunPath, [string]$RemoteBase, [string]$LocalBase) {
    Assert-SafeRelativePath $SelectedRunPath
    $selectedRunId = Split-Path -Leaf $SelectedRunPath
    Assert-SafeRunId $selectedRunId
    $remoteRun = "$RemoteBase/$SelectedRunPath"
    $windowsRelativePath = $SelectedRunPath.Replace('/', [System.IO.Path]::DirectorySeparatorChar)
    $localRun = Join-Path $LocalBase $windowsRelativePath
    New-Item -ItemType Directory -Force -Path $localRun | Out-Null

    # Older captures can predate the analysis pipeline. They remain usable as
    # raw-data/metadata archives, so do not abort a batch when this directory
    # is absent.
    $remoteAnalysis = "$remoteRun/range_analysis"
    $rangeAnalysisProbe = @(Invoke-Checked "ssh" @($Target, "if [ -d '$remoteAnalysis' ]; then printf yes; fi"))
    $hasRangeAnalysis = if ($rangeAnalysisProbe) { $rangeAnalysisProbe[-1].Trim() } else { "" }
    if ($hasRangeAnalysis -eq "yes") {
        $localAnalysis = Join-Path $localRun "range_analysis"
        Copy-RemoteTreeMissing $Target $remoteAnalysis $localAnalysis
    }
    else {
        Write-Host "[SKIP] No range_analysis directory for old run: $SelectedRunPath" -ForegroundColor Yellow
    }

    # These top-level files can be absent in older runs, so copy them only when present.
    Copy-RemoteFileIfMissing $Target $remoteRun $localRun "output_analysis.md" | Out-Null
    Copy-RemoteFileIfMissing $Target $remoteRun $localRun "post_capture_analysis.json" | Out-Null
    Copy-RemoteFileIfMissing $Target $remoteRun $localRun "adc_data_*.json" | Out-Null
    $cfgSnapshot = Copy-RemoteFileIfMissing $Target $remoteRun $localRun "capture_config.cfg"
    if (-not $cfgSnapshot) {
        Copy-LegacyCaptureCfg $Target $remoteRun $localRun
    }

    if ($IncludeBin -or $AnalyzeOnPc) {
        $binPath = Copy-RemoteFileIfMissing $Target $remoteRun $localRun "adc_data_*.bin"
        if (-not $binPath) {
            throw "No raw ADC BIN found in remote run: $remoteRun"
        }
    }

    $dashboardPath = Join-Path $localRun 'range_analysis\diagnostic_dashboard.png'
    Write-Host "[DONE] Run: $SelectedRunPath" -ForegroundColor Green
    Write-Host "[DONE] Local analysis: $(Join-Path $localRun 'range_analysis')" -ForegroundColor Green
    Write-Host "[OPEN] $dashboardPath" -ForegroundColor Green
    return $dashboardPath
}

$networkAddresses = @{
    lab = "172.20.10.10"
    hotspot = "10.42.0.1"
    phone = "192.168.43.36"
}
$effectivePiHost = $networkAddresses[$NetworkMode]
Write-Host "[MODE] ${NetworkMode}: using saved Raspberry Pi address $effectivePiHost" -ForegroundColor Cyan

$remoteTarget = "${PiUser}@${effectivePiHost}"
$hasRemoteCaptureRoot = -not [string]::IsNullOrWhiteSpace($RemoteCaptureRoot)
$hasLocalCaptureRoot = -not [string]::IsNullOrWhiteSpace($LocalCaptureRoot)
if ($hasRemoteCaptureRoot -ne $hasLocalCaptureRoot) {
    throw "Specify both RemoteCaptureRoot and LocalCaptureRoot, or neither for all-family sync."
}
if ($hasRemoteCaptureRoot) {
    $effectiveRemoteRoot = $RemoteCaptureRoot
    $effectiveLocalRoot = $LocalCaptureRoot
    $allFamilies = $false
}
else {
    $effectiveRemoteRoot = $RemoteRunsRoot
    $effectiveLocalRoot = $LocalRunsRoot
    $allFamilies = $true
}
Write-Host "[ROOT] Remote: $effectiveRemoteRoot" -ForegroundColor Cyan
Write-Host "[ROOT] Local:  $effectiveLocalRoot" -ForegroundColor Cyan

$remoteRunPaths = Get-RemoteRunPaths $remoteTarget $effectiveRemoteRoot $allFamilies
if (-not $remoteRunPaths) {
    throw "No capture directories found below $effectiveRemoteRoot"
}
$localRunPathSet = Get-LocalRunPathSet $effectiveLocalRoot $allFamilies
Write-Host "[INVENTORY] Pi timestamped runs: $($remoteRunPaths.Count); local run directories: $($localRunPathSet.Count)" -ForegroundColor Cyan

if ($RunId) {
    Assert-SafeRunId $RunId
    $selectedRunPaths = @($remoteRunPaths | Where-Object { (Split-Path -Leaf $_) -eq $RunId })
    if ($selectedRunPaths.Count -eq 0) {
        throw "RunId not found below ${effectiveRemoteRoot}: $RunId"
    }
}
else {
    # Default policy intentionally uses run-directory existence rather than
    # repeated per-file probes. A local timestamped run is treated as archived
    # and will not be queried on the Pi again; only wholly new Pi runs transfer.
    $selectedRunPaths = @($remoteRunPaths | Where-Object {
        if ($localRunPathSet.ContainsKey($_)) {
            Write-Host "[SKIP] Local run already exists: $_" -ForegroundColor DarkGray
            $false
        }
        else {
            $true
        }
    })
}

if ($selectedRunPaths.Count -eq 0) {
    Write-Host "[SKIP] No Pi capture directories are missing locally." -ForegroundColor Green
}
else {
    Write-Host "[PLAN] New Pi runs to transfer (newest first): $($selectedRunPaths -join ', ')" -ForegroundColor Cyan
}

$lastDashboard = ""
foreach ($selectedRunPath in $selectedRunPaths) {
    $lastDashboard = Copy-OneRun $remoteTarget $selectedRunPath $effectiveRemoteRoot $effectiveLocalRoot
}

if ($AnalyzeOnPc) {
    $pcAnalysisScript = Join-Path $PSScriptRoot "analyze_radar_captures.ps1"
    foreach ($selectedRunPath in $selectedRunPaths) {
        $windowsRelativePath = $selectedRunPath.Replace('/', [System.IO.Path]::DirectorySeparatorChar)
        $localRunPath = Join-Path $effectiveLocalRoot $windowsRelativePath
        $pcAnalysisArguments = @(
            "-NoProfile",
            "-ExecutionPolicy", "Bypass",
            "-File", $pcAnalysisScript,
            "-RunFolder", $localRunPath,
            "-MaxRangeM", ([string]::Format([Globalization.CultureInfo]::InvariantCulture, "{0}", $MaxRangeM))
        )
        Write-Host "[RUN] powershell $($pcAnalysisArguments -join ' ')" -ForegroundColor Cyan
        & powershell @pcAnalysisArguments
        if ($LASTEXITCODE -ne 0) {
            throw "PC analysis failed for $selectedRunPath with exit code ${LASTEXITCODE}"
        }
        $lastDashboard = Join-Path $localRunPath "pc_analysis\range_analysis\diagnostic_dashboard.png"
    }
}

if ($OpenDashboard -and $lastDashboard -and (Test-Path -LiteralPath $lastDashboard)) {
    Start-Process -FilePath $lastDashboard
}
