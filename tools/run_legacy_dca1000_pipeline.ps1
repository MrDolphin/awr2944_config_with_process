<#
.SYNOPSIS
  Test runner for the legacy AWR2944 + DCA1000 + MATLAB capture workflow.

.DESCRIPTION
  This script intentionally does not touch radar_server.py. It automates the
  previous manual workflow:
    1. copy helper/cfg to the router or Raspberry Pi
    2. configure AWR2944 over the remote serial CLI, skipping sensorStart
    3. start MATLAB locally to configure DCA1000 and wait for UDP data
    4. after MATLAB writes a ready flag, start the radar remotely
    5. wait for MATLAB to save the .mat file, then stop the radar

.EXAMPLE
  .\tools\run_legacy_dca1000_pipeline.ps1 `
    -RemoteHost pi@192.168.33.30 `
    -RemoteSerialPort /dev/ttyACM0 `
    -CfgPath mathlab\Python4IWR\cfg\profile.cfg `
    -MatlabExe "C:\Program Files\MATLAB\R2023b\bin\matlab.exe"
#>

[CmdletBinding()]
param(
    [string]$RemoteHost = "pi@192.168.33.30",
    [string]$RemoteWorkDir = "/tmp/awr2944_dca1000",
    [string]$RemoteSerialPort = "/dev/ttyACM0",
    [string]$CfgPath = "mathlab\Python4IWR\cfg\profile.cfg",
    [string]$MatlabExe = "matlab",
    [string]$PostProcDir = "",
    [string]$OutputDir = "captures\legacy_mat",
    [string]$OutputPrefix = "legacy_dca1000",
    [int]$NumFrames = 256,
    [int]$ChirpNum = 255,
    [int]$AdcSamples = 256,
    [int]$ReadyTimeoutSec = 60,
    [switch]$ResumeStart,
    [switch]$SkipRemoteCopy,
    [switch]$SkipRemoteConfigure,
    [switch]$SkipRemoteStart,
    [switch]$NoDcaControl
)

$ErrorActionPreference = "Stop"

function Resolve-RepoPath([string]$PathValue) {
    if ([System.IO.Path]::IsPathRooted($PathValue)) {
        return (Resolve-Path -LiteralPath $PathValue).Path
    }
    return (Resolve-Path -LiteralPath (Join-Path $PWD $PathValue)).Path
}

function Quote-MatlabString([string]$Value) {
    return $Value.Replace("'", "''")
}

function Invoke-Checked([string]$FilePath, [string[]]$Arguments) {
    Write-Host "[RUN] $FilePath $($Arguments -join ' ')" -ForegroundColor Cyan
    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $FilePath"
    }
}

$repoRoot = (Resolve-Path -LiteralPath $PWD).Path
$cfgFullPath = Resolve-RepoPath $CfgPath
$remoteHelperFullPath = Resolve-RepoPath "tools\awr2944_cli_control.py"

if (-not $PostProcDir) {
    $PostProcDir = Join-Path $repoRoot "mathlab\PostProc"
}
$postProcFullPath = Resolve-RepoPath $PostProcDir

$outputFullPath = if ([System.IO.Path]::IsPathRooted($OutputDir)) {
    $OutputDir
} else {
    Join-Path $repoRoot $OutputDir
}
New-Item -ItemType Directory -Force -Path $outputFullPath | Out-Null

$flagDir = Join-Path $outputFullPath "_flags"
New-Item -ItemType Directory -Force -Path $flagDir | Out-Null
$readyFlag = Join-Path $flagDir "dca1000_ready.flag"
Remove-Item -LiteralPath $readyFlag -Force -ErrorAction SilentlyContinue

$remoteHelper = "$RemoteWorkDir/awr2944_cli_control.py"
$remoteCfg = "$RemoteWorkDir/profile.cfg"

if (-not $SkipRemoteCopy) {
    Invoke-Checked "ssh" @($RemoteHost, "mkdir -p '$RemoteWorkDir'")
    Invoke-Checked "scp" @($remoteHelperFullPath, "${RemoteHost}:$remoteHelper")
    Invoke-Checked "scp" @($cfgFullPath, "${RemoteHost}:$remoteCfg")
}

if (-not $SkipRemoteConfigure) {
    Invoke-Checked "ssh" @(
        $RemoteHost,
        "python3 '$remoteHelper' --port '$RemoteSerialPort' configure --cfg '$remoteCfg' --no-start"
    )
}

$runDcaControlText = if ($NoDcaControl) { "false" } else { "true" }
$matlabCmd = @"
addpath('$(Quote-MatlabString (Join-Path $repoRoot 'mathlab'))');
run_legacy_dca1000_capture_once( ...
    'PostProcDir','$(Quote-MatlabString $postProcFullPath)', ...
    'OutputDir','$(Quote-MatlabString $outputFullPath)', ...
    'OutputPrefix','$(Quote-MatlabString $OutputPrefix)', ...
    'ReadyFlag','$(Quote-MatlabString $readyFlag)', ...
    'NumFrames',$NumFrames, ...
    'ChirpNum',$ChirpNum, ...
    'AdcSamples',$AdcSamples, ...
    'RunDcaControl',$runDcaControlText);
"@
$matlabCmd = ($matlabCmd -replace "`r?`n", " ")

Write-Host "[MATLAB] Starting local capture process..." -ForegroundColor Cyan
$matlabProcess = Start-Process -FilePath $MatlabExe -ArgumentList @("-batch", $matlabCmd) -PassThru

try {
    $deadline = (Get-Date).AddSeconds($ReadyTimeoutSec)
    while (-not (Test-Path -LiteralPath $readyFlag)) {
        if ($matlabProcess.HasExited) {
            throw "MATLAB exited before DCA1000 ready flag appeared. ExitCode=$($matlabProcess.ExitCode)"
        }
        if ((Get-Date) -gt $deadline) {
            throw "Timed out waiting for DCA1000 ready flag: $readyFlag"
        }
        Start-Sleep -Milliseconds 500
    }
    Write-Host "[OK] DCA1000 receiver ready. Starting radar..." -ForegroundColor Green

    if (-not $SkipRemoteStart) {
        $startArgs = if ($ResumeStart) { "--resume" } else { "" }
        Invoke-Checked "ssh" @(
            $RemoteHost,
            "python3 '$remoteHelper' --port '$RemoteSerialPort' start $startArgs"
        )
    }

    Wait-Process -Id $matlabProcess.Id
    if ($matlabProcess.ExitCode -ne 0) {
        throw "MATLAB capture failed. ExitCode=$($matlabProcess.ExitCode)"
    }
    Write-Host "[DONE] MATLAB capture finished. Output directory: $outputFullPath" -ForegroundColor Green
}
finally {
    if (-not $SkipRemoteStart) {
        try {
            Invoke-Checked "ssh" @(
                $RemoteHost,
                "python3 '$remoteHelper' --port '$RemoteSerialPort' stop"
            )
        } catch {
            Write-Warning "Failed to stop radar remotely: $_"
        }
    }
}
