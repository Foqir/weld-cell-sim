# Builds the image stage by stage (deps -> hdr-build -> final).
# After every finished stage all its layers are exported to backup\buildcache, so a network
# failure or a Docker reset never forces re-downloading finished parts. The final image is
# also saved to backup\weld-cell-sim.tar (restore with restore.bat, no internet needed).
# Usage: build.ps1 [-Stages deps,hdr-build,final] [-NoSave]
param(
    [string[]]$Stages = @('deps', 'hdr-build', 'final'),
    [switch]$NoSave
)
Set-Location $PSScriptRoot
$env:Path = "C:\Program Files\Docker\Docker\resources\bin;" + $env:Path
$backup = Join-Path $PSScriptRoot 'backup'
$cache = Join-Path $backup 'buildcache'
$logs = Join-Path $backup 'logs'
New-Item -ItemType Directory -Force $cache, $logs | Out-Null

foreach ($stage in $Stages) {
    $tag = if ($stage -eq 'final') { 'weld-cell-sim:latest' } else { "weld-cell-sim:$stage" }
    $log = Join-Path $logs "$stage.log"
    $ok = $false
    for ($attempt = 1; $attempt -le 3 -and -not $ok; $attempt++) {
        Write-Host "=== [$stage] attempt $attempt $(Get-Date -Format T) ===" -ForegroundColor Cyan
        $cacheFrom = ''
        if (Test-Path (Join-Path $cache 'index.json')) { $cacheFrom = "--cache-from type=local,src=`"$cache`"" }
        cmd /c "docker build --progress=plain --target $stage -t $tag $cacheFrom --cache-to type=local,dest=`"$cache`",mode=max . 2>&1" |
            Tee-Object -FilePath $log -Append
        $ok = $LASTEXITCODE -eq 0
        if (-not $ok) { Start-Sleep 30 }
    }
    if (-not $ok) {
        Write-Host "Stage '$stage' failed 3 times, see $log" -ForegroundColor Red
        exit 1
    }
    Write-Host "=== [$stage] done, layers saved to backup\buildcache ===" -ForegroundColor Green
}

if (-not $NoSave -and $Stages -contains 'final') {
    Write-Host "Saving final image to backup\weld-cell-sim.tar ..." -ForegroundColor Cyan
    cmd /c "docker save weld-cell-sim:latest -o `"$backup\weld-cell-sim.tar`""
    if ($LASTEXITCODE -ne 0) { exit 1 }
}
Write-Host "BUILD COMPLETE" -ForegroundColor Green
