$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$stateDir = Join-Path $env:LOCALAPPDATA 'WheeltecFollow'
New-Item -Path $stateDir -ItemType Directory -Force | Out-Null
Set-Location -LiteralPath $project
$wsl = Join-Path $env:WINDIR 'System32\wsl.exe'
$curl = Join-Path $env:WINDIR 'System32\curl.exe'
$url = 'http://127.0.0.1:8769/'

function Open-ControlPageWhenAvailable {
    # Windows can expose the WSL loopback forward after the Linux check succeeds.
    for ($attempt = 0; $attempt -lt 16; $attempt++) {
        $response = & $curl --noproxy '*' --fail --silent --max-time 2 "${url}healthz"
        if ($LASTEXITCODE -eq 0) {
            $health = $response | ConvertFrom-Json -ErrorAction SilentlyContinue
            if ($health.backend -in 'robot', 'wsl' -and $health.view_only -eq $false) {
                Start-Process $url -ErrorAction Stop
                Get-Date -Format o | Set-Content -LiteralPath (Join-Path $stateDir 'last-open.txt') -ErrorAction Stop
                return
            }
        }
        Start-Sleep -Milliseconds 750
    }
    throw 'The Wheeltec console did not become reachable from Windows.'
}

$launcher = [System.Threading.Mutex]::new($false, 'Local\WheeltecFollowConsole')
try {
    try { $ownsLauncher = $launcher.WaitOne(0) }
    catch [System.Threading.AbandonedMutexException] { $ownsLauncher = $true }
    if (-not $ownsLauncher) { exit 0 }

    $launchFile = Join-Path $stateDir 'watch-instance.txt'
    $launchToken = [guid]::NewGuid().ToString('N')
    Set-Content -LiteralPath $launchFile -Value $launchToken -Encoding Ascii

    # WSL can write a proxy notice to stderr even when startup succeeds.
    $ErrorActionPreference = 'Continue'
    & $wsl -d Ubuntu-F --exec bash ./ops/watch-follow.sh $launchFile $launchToken 2>> (Join-Path $stateDir 'watch.log') |
        ForEach-Object {
            if ($_ -eq 'WHEELTEC_READY') {
                Open-ControlPageWhenAvailable
            }
        }
    # A watcher has no normal exit while the task is running. Let Task Scheduler retry.
    if ($LASTEXITCODE -eq 0) { exit 1 }
    exit $LASTEXITCODE
} finally {
    if ($ownsLauncher) { $launcher.ReleaseMutex() }
    $launcher.Dispose()
}
