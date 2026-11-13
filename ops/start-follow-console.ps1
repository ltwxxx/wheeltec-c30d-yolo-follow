$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$stateDir = Join-Path $env:LOCALAPPDATA 'WheeltecFollow'
New-Item -Path $stateDir -ItemType Directory -Force | Out-Null
Set-Location -LiteralPath $project
$wsl = Join-Path $env:WINDIR 'System32\wsl.exe'
$url = 'http://127.0.0.1:8769/'

# WSL can write a proxy notice to stderr even when startup succeeds.
$ErrorActionPreference = 'Continue'
& $wsl -d Ubuntu-F --exec bash ./ops/watch-follow.sh 2>> (Join-Path $stateDir 'watch.log') |
    ForEach-Object {
        if ($_ -eq 'WHEELTEC_READY') {
            $openUrl = $url
            try {
                Invoke-RestMethod -Uri "${url}healthz" -TimeoutSec 2 -ErrorAction Stop | Out-Null
            } catch {
                $fallback = 'http://127.0.0.1:8769/'
                try {
                    $health = Invoke-RestMethod -Uri "${fallback}healthz" -TimeoutSec 2 -ErrorAction Stop
                    if ($health.backend -eq 'robot' -and -not $health.view_only) {
                        $openUrl = $fallback
                    }
                } catch {}
            }
            Start-Process $openUrl -ErrorAction Stop
            Get-Date -Format o | Set-Content -LiteralPath (Join-Path $stateDir 'last-open.txt') -ErrorAction Stop
        }
    }
exit $LASTEXITCODE
