$ErrorActionPreference = 'Stop'
$workspacePath = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$pythonPath = Join-Path $workspacePath '.venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $pythonPath)) { $pythonPath = 'python' }
& $pythonPath (Join-Path $PSScriptRoot 'backend/server.py') --workspace $workspacePath @args
exit $LASTEXITCODE
