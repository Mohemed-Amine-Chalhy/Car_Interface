[CmdletBinding()]
param(
    [switch]$Check
)

$ErrorActionPreference = "Stop"
$RepositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location -LiteralPath $RepositoryRoot

$UvCommand = Get-Command uv -ErrorAction SilentlyContinue
if ($null -eq $UvCommand) {
    throw "uv is required. Install it from https://docs.astral.sh/uv/getting-started/installation/ and rerun this script."
}

$PythonVersion = (Get-Content -LiteralPath ".python-version" -Raw).Trim()
& uv python install $PythonVersion
if ($LASTEXITCODE -ne 0) { throw "Failed to install Python $PythonVersion." }

$SyncArguments = @("sync", "--locked", "--all-groups")
& uv @SyncArguments
if ($LASTEXITCODE -ne 0) { throw "Failed to synchronize the project environment." }

& uv run --locked --group dev pre-commit install --install-hooks --hook-type pre-commit --hook-type pre-push
if ($LASTEXITCODE -ne 0) { throw "Failed to install Git hooks." }

& uv run --locked --group dev python scripts/dev.py doctor
if ($LASTEXITCODE -ne 0) { throw "Environment diagnostics failed." }

if ($Check) {
    & uv run --locked --group dev python scripts/dev.py check
    if ($LASTEXITCODE -ne 0) { throw "Project checks failed." }
}

Write-Host "Environment ready. Run: uv run python scripts/dev.py run-sim"
