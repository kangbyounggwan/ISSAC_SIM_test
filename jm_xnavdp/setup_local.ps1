param([string]$BasePython = '')
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$lock = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'upstream_lock.json') -Raw | ConvertFrom-Json
if (-not $BasePython) { $BasePython = Join-Path $root '.conda/wb7-pink-runtime/python.exe' }
$repo = Join-Path $root $lock.code_path
$model = Join-Path $root $lock.model_path
$venv = Join-Path $root '.conda/xnavdp-runtime'
$python = Join-Path $venv 'Scripts/python.exe'
if (-not (Test-Path -LiteralPath $repo)) {
    & git clone --no-checkout $lock.code_url $repo
    if ($LASTEXITCODE -ne 0) { throw 'Clone failed' }
    & git -C $repo checkout --detach $lock.code_commit
    if ($LASTEXITCODE -ne 0) { throw 'Pinned checkout failed' }
}
$commit = & git -C $repo rev-parse HEAD
if ($commit -ne $lock.code_commit) { throw 'Existing upstream checkout differs; not overwriting it' }
New-Item -ItemType Directory -Force -Path (Split-Path $model -Parent) | Out-Null
if (-not (Test-Path -LiteralPath $model)) {
    $partial = "$model.part"
    if (Test-Path -LiteralPath $partial) { throw 'Partial download exists; inspect it before retrying' }
    $url = "https://huggingface.co/$($lock.model_repo)/resolve/$($lock.model_revision)/$($lock.model_file)"
    & curl.exe -L --fail --retry 2 --output $partial $url
    if ($LASTEXITCODE -ne 0) { throw 'Checkpoint download failed' }
    if ((Get-FileHash -LiteralPath $partial -Algorithm SHA256).Hash.ToLower() -ne $lock.model_sha256) {
        throw 'Downloaded checksum mismatch'
    }
    Move-Item -LiteralPath $partial -Destination $model
}
if ((Get-FileHash -LiteralPath $model -Algorithm SHA256).Hash.ToLower() -ne $lock.model_sha256) {
    throw 'Checkpoint checksum mismatch'
}
if (-not (Test-Path -LiteralPath $python)) {
    & $BasePython -m venv $venv
    if ($LASTEXITCODE -ne 0) { throw 'Virtual environment creation failed' }
}
& $python -m pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
if ($LASTEXITCODE -ne 0) { throw 'PyTorch installation failed' }
& $python -m pip install -r (Join-Path $PSScriptRoot 'requirements-inference.txt')
if ($LASTEXITCODE -ne 0) { throw 'Inference dependencies failed' }
& $python -m pip check
if ($LASTEXITCODE -ne 0) { throw 'Dependency check failed' }
Write-Output 'X-NavDP isolated inference setup ready. H1 actuation is NOT enabled.'
