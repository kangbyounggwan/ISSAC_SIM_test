param([string]$RunName="h1_nav_001",[int]$NumEnvs=512,[int]$Iterations=1000,[switch]$Smoke)
$ErrorActionPreference="Stop"
$Out=Join-Path $PSScriptRoot "runs\$RunName"
if(Test-Path -LiteralPath $Out){throw "Output exists. Use a new RunName: $Out"}
New-Item -ItemType Directory -Path $Out | Out-Null
$ArgsList=@((Join-Path $PSScriptRoot "train.py"),"--out",$Out,"--num_envs",$NumEnvs,"--max_iterations",$Iterations,"--headless","--device","cuda:0")
if($Smoke){$ArgsList+="--smoke"}
$OldConda=$env:CONDA_PREFIX
Remove-Item Env:CONDA_PREFIX -ErrorAction SilentlyContinue
try{
    & C:\isaacsim\python.bat @ArgsList *> (Join-Path $Out "console.log")
} finally {
    if($null -ne $OldConda){$env:CONDA_PREFIX=$OldConda}
}
$StatusPath=Join-Path $Out "status.json"
if(-not(Test-Path -LiteralPath $StatusPath)){throw "No training status; see $Out\console.log"}
$Status=Get-Content -LiteralPath $StatusPath -Encoding UTF8 -Raw | ConvertFrom-Json
$Status | ConvertTo-Json -Depth 6
if($Status.state -eq "FAILED"){throw "RL training failed. See $Out\exception.txt"}
