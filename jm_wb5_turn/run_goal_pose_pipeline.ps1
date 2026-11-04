param(
    [string]$RunName = 'train_001',
    [int]$NumEnvs = 512,
    [int]$Iterations = 1000,
    [string]$Readiness = 'artifacts/wb5_goal_turn_20261002/preflight_003'
)
$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $Root
if ($RunName -notmatch '^[a-zA-Z0-9_-]+$') { throw 'RunName must be a simple directory name' }
$Train = "artifacts/wb5_goal_turn_20261002/$RunName"
$Video = "artifacts/wb5_goal_turn_20261002/${RunName}_video"
if ((Test-Path -LiteralPath $Train) -or (Test-Path -LiteralPath $Video)) {
    throw 'Fresh output directories required; nothing will be overwritten'
}
& C:/isaacsim/python.bat -u jm_wb5_turn/train_goal_pose.py --out $Train --num_envs $NumEnvs --max_iterations $Iterations --readiness $Readiness --headless --device cuda:0
$State = Get-Content -LiteralPath "$Train/status.json" -Raw | ConvertFrom-Json
if ($State.state -ne 'TRAINING_COMPLETE') { throw 'Training did not complete; video stage not started' }
& C:/isaacsim/python.bat -u jm_wb5_turn/record_goal_pose.py --policy "$Train/policy_goal_pose_candidate.pt" --out $Video --headless --device cuda:0
$State = Get-Content -LiteralPath "$Video/status.json" -Raw | ConvertFrom-Json
if ($State.state -ne 'EVALUATION_COMPLETE') { throw 'Video evaluation failed; do not claim completion' }
& C:/isaacsim/python.bat jm_wb5_turn/verify_recordings.py $Video
$Qa = Get-Content -LiteralPath "$Video/video_qa.json" -Raw | ConvertFrom-Json
$Bad = @($Qa.PSObject.Properties | Where-Object { -not $_.Value.passed })
if ($Bad.Count -gt 0 -or @($Qa.PSObject.Properties).Count -ne 6) { throw 'Six 720p videos did not pass file QA' }
Write-Output 'PIPELINE_COMPLETE: training and video file QA complete; inspect motion metrics before promotion.'
