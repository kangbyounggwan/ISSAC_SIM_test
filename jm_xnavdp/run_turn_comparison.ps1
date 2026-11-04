param([string]$RunTag = '003')
$ErrorActionPreference = 'Stop'
$trials = @(
    @('official','official','wb5',-1,'official_wb5contract_neg'),
    @('wb5','official','wb5',-1,'wb5_original_neg'),
    @('official','jm','wb5',-1,'official_jm_neg'),
    @('wb5','jm','wb5',-1,'wb5_jm_neg'),
    @('official','official','wb5',1,'official_wb5contract_pos'),
    @('wb5','official','wb5',1,'wb5_original_pos'),
    @('official','jm','wb5',1,'official_jm_pos'),
    @('wb5','jm','wb5',1,'wb5_jm_pos')
)
foreach ($trial in $trials) {
    $output = "artifacts/xnavdp_20260930/turn_compare_$($trial[4])_$RunTag"
    if (Test-Path -LiteralPath $output) { throw "Fresh output required: $output" }
    & 'C:/isaacsim/python.bat' jm_xnavdp/compare_turn_policies.py --policy $trial[0] --asset $trial[1] --contract $trial[2] --sign $trial[3] --out $output *> "$output.log"
    if (-not (Test-Path -LiteralPath "$output/result.json")) { throw "Comparison failed to produce result: $output" }
    Get-Content -LiteralPath "$output/result.json"
}
