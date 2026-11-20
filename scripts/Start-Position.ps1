& {
    # Use the existing, separately verified motor/sensor position scripts.
    # Each child closes its serial port before returning to this menu.
    $positionScripts = @{
        '1' = (Join-Path $PSScriptRoot 'Test-MDD10A-Position.ps1')
        '2' = (Join-Path $PSScriptRoot 'Test-MDD10A2-Position.ps1')
    }
    while ($true) {
        Write-Host ''
        Write-Host '모터 1 → 센서 1(PA0), 중앙 ADC 522'
        Write-Host '모터 2 → 센서 2(PA1), 중앙 ADC 529'
        Write-Host '한 번에 선택한 모터 하나만 움직입니다.'
        $selectedMotor = (Read-Host '모터 선택: 1 / 2 / Q(전체 종료)').Trim().ToUpperInvariant()
        if ($selectedMotor -eq 'Q') { break }
        if (!$positionScripts.ContainsKey($selectedMotor)) {
            Write-Host '1, 2 또는 Q를 입력하세요.'
            continue
        }
        $positionScriptPath = $positionScripts[$selectedMotor]
        if (!(Test-Path -LiteralPath $positionScriptPath -PathType Leaf)) {
            Write-Host "스크립트를 찾지 못했습니다: $positionScriptPath" -ForegroundColor Yellow
            break
        }
        Write-Host "모터 $selectedMotor 선택. 다음 메뉴에서 HOVER / UP / DOWN을 입력하세요." -ForegroundColor Cyan
        Write-Host '이동 중 S: 정지. 명령 메뉴에서 Q: 모터 선택으로 돌아가기.'
        try {
            & ([scriptblock]::Create((Get-Content -Raw -Encoding UTF8 -LiteralPath $positionScriptPath)))
        }
        catch {
            Write-Host ('시험 중단: ' + $_.Exception.Message) -ForegroundColor Yellow
        }
    }
}
