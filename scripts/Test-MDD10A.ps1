& {
    $motorPort = $null
    function Read-Reply([string]$pattern, [int]$timeoutMs = 3000) {
        $timer = [Diagnostics.Stopwatch]::StartNew()
        while ($timer.ElapsedMilliseconds -lt $timeoutMs) {
            try { $line = $motorPort.ReadLine().Trim() }
            catch [System.TimeoutException] { continue }
            if (!$line) { continue }
            Write-Host "< $line"
            if ($line.StartsWith('ERR')) { throw $line }
            if ($line -match $pattern) { return $line }
        }
        throw "응답 시간 초과: $pattern"
    }
    function Send-Command([string]$command, [string]$pattern) {
        Write-Host "> $command"
        $motorPort.WriteLine($command)
        $null = Read-Reply $pattern
    }
    try {
        $portName = Read-Host 'COM 번호 (Enter=COM5)'
        if ([string]::IsNullOrWhiteSpace($portName)) { $portName = 'COM5' }
        $motorText = Read-Host '모터 번호: 1 또는 2 (Enter=1)'
        if (!$motorText) { $motorText = '1' }
        if ($motorText -notmatch '^[12]$') { throw '모터 번호는 1 또는 2입니다.' }
        $directionText = Read-Host '방향: 1 또는 -1 (Enter=1)'
        if (!$directionText) { $directionText = '1' }
        if ($directionText -notmatch '^(1|-1)$') { throw '방향은 1 또는 -1입니다.' }
        $pwmText = Read-Host 'PWM: 1~100% (첫 무부하 시험 권장 30, Enter=30)'
        if (!$pwmText) { $pwmText = '30' }
        $pwmValue = 0
        if (![int]::TryParse($pwmText, [ref]$pwmValue) -or $pwmValue -lt 1 -or $pwmValue -gt 100) {
            throw 'PWM은 1~100 정수입니다.'
        }
        $secondsText = Read-Host '구동 시간(초): 0.02~20 (Enter=1)'
        if (!$secondsText) { $secondsText = '1' }
        $secondsValue = 0.0
        if (![double]::TryParse($secondsText, [Globalization.NumberStyles]::Float,
                [Globalization.CultureInfo]::InvariantCulture, [ref]$secondsValue) -or
            [double]::IsNaN($secondsValue) -or $secondsValue -lt 0.02 -or $secondsValue -gt 20) {
            throw '시간은 0.02~20초입니다. 소수점은 .을 사용하세요.'
        }
        $durationMs = [int][Math]::Round($secondsValue * 1000)
        $signedPwm = [int]$directionText * $pwmValue

        $motorPort = [System.IO.Ports.SerialPort]::new(
            $portName.Trim(), 115200, [System.IO.Ports.Parity]::None,
            8, [System.IO.Ports.StopBits]::One)
        $motorPort.Handshake = [System.IO.Ports.Handshake]::None
        $motorPort.NewLine = "`n"
        $motorPort.ReadTimeout = 100
        $motorPort.WriteTimeout = 1000
        $motorPort.Open()
        Start-Sleep -Milliseconds 300
        $motorPort.DiscardInBuffer()
        Send-Command 'STOP' '^OK STOP$'
        Send-Command 'TRACE OFF' '^OK TRACE OFF'
        Send-Command "DRIVER MDD10A $motorText" "^OK DRIVER MDD10A $motorText JOG_ONLY$"
        Send-Command 'STATUS' '^G431 MOTOR:'
        Send-Command 'SENSOR BOTH' '^SENSORS raw1='

        Write-Host "MDD10A 모터 $motorText / PWM $signedPwm% / $secondsValue 초"
        $confirm = Read-Host '처음에는 기어 부하를 빼고 시험하세요. 시작하려면 Y'
        if ($confirm -ne 'Y') { Write-Host '취소했습니다.'; return }
        Start-Sleep -Milliseconds 150
        Send-Command 'ARM' '^OK ARM'
        Send-Command "JOG $signedPwm $durationMs" '^OK JOG$'
        Write-Host '구동 중 S 키를 누르면 정지합니다. (Ctrl+C도 중단)'

        $timer = [Diagnostics.Stopwatch]::StartNew()
        $done = $false
        while ($timer.ElapsedMilliseconds -lt ($durationMs + 5000)) {
            if (![Console]::IsInputRedirected -and [Console]::KeyAvailable) {
                $key = [Console]::ReadKey($true)
                if ($key.Key -eq [ConsoleKey]::S) {
                    Send-Command 'STOP' '^OK STOP$'
                    $done = $true
                    break
                }
            }
            try { $line = $motorPort.ReadLine().Trim() }
            catch [System.TimeoutException] { continue }
            if (!$line) { continue }
            Write-Host "< $line"
            if ($line.StartsWith('ERR')) { throw $line }
            if ($line -eq 'DONE STOP') { $done = $true; break }
        }
        if (!$done) { throw '자동 정지 응답을 받지 못했습니다. STOP을 보냅니다.' }
        Send-Command 'SENSOR BOTH' '^SENSORS raw1='
        Send-Command 'STATUS' '^G431 MOTOR:'
    }
    catch { Write-Host ('확인 필요: ' + $_.Exception.Message) -ForegroundColor Yellow }
    finally {
        if ($null -ne $motorPort) {
            if ($motorPort.IsOpen) {
                try { $motorPort.WriteLine('STOP') } catch {}
                $motorPort.Close()
            }
            $motorPort.Dispose()
        }
    }
}
