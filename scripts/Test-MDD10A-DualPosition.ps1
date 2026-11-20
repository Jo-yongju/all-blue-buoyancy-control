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
        Send-Command 'DUAL ON' '^OK DUAL ON'
        Send-Command 'DUAL STATUS' '^DUAL '
        Send-Command 'SENSOR BOTH' '^SENSORS raw1='
        Write-Host '두 모터 동시 위치 시험: 모터1=센서1(PA0), 모터2=센서2(PA1).'
        Write-Host 'UP=둘 다 0, HOVER=센서1 522 / 센서2 529, DOWN=둘 다 4095.'
        Write-Host '감속 없이 PWM 50%로 이동합니다. 도착/목표 통과 시 정지합니다.'
        Write-Host '양끝은 ADC 포화가 처음 관측되는 위치입니다. 기계적 끝이나 정확한 물양을 뜻하지 않습니다.'
        Write-Host '두 모터 모두 -1=흡입=ADC 증가. 각각 도착하면 먼저 멈추며 속도 동기화 시험은 아닙니다.'
        Write-Host '센서 없는 피스톤이 먼저 벽에 닿거나 나사봉이 헛도는 상태라면 조립을 정리한 뒤 진행하세요.'
        Write-Host '먼저 HOVER로 확인하세요. 이동 중 S 키로 정지할 수 있습니다.'
        $confirm = Read-Host '위 구성을 확인했으면 Y 입력'
        if ($confirm -ne 'Y') { return }
        $firstMove = $true
        while ($true) {
            $choice = (Read-Host '명령: HOVER / UP / DOWN / Q(종료)').Trim().ToUpperInvariant()
            if ($choice -eq 'Q') { break }
            if ($choice -notin @('HOVER', 'UP', 'DOWN')) { continue }
            if ($firstMove -and $choice -ne 'HOVER') {
                Write-Host '첫 동시 시험은 HOVER를 선택하세요.'
                continue
            }
            Write-Host '--- 이동 전 ---'
            Send-Command 'SENSOR BOTH' '^SENSORS raw1='
            # SENSOR stops/disarms. Wait for restart gap, then ARM once per move.
            Start-Sleep -Milliseconds 300
            Send-Command 'ARM' '^OK ARM'
            Send-Command "DUAL $choice" '^OK DUAL MOVE$'
            $timer = [Diagnostics.Stopwatch]::StartNew()
            $complete = $false
            while ($timer.ElapsedMilliseconds -lt 65000) {
                if (![Console]::IsInputRedirected -and [Console]::KeyAvailable) {
                    $key = [Console]::ReadKey($true)
                    if ($key.Key -eq [ConsoleKey]::S) {
                        Send-Command 'STOP' '^OK STOP$'
                        Write-Host '사용자 중단: 목표 도착으로 판정하지 않습니다.'
                        $complete = $true
                        break
                    }
                }
                try { $line = $motorPort.ReadLine().Trim() }
                catch [System.TimeoutException] { continue }
                if (!$line) { continue }
                Write-Host "< $line"
                if ($line.StartsWith('ERR')) { throw $line }
                if ($line -match '^DONE DUAL DONE\b') {
                    $firstMove = $false
                    Write-Host '두 모터 모두 목표 도착 정지 확인.' -ForegroundColor Green
                    $complete = $true
                    break
                }
            }
            if (!$complete) { throw '이동 완료 응답 시간 초과. STOP을 보냅니다.' }
            Start-Sleep -Milliseconds 300
            Write-Host '--- 정지 0.3초 후 (관성 이동 포함) ---'
            Send-Command 'SENSOR BOTH' '^SENSORS raw1='
            Send-Command 'DUAL STATUS' '^DUAL '
            Send-Command 'STATS' '^STATS '
            Send-Command 'RTOS' '^RTOS '
        }
    }
    catch {
        Write-Host ('중단: ' + $_.Exception.Message) -ForegroundColor Yellow
        if ($null -ne $motorPort -and $motorPort.IsOpen) {
            try {
                Send-Command 'STOP' '^OK STOP$'
                Send-Command 'DUAL STATUS' '^DUAL '
                Send-Command 'STATS' '^STATS '
            } catch { Write-Host '진단 응답을 읽지 못했습니다. 연결 상태를 확인하세요.' }
        }
    }
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
