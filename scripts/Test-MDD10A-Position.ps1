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
        Send-Command 'DRIVER MDD10A 1' '^OK DRIVER MDD10A 1'
        Send-Command 'BENCH1 ON' '^OK BENCH1 MDD10A1 PA0'
        Send-Command 'BENCH1 STATUS' '^BENCH1 enabled=1'
        Send-Command 'STATUS' '^G431 MOTOR:'
        Send-Command 'SENSOR BOTH' '^SENSORS raw1='
        Write-Host '모터 1 + 센서 1(PA0). 모터 2는 구동하지 않습니다.'
        Write-Host 'UP=ADC 0(배출), HOVER=522(측정한 중앙), DOWN=4095(흡입).'
        Write-Host '감속 없이 PWM 50%로 이동합니다. 도착/목표 통과 시 정지합니다.'
        Write-Host '양끝은 ADC 포화가 처음 관측되는 위치입니다. 기계적 끝이나 정확한 물양을 뜻하지 않습니다.'
        Write-Host '먼저 HOVER로 확인하세요. 이동 중 S 키로 정지할 수 있습니다.'
        $confirm = Read-Host '위 구성을 확인했으면 Y 입력'
        if ($confirm -ne 'Y') { return }
        while ($true) {
            $choice = (Read-Host '명령: HOVER / UP / DOWN / Q(종료)').Trim().ToUpperInvariant()
            if ($choice -eq 'Q') { break }
            if ($choice -notin @('HOVER', 'UP', 'DOWN')) { continue }
            Write-Host '--- 이동 전 ---'
            Send-Command 'SENSOR BOTH' '^SENSORS raw1='
            # SENSOR stops/disarms. Wait for restart gap, then ARM once per move.
            Start-Sleep -Milliseconds 300
            Send-Command 'ARM' '^OK ARM'
            Send-Command "BUOY $choice" '^OK MOVE MDD10A1 SENSOR1$'
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
                if ($line -match '^DONE MOVE DONE\b') {
                    Write-Host '목표 도착 정지 확인.' -ForegroundColor Green
                    $complete = $true
                    break
                }
            }
            if (!$complete) { throw '이동 완료 응답 시간 초과. STOP을 보냅니다.' }
            Start-Sleep -Milliseconds 300
            Write-Host '--- 정지 0.3초 후 (관성 이동 포함) ---'
            Send-Command 'SENSOR BOTH' '^SENSORS raw1='
            Send-Command 'STATUS' '^G431 MOTOR:'
            Send-Command 'STATS' '^STATS '
            Send-Command 'RTOS' '^RTOS '
        }
    }
    catch {
        Write-Host ('중단: ' + $_.Exception.Message) -ForegroundColor Yellow
        if ($null -ne $motorPort -and $motorPort.IsOpen) {
            try {
                Send-Command 'STOP' '^OK STOP$'
                Send-Command 'STATUS' '^G431 MOTOR:'
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
