[CmdletBinding()]
param(
    [ValidateSet('cycle', 'ready', 'uphill', 'downhill', 'braking', 'hazard')]
    [string]$Scenario = 'cycle',
    [ValidatePattern('^COM\d+$')]
    [string]$Port = 'COM7',
    [ValidateRange(1, 60)]
    [int]$Seconds = 8
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Register order: version, heartbeat, valid, speed, hands, crosswalk,
# seconds, distance, pitch, tof, hazard, walker, braking, faults,
# host_link, flags.
$states = [ordered]@{
    ready = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 83)
    uphill = [uint16[]](2, 0, 63, 85, 3, 2, 65535, 35, 85, 0, 0, 2, 0, 0, 1, 3)
    downhill = [uint16[]](2, 0, 63, 70, 3, 4, 9, 21, 65466, 0, 0, 2, 0, 0, 1, 67)
    braking = [uint16[]](2, 0, 63, 20, 3, 2, 5, 15, 0, 0, 0, 2, 1, 0, 1, 67)
    hazard = [uint16[]](2, 0, 63, 0, 3, 2, 65535, 15, 0, 3, 1, 2, 0, 0, 1, 3)
}

function Get-ModbusCrc {
    param([byte[]]$Data)
    [uint16]$crc = 0xFFFF
    foreach ($value in $Data) {
        $crc = [uint16]($crc -bxor [uint16]$value)
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 1) -ne 0) {
                $crc = [uint16](($crc -shr 1) -bxor 0xA001)
            } else {
                $crc = [uint16]($crc -shr 1)
            }
        }
    }
    return $crc
}

function New-WriteFrame {
    param([uint16[]]$Words, [uint16]$Heartbeat)
    [uint16[]]$payloadWords = $Words.Clone()
    $payloadWords[1] = $Heartbeat

    $frame = [Collections.Generic.List[byte]]::new()
    $frame.AddRange([byte[]](1, 0x10, 0, 0, 0, 16, 32))
    foreach ($word in $payloadWords) {
        $frame.Add([byte](($word -shr 8) -band 255))
        $frame.Add([byte]($word -band 255))
    }
    [uint16]$crc = Get-ModbusCrc $frame.ToArray()
    $frame.Add([byte]($crc -band 255))
    $frame.Add([byte](($crc -shr 8) -band 255))
    return $frame.ToArray()
}

$sequence = if ($Scenario -eq 'cycle') {
    @('ready', 'uphill', 'downhill', 'braking', 'hazard', 'link_lost')
} else {
    @($Scenario)
}

$serial = [IO.Ports.SerialPort]::new($Port, 19200, 'None', 8, 'One')
$serial.Handshake = 'None'
$serial.ReadTimeout = 50
$serial.WriteTimeout = 1000
$heartbeat = 0

try {
    $serial.Open()
    foreach ($name in $sequence) {
        if ($name -eq 'link_lost') {
            Write-Host "[link_lost] no snapshots for $Seconds seconds"
            Start-Sleep -Seconds $Seconds
            continue
        }

        $ackBytes = 0
        $writes = $Seconds * 5
        Write-Host "[$name] sending for $Seconds seconds"
        for ($index = 0; $index -lt $writes; $index++) {
            $heartbeat = ($heartbeat % 250) + 1
            [byte[]]$packet = New-WriteFrame $states[$name] $heartbeat
            $serial.Write($packet, 0, $packet.Length)
            Start-Sleep -Milliseconds 200
            while ($serial.BytesToRead -gt 0) {
                [void]$serial.ReadByte()
                $ackBytes++
            }
        }
        Write-Host "[$name] Modbus response bytes: $ackBytes"
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
