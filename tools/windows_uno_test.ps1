[CmdletBinding()]
param(
    [ValidateSet('status', 'encoder', 'motor')]
    [string]$Mode = 'status',
    [string]$DrivePort = 'COM4',
    [string]$TerrainPort = 'COM3',
    [switch]$EnableMotor,
    [ValidateRange(1, 30)]
    [int]$DurationSeconds = 10,
    [ValidateRange(-3000, 3000)]
    [int]$TargetMradS = 1000
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ProtocolVersion = 3
$TypeHello = 0x01
$TypeSessionStart = 0x02
$TypeCommand = 0x10
$TypeTelemetry = 0x20
$TypeTerrainTelemetry = 0x21
$CapWheelEncoder = 1
$CapPressureTelemetry = 0x40
$CapTof10120 = 0x100
$StatusEncoderSampleValid = 1 -shl 11

function Add-U16 {
    param([System.Collections.Generic.List[byte]]$List, [uint16]$Value)
    [void]$List.Add([byte]($Value -band 0xff))
    [void]$List.Add([byte](($Value -shr 8) -band 0xff))
}

function Add-U32 {
    param([System.Collections.Generic.List[byte]]$List, [uint32]$Value)
    foreach ($shift in 0, 8, 16, 24) {
        [void]$List.Add([byte](($Value -shr $shift) -band 0xff))
    }
}

function Add-Bytes {
    param(
        [System.Collections.Generic.List[byte]]$List,
        [byte[]]$Values
    )
    foreach ($value in $Values) {
        [void]$List.Add($value)
    }
}

function Get-UptimeMilliseconds {
    $bytes = [BitConverter]::GetBytes([Environment]::TickCount)
    return [BitConverter]::ToUInt32($bytes, 0)
}

function Get-Crc16 {
    param([byte[]]$Data)
    [int]$crc = 0xffff
    foreach ($value in $Data) {
        $crc = $crc -bxor ([int]$value -shl 8)
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = (($crc -shl 1) -bxor 0x1021) -band 0xffff
            } else {
                $crc = ($crc -shl 1) -band 0xffff
            }
        }
    }
    return [uint16]$crc
}

function ConvertTo-Cobs {
    param([byte[]]$Data)
    $output = [System.Collections.Generic.List[byte]]::new()
    [void]$output.Add(0)
    $codeIndex = 0
    [int]$code = 1
    foreach ($value in $Data) {
        if ($value -eq 0) {
            $output[$codeIndex] = [byte]$code
            $codeIndex = $output.Count
            [void]$output.Add(0)
            $code = 1
            continue
        }
        [void]$output.Add($value)
        $code++
        if ($code -eq 0xff) {
            $output[$codeIndex] = [byte]$code
            $codeIndex = $output.Count
            [void]$output.Add(0)
            $code = 1
        }
    }
    $output[$codeIndex] = [byte]$code
    return [byte[]]$output.ToArray()
}

function ConvertFrom-Cobs {
    param([byte[]]$Data)
    if ($Data.Count -eq 0) {
        throw 'empty COBS packet'
    }
    $output = [System.Collections.Generic.List[byte]]::new()
    $index = 0
    while ($index -lt $Data.Count) {
        [int]$code = $Data[$index]
        $index++
        if ($code -eq 0 -or $index + $code - 1 -gt $Data.Count) {
            throw 'invalid COBS packet'
        }
        for ($offset = 1; $offset -lt $code; $offset++) {
            [void]$output.Add($Data[$index])
            $index++
        }
        if ($code -ne 0xff -and $index -lt $Data.Count) {
            [void]$output.Add(0)
        }
    }
    return [byte[]]$output.ToArray()
}

function New-ProtocolFrame {
    param(
        [byte]$Type,
        [uint16]$Sequence,
        [uint32]$SessionId,
        [byte[]]$Payload
    )
    $raw = [System.Collections.Generic.List[byte]]::new()
    foreach ($value in [byte[]]@($ProtocolVersion, $Type, 0, 0)) {
        [void]$raw.Add($value)
    }
    Add-U16 $raw $Sequence
    Add-U16 $raw ([uint16]$Payload.Count)
    Add-U32 $raw $SessionId
    Add-U32 $raw (Get-UptimeMilliseconds)
    Add-Bytes $raw $Payload
    Add-U16 $raw (Get-Crc16 ([byte[]]$raw.ToArray()))
    $encoded = [System.Collections.Generic.List[byte]]::new()
    Add-Bytes $encoded (ConvertTo-Cobs ([byte[]]$raw.ToArray()))
    [void]$encoded.Add(0)
    return [byte[]]$encoded.ToArray()
}

function ConvertFrom-ProtocolFrame {
    param([byte[]]$Encoded)
    $raw = ConvertFrom-Cobs $Encoded
    if ($raw.Count -lt 18 -or $raw[0] -ne $ProtocolVersion) {
        throw 'invalid protocol frame'
    }
    $payloadLength = [BitConverter]::ToUInt16($raw, 6)
    if ($raw.Count -ne 18 + $payloadLength) {
        throw 'protocol payload length mismatch'
    }
    $receivedCrc = [BitConverter]::ToUInt16($raw, $raw.Count - 2)
    $checked = [byte[]]$raw[0..($raw.Count - 3)]
    if ((Get-Crc16 $checked) -ne $receivedCrc) {
        throw 'protocol CRC mismatch'
    }
    $payload = if ($payloadLength -eq 0) {
        [byte[]]@()
    } else {
        [byte[]]$raw[16..(15 + $payloadLength)]
    }
    return [pscustomobject]@{
        Type = [int]$raw[1]
        Sequence = [BitConverter]::ToUInt16($raw, 4)
        SessionId = [BitConverter]::ToUInt32($raw, 8)
        Payload = $payload
    }
}

function Open-UnoPort {
    param([string]$Name)
    $port = [System.IO.Ports.SerialPort]::new(
        $Name, 115200, 'None', 8, 'One'
    )
    $port.ReadTimeout = 50
    $port.WriteTimeout = 500
    $port.DtrEnable = $true
    $port.RtsEnable = $false
    $port.Open()
    Start-Sleep -Milliseconds 2000
    $port.DiscardInBuffer()
    $port.DiscardOutBuffer()
    return $port
}

function Read-ProtocolFrame {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [int]$TimeoutMs = 1000
    )
    $packet = [System.Collections.Generic.List[byte]]::new()
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $TimeoutMs) {
        if ($Port.BytesToRead -le 0) {
            Start-Sleep -Milliseconds 2
            continue
        }
        $value = $Port.ReadByte()
        if ($value -lt 0) {
            continue
        }
        if ($value -eq 0) {
            if ($packet.Count -eq 0) {
                continue
            }
            try {
                $frame = ConvertFrom-ProtocolFrame ([byte[]]$packet.ToArray())
                Write-Verbose (
                    'RX {0}: type=0x{1:x2} sequence={2} session=0x{3:x8}' -f
                    $Port.PortName, $frame.Type, $frame.Sequence,
                    $frame.SessionId
                )
                return $frame
            } catch {
                Write-Verbose (
                    "RX $($Port.PortName) dropped: $($_.Exception.Message)"
                )
                $packet.Clear()
                continue
            }
        }
        [void]$packet.Add([byte]$value)
        if ($packet.Count -gt 160) {
            $packet.Clear()
        }
    }
    return $null
}

function Wait-Hello {
    param([System.IO.Ports.SerialPort]$Port, [int]$TimeoutMs = 4000)
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $TimeoutMs) {
        $frame = Read-ProtocolFrame $Port 500
        if ($null -eq $frame -or $frame.Type -ne $TypeHello) {
            continue
        }
        if ($frame.SessionId -ne 0 -or $frame.Payload.Count -ne 8) {
            continue
        }
        $hello = [pscustomobject]@{
            BootId = [BitConverter]::ToUInt32($frame.Payload, 0)
            Capabilities = [BitConverter]::ToUInt32($frame.Payload, 4)
        }
        Write-Verbose (
            'HELLO {0}: boot=0x{1:x8} capabilities=0x{2:x8}' -f
            $Port.PortName, $hello.BootId, $hello.Capabilities
        )
        return $hello
    }
    throw "no valid HELLO received from $($Port.PortName)"
}

function New-SessionId {
    $bytes = [byte[]]::new(4)
    $generator = [Security.Cryptography.RandomNumberGenerator]::Create()
    try {
        $generator.GetBytes($bytes)
    } finally {
        $generator.Dispose()
    }
    $value = [BitConverter]::ToUInt32($bytes, 0)
    if ($value -eq 0) {
        return [uint32]1
    }
    return [uint32]$value
}

function Write-Frame {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [byte]$Type,
        [uint16]$Sequence,
        [uint32]$SessionId,
        [byte[]]$Payload
    )
    [byte[]]$frame = New-ProtocolFrame $Type $Sequence $SessionId $Payload
    Write-Verbose (
        'TX {0}: {1}' -f $Port.PortName,
        (($frame | ForEach-Object { '{0:x2}' -f $_ }) -join '')
    )
    $Port.Write($frame, 0, $frame.Count)
    $Port.BaseStream.Flush()
}

function Start-UnoSession {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [uint32]$BootId,
        [uint32]$SessionId,
        [uint16]$Sequence
    )
    Write-Frame $Port $TypeSessionStart $Sequence $SessionId (
        [BitConverter]::GetBytes($BootId)
    )
}

function New-CommandPayload {
    param([int]$Target, [bool]$Enabled)
    $payload = [System.Collections.Generic.List[byte]]::new()
    Add-Bytes $payload ([BitConverter]::GetBytes([int32]$Target))
    Add-U16 $payload 200
    [byte]$enabledValue = 0
    if ($Enabled) {
        $enabledValue = 1
    }
    [void]$payload.Add($enabledValue)
    [void]$payload.Add(0)
    return [byte[]]$payload.ToArray()
}

function Wait-Telemetry {
    param(
        [System.IO.Ports.SerialPort]$Port,
        [int]$ExpectedType,
        [uint32]$SessionId,
        [int]$TimeoutMs = 3000
    )
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $TimeoutMs) {
        $frame = Read-ProtocolFrame $Port 500
        if ($null -ne $frame -and
            $frame.Type -eq $ExpectedType -and
            $frame.SessionId -eq $SessionId) {
            return $frame
        }
    }
    throw "no telemetry received from $($Port.PortName)"
}

function Test-DriveStatus {
    param([string]$PortName)
    $port = Open-UnoPort $PortName
    try {
        $hello = Wait-Hello $port
        if (($hello.Capabilities -band $CapPressureTelemetry) -eq 0) {
            throw "$PortName is not Drive firmware"
        }
        $sessionId = New-SessionId
        Start-UnoSession $port $hello.BootId $sessionId 0
        Write-Frame $port $TypeCommand 1 $sessionId (
            New-CommandPayload 0 $false
        )
        $frame = Wait-Telemetry $port $TypeTelemetry $sessionId
        $status = [BitConverter]::ToUInt16($frame.Payload, 26)
        $fault = [BitConverter]::ToUInt16($frame.Payload, 28)
        [pscustomobject]@{
            Port = $PortName
            Role = 'Drive'
            Link = (($status -band 1) -ne 0)
            Armed = (($status -band 2) -ne 0)
            Deadman = (($status -band 4) -ne 0)
            EncoderAvailable = (($hello.Capabilities -band $CapWheelEncoder) -ne 0)
            EncoderSampleValid = (($status -band $StatusEncoderSampleValid) -ne 0)
            FaultBits = ('0x{0:x4}' -f $fault)
            PressureLeft = [BitConverter]::ToUInt16($frame.Payload, 32)
            PressureRight = [BitConverter]::ToUInt16($frame.Payload, 34)
        }
    } finally {
        if ($port.IsOpen) {
            $port.Close()
        }
    }
}

function Test-TerrainStatus {
    param([string]$PortName)
    $port = Open-UnoPort $PortName
    try {
        $hello = Wait-Hello $port
        if (($hello.Capabilities -band $CapTof10120) -eq 0) {
            throw "$PortName is not Terrain firmware"
        }
        $sessionId = New-SessionId
        Start-UnoSession $port $hello.BootId $sessionId 0
        $frame = Wait-Telemetry $port $TypeTerrainTelemetry $sessionId
        [pscustomobject]@{
            Port = $PortName
            Role = 'Terrain'
            TofValid = ($frame.Payload[2] -eq 1)
            DistanceMm = [BitConverter]::ToUInt16($frame.Payload, 0)
            Alert = [int]$frame.Payload[3]
            FaultBits = ('0x{0:x4}' -f (
                [BitConverter]::ToUInt16($frame.Payload, 12)
            ))
        }
    } finally {
        if ($port.IsOpen) {
            $port.Close()
        }
    }
}

function Watch-WheelEncoder {
    param([string]$PortName, [int]$Seconds)
    $port = Open-UnoPort $PortName
    $sessionId = [uint32]0
    [uint16]$sequence = 0
    try {
        $hello = Wait-Hello $port
        if (($hello.Capabilities -band $CapWheelEncoder) -eq 0) {
            throw 'wheel encoder adapter is not enabled in Drive firmware'
        }
        $sessionId = New-SessionId
        Start-UnoSession $port $hello.BootId $sessionId $sequence
        $sequence++
        Write-Frame $port $TypeCommand $sequence $sessionId (
            New-CommandPayload 0 $false
        )
        $sequence++
        [void](Wait-Telemetry $port $TypeTelemetry $sessionId)

        Write-Host 'Motor disabled. Rotate the encoder-equipped wheels by hand.'
        Write-Host (
            ' time sample_valid left_position right_position ' +
            'left_mrad_s right_mrad_s'
        )
        $timer = [Diagnostics.Stopwatch]::StartNew()
        [long]$nextCommandMs = 0
        [long]$nextPrintMs = 0
        while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
            if ($timer.ElapsedMilliseconds -ge $nextCommandMs) {
                Write-Frame $port $TypeCommand $sequence $sessionId (
                    New-CommandPayload 0 $false
                )
                $sequence++
                $nextCommandMs += 100
            }
            $frame = Read-ProtocolFrame $port 50
            if ($null -eq $frame -or
                $frame.Type -ne $TypeTelemetry -or
                $frame.SessionId -ne $sessionId -or
                $timer.ElapsedMilliseconds -lt $nextPrintMs) {
                continue
            }
            $status = [BitConverter]::ToUInt16($frame.Payload, 26)
            Write-Host ('{0,5:N1}s {1,12} {2,13} {3,14} {4,11} {5,12}' -f
                $timer.Elapsed.TotalSeconds,
                (($status -band $StatusEncoderSampleValid) -ne 0),
                [BitConverter]::ToInt32($frame.Payload, 0),
                [BitConverter]::ToInt32($frame.Payload, 4),
                [BitConverter]::ToInt32($frame.Payload, 8),
                [BitConverter]::ToInt32($frame.Payload, 12))
            $nextPrintMs += 200
        }
    } finally {
        if ($port.IsOpen -and $sessionId -ne 0) {
            try {
                Write-Frame $port $TypeCommand $sequence $sessionId (
                    New-CommandPayload 0 $false
                )
            } catch {
            }
        }
        if ($port.IsOpen) {
            $port.Close()
        }
    }
}

function Start-DriveMotorTest {
    param([string]$PortName, [int]$Target, [int]$Seconds)
    if (-not $EnableMotor) {
        throw 'motor mode requires -EnableMotor'
    }
    if ($Target -eq 0) {
        throw 'motor target must be nonzero'
    }
    $port = Open-UnoPort $PortName
    $sessionId = [uint32]0
    [uint16]$sequence = 0
    try {
        $hello = Wait-Hello $port
        $sessionId = New-SessionId
        Start-UnoSession $port $hello.BootId $sessionId $sequence
        $sequence++
        Write-Frame $port $TypeCommand $sequence $sessionId (
            New-CommandPayload 0 $false
        )
        $sequence++
        [void](Wait-Telemetry $port $TypeTelemetry $sessionId)
        Write-Host ((
                "Drive enabled for {0}s at target {1} mrad/s. " +
                'Keep both wheels lifted and the physical power cutoff ready.'
            ) -f $Seconds, $Target)
        $timer = [Diagnostics.Stopwatch]::StartNew()
        while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
            Write-Frame $port $TypeCommand $sequence $sessionId (
                New-CommandPayload $Target $true
            )
            $sequence++
            [void](Read-ProtocolFrame $port 20)
            Start-Sleep -Milliseconds 30
        }
    } finally {
        if ($port.IsOpen -and $sessionId -ne 0) {
            try {
                Write-Frame $port $TypeCommand $sequence $sessionId (
                    New-CommandPayload 0 $false
                )
                Start-Sleep -Milliseconds 100
            } catch {
            }
        }
        if ($port.IsOpen) {
            $port.Close()
        }
        Write-Host 'Drive stop command sent; serial port closed.'
    }
}

switch ($Mode) {
    'status' {
        Test-DriveStatus $DrivePort | Format-List
        Test-TerrainStatus $TerrainPort | Format-List
    }
    'encoder' {
        Watch-WheelEncoder $DrivePort $DurationSeconds
    }
    'motor' {
        Start-DriveMotorTest $DrivePort $TargetMradS $DurationSeconds
    }
}
