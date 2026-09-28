[CmdletBinding()]
param(
    [ValidateSet(
        'cycle', 'ready', 'uphill', 'downhill', 'braking', 'hazard',
        'surface', 'surface_cycle', 'smooth', 'rough', 'wet', 'gravel',
        'step', 'hole', 'layout', 'location_cycle', 'suseo', 'seonsa',
        'nangok', 'unnamed', 'konkuk'
    )]
    [string]$Scenario = 'cycle',
    [ValidatePattern('^COM\d+$')]
    [string]$Port = 'COM7',
    [ValidateRange(1, 60)]
    [int]$Seconds = 8
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Register order: 16 status words followed by 10 packed ASCII location words.
$states = [ordered]@{
    ready = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 83)
    uphill = [uint16[]](2, 0, 63, 85, 3, 2, 65535, 35, 85, 0, 0, 2, 0, 0, 1, 3)
    downhill = [uint16[]](2, 0, 63, 70, 3, 4, 9, 21, 65466, 0, 0, 2, 0, 0, 1, 67)
    braking = [uint16[]](2, 0, 63, 20, 3, 2, 5, 15, 0, 0, 0, 2, 1, 0, 1, 67)
    hazard = [uint16[]](2, 0, 63, 0, 3, 2, 65535, 15, 0, 3, 1, 2, 0, 0, 1, 3)
    # Surface confidence is quantized to five bits for the LCD snapshot.
    smooth = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 59859)
    rough = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 51923)
    wet = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 50131)
    gravel = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 48339)
    step = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 56787)
    hole = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 61139)
    # Long strings for checking font size and clipping on every card.
    layout = [uint16[]](2, 0, 61, 65534, 0, 4, 65534, 65534, 63736, 4, 1, 5, 0, 1, 1, 64736)
    # Backward-compatible alias: WET at 24/31 confidence (LCD displays 77%).
    surface = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 50131)
    suseo = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 83)
    seonsa = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 83)
    nangok = [uint16[]](2, 0, 63, 125, 3, 3, 15, 42, 0, 0, 0, 2, 0, 0, 1, 83)
    unnamed = [uint16[]](2, 0, 63, 125, 3, 2, 65535, 42, 0, 0, 0, 2, 0, 0, 1, 3)
    konkuk = [uint16[]](2, 0, 63, 125, 3, 3, 18, 36, 0, 0, 0, 2, 0, 0, 1, 50131)
}

$locations = @{
    ready = 'SUSEO STN'
    uphill = 'SEONSA JCT'
    downhill = 'NANGOK POST OFFICE'
    braking = 'SCHOOL MAIN GATE'
    hazard = 'HOSPITAL FRONT'
    smooth = 'SUSEO STN'
    rough = 'SUSEO STN'
    wet = 'SUSEO STN'
    gravel = 'SUSEO STN'
    step = 'SUSEO STN'
    hole = 'SUSEO STN'
    layout = 'NANGOK POST OFFICE'
    surface = 'SUSEO STN'
    suseo = 'SUSEO STN'
    seonsa = 'SEONSA JCT'
    nangok = 'NANGOK POST OFFICE'
    unnamed = ''
    konkuk = 'KONKUK UNIV STN'
}

foreach ($name in @($states.Keys)) {
    [uint16[]]$expanded = New-Object 'UInt16[]' 26
    [Array]::Copy($states[$name], $expanded, 16)
    $expanded[0] = 3
    [byte[]]$label = [Text.Encoding]::ASCII.GetBytes($locations[$name])
    for ($index = 0; $index -lt [Math]::Min($label.Length, 20); $index += 2) {
        [uint16]$word = [uint16]$label[$index] -shl 8
        if ($index + 1 -lt $label.Length) {
            $word = $word -bor [uint16]$label[$index + 1]
        }
        $expanded[16 + [int]($index / 2)] = $word
    }
    $states[$name] = $expanded
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

    if ($payloadWords.Length -ne 26) {
        throw "Expected 26 HMI registers, got $($payloadWords.Length)"
    }
    $frame = [Collections.Generic.List[byte]]::new()
    $frame.AddRange([byte[]](1, 0x10, 0, 0, 0, 26, 52))
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
} elseif ($Scenario -eq 'surface_cycle') {
    @('smooth', 'rough', 'wet', 'gravel', 'step', 'hole')
} elseif ($Scenario -eq 'location_cycle') {
    @('suseo', 'seonsa', 'nangok', 'unnamed')
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
