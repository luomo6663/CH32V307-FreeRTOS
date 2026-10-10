param(
    [string]$Port = "COM8",
    [int]$Baud = 115200,
    [double]$Degrees = 360,
    [double]$Rpm = 5.0,
    [int]$SettleMs = 1000
)

$ErrorActionPreference = "Stop"

if($Rpm -le 0) {
    throw "Rpm must be greater than zero"
}

$serial = New-Object System.IO.Ports.SerialPort(
    $Port,
    $Baud,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.NewLine = "`n"
$serial.ReadTimeout = 300
$serial.WriteTimeout = 500

function Send-Command {
    param([string]$Command)

    $serial.WriteLine($Command)
    Start-Sleep -Milliseconds 120
    return $serial.ReadExisting()
}

try {
    $serial.Open()
    Start-Sleep -Milliseconds 150
    [void]$serial.ReadExisting()

    Write-Output (Send-Command "TURN ZERO")
    Write-Output ("turn {0} deg at {1} rpm" -f $Degrees, $Rpm)
    Write-Output (Send-Command ("TURN {0} {1}" -f $Degrees, $Rpm))

    $moveMs = [int]([math]::Abs($Degrees) * 60000.0 / (360.0 * $Rpm))
    Start-Sleep -Milliseconds ($moveMs + $SettleMs)

    Write-Output (Send-Command "TURN")
}
finally {
    if($serial.IsOpen) {
        try {
            $serial.WriteLine("TURN STOP")
        }
        catch {
        }
        $serial.Close()
    }
}
