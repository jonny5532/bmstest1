param(
    [string]$Port = "COM13"
)

# Opening the CDC port at 1200 baud asks Pico SDK USB stdio firmware to
# reboot into BOOTSEL mode
$sp = New-Object System.IO.Ports.SerialPort($Port, 1200)
try {
    $sp.Open()
    Start-Sleep -Milliseconds 300
    $sp.Close()
    Write-Host "1200 baud touch sent to $Port"
} catch {
    Write-Host ("open failed: " + $_.Exception.Message)
}
