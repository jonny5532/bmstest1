param(
    [string]$Port = "COM13",
    [string]$Command = "adc",
    [int]$Seconds = 5,
    [string]$OutFile = "serial_capture.txt"
)

$sp = New-Object System.IO.Ports.SerialPort($Port, 115200)
$sp.ReadTimeout = 500
$sp.DtrEnable = $true
$sp.NewLine = "`n"
$sp.Open()
Start-Sleep -Milliseconds 500
$sp.ReadExisting() | Out-Null
$sp.WriteLine($Command)
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$buf = ""
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try { $buf += $sp.ReadExisting() } catch {}
    Start-Sleep -Milliseconds 100
}
$sp.Close()
Set-Content -Path $OutFile -Value $buf
Write-Host "Captured $($buf.Length) chars to $OutFile"
