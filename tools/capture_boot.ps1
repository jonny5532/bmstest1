param(
    [string]$Port = "COM13",
    [int]$Seconds = 8,
    [string]$OutFile = "boot_capture.txt"
)

# Send restart command
$sp = New-Object System.IO.Ports.SerialPort($Port, 115200)
$sp.ReadTimeout = 500
$sp.DtrEnable = $true
$sp.NewLine = "`n"
$sp.Open()
Start-Sleep -Milliseconds 300
$sp.ReadExisting() | Out-Null
$sp.WriteLine("restart")
Start-Sleep -Milliseconds 200
$sp.Close()
Write-Host "Restart sent, waiting for port to come back..."

# Wait for the port to re-enumerate and reconnect
$buf = ""
$deadline = (Get-Date).AddSeconds(20)
$sp2 = $null
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Milliseconds 250
    try {
        $sp2 = New-Object System.IO.Ports.SerialPort($Port, 115200)
        $sp2.ReadTimeout = 500
        $sp2.DtrEnable = $true
        $sp2.Open()
        break
    } catch {
        $sp2 = $null
    }
}
if ($null -eq $sp2) {
    Write-Host "Failed to reopen $Port"
    exit 1
}
Write-Host "Reconnected to $Port, capturing..."
$sw = [System.Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try { $buf += $sp2.ReadExisting() } catch {}
    Start-Sleep -Milliseconds 100
}
$sp2.Close()
Set-Content -Path $OutFile -Value $buf
Write-Host "Captured $($buf.Length) chars to $OutFile"
