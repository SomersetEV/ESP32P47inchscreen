# capture_log.ps1 — read the board's USB-serial-JTAG output for a fixed time and exit.
#
# idf.py monitor never returns, which makes it useless from an automated shell.
# This opens the port, releases the board from reset, listens for -Seconds, then
# writes everything it saw to -Out and prints it.
#
#   idf.py -p COM5 flash
#   powershell -File tools/capture_log.ps1

param([string]$Port = "COM5", [int]$Seconds = 25, [string]$Out = "build/serial.log",
      [switch]$NoReset)

# On this board the port is the P4's native USB-serial-JTAG, where DTR/RTS map
# straight onto boot-mode and reset. Asserting them the way a CP210x adapter
# needs drops the chip into "waiting for download" instead of running the app,
# so both lines are held low and the board is left to run.
$p = New-Object System.IO.Ports.SerialPort $Port, 115200, None, 8, One
$p.ReadTimeout = 500
$p.Open()
$p.DtrEnable = $false
$p.RtsEnable = $false
Start-Sleep -Milliseconds 100

# Reset the board now that we are already listening, so the boot log is not
# missed. On USB-serial-JTAG, RTS alone pulses the reset line while DTR stays
# low, which keeps the chip out of download mode.
if (-not $NoReset) {
    $p.RtsEnable = $true
    Start-Sleep -Milliseconds 120
    $p.RtsEnable = $false
    Start-Sleep -Milliseconds 100
}

$end = (Get-Date).AddSeconds($Seconds)
$sb = New-Object Text.StringBuilder
while ((Get-Date) -lt $end) {
    try { [void]$sb.Append($p.ReadExisting()) } catch {}
    Start-Sleep -Milliseconds 50
}
$p.Close()

$dir = Split-Path -Parent $Out
if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Force $dir | Out-Null }
$sb.ToString() | Out-File -Encoding utf8 $Out
Get-Content $Out
