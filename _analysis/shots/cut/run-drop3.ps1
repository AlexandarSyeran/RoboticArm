Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
$exe = "C:\Users\aleks\source\repos\RoboticArm\bin\Debug\net10.0-windows\RoboticArm.exe"
$err = Join-Path $out "drop3-stderr.txt"
$log = Join-Path $out "drop3-log.txt"
Remove-Item $err, $log -ErrorAction SilentlyContinue

Get-Process RoboticArm -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Seconds 2
$p = Start-Process $exe -PassThru -RedirectStandardError $err -RedirectStandardOutput $log
Start-Sleep -Seconds 6

$root = [System.Windows.Automation.AutomationElement]::RootElement
$cond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Robotic Arm Simulator")
$win = $root.FindFirst([System.Windows.Automation.TreeScope]::Children, $cond)
if ($win -eq $null) { Write-Output "window not found"; exit }

$btnCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Start operation")
$startBtn = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $btnCond)
Write-Output "start button found: $($startBtn -ne $null)"
$startBtn.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()

for ($i = 0; $i -lt 60; $i++) {
    Start-Sleep -Seconds 5
    if ($p.HasExited) {
        Write-Output "PROCESS EXITED at t+$((($i + 1) * 5) + 6)s exitCode=$($p.ExitCode)"
        break
    }
    if ((($i + 1) * 5) % 30 -eq 0) {
        Write-Output "alive t+$((($i + 1) * 5) + 6)s"
    }
}
if (-not $p.HasExited) { Write-Output "still running after poll window" }
if (Test-Path $err) {
    $e = Get-Content $err -Raw
    if ($e) { Write-Output "---- stderr ----"; Write-Output $e }
}
if (Test-Path $log) {
    $l = Get-Content $log -Raw
    if ($l) { Write-Output "---- stdout ----"; Write-Output $l }
}
