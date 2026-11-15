Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
Get-ChildItem $out -Filter "f-*.png" | Remove-Item -Force

function Grab([string]$name) {
    $b = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
    $bmp = New-Object System.Drawing.Bitmap $b.Width, $b.Height
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.CopyFromScreen($b.Location, [System.Drawing.Point]::Empty, $b.Size)
    $g.Dispose()
    $bmp.Save((Join-Path $out $name), [System.Drawing.Imaging.ImageFormat]::Png)
    $bmp.Dispose()
}

$exe = "C:\Users\aleks\source\repos\RoboticArm\bin\Debug\net10.0-windows\RoboticArm.exe"
Get-Process RoboticArm -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Process $exe
Start-Sleep -Seconds 4

$root = [System.Windows.Automation.AutomationElement]::RootElement
$cond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Robotic Arm Simulator")
$win = $root.FindFirst([System.Windows.Automation.TreeScope]::Children, $cond)
if ($win -eq $null) { Write-Output "window not found"; exit 1 }

$c = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Start operation")
$b = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $c)
if ($b -eq $null) { Write-Output "start button not found"; exit 1 }
$b.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()

$t0 = Get-Date
$marks = 2, 4, 6, 8, 10, 12, 15, 18, 22, 26, 30
foreach ($s in $marks) {
    while (((Get-Date) - $t0).TotalSeconds -lt $s) { Start-Sleep -Milliseconds 200 }
    Grab ("f-{0:00}s.png" -f $s)
}
Get-Process RoboticArm -ErrorAction SilentlyContinue | Stop-Process -Force
Write-Output "done"
