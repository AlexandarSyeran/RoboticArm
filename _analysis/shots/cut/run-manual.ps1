Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
Get-ChildItem $out -Filter "m-*.png" | Remove-Item -Force

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

function ClickButton([string]$name) {
    $c = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::NameProperty, $name)
    $b = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $c)
    if ($b -eq $null) { Write-Output "button '$name' not found"; return }
    $b.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
}

function FindText([string]$name) {
    $c = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::NameProperty, $name)
    return $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $c)
}

$sliderCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::Slider)
$sliders = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $sliderCond)
$speed = $sliders[$sliders.Count - 1]
$speed.GetCurrentPattern([System.Windows.Automation.RangeValuePattern]::Pattern).SetValue(1.0)
Start-Sleep -Milliseconds 400

ClickButton "Pick Object"
$heldCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty,
    "Manual pick: object held - move the arm or press Place Object")
$deadline = (Get-Date).AddSeconds(60)
while ((Get-Date) -lt $deadline -and (FindText "Manual pick: object held - move the arm or press Place Object") -eq $null) {
    Start-Sleep -Milliseconds 500
}
Write-Output "pick1: $($null -ne (FindText 'Manual pick: object held - move the arm or press Place Object'))"
Grab "m-01-picked.png"

ClickButton "Place Object"
$placedName = "Manual place: object placed in drop zone"
$deadline = (Get-Date).AddSeconds(90)
while ((Get-Date) -lt $deadline -and (FindText $placedName) -eq $null) {
    Start-Sleep -Milliseconds 500
}
Write-Output "place: $($null -ne (FindText $placedName))"
Start-Sleep -Milliseconds 800
Grab "m-02-placed-retracted.png"

ClickButton "Pick Object"
Start-Sleep -Seconds 2
$err = FindText "No collision-free route to the pickup zone."
$moving = FindText "Manual pick: moving to pickup zone"
$above = FindText "Manual pick: moving above object"
Write-Output "pick2 error: $($null -ne $err) | moving: $($null -ne $moving) | approach: $($null -ne $above)"
Grab "m-03-pick2.png"

$deadline = (Get-Date).AddSeconds(60)
while ((Get-Date) -lt $deadline -and (FindText "Manual pick: object held - move the arm or press Place Object") -eq $null) {
    Start-Sleep -Milliseconds 500
}
Write-Output "pick2 done: $($null -ne (FindText 'Manual pick: object held - move the arm or press Place Object'))"
Grab "m-04-pick2-done.png"
Write-Output "done"
