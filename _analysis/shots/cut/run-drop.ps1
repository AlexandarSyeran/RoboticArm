Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
Get-ChildItem $out -Filter "d-*.png" | Remove-Item -Force

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
$sliders[$sliders.Count - 1].GetCurrentPattern([System.Windows.Automation.RangeValuePattern]::Pattern).SetValue(1.0)
Start-Sleep -Milliseconds 400

ClickButton "Start operation"
$toastCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty,
    "Transport operation finished. Press Reset Auto Objects to repeat.")
$deadline = (Get-Date).AddSeconds(220)
$found = $false
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 3
    if (FindText "Transport operation finished. Press Reset Auto Objects to repeat.") {
        $found = $true
        break
    }
}
Write-Output "finished toast: $found"
Start-Sleep -Milliseconds 600
Grab "d-01-auto-drop-layout.png"
Write-Output "done"
