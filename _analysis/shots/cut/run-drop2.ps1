Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"

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
Start-Sleep -Seconds 2
Start-Process $exe
Start-Sleep -Seconds 6

$root = [System.Windows.Automation.AutomationElement]::RootElement
$cond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Robotic Arm Simulator")
$win = $root.FindFirst([System.Windows.Automation.TreeScope]::Children, $cond)
if ($win -eq $null) { Write-Output "window not found"; exit }

$textCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::Text)
$btnCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Start operation")
$startBtn = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $btnCond)
Write-Output "start button found: $($startBtn -ne $null)"
$startBtn.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()

function DumpTexts([string]$tag) {
    $texts = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $textCond)
    $names = @()
    foreach ($t in $texts) {
        $n = $t.Current.Name
        if ($n -match "Automation|Operation|objects detected|no objects|Manual|Cutting") { $names += $n }
    }
    Write-Output "[$tag] $($names -join ' | ')"
}

Start-Sleep -Seconds 3
DumpTexts "t+3s"
Grab "d2-03s.png"
Start-Sleep -Seconds 20
DumpTexts "t+23s"
Grab "d2-23s.png"

$deadline = (Get-Date).AddSeconds(200)
$found = $false
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 3
    DumpTexts "poll"
    $c = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::NameProperty,
        "Transport operation finished. Press Reset Auto Objects to repeat.")
    $toast = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $c)
    if ($toast -ne $null) {
        $found = $true
        break
    }
}
Write-Output "toast element found: $found"
Start-Sleep -Milliseconds 600
DumpTexts "end"
Grab "d2-end.png"
