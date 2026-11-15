Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
New-Item -ItemType Directory -Force -Path $out | Out-Null

function Grab([string]$name) {
    $b = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
    $bmp = New-Object System.Drawing.Bitmap $b.Width, $b.Height
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.CopyFromScreen($b.Location, [System.Drawing.Point]::Empty, $b.Size)
    $g.Dispose()
    $bmp.Save((Join-Path $out $name), [System.Drawing.Imaging.ImageFormat]::Png)
    $bmp.Dispose()
}

$root = [System.Windows.Automation.AutomationElement]::RootElement
$cond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Robotic Arm Simulator")
$wins = $root.FindAll([System.Windows.Automation.TreeScope]::Children, $cond)
Write-Output "windows: $($wins.Count)"

$win = $null
foreach ($w in $wins) {
    $btnCond = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::NameProperty, "Activate Tool")
    $btn = $w.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $btnCond)
    if ($btn -ne $null) { $win = $w; break }
}
if ($win -eq $null) { Write-Output "no usable window"; exit 1 }

$comboCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::ComboBox)
$combo = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $comboCond)
Write-Output "combo found: $($combo -ne $null)"

$expand = $combo.GetCurrentPattern([System.Windows.Automation.ExpandCollapsePattern]::Pattern)
$expand.Expand()
Start-Sleep -Milliseconds 300

$itemCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Cut")
$item = $null
foreach ($scope in @([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.TreeScope]::Children)) {
    $item = $combo.FindFirst($scope, $itemCond)
    if ($item -ne $null) { break }
    $item = $win.FindFirst($scope, $itemCond)
    if ($item -ne $null) { break }
}
if ($item -ne $null) {
    $sel = $item.GetCurrentPattern([System.Windows.Automation.SelectionItemPattern]::Pattern)
    $sel.Select()
    Write-Output "selected Cut"
} else {
    Write-Output "Cut item not found"
}
$expand.Collapse()
Start-Sleep -Milliseconds 300

Grab "00-before.png"

$btnCond2 = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Activate Tool")
$button = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $btnCond2)
$invoke = $button.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern)
$invoke.Invoke()

for ($i = 1; $i -le 8; $i++) {
    Start-Sleep -Milliseconds 110
    Grab ("{0:D2}-flash.png" -f $i)
}
Grab "09-after.png"
Write-Output "done"
