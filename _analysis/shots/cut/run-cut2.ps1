Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
New-Item -ItemType Directory -Force -Path $out | Out-Null
Get-ChildItem $out -Filter "c2-*.png" | Remove-Item -Force

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
if ($wins.Count -eq 0) { exit 1 }
$win = $wins[0]

$comboCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::ComboBox)
$combo = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $comboCond)
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
if ($item -eq $null) { Write-Output "Cut item not found"; exit 1 }
$sel = $item.GetCurrentPattern([System.Windows.Automation.SelectionItemPattern]::Pattern)
$sel.Select()
$expand.Collapse()
Start-Sleep -Milliseconds 300
Write-Output "selected Cut"

Grab "c2-00-before.png"

$btnCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Activate Tool")
$button = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $btnCond)
$invoke = $button.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern)
$invoke.Invoke()
Write-Output "activated"

$marks = @(2.5, 4.5, 5.5, 7.0, 9.5, 12.5, 15.5, 18.5, 21.5, 23.0, 26.0)
$prev = 0.0
$index = 1
foreach ($mark in $marks) {
    Start-Sleep -Milliseconds ([int](($mark - $prev) * 1000))
    $prev = $mark
    Grab ("c2-{0:D2}-t{1}.png" -f $index, $mark)
    $index++
}

$textCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::Text)
$texts = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $textCond)
$found = $false
foreach ($t in $texts) {
    $name = $t.Current.Name
    if ($name -like "*Cutting operation finished*") { $found = $true; Write-Output "TOAST: $name" }
    if ($name -like "Cutting:*") { Write-Output "STATUS: $name" }
}
Write-Output "toast visible: $found"
Write-Output "done"
