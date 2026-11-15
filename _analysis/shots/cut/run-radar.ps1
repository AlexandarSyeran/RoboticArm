Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
Get-ChildItem $out -Filter "r-*.png" | Remove-Item -Force

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
$win = $wins[0]

function ClickButton([string]$name) {
    $c = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::NameProperty, $name)
    $b = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $c)
    if ($b -eq $null) { Write-Output "button '$name' not found"; return $false }
    $b.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
    return $true
}

function GrabTab($file) {
    Grab $file
}

$textCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::Text)
$allCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::Edit)

function DumpRadar([string]$tag) {
    $texts = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $textCond)
    foreach ($t in $texts) {
        $n = $t.Current.Name
        if ($n -match "objects? detected|^#1:|^-\s*$|^#\d+:") {
            Write-Output "[$tag] $n"
        }
    }
}

Grab "r-01-start.png"
DumpRadar "start"

# Switch to Cut and activate.
$comboCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::ComboBox)
$combo = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $comboCond)
$expand = $combo.GetCurrentPattern([System.Windows.Automation.ExpandCollapsePattern]::Pattern)
$expand.Expand()
Start-Sleep -Milliseconds 300
$itemCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Cut")
$item = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $itemCond)
$item.GetCurrentPattern([System.Windows.Automation.SelectionItemPattern]::Pattern).Select()
$expand.Collapse()
Start-Sleep -Milliseconds 300
ClickButton "Activate Tool" | Out-Null
Start-Sleep -Milliseconds 9000
Grab "r-02-cutting.png"
DumpRadar "mid-cut"
Start-Sleep -Milliseconds 14000
Grab "r-03-cut-done.png"
DumpRadar "cut done"

# Switch back to Grip and start the operation.
$expand.Expand()
Start-Sleep -Milliseconds 300
$gripCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Grip")
$grip = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $gripCond)
$grip.GetCurrentPattern([System.Windows.Automation.SelectionItemPattern]::Pattern).Select()
$expand.Collapse()
Start-Sleep -Milliseconds 300
ClickButton "Start operation" | Out-Null
Start-Sleep -Milliseconds 3000
Grab "r-04-transport.png"
DumpRadar "transport"
Start-Sleep -Milliseconds 9000
DumpRadar "transport-later"
Grab "r-05-transport-later.png"
Write-Output "done"
