Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

$out = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
Get-ChildItem $out -Filter "u5-*.png" | Remove-Item -Force

function GrabPanel([string]$name) {
    $b = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
    $full = New-Object System.Drawing.Bitmap $b.Width, $b.Height
    $g = [System.Drawing.Graphics]::FromImage($full)
    $g.CopyFromScreen($b.Location, [System.Drawing.Point]::Empty, $b.Size)
    $g.Dispose()
    $crop = New-Object System.Drawing.Bitmap 300, 702
    $cg = [System.Drawing.Graphics]::FromImage($crop)
    $cg.DrawImage($full, (New-Object System.Drawing.Rectangle 0, 0, 300, 702),
                  (New-Object System.Drawing.Rectangle 724, 0, 300, 702),
                  [System.Drawing.GraphicsUnit]::Pixel)
    $cg.Dispose()
    $scaled = New-Object System.Drawing.Bitmap 450, 1053
    $sg = [System.Drawing.Graphics]::FromImage($scaled)
    $sg.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
    $sg.DrawImage($crop, 0, 0, 450, 1053)
    $sg.Dispose()
    $scaled.Save((Join-Path $out $name), [System.Drawing.Imaging.ImageFormat]::Png)
    $full.Dispose(); $crop.Dispose(); $scaled.Dispose()
}

$exe = "C:\Users\aleks\source\repos\RoboticArm\bin\Debug\net10.0-windows\RoboticArm.exe"
Get-Process RoboticArm -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Process $exe
Start-Sleep -Seconds 4

$root = [System.Windows.Automation.AutomationElement]::RootElement
$cond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Robotic Arm Simulator")
$win = $root.FindFirst([System.Windows.Automation.TreeScope]::Children, $cond)

# Scroll the command panel back to the top.
$all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
    [System.Windows.Automation.Condition]::TrueCondition)
foreach ($e in $all) {
    $p = $null
    if ($e.TryGetCurrentPattern([System.Windows.Automation.ScrollPattern]::Pattern, [ref]$p)) {
        if ($p.VerticallyScrollable) {
            $p.SetScrollPercent([System.Windows.Automation.ScrollPattern]::NoScroll, 0)
            break
        }
    }
}
Start-Sleep -Milliseconds 500
GrabPanel "u5-01-panel-top.png"

# Expand the tool-mode combo to check the dark drop-down.
$comboCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::ComboBox)
$combo = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $comboCond)
$expand = $combo.GetCurrentPattern([System.Windows.Automation.ExpandCollapsePattern]::Pattern)
$expand.Expand()
Start-Sleep -Milliseconds 600
GrabPanel "u5-02-combo-open.png"
$expand.Collapse()
Start-Sleep -Milliseconds 300

# Scroll to the bottom to check the automation section.
$all = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants,
    [System.Windows.Automation.Condition]::TrueCondition)
foreach ($e in $all) {
    $p = $null
    if ($e.TryGetCurrentPattern([System.Windows.Automation.ScrollPattern]::Pattern, [ref]$p)) {
        if ($p.VerticallyScrollable) {
            $p.SetScrollPercent([System.Windows.Automation.ScrollPattern]::NoScroll, 100)
            break
        }
    }
}
Start-Sleep -Milliseconds 500
GrabPanel "u5-03-panel-bottom.png"
Write-Output "done"
