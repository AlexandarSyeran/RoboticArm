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

$root = [System.Windows.Automation.AutomationElement]::RootElement
$cond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::NameProperty, "Robotic Arm Simulator")
$win = $root.FindFirst([System.Windows.Automation.TreeScope]::Children, $cond)
if ($win -eq $null) { Write-Output "no window"; exit 1 }

function ClickButton([string]$name) {
    $c = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::NameProperty, $name)
    $b = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $c)
    if ($b -eq $null) { Write-Output "button '$name' not found"; return $false }
    $b.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
    return $true
}

ClickButton "Reset Auto Objects" | Out-Null
Start-Sleep -Milliseconds 900
Grab "c3-01-reset.png"

$textCond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ControlTypeProperty,
    [System.Windows.Automation.ControlType]::Text)
function FindToast() {
    $texts = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $textCond)
    foreach ($t in $texts) {
        if ($t.Current.Name -like "*Cutting operation finished*") { return $true }
    }
    return $false
}
Write-Output "toast after reset: $(FindToast)"

ClickButton "Activate Tool" | Out-Null
Start-Sleep -Milliseconds 3000
Grab "c3-02-cycle2.png"
$status = ""
$texts = $win.FindAll([System.Windows.Automation.TreeScope]::Descendants, $textCond)
foreach ($t in $texts) {
    if ($t.Current.Name -like "Cutting:*") { $status = $t.Current.Name }
}
Write-Output "cycle 2 status: $status"
Write-Output "toast during cycle 2: $(FindToast)"
Start-Sleep -Milliseconds 24000
Grab "c3-03-cycle2-end.png"
Write-Output "toast after cycle 2: $(FindToast)"
Write-Output "done"
