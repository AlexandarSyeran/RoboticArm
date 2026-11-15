param(
    [int]$Seconds = 180,
    [string]$ShotDir = "$env:TEMP\robot-shots",
    [string]$Button = "Test obstacle avoidance"
)

Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms

New-Item -ItemType Directory -Force -Path $ShotDir | Out-Null

$proc = Get-Process RoboticArm -ErrorAction SilentlyContinue
if (-not $proc) {
    $exe = "C:\Users\aleks\source\repos\RoboticArm\bin\Debug\net10.0-windows\RoboticArm.exe"
    Start-Process -FilePath $exe | Out-Null
    Start-Sleep -Seconds 4
    $proc = Get-Process RoboticArm -ErrorAction Stop
}

$root = [System.Windows.Automation.AutomationElement]::RootElement
$cond = New-Object System.Windows.Automation.PropertyCondition(
    [System.Windows.Automation.AutomationElement]::ProcessIdProperty, $proc.Id)
$window = $null
for ($i = 0; $i -lt 30 -and -not $window; $i++) {
    $window = $root.FindFirst([System.Windows.Automation.TreeScope]::Children, $cond)
    if (-not $window) { Start-Sleep -Milliseconds 500 }
}
if (-not $window) { Write-Output "WINDOW NOT FOUND"; exit 1 }
Write-Output ("window: " + $window.Current.Name)

$button = $null
for ($i = 0; $i -lt 30 -and -not $button; $i++) {
    $all = $window.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
    foreach ($el in $all) {
        if ($el.Current.Name -eq $Button -and $el.Current.ControlType.ProgrammaticName -like "*Button*") {
            $button = $el
            break
        }
    }
    if (-not $button) { Start-Sleep -Milliseconds 500 }
}
if (-not $button) { Write-Output "BUTTON NOT FOUND"; exit 1 }
Write-Output ("clicking " + $Button)
$button.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()

$statusNames = @()
$deadline = (Get-Date).AddSeconds($Seconds)
$shotIndex = 0
$lastStatus = ""

while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 2
    $all = $window.FindAll([System.Windows.Automation.TreeScope]::Descendants, [System.Windows.Automation.Condition]::TrueCondition)
    $status = $null
    foreach ($el in $all) {
        $n = $el.Current.Name
        if ($n -and ($n.StartsWith("Automation:") -or $n.StartsWith("Operation") -or $n.StartsWith("Manual"))) {
            $status = $n
            break
        }
    }

    if ($status -and $status -ne $lastStatus) {
        Write-Output ("[{0:HH:mm:ss}] {1}" -f (Get-Date), $status)
        $lastStatus = $status
    }

    if ($status -eq "Operation finished") { Write-Output "FINISHED"; break }
}

Write-Output ("last status: " + $lastStatus)
