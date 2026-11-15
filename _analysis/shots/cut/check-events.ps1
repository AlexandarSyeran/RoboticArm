$events = Get-WinEvent -FilterHashtable @{LogName='Application'; StartTime=(Get-Date).AddMinutes(-25)} -MaxEvents 60
$events | Where-Object { $_.ProviderName -match 'NET Runtime|Application Error|Windows Error Reporting' } |
    Select-Object -First 8 | ForEach-Object {
        Write-Output "---- $($_.TimeCreated) [$($_.ProviderName)] ----"
        Write-Output $_.Message
    }
