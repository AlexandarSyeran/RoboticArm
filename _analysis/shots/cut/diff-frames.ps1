Add-Type -AssemblyName System.Drawing

$dir = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
$base = [System.Drawing.Bitmap]::FromFile((Join-Path $dir "00-before.png"))

Get-ChildItem $dir -Filter "*flash.png" | Sort-Object Name | ForEach-Object {
    $bmp = [System.Drawing.Bitmap]::FromFile($_.FullName)
    $count = 0
    $minX = 1e9; $maxX = -1; $minY = 1e9; $maxY = -1
    $maxDiff = 0
    for ($y = 0; $y -lt $bmp.Height; $y += 2) {
        for ($x = 0; $x -lt $bmp.Width; $x += 2) {
            $a = $base.GetPixel($x, $y)
            $b = $bmp.GetPixel($x, $y)
            $d = [Math]::Abs($a.R - $b.R) + [Math]::Abs($a.G - $b.G) + [Math]::Abs($a.B - $b.B)
            if ($d -gt 60) {
                $count++
                if ($d -gt $maxDiff) { $maxDiff = $d }
                if ($x -lt $minX) { $minX = $x }
                if ($x -gt $maxX) { $maxX = $x }
                if ($y -lt $minY) { $minY = $y }
                if ($y -gt $maxY) { $maxY = $y }
            }
        }
    }
    $bmp.Dispose()
    if ($count -gt 0) {
        Write-Output ("{0}: changed={1} bbox=({2},{3})-({4},{5}) maxdiff={6}" -f $_.Name, $count, $minX, $minY, $maxX, $maxY, $maxDiff)
    } else {
        Write-Output ("{0}: no change" -f $_.Name)
    }
}
$base.Dispose()
