Add-Type -AssemblyName System.Drawing

$dir = "C:\Users\aleks\Documents\Qoder\2026-11-13\8c076e37\_analysis\shots\cut"
$src = @("00-before.png", "02-flash.png", "06-flash.png")
$rect = New-Object System.Drawing.Rectangle 240, 430, 120, 120

foreach ($name in $src) {
    $bmp = [System.Drawing.Bitmap]::FromFile((Join-Path $dir $name))
    $crop = $bmp.Clone($rect, $bmp.PixelFormat)
    $big = New-Object System.Drawing.Bitmap ($rect.Width * 5), ($rect.Height * 5)
    $g = [System.Drawing.Graphics]::FromImage($big)
    $g.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::NearestNeighbor
    $g.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::Half
    $g.DrawImage($crop, 0, 0, $big.Width, $big.Height)
    $g.Dispose()
    $big.Save((Join-Path $dir ("zoom-" + $name)), [System.Drawing.Imaging.ImageFormat]::Png)
    $big.Dispose(); $crop.Dispose(); $bmp.Dispose()
    Write-Output "cropped $name"
}
