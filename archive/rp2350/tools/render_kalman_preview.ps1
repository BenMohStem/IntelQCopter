param(
    [Parameter(Mandatory=$true)][string]$CsvPath,
    [Parameter(Mandatory=$true)][string]$OutputPath
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$rows = @(Import-Csv -LiteralPath $CsvPath)
if ($rows.Count -lt 2) { throw 'Need at least two Kalman Monte Carlo rows.' }

$width=1200; $height=680; $left=145; $right=1140; $top=135; $bottom=525
$bitmap=[System.Drawing.Bitmap]::new($width,$height)
$graphics=[System.Drawing.Graphics]::FromImage($bitmap)
$graphics.SmoothingMode=[System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$graphics.TextRenderingHint=[System.Drawing.Text.TextRenderingHint]::AntiAliasGridFit
$white=[System.Drawing.Brushes]::White; $ink=[System.Drawing.Color]::FromArgb(23,32,51)
$muted=[System.Drawing.Color]::FromArgb(82,99,122); $grid=[System.Drawing.Color]::FromArgb(215,224,234)
$blue=[System.Drawing.Color]::FromArgb(37,99,235); $orange=[System.Drawing.Color]::FromArgb(234,88,12)
$titleFont=[System.Drawing.Font]::new('Segoe UI',22,[System.Drawing.FontStyle]::Bold)
$font=[System.Drawing.Font]::new('Segoe UI',11)
$smallFont=[System.Drawing.Font]::new('Segoe UI',10)
$inkBrush=[System.Drawing.SolidBrush]::new($ink); $mutedBrush=[System.Drawing.SolidBrush]::new($muted)
$bluePen=[System.Drawing.Pen]::new($blue,3); $orangePen=[System.Drawing.Pen]::new($orange,3)
$gridPen=[System.Drawing.Pen]::new($grid,1); $outlinePen=[System.Drawing.Pen]::new($ink,1.2)
try {
    $graphics.Clear([System.Drawing.Color]::White)
    $graphics.DrawString('Kalman uncertainty: measured error vs filter prediction',$titleFont,$inkBrush,48,28)
    $graphics.DrawString('Production C roll filter · 500 seeded synthetic trials · 2 ms steps · software-only result',$font,$mutedBrush,50,72)
    $maxY=0.0
    foreach($row in $rows){$maxY=[math]::Max($maxY,[double]$row.empirical_error_std_deg); $maxY=[math]::Max($maxY,[double]$row.kalman_std_deg)}
    $maxY=[math]::Ceiling(($maxY*1.12)/0.05)*0.05
    for($tick=0;$tick -le 5;$tick++){
        $yVal=$maxY*$tick/5; $y=$bottom-($bottom-$top)*$tick/5
        $graphics.DrawLine($gridPen,$left,[int]$y,$right,[int]$y)
        $graphics.DrawString(('{0:F2}' -f $yVal),$smallFont,$mutedBrush,88,[single]($y-10))
        $x=$left+($right-$left)*$tick/5; $graphics.DrawLine($gridPen,[int]$x,$top,[int]$x,$bottom)
        $graphics.DrawString(('{0:F1}' -f ([double]$rows[-1].time_s*$tick/5)),$smallFont,$mutedBrush,[single]($x-12),$bottom+8)
    }
    $graphics.DrawRectangle($outlinePen,$left,$top,$right-$left,$bottom-$top)
    $pEmp=[System.Drawing.PointF[]]::new($rows.Count); $pKal=[System.Drawing.PointF[]]::new($rows.Count)
    $lastTime=[double]$rows[-1].time_s
    for($i=0;$i -lt $rows.Count;$i++){
        $x=$left+($right-$left)*([double]$rows[$i].time_s/$lastTime)
        $pEmp[$i]=[System.Drawing.PointF]::new([single]$x,[single]($bottom-($bottom-$top)*([double]$rows[$i].empirical_error_std_deg/$maxY)))
        $pKal[$i]=[System.Drawing.PointF]::new([single]$x,[single]($bottom-($bottom-$top)*([double]$rows[$i].kalman_std_deg/$maxY)))
    }
    $graphics.DrawLines($bluePen,$pEmp); $graphics.DrawLines($orangePen,$pKal)
    $state=$graphics.Save()
    $graphics.TranslateTransform(34,[single](($top+$bottom)/2)); $graphics.RotateTransform(-90)
    $graphics.DrawString('Roll standard deviation (degrees)',$font,$inkBrush,-112,-8)
    $graphics.Restore($state)
    $graphics.DrawString('Time (seconds)',$font,$inkBrush,([single](($left+$right)/2-48)),$bottom+32)
    $legendY=590
    $graphics.DrawLine($bluePen,300,$legendY+8,345,$legendY+8)
    $graphics.DrawString('Empirical estimation error σ',$font,$inkBrush,354,$legendY)
    $graphics.DrawLine($orangePen,650,$legendY+8,695,$legendY+8)
    $graphics.DrawString('Filter predicted √P00',$font,$inkBrush,704,$legendY)
    $end=$rows[-1]
    $summaryValues=@([double]$end.empirical_error_std_deg,[double]$end.kalman_std_deg,[double]$end.rmse_deg,(100*[double]$end.coverage_1sigma),(100*[double]$end.coverage_2sigma))
    $summary='At 5 s: empirical σ {0:F4}° · predicted σ {1:F4}° · RMSE {2:F4}° · ±1σ coverage {3:F1}% · ±2σ {4:F1}%' -f $summaryValues
    $graphics.DrawString($summary,$font,$mutedBrush,75,635)
    $directory=Split-Path -Parent $OutputPath
    if($directory){New-Item -ItemType Directory -Force $directory | Out-Null}
    $bitmap.Save($OutputPath,[System.Drawing.Imaging.ImageFormat]::Png)
    Write-Output "Wrote $OutputPath from $($rows.Count) saved time bins."
}
finally {
    $graphics.Dispose(); $bitmap.Dispose(); $titleFont.Dispose(); $font.Dispose(); $smallFont.Dispose()
    $inkBrush.Dispose(); $mutedBrush.Dispose(); $bluePen.Dispose(); $orangePen.Dispose(); $gridPen.Dispose(); $outlinePen.Dispose()
}
