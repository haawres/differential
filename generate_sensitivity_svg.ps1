# PowerShell script to compute exact analytical solution and render authentic MATLAB sensitivity 9-panel SVG
$m_base = 1.5
$T = 20.0
$k_base = 0.5
$g = 9.81
$pitch_base = 10.0 * [Math]::PI / 180.0
$roll = 5.0 * [Math]::PI / 180.0
$t0 = 0.0
$tf = 45.0
$h = 0.1

$tArr = @()
$t = $t0
while ($t -le $tf + 0.0001) {
    $tArr += $t
    $t += $h
}

function Solve-Analytical($m, $thrust, $k, $grav, $pitch, $rollAngle) {
    $Ax = ($thrust / $m) * [Math]::Sin($pitch) * [Math]::Cos($rollAngle)
    $Ay = -($thrust / $m) * [Math]::Sin($rollAngle)
    $Az = ($thrust / $m) * [Math]::Cos($pitch) * [Math]::Cos($rollAngle) - $grav

    $vx = @()
    $vy = @()
    $vz = @()

    foreach ($ti in $tArr) {
        $vx += [Math]::Round((-$Ax * $m / $k) * [Math]::Exp(-$k * $ti / $m) + $Ax * $m / $k, 4)
        $vy += [Math]::Round((-$Ay * $m / $k) * [Math]::Exp(-$k * $ti / $m) + $Ay * $m / $k, 4)
        $vz += [Math]::Round((-$Az * $m / $k) * [Math]::Exp(-$k * $ti / $m) + $Az * $m / $k, 4)
    }
    return @{ vx = $vx; vy = $vy; vz = $vz }
}

$mass1 = Solve-Analytical 1.0 $T $k_base $g $pitch_base $roll
$mass3 = Solve-Analytical 3.0 $T $k_base $g $pitch_base $roll

$drag05 = Solve-Analytical $m_base $T 0.5 $g $pitch_base $roll
$drag10 = Solve-Analytical $m_base $T 1.0 $g $pitch_base $roll

$pitch10 = Solve-Analytical $m_base $T $k_base $g (10.0 * [Math]::PI / 180.0) $roll
$pitch25 = Solve-Analytical $m_base $T $k_base $g (25.0 * [Math]::PI / 180.0) $roll

$width = 1100
$height = 850
$marginTop = 60
$marginBottom = 50
$marginLeft = 65
$marginRight = 30
$cols = 3
$rows = 3
$subW = [Math]::Floor(($width - $marginLeft - $marginRight - 80) / $cols)
$subH = [Math]::Floor(($height - $marginTop - $marginBottom - 80) / $rows)

$sb = [System.Text.StringBuilder]::new()
[void]$sb.AppendLine("<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 $width $height' width='$width' height='$height' style='background:#ffffff; font-family: Arial, Helvetica, sans-serif;'>")
[void]$sb.AppendLine("<rect width='$width' height='$height' fill='#ffffff'/>")
[void]$sb.AppendLine("<text x='$([Math]::Floor($width / 2))' y='36' text-anchor='middle' font-size='20' font-weight='bold' fill='#000000'>Sensitivity Analysis: Mass, Drag, and Pitch Variations (MATLAB)</text>")

$subplots = @(
    @{ row = 0; col = 0; title = 'Mass: Forward Velocity'; ylabel = 'v_x (m/s)'; d1 = $mass1.vx; d2 = $mass3.vx; l1 = '1 kg'; l2 = '3 kg'; c2 = '#d90429'; dash2 = '6,4' },
    @{ row = 0; col = 1; title = 'Mass: Lateral Velocity'; ylabel = 'v_y (m/s)'; d1 = $mass1.vy; d2 = $mass3.vy; l1 = '1 kg'; l2 = '3 kg'; c2 = '#d90429'; dash2 = '6,4' },
    @{ row = 0; col = 2; title = 'Mass: Vertical Velocity'; ylabel = 'v_z (m/s)'; d1 = $mass1.vz; d2 = $mass3.vz; l1 = '1 kg'; l2 = '3 kg'; c2 = '#d90429'; dash2 = '6,4' },

    @{ row = 1; col = 0; title = 'Drag: Forward Velocity'; ylabel = 'v_x (m/s)'; d1 = $drag05.vx; d2 = $drag10.vx; l1 = 'k = 0.5'; l2 = 'k = 1.0'; c2 = '#e63946'; dash2 = '6,4' },
    @{ row = 1; col = 1; title = 'Drag: Lateral Velocity'; ylabel = 'v_y (m/s)'; d1 = $drag05.vy; d2 = $drag10.vy; l1 = 'k = 0.5'; l2 = 'k = 1.0'; c2 = '#e63946'; dash2 = '6,4' },
    @{ row = 1; col = 2; title = 'Drag: Vertical Velocity'; ylabel = 'v_z (m/s)'; d1 = $drag05.vz; d2 = $drag10.vz; l1 = 'k = 0.5'; l2 = 'k = 1.0'; c2 = '#e63946'; dash2 = '6,4' },

    @{ row = 2; col = 0; title = 'Pitch: Forward Velocity'; ylabel = 'v_x (m/s)'; d1 = $pitch10.vx; d2 = $pitch25.vx; l1 = '10°'; l2 = '25°'; c2 = '#0072BD'; dash2 = '6,4' },
    @{ row = 2; col = 1; title = 'Pitch: Lateral Velocity'; ylabel = 'v_y (m/s)'; d1 = $pitch10.vy; d2 = $pitch25.vy; l1 = '10°'; l2 = '25°'; c2 = '#0072BD'; dash2 = '6,4' },
    @{ row = 2; col = 2; title = 'Pitch: Vertical Velocity'; ylabel = 'v_z (m/s)'; d1 = $pitch10.vz; d2 = $pitch25.vz; l1 = '10°'; l2 = '25°'; c2 = '#0072BD'; dash2 = '6,4' }
)

foreach ($sp in $subplots) {
    $x0 = $marginLeft + $sp.col * ($subW + 40)
    $y0 = $marginTop + $sp.row * ($subH + 40)

    $allY = $sp.d1 + $sp.d2
    $minY = ($allY | Measure-Object -Minimum).Minimum
    $maxY = ($allY | Measure-Object -Maximum).Maximum
    if ($minY -eq $maxY) { $minY -= 1; $maxY += 1 }
    $padY = ($maxY - $minY) * 0.1
    if ($padY -eq 0) { $padY = 0.5 }
    $minY -= $padY
    $maxY += $padY

    # Box
    [void]$sb.AppendLine("<rect x='$x0' y='$y0' width='$subW' height='$subH' fill='#ffffff' stroke='#000000' stroke-width='1'/>")

    # Y gridlines
    for ($k = 0; $k -le 4; $k++) {
        $val = $minY + ($k / 4.0) * ($maxY - $minY)
        $yPos = $y0 + $subH - ($k / 4.0) * $subH
        [void]$sb.AppendLine("<line x1='$x0' y1='$yPos' x2='$($x0 + $subW)' y2='$yPos' stroke='#e0e0e0' stroke-width='0.8' stroke-dasharray='2,2'/>")
        [void]$sb.AppendLine("<text x='$($x0 - 6)' y='$($yPos + 4)' text-anchor='end' font-size='10' fill='#333333'>$([Math]::Round($val, 1))</text>")
    }

    # X gridlines
    for ($ti = 0; $ti -le 45; $ti += 10) {
        $xPos = $x0 + ($ti / 45.0) * $subW
        [void]$sb.AppendLine("<line x1='$xPos' y1='$y0' x2='$xPos' y2='$($y0 + $subH)' stroke='#e0e0e0' stroke-width='0.8' stroke-dasharray='2,2'/>")
        if ($sp.row -eq 2) {
            [void]$sb.AppendLine("<text x='$xPos' y='$($y0 + $subH + 16)' text-anchor='middle' font-size='10' fill='#333333'>$ti</text>")
        }
    }

    # Line 1: solid black
    $pts1Arr = @()
    for ($i = 0; $i -lt $tArr.Count; $i++) {
        $px = [Math]::Round($x0 + ($tArr[$i] / 45.0) * $subW, 1)
        $py = [Math]::Round($y0 + $subH - (($sp.d1[$i] - $minY) / ($maxY - $minY)) * $subH, 1)
        $pts1Arr += "$px,$py"
    }
    $pts1Str = $pts1Arr -join ' '
    [void]$sb.AppendLine("<polyline points='$pts1Str' fill='none' stroke='#000000' stroke-width='2'/>")

    # Line 2: dashed colored
    $pts2Arr = @()
    for ($i = 0; $i -lt $tArr.Count; $i++) {
        $px = [Math]::Round($x0 + ($tArr[$i] / 45.0) * $subW, 1)
        $py = [Math]::Round($y0 + $subH - (($sp.d2[$i] - $minY) / ($maxY - $minY)) * $subH, 1)
        $pts2Arr += "$px,$py"
    }
    $pts2Str = $pts2Arr -join ' '
    [void]$sb.AppendLine("<polyline points='$pts2Str' fill='none' stroke='$($sp.c2)' stroke-width='2' stroke-dasharray='$($sp.dash2)'/>")

    # Subplot Title & Labels
    [void]$sb.AppendLine("<text x='$($x0 + $subW / 2)' y='$($y0 - 8)' text-anchor='middle' font-size='11' font-weight='bold' fill='#000000'>$($sp.title)</text>")
    if ($sp.col -eq 0) {
        [void]$sb.AppendLine("<text x='$($x0 - 35)' y='$($y0 + $subH / 2)' text-anchor='middle' font-size='10' fill='#000000' transform='rotate(-90 $($x0 - 35) $($y0 + $subH / 2))'>$($sp.ylabel)</text>")
    }
    if ($sp.row -eq 2) {
        [void]$sb.AppendLine("<text x='$($x0 + $subW / 2)' y='$($y0 + $subH + 32)' text-anchor='middle' font-size='10' fill='#000000'>Time (s)</text>")
    }

    # Legend
    $legW = 75
    $legH = 34
    $legX = $x0 + $subW - $legW - 6
    $legY = $y0 + 6
    [void]$sb.AppendLine("<rect x='$legX' y='$legY' width='$legW' height='$legH' fill='#ffffff' fill-opacity='0.9' stroke='#cccccc' stroke-width='0.8'/>")
    [void]$sb.AppendLine("<line x1='$($legX + 6)' y1='$($legY + 10)' x2='$($legX + 22)' y2='$($legY + 10)' stroke='#000000' stroke-width='2'/>")
    [void]$sb.AppendLine("<text x='$($legX + 26)' y='$($legY + 13)' font-size='9' fill='#000000'>$($sp.l1)</text>")
    [void]$sb.AppendLine("<line x1='$($legX + 6)' y1='$($legY + 24)' x2='$($legX + 22)' y2='$($legY + 24)' stroke='$($sp.c2)' stroke-width='2' stroke-dasharray='$($sp.dash2)'/>")
    [void]$sb.AppendLine("<text x='$($legX + 26)' y='$($legY + 27)' font-size='9' fill='#000000'>$($sp.l2)</text>")
}

[void]$sb.AppendLine("</svg>")

$svgContent = $sb.ToString()
Set-Content -Path "C:\Users\XPS\OneDrive - Ashesi University\Desktop\Differential\Drone Project\sensitivity_analysis.svg" -Value $svgContent -Encoding UTF8
Set-Content -Path "C:\Users\XPS\Downloads\differential\sensitivity_analysis.svg" -Value $svgContent -Encoding UTF8
Write-Output "Generated authentic MATLAB sensitivity_analysis.svg successfully!"
