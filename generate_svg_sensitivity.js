const fs = require('fs');

// Parameters from parameters.m
const m_base = 1.5;
const T = 20.0;
const k_base = 0.5;
const g = 9.81;
const pitch_base = 10.0 * Math.PI / 180.0;
const roll = 5.0 * Math.PI / 180.0;
const t0 = 0.0;
const tf = 45.0;
const h = 0.1;

const tArr = [];
for (let t = t0; t <= tf + 0.0001; t += h) {
    tArr.push(t);
}

function solveAnalytical(m, thrust, k, grav, pitch, rollAngle) {
    const Ax = (thrust / m) * Math.sin(pitch) * Math.cos(rollAngle);
    const Ay = -(thrust / m) * Math.sin(rollAngle);
    const Az = (thrust / m) * Math.cos(pitch) * Math.cos(rollAngle) - grav;

    const vx = [];
    const vy = [];
    const vz = [];

    for (let i = 0; i < tArr.length; i++) {
        const ti = tArr[i];
        vx.push((-Ax * m / k) * Math.exp(-k * ti / m) + Ax * m / k);
        vy.push((-Ay * m / k) * Math.exp(-k * ti / m) + Ay * m / k);
        vz.push((-Az * m / k) * Math.exp(-k * ti / m) + Az * m / k);
    }
    return { vx, vy, vz };
}

// 1. Mass variations: 1.0 kg vs 3.0 kg
const mass1 = solveAnalytical(1.0, T, k_base, g, pitch_base, roll);
const mass3 = solveAnalytical(3.0, T, k_base, g, pitch_base, roll);

// 2. Drag variations: 0.5 vs 1.0
const drag05 = solveAnalytical(m_base, T, 0.5, g, pitch_base, roll);
const drag10 = solveAnalytical(m_base, T, 1.0, g, pitch_base, roll);

// 3. Pitch variations: 10 deg vs 25 deg
const pitch10 = solveAnalytical(m_base, T, k_base, g, 10.0 * Math.PI / 180.0, roll);
const pitch25 = solveAnalytical(m_base, T, k_base, g, 25.0 * Math.PI / 180.0, roll);

// Construct SVG
const width = 1100;
const height = 850;
const margin = { top: 60, bottom: 40, left: 60, right: 30 };
const cols = 3;
const rows = 3;
const subW = (width - margin.left - margin.right - 80) / cols;
const subH = (height - margin.top - margin.bottom - 80) / rows;

let svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${width} ${height}" width="${width}" height="${height}" style="background:#ffffff; font-family: Arial, Helvetica, sans-serif;">\n`;
svg += `<rect width="${width}" height="${height}" fill="#ffffff"/>\n`;
svg += `<text x="${width / 2}" y="35" text-anchor="middle" font-size="20" font-weight="bold" fill="#000000">Sensitivity Analysis: Mass, Drag, and Pitch Variations (MATLAB)</text>\n`;

const subplots = [
    { row: 0, col: 0, title: 'Mass: Forward Velocity', ylabel: 'v_x (m/s)', d1: mass1.vx, d2: mass3.vx, l1: '1 kg', l2: '3 kg', c2: '#d90429', dash2: '6,4' },
    { row: 0, col: 1, title: 'Mass: Lateral Velocity', ylabel: 'v_y (m/s)', d1: mass1.vy, d2: mass3.vy, l1: '1 kg', l2: '3 kg', c2: '#d90429', dash2: '6,4' },
    { row: 0, col: 2, title: 'Mass: Vertical Velocity', ylabel: 'v_z (m/s)', d1: mass1.vz, d2: mass3.vz, l1: '1 kg', l2: '3 kg', c2: '#d90429', dash2: '6,4' },

    { row: 1, col: 0, title: 'Drag: Forward Velocity', ylabel: 'v_x (m/s)', d1: drag05.vx, d2: drag10.vx, l1: 'k = 0.5', l2: 'k = 1.0', c2: '#e63946', dash2: '6,4' },
    { row: 1, col: 1, title: 'Drag: Lateral Velocity', ylabel: 'v_y (m/s)', d1: drag05.vy, d2: drag10.vy, l1: 'k = 0.5', l2: 'k = 1.0', c2: '#e63946', dash2: '6,4' },
    { row: 1, col: 2, title: 'Drag: Vertical Velocity', ylabel: 'v_z (m/s)', d1: drag05.vz, d2: drag10.vz, l1: 'k = 0.5', l2: 'k = 1.0', c2: '#e63946', dash2: '6,4' },

    { row: 2, col: 0, title: 'Pitch: Forward Velocity', ylabel: 'v_x (m/s)', d1: pitch10.vx, d2: pitch25.vx, l1: '10°', l2: '25°', c2: '#0072BD', dash2: '6,4' },
    { row: 2, col: 1, title: 'Pitch: Lateral Velocity', ylabel: 'v_y (m/s)', d1: pitch10.vy, d2: pitch25.vy, l1: '10°', l2: '25°', c2: '#0072BD', dash2: '6,4' },
    { row: 2, col: 2, title: 'Pitch: Vertical Velocity', ylabel: 'v_z (m/s)', d1: pitch10.vz, d2: pitch25.vz, l1: '10°', l2: '25°', c2: '#0072BD', dash2: '6,4' }
];

subplots.forEach(sp => {
    const x0 = margin.left + sp.col * (subW + 40);
    const y0 = margin.top + sp.row * (subH + 40);

    const allY = sp.d1.concat(sp.d2);
    let minY = Math.min(...allY);
    let maxY = Math.max(...allY);
    if (minY === maxY) { minY -= 1; maxY += 1; }
    const padY = (maxY - minY) * 0.1 || 0.5;
    minY -= padY;
    maxY += padY;

    // Grid and Box
    svg += `<rect x="${x0}" y="${y0}" width="${subW}" height="${subH}" fill="#ffffff" stroke="#000000" stroke-width="1"/>\n`;

    // Horizontal gridlines
    const numYTicks = 5;
    for (let k = 0; k <= numYTicks; k++) {
        const val = minY + (k / numYTicks) * (maxY - minY);
        const yPos = y0 + subH - (k / numYTicks) * subH;
        svg += `<line x1="${x0}" y1="${yPos}" x2="${x0 + subW}" y2="${yPos}" stroke="#e0e0e0" stroke-width="0.8" stroke-dasharray="2,2"/>\n`;
        svg += `<text x="${x0 - 6}" y="${yPos + 4}" text-anchor="end" font-size="10" fill="#333333">${val.toFixed(1)}</text>\n`;
    }

    // Vertical gridlines
    for (let ti = 0; ti <= 45; ti += 10) {
        const xPos = x0 + (ti / 45) * subW;
        svg += `<line x1="${xPos}" y1="${y0}" x2="${xPos}" y2="${y0 + subH}" stroke="#e0e0e0" stroke-width="0.8" stroke-dasharray="2,2"/>\n`;
        if (sp.row === 2) {
            svg += `<text x="${xPos}" y="${y0 + subH + 14}" text-anchor="middle" font-size="10" fill="#333333">${ti}</text>\n`;
        }
    }

    // Line 1: solid black
    let pts1 = sp.d1.map((v, idx) => {
        const px = x0 + (tArr[idx] / 45) * subW;
        const py = y0 + subH - ((v - minY) / (maxY - minY)) * subH;
        return `${px.toFixed(1)},${py.toFixed(1)}`;
    }).join(' ');
    svg += `<polyline points="${pts1}" fill="none" stroke="#000000" stroke-width="2"/>\n`;

    // Line 2: dashed
    let pts2 = sp.d2.map((v, idx) => {
        const px = x0 + (tArr[idx] / 45) * subW;
        const py = y0 + subH - ((v - minY) / (maxY - minY)) * subH;
        return `${px.toFixed(1)},${py.toFixed(1)}`;
    }).join(' ');
    svg += `<polyline points="${pts2}" fill="none" stroke="${sp.c2}" stroke-width="2" stroke-dasharray="${sp.dash2}"/>\n`;

    // Titles & Labels
    svg += `<text x="${x0 + subW / 2}" y="${y0 - 8}" text-anchor="middle" font-size="11" font-weight="bold" fill="#000000">${sp.title}</text>\n`;
    if (sp.col === 0) {
        svg += `<text x="${x0 - 32}" y="${y0 + subH / 2}" text-anchor="middle" font-size="10" fill="#000000" transform="rotate(-90 ${x0 - 32} ${y0 + subH / 2})">${sp.ylabel}</text>\n`;
    }
    if (sp.row === 2) {
        svg += `<text x="${x0 + subW / 2}" y="${y0 + subH + 30}" text-anchor="middle" font-size="10" fill="#000000">Time (s)</text>\n`;
    }

    // Legend box inside top right
    const legW = 75;
    const legH = 34;
    const legX = x0 + subW - legW - 6;
    const legY = y0 + 6;
    svg += `<rect x="${legX}" y="${legY}" width="${legW}" height="${legH}" fill="#ffffff" fill-opacity="0.9" stroke="#cccccc" stroke-width="0.8"/>\n`;
    svg += `<line x1="${legX + 6}" y1="${legY + 10}" x2="${legX + 22}" y2="${legY + 10}" stroke="#000000" stroke-width="2"/>\n`;
    svg += `<text x="${legX + 26}" y="${legY + 13}" font-size="9" fill="#000000">${sp.l1}</text>\n`;
    svg += `<line x1="${legX + 6}" y1="${legY + 24}" x2="${legX + 22}" y2="${legY + 24}" stroke="${sp.c2}" stroke-width="2" stroke-dasharray="${sp.dash2}"/>\n`;
    svg += `<text x="${legX + 26}" y="${legY + 27}" font-size="9" fill="#000000">${sp.l2}</text>\n`;
});

svg += `</svg>`;

fs.writeFileSync('C:/Users/XPS/OneDrive - Ashesi University/Desktop/Differential/Drone Project/sensitivity_analysis.svg', svg);
fs.writeFileSync('C:/Users/XPS/Downloads/differential/sensitivity_analysis.svg', svg);
console.log('Saved sensitivity_analysis.svg successfully!');
