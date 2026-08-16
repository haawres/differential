// =========================================================================
// KEMS - UAV Autonomous Navigation Project — Client-Side Application Engine
// Author: KEMS UAV Research Group (MATH221 & CE122)
// Features: SPA Router, Interactive HTML5 Canvas Flight Simulator, Authentic MATLAB Palette Chart Visualizations
// =========================================================================

// =========================================================================
// 1. Single Page Application (SPA) Router
// =========================================================================
function navigateToPage(targetPageId) {
    const pages = document.querySelectorAll('.page-section');
    pages.forEach(page => {
        page.classList.remove('active-page');
    });

    const activeTarget = document.getElementById(targetPageId);
    if (activeTarget) {
        activeTarget.classList.add('active-page');
        window.scrollTo({ top: 0, behavior: 'smooth' });
    }

    // Update active state in top navigation
    const navLinks = document.querySelectorAll('.nav-link');
    navLinks.forEach(link => {
        link.classList.remove('active');
        const href = link.getAttribute('href');
        if (href === `#${targetPageId}`) {
            link.classList.add('active');
        }
    });

    // Close mobile navigation drawer if open
    const navBar = document.getElementById('nav-bar');
    if (navBar && navBar.classList.contains('open-nav')) {
        navBar.classList.remove('open-nav');
    }

    // Handle Simulator Canvas Resizing & Starting
    if (targetPageId === 'model-shortcommings') {
        setTimeout(() => {
            resizeCanvas();
            resetFlight();
            if (!animFrameId) {
                animFrameId = requestAnimationFrame(simulationLoop);
            }
        }, 80);
    }

    // Initialize Chart.js benchmarks when navigating to results or sensitivity
    if (targetPageId === 'our-results-analysis') {
        setTimeout(initSolverCharts, 100);
    } else if (targetPageId === 'parameter-sensitivity-analysis') {
        setTimeout(initSensitivityCharts, 100);
    }
}

// Handle Browser Hash Routing (e.g. #equations-explained)
window.addEventListener('DOMContentLoaded', () => {
    const currentHash = window.location.hash.substring(1);
    if (currentHash && document.getElementById(currentHash)) {
        navigateToPage(currentHash);
    } else {
        navigateToPage('home');
    }

    // Mobile Drawer Toggle
    const menuToggle = document.getElementById('menu-toggle');
    const navBar = document.getElementById('nav-bar');
    if (menuToggle && navBar) {
        menuToggle.addEventListener('click', () => {
            navBar.classList.toggle('open-nav');
        });
    }

    // Initialize Simulation Controls
    initSimulator();
});

window.addEventListener('hashchange', () => {
    const currentHash = window.location.hash.substring(1);
    if (currentHash && document.getElementById(currentHash)) {
        navigateToPage(currentHash);
    }
});


// =========================================================================
// 2. Interactive Scientific Picture Depiction Explorer
// =========================================================================
const depictionData = {
    thrust: {
        title: "1. Motor Thrust & Propeller Lift Forces (f₁, f₂, f₃, f₄)",
        text: "Each spinning rotor creates upward thrust proportional to the square of its rotational speed: f_i = b · ω_i². The sum of all 4 forces creates the total upward thrust U₁ = f₁ + f₂ + f₃ + f₄. When U₁ exceeds total weight (m · g = 18.52 N), the drone accelerates upward into the sky.",
        btnId: "btn-dep-thrust"
    },
    angles: {
        title: "2. Rotational Tilt & Banking Angles (Roll φ, Pitch θ, Yaw ψ)",
        text: "A quadrotor is under-actuated: it cannot move sideways or forward without tilting first! Spinning back motor 3 faster than front motor 1 tilts the drone forward (Pitch θ > 0), redirecting part of upward lift U₁ horizontally to drive forward motion in X.",
        btnId: "btn-dep-angles"
    },
    lidar: {
        title: "3. 2D LiDAR Rangefinder & Bat Algorithm Obstacle Avoidance",
        text: "The onboard LiDAR emits a continuous 2D horizontal laser fan. When an acacia tree or stray livestock enters the 1.0-meter safety threshold, the algorithm injects a perpendicular tangential steering force into the equations of motion, smoothly banking the drone around the obstacle.",
        btnId: "btn-dep-lidar"
    },
    pid: {
        title: "4. Dual PID Flight Control Loop (The Robot Brain)",
        text: "Continuously compares desired flight coordinates against onboard IMU gyroscope/accelerometer measurements. It calculates e(t) = target - actual and dynamically modulates individual motor speeds to cancel out error with optimal 2.65% damping.",
        btnId: "btn-dep-pid"
    }
};

function highlightDepiction(type) {
    const info = depictionData[type] || depictionData.thrust;
    const titleEl = document.getElementById('depiction-detail-title');
    const textEl = document.getElementById('depiction-detail-text');

    if (titleEl && textEl) {
        titleEl.textContent = info.title;
        textEl.textContent = info.text;
    }

    // Toggle active button style
    document.querySelectorAll('.interactive-callout-btn').forEach(btn => {
        btn.classList.remove('active-callout');
    });

    const activeBtn = document.getElementById(info.btnId);
    if (activeBtn) activeBtn.classList.add('active-callout');
}


// =========================================================================
// 3. Interactive HTML5 Canvas Flight Simulator Engine
// =========================================================================
let canvas, ctx;
let simRunning = true;
let animFrameId = null;
let rotorAngle = 0;

// Drone State Vector: [x, y, z, vx, vy, vz, phi, theta, psi]
const drone = {
    x: 50,
    y: 340,
    z: 5.0,
    vx: 0,
    vy: 0,
    vz: 0,
    phi: 0,
    theta: 0,
    psi: 0,
    mass: 1.888,
    trail: []
};

// Simulation Configuration
const simConfig = {
    scene: 'single',        // 'single' or 'three'
    controller: 'itae',     // 'itae' or 'ise'
    threshold: 1.0,         // 1.0m (safe) or 0.5m (crash)
    windX: 0.0,             // Lateral wind disturbance (N)
    lag: 0.05,              // Sensor feedback lag (s)
    speedFactor: 1.3,
    target: { x: 620, y: 75 }
};

// Obstacle Database
const obstacleScenes = {
    single: [
        { x: 340, y: 210, r: 40, label: 'Acacia Tree Obstacle' }
    ],
    three: [
        { x: 220, y: 270, r: 32, label: 'Grazing Sheep Flock' },
        { x: 380, y: 190, r: 42, label: 'Acacia Tree' },
        { x: 520, y: 130, r: 30, label: 'Farm Perimeter Gate' }
    ]
};

// Sensor Latency Queue
let stateHistoryQueue = [];

function initSimulator() {
    canvas = document.getElementById('simulation-canvas');
    if (!canvas) return;

    ctx = canvas.getContext('2d');
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    // Attach Sliders
    const slideWind = document.getElementById('slide-wind');
    const valWind = document.getElementById('val-wind');
    if (slideWind && valWind) {
        slideWind.addEventListener('input', (e) => {
            simConfig.windX = parseFloat(e.target.value);
            valWind.textContent = `${simConfig.windX.toFixed(2)} N`;
        });
    }

    const slideMass = document.getElementById('slide-mass');
    const valMass = document.getElementById('val-mass');
    if (slideMass && valMass) {
        slideMass.addEventListener('input', (e) => {
            drone.mass = parseFloat(e.target.value);
            valMass.textContent = `${drone.mass.toFixed(2)} kg`;
        });
    }

    const slideLag = document.getElementById('slide-lag');
    const valLag = document.getElementById('val-lag');
    if (slideLag && valLag) {
        slideLag.addEventListener('input', (e) => {
            simConfig.lag = parseFloat(e.target.value);
            valLag.textContent = `${simConfig.lag.toFixed(2)} s`;
        });
    }

    // Play / Pause & Reset
    const btnPlayPause = document.getElementById('btn-play-pause');
    if (btnPlayPause) {
        btnPlayPause.addEventListener('click', () => {
            simRunning = !simRunning;
            btnPlayPause.textContent = simRunning ? 'Pause Flight' : 'Resume Flight';
        });
    }

    const btnResetSim = document.getElementById('btn-reset-sim');
    if (btnResetSim) {
        btnResetSim.addEventListener('click', resetFlight);
    }

    resetFlight();
    if (!animFrameId) {
        animFrameId = requestAnimationFrame(simulationLoop);
    }
}

function resizeCanvas() {
    if (!canvas) return;
    const parent = canvas.parentElement;
    const parentWidth = parent ? parent.clientWidth : 720;
    canvas.width = parentWidth > 100 ? parentWidth : 760;
    canvas.height = 420;
    simConfig.target = { x: canvas.width - 60, y: 70 };
}

function resetFlight() {
    drone.x = 50;
    drone.y = canvas ? canvas.height - 60 : 340;
    drone.vx = 0;
    drone.vy = 0;
    drone.vz = 0;
    drone.phi = 0;
    drone.theta = 0;
    drone.trail = [];
    stateHistoryQueue = [];
}

// Mode Selection Handlers
function setSimScene(sceneType) {
    simConfig.scene = sceneType;
    const btnSingle = document.getElementById('scene-single');
    const btnThree = document.getElementById('scene-three');
    if (btnSingle) btnSingle.classList.toggle('active-mode', sceneType === 'single');
    if (btnThree) btnThree.classList.toggle('active-mode', sceneType === 'three');
    resetFlight();
}

function setSimController(ctrlType) {
    simConfig.controller = ctrlType;
    const btnItae = document.getElementById('ctrl-itae');
    const btnIse = document.getElementById('ctrl-ise');
    if (btnItae) btnItae.classList.toggle('active-mode', ctrlType === 'itae');
    if (btnIse) btnIse.classList.toggle('active-mode', ctrlType === 'ise');
    resetFlight();
}

function setSimThreshold(threshVal) {
    simConfig.threshold = threshVal;
    const btn10 = document.getElementById('thresh-10');
    const btn05 = document.getElementById('thresh-05');
    if (btn10) btn10.classList.toggle('active-mode', threshVal === 1.0);
    if (btn05) btn05.classList.toggle('active-mode', threshVal === 0.5);
    resetFlight();
}

// Main Physics & Simulation Loop
function simulationLoop() {
    if (simRunning) {
        updatePhysics(0.016);
    }
    renderScene();
    updateTelemetry();
    rotorAngle += 0.35;
    animFrameId = requestAnimationFrame(simulationLoop);
}

function updatePhysics(dt) {
    // Latency Queue
    stateHistoryQueue.push({ x: drone.x, y: drone.y, time: performance.now() });
    const lagMs = simConfig.lag * 1000;
    const now = performance.now();
    while (stateHistoryQueue.length > 0 && (now - stateHistoryQueue[0].time) > lagMs) {
        stateHistoryQueue.shift();
    }
    const delayedState = stateHistoryQueue.length > 0 ? stateHistoryQueue[0] : { x: drone.x, y: drone.y };

    // Goal Attraction Vector
    const dx = simConfig.target.x - delayedState.x;
    const dy = simConfig.target.y - delayedState.y;
    const distToGoal = Math.sqrt(dx * dx + dy * dy);

    // Controller Tuning
    let kp = (simConfig.controller === 'itae') ? 1.4 : 3.0;
    let kd = (simConfig.controller === 'itae') ? 1.6 : 0.7;

    let fx = (dx / (distToGoal + 1e-4)) * kp * 60 - drone.vx * kd;
    let fy = (dy / (distToGoal + 1e-4)) * kp * 60 - drone.vy * kd;

    // Obstacle Avoidance (Bat Algorithm Repulsion)
    const currentObstacles = obstacleScenes[simConfig.scene] || obstacleScenes.single;
    currentObstacles.forEach(obs => {
        const ox = obs.x - drone.x;
        const oy = obs.y - drone.y;
        const dObs = Math.sqrt(ox * ox + oy * oy);
        const warningRadius = obs.r + (simConfig.threshold * 55);

        if (dObs < warningRadius) {
            const penetration = (warningRadius - dObs) / warningRadius;
            const repAngle = Math.atan2(oy, ox);
            // Tangential evasion vector
            const tangentAngle = repAngle - Math.PI / 2;
            const repStrength = penetration * penetration * 420;

            fx -= Math.cos(repAngle) * repStrength;
            fy -= Math.sin(repAngle) * repStrength;
            fx += Math.cos(tangentAngle) * repStrength * 0.9;
            fy += Math.sin(tangentAngle) * repStrength * 0.9;
        }
    });

    // Body Drag (-k * v)
    const dragCoeff = 0.45;
    const dragX = -dragCoeff * drone.vx;
    const dragY = -dragCoeff * drone.vy;

    // Accelerations derived from ODEs: a = (F_control + F_drag + F_wind) / m
    const ax = (fx + dragX + simConfig.windX * 80) / drone.mass;
    const ay = (fy + dragY) / drone.mass;

    // Forward Euler Integration Step
    drone.vx += ax * dt;
    drone.vy += ay * dt;
    drone.x += drone.vx * dt * simConfig.speedFactor;
    drone.y += drone.vy * dt * simConfig.speedFactor;

    // Banking Roll Angle
    drone.phi = Math.max(-0.45, Math.min(0.45, drone.vx * 0.08));

    // Flight Trail History
    drone.trail.push({ x: drone.x, y: drone.y });
    if (drone.trail.length > 350) drone.trail.shift();

    // Reached Goal Check
    if (distToGoal < 20) {
        drone.vx *= 0.85;
        drone.vy *= 0.85;
    }
}

function renderScene() {
    if (!ctx || !canvas) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Grid Background
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
    ctx.lineWidth = 1;
    for (let x = 0; x < canvas.width; x += 40) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
    }
    for (let y = 0; y < canvas.height; y += 40) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
    }

    // 2. Draw Obstacles (Trees / Sheep)
    const currentObstacles = obstacleScenes[simConfig.scene] || obstacleScenes.single;
    let collisionDetected = false;

    currentObstacles.forEach(obs => {
        const warningRadius = obs.r + (simConfig.threshold * 55);
        const distToObs = Math.hypot(obs.x - drone.x, obs.y - drone.y);

        if (distToObs < (obs.r + 14)) {
            collisionDetected = true;
        }

        // Safety Margin Circle
        ctx.strokeStyle = (simConfig.threshold === 1.0) ? 'rgba(0, 180, 216, 0.4)' : 'rgba(231, 111, 81, 0.45)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([5, 4]);
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, warningRadius, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        // Obstacle Body
        const gradient = ctx.createRadialGradient(obs.x, obs.y, 5, obs.x, obs.y, obs.r);
        gradient.addColorStop(0, 'rgba(231, 111, 81, 0.75)');
        gradient.addColorStop(1, 'rgba(231, 111, 81, 0.25)');
        ctx.fillStyle = gradient;
        ctx.strokeStyle = '#e76f51';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, obs.r, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Label
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 11px Inter';
        ctx.fillText(obs.label, obs.x - 45, obs.y + obs.r + 16);
    });

    // 3. Target Goal
    ctx.fillStyle = '#2ec4b6';
    ctx.beginPath();
    ctx.arc(simConfig.target.x, simConfig.target.y, 12, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#2ec4b6';
    ctx.font = 'bold 11px JetBrains Mono';
    ctx.fillText('TARGET GOAL (5m Hover)', simConfig.target.x - 70, simConfig.target.y - 18);

    // 4. Flight Path Trail
    if (drone.trail.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = (simConfig.controller === 'itae') ? '#00b4d8' : '#f4a261';
        ctx.lineWidth = 3;
        ctx.shadowColor = (simConfig.controller === 'itae') ? '#00b4d8' : '#f4a261';
        ctx.shadowBlur = 8;
        ctx.moveTo(drone.trail[0].x, drone.trail[0].y);
        for (let i = 1; i < drone.trail.length; i++) {
            ctx.lineTo(drone.trail[i].x, drone.trail[i].y);
        }
        ctx.stroke();
        ctx.shadowBlur = 0;
    }

    // 5. 2D LiDAR Scanning Cone
    ctx.strokeStyle = 'rgba(0, 180, 216, 0.28)';
    ctx.fillStyle = 'rgba(0, 180, 216, 0.06)';
    ctx.beginPath();
    ctx.arc(drone.x, drone.y, 85, -0.6 + drone.phi, 0.6 + drone.phi);
    ctx.lineTo(drone.x, drone.y);
    ctx.fill();
    ctx.stroke();

    // 6. Draw Quadcopter Body & Spinning Rotors
    ctx.save();
    ctx.translate(drone.x, drone.y);
    ctx.rotate(drone.phi);

    // Drone Arms (+)
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-18, 0); ctx.lineTo(18, 0);
    ctx.moveTo(0, -18); ctx.lineTo(0, 18);
    ctx.stroke();

    // 4 Motors & Spinning Propellers
    const motorPositions = [
        { x: -18, y: 0 }, { x: 18, y: 0 },
        { x: 0, y: -18 }, { x: 0, y: 18 }
    ];

    motorPositions.forEach((pos, idx) => {
        // Motor Mount
        ctx.fillStyle = '#2ec4b6';
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 4, 0, Math.PI * 2);
        ctx.fill();

        // Spinning Propeller
        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate((idx % 2 === 0 ? 1 : -1) * rotorAngle);
        ctx.strokeStyle = 'rgba(0, 180, 216, 0.85)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-10, 0); ctx.lineTo(10, 0);
        ctx.stroke();
        ctx.restore();
    });

    // Center Fuselage & Camera Gimbal
    ctx.fillStyle = '#070c1e';
    ctx.strokeStyle = '#00b4d8';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.restore();

    // Status Banner in Canvas
    if (collisionDetected) {
        ctx.fillStyle = 'rgba(231, 111, 81, 0.92)';
        ctx.fillRect(15, 15, 230, 30);
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 12px Inter';
        ctx.fillText('⚠️ WARNING: COLLISION DETECTED!', 25, 35);
    } else {
        ctx.fillStyle = 'rgba(46, 196, 182, 0.9)';
        ctx.fillRect(15, 15, 190, 30);
        ctx.fillStyle = '#070c1e';
        ctx.font = 'bold 12px Inter';
        ctx.fillText('✅ SAFE CLEARANCE ACTIVE', 25, 35);
    }
}

function updateTelemetry() {
    const telemetry = document.getElementById('telemetry-readout');
    if (telemetry) {
        const speed = Math.sqrt(drone.vx * drone.vx + drone.vy * drone.vy) * 0.1;
        telemetry.textContent = `X: ${(drone.x * 0.1).toFixed(1)}m | Y: ${(drone.y * 0.1).toFixed(1)}m | Speed: ${speed.toFixed(2)}m/s | Mass: ${drone.mass.toFixed(2)}kg | Wind: ${simConfig.windX.toFixed(2)}N`;
    }
}


// =========================================================================
// 4. Chart.js Interactive Benchmark Visualizations (Authentic MATLAB Color Palette)
// MATLAB Default 'lines' Palette:
// [1] Blue:   #0072BD (rgb: 0, 114, 189)
// [2] Orange: #D95319 (rgb: 217, 83, 25)
// [3] Yellow: #EDB120 (rgb: 237, 177, 32)
// [4] Purple: #7E2F8E (rgb: 126, 47, 142)
// [5] Green:  #77AC30 (rgb: 119, 172, 48)
// [6] Cyan:   #4DBEEE (rgb: 77, 190, 238)
// [7] Maroon: #A2142F (rgb: 162, 20, 47)
// =========================================================================
let chartRuntime = null;
let chartRmse = null;
let chartConvergence = null;
let chartStability = null;
let chartSensKd = null;
let chartSensMass = null;
let chartSensKp = null;

function initSolverCharts() {
    // 1. Solver Runtime Bar Chart (ode45 vs RK4 vs Euler) - Authentic MATLAB Bar Style
    const ctxRuntime = document.getElementById('chart-solver-runtime');
    if (ctxRuntime && !chartRuntime) {
        chartRuntime = new Chart(ctxRuntime, {
            type: 'bar',
            data: {
                labels: ['MATLAB ode45', 'Classical RK4', 'Forward Euler'],
                datasets: [{
                    label: 'Runtime (ms)',
                    data: [1.84, 4.20, 3.10],
                    backgroundColor: ['#0072BD', '#D95319', '#7E2F8E'],
                    borderColor: '#222222',
                    borderWidth: 1,
                    borderRadius: 2
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { labels: { color: '#111827', font: { weight: '600' } } }
                },
                scales: {
                    y: { 
                        grid: { color: '#e5e7eb' }, 
                        ticks: { color: '#111827', font: { weight: '600' } },
                        title: { display: true, text: 'Execution Time (ms)', color: '#111827', font: { weight: '700' } }
                    },
                    x: { 
                        grid: { display: false }, 
                        ticks: { color: '#111827', font: { weight: '700' } } 
                    }
                }
            }
        });
    }

    // 2. Solver Accuracy (RMSE) Bar Chart - Authentic MATLAB Bar Style
    const ctxRmse = document.getElementById('chart-solver-rmse');
    if (ctxRmse && !chartRmse) {
        chartRmse = new Chart(ctxRmse, {
            type: 'bar',
            data: {
                labels: ['MATLAB ode45', 'Classical RK4', 'Forward Euler'],
                datasets: [{
                    label: 'RMSE Error vs. Exact Benchmark (m)',
                    data: [0.0000412, 0.0000894, 0.0482],
                    backgroundColor: ['#0072BD', '#77AC30', '#A2142F'],
                    borderColor: '#222222',
                    borderWidth: 1,
                    borderRadius: 2
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { labels: { color: '#111827', font: { weight: '600' } } }
                },
                scales: {
                    y: { 
                        type: 'logarithmic',
                        grid: { color: '#e5e7eb' }, 
                        ticks: { color: '#111827', font: { weight: '600' } },
                        title: { display: true, text: 'RMSE (m) [Log Scale]', color: '#111827', font: { weight: '700' } }
                    },
                    x: { 
                        grid: { display: false }, 
                        ticks: { color: '#111827', font: { weight: '700' } } 
                    }
                }
            }
        });
    }

    // 3. Step-Size Convergence Log-Log Chart - Authentic MATLAB Line Style
    const ctxConv = document.getElementById('chart-convergence');
    if (ctxConv && !chartConvergence) {
        chartConvergence = new Chart(ctxConv, {
            type: 'line',
            data: {
                labels: ['0.20s', '0.10s', '0.05s', '0.02s', '0.01s', '0.005s'],
                datasets: [
                    {
                        label: 'Forward Euler (Slope = 1, O(h))',
                        data: [0.48, 0.24, 0.12, 0.048, 0.024, 0.012],
                        borderColor: '#A2142F',
                        backgroundColor: '#A2142F',
                        borderWidth: 2,
                        pointRadius: 4,
                        tension: 0
                    },
                    {
                        label: 'Classical RK4 (Slope = 4, O(h⁴))',
                        data: [0.091, 0.0057, 0.00035, 0.000089, 0.0000056, 0.00000012],
                        borderColor: '#0072BD',
                        backgroundColor: '#0072BD',
                        borderWidth: 2,
                        pointRadius: 4,
                        tension: 0
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { labels: { color: '#111827', font: { weight: '700' } } }
                },
                scales: {
                    y: { 
                        type: 'logarithmic',
                        grid: { color: '#e5e7eb' }, 
                        ticks: { color: '#111827', font: { weight: '600' } },
                        title: { display: true, text: 'Global Error (RMSE, Log Scale)', color: '#111827', font: { weight: '700' } }
                    },
                    x: { 
                        grid: { color: '#f3f4f6' }, 
                        ticks: { color: '#111827', font: { weight: '700' } },
                        title: { display: true, text: 'Time Step Size h (seconds)', color: '#111827', font: { weight: '700' } }
                    }
                }
            }
        });
    }

    // 4. Numerical Stability Chart - Authentic MATLAB Line Style
    const ctxStab = document.getElementById('chart-stability');
    if (ctxStab && !chartStability) {
        chartStability = new Chart(ctxStab, {
            type: 'line',
            data: {
                labels: ['0s', '1s', '2s', '3s', '4s', '5s', '6s', '7s', '8s', '9s', '10s'],
                datasets: [
                    {
                        label: 'Exact Analytical Benchmark',
                        data: [0.0, 2.5, 4.4, 5.13, 5.04, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#000000',
                        borderWidth: 2.5,
                        pointRadius: 0,
                        tension: 0
                    },
                    {
                        label: 'Euler (h = 0.02s - Stable)',
                        data: [0.0, 2.45, 4.38, 5.18, 5.05, 5.01, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#0072BD',
                        borderDash: [5, 4],
                        pointRadius: 3
                    },
                    {
                        label: 'Euler (h = 0.10s - Severe Oscillations)',
                        data: [0.0, 2.8, 5.4, 4.2, 5.8, 4.5, 5.4, 4.7, 5.2, 4.9, 5.0],
                        borderColor: '#D95319',
                        borderWidth: 2,
                        pointRadius: 4
                    },
                    {
                        label: 'Euler (h = 0.25s - Diverging Blowup)',
                        data: [0.0, 3.8, 7.2, 1.8, 9.4, 0.2, 12.5, 0.0, 16.0, 0.0, 22.0],
                        borderColor: '#A2142F',
                        borderWidth: 2,
                        pointRadius: 4
                    },
                    {
                        label: 'RK4 (h = 0.10s - Stable)',
                        data: [0.0, 2.5, 4.4, 5.13, 5.04, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#77AC30',
                        borderDash: [2, 2],
                        pointRadius: 3
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { labels: { color: '#111827', font: { weight: '700' } } }
                },
                scales: {
                    y: { 
                        min: 0, max: 12,
                        grid: { color: '#e5e7eb' }, 
                        ticks: { color: '#111827', font: { weight: '600' } },
                        title: { display: true, text: 'Altitude z (meters)', color: '#111827', font: { weight: '700' } }
                    },
                    x: { 
                        grid: { color: '#f3f4f6' }, 
                        ticks: { color: '#111827', font: { weight: '700' } },
                        title: { display: true, text: 'Time (seconds)', color: '#111827', font: { weight: '700' } }
                    }
                }
            }
        });
    }
}

// 5. Parameter Sensitivity Time-Series Response Charts (Authentic MATLAB Palette)
function initSensitivityCharts() {
    // Chart 1: Kd Damping Sensitivity
    const ctxKd = document.getElementById('chart-sens-kd');
    if (ctxKd && !chartSensKd) {
        chartSensKd = new Chart(ctxKd, {
            type: 'line',
            data: {
                labels: ['0s', '1s', '2s', '3s', '4s', '5s', '6s', '7s', '8s', '9s', '10s'],
                datasets: [
                    {
                        label: '-20% Kd (2.92) - Overshoot 7.80%',
                        data: [0.0, 2.7, 4.7, 5.39, 4.88, 5.05, 4.98, 5.01, 5.00, 5.00, 5.00],
                        borderColor: '#A2142F',
                        borderWidth: 2,
                        tension: 0
                    },
                    {
                        label: '-10% Kd (3.28) - Overshoot 4.90%',
                        data: [0.0, 2.6, 4.5, 5.25, 4.94, 5.02, 4.99, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#D95319',
                        borderWidth: 2,
                        tension: 0
                    },
                    {
                        label: 'Baseline Kd (3.65) - Optimal 2.65%',
                        data: [0.0, 2.5, 4.4, 5.13, 5.02, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#0072BD',
                        borderWidth: 3,
                        tension: 0
                    },
                    {
                        label: '+10% Kd (4.01) - Well Damped 1.20%',
                        data: [0.0, 2.4, 4.2, 5.06, 5.01, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#77AC30',
                        borderWidth: 2,
                        tension: 0
                    },
                    {
                        label: '+20% Kd (4.38) - Overdamped 0.40%',
                        data: [0.0, 2.3, 4.0, 5.02, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#7E2F8E',
                        borderWidth: 2,
                        tension: 0
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { labels: { color: '#111827', font: { weight: '600', size: 10 } } }
                },
                scales: {
                    y: { 
                        min: 0, max: 6.0, 
                        grid: { color: '#e5e7eb' }, 
                        ticks: { color: '#111827' },
                        title: { display: true, text: 'Altitude z (m)', color: '#111827', font: { weight: '700' } }
                    },
                    x: { 
                        grid: { color: '#f3f4f6' }, 
                        ticks: { color: '#111827' },
                        title: { display: true, text: 'Time (s)', color: '#111827', font: { weight: '700' } }
                    }
                }
            }
        });
    }

    // Chart 2: Mass Sensitivity
    const ctxMass = document.getElementById('chart-sens-mass');
    if (ctxMass && !chartSensMass) {
        chartSensMass = new Chart(ctxMass, {
            type: 'line',
            data: {
                labels: ['0s', '1s', '2s', '3s', '4s', '5s', '6s', '7s', '8s', '9s', '10s'],
                datasets: [
                    {
                        label: '-20% Mass (1.51 kg) - Fast Rise',
                        data: [0.0, 2.8, 4.7, 5.06, 5.01, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#77AC30',
                        borderWidth: 2,
                        tension: 0
                    },
                    {
                        label: 'Baseline Mass (1.89 kg) - Standard',
                        data: [0.0, 2.5, 4.4, 5.13, 5.02, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#0072BD',
                        borderWidth: 3,
                        tension: 0
                    },
                    {
                        label: '+20% Mass (2.27 kg) - Heavy Camera Payload (Slow Rise)',
                        data: [0.0, 2.1, 3.8, 4.85, 5.18, 5.04, 5.01, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#A2142F',
                        borderWidth: 2,
                        tension: 0
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { labels: { color: '#111827', font: { weight: '600', size: 10 } } }
                },
                scales: {
                    y: { 
                        min: 0, max: 6.0, 
                        grid: { color: '#e5e7eb' }, 
                        ticks: { color: '#111827' },
                        title: { display: true, text: 'Altitude z (m)', color: '#111827', font: { weight: '700' } }
                    },
                    x: { 
                        grid: { color: '#f3f4f6' }, 
                        ticks: { color: '#111827' },
                        title: { display: true, text: 'Time (s)', color: '#111827', font: { weight: '700' } }
                    }
                }
            }
        });
    }

    // Chart 3: Kp Spring Gain Sensitivity
    const ctxKp = document.getElementById('chart-sens-kp');
    if (ctxKp && !chartSensKp) {
        chartSensKp = new Chart(ctxKp, {
            type: 'line',
            data: {
                labels: ['0s', '1s', '2s', '3s', '4s', '5s', '6s', '7s', '8s', '9s', '10s'],
                datasets: [
                    {
                        label: 'Kp = 4.0 (-20% Spring) - Slow Rise',
                        data: [0.0, 2.2, 4.0, 4.95, 5.03, 5.01, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#D95319',
                        borderWidth: 2,
                        tension: 0
                    },
                    {
                        label: 'Kp = 5.0 (Baseline) - Optimal Rise',
                        data: [0.0, 2.5, 4.4, 5.13, 5.02, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#0072BD',
                        borderWidth: 3,
                        tension: 0
                    },
                    {
                        label: 'Kp = 6.0 (+20% Spring) - Overshoot 4.80%',
                        data: [0.0, 2.8, 4.7, 5.24, 4.96, 5.02, 5.00, 5.00, 5.00, 5.00, 5.00],
                        borderColor: '#A2142F',
                        borderWidth: 2,
                        tension: 0
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: { labels: { color: '#111827', font: { weight: '600', size: 10 } } }
                },
                scales: {
                    y: { 
                        min: 0, max: 6.0, 
                        grid: { color: '#e5e7eb' }, 
                        ticks: { color: '#111827' },
                        title: { display: true, text: 'Altitude z (m)', color: '#111827', font: { weight: '700' } }
                    },
                    x: { 
                        grid: { color: '#f3f4f6' }, 
                        ticks: { color: '#111827' },
                        title: { display: true, text: 'Time (s)', color: '#111827', font: { weight: '700' } }
                    }
                }
            }
        });
    }
}
