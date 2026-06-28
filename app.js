// ==========================================
// 1. Navigation & Page Router (Wix Transitions)
// ==========================================

function navigateToPage(targetId) {
    const activePage = document.querySelector('.page-section.active-page');
    const targetPage = document.getElementById(targetId);
    
    if (!targetPage) return;
    if (activePage && activePage.id === targetId) return;

    // Reset all sub-dropdown states for mobile
    const dropdowns = document.querySelectorAll('.has-dropdown');
    dropdowns.forEach(d => d.classList.remove('mobile-dropdown-open'));

    // Highlight current active header link (including dropdown parents)
    const navLinks = document.querySelectorAll('.nav-link');
    navLinks.forEach(link => {
        const href = link.getAttribute('href');
        const isParentModel = href === '#the-model' && ['detailed-analysis', 'equations-explained', 'parameter-definition'].includes(targetId);
        const isParentContrib = href === '#copy-of-the-model' && ['model-shortcommings', 'parameter-sensitivity-analysis', 'future-improvements', 'applied-prgramming-aspect'].includes(targetId);
        
        if (href === '#' + targetId || isParentModel || isParentContrib) {
            link.classList.add('active');
        } else {
            link.classList.remove('active');
        }
    });

    // Animate transition using active-page and is-visible classes
    if (activePage) {
        activePage.classList.remove('is-visible');
        activePage.classList.remove('active-page');
    }
    
    targetPage.classList.add('active-page');
    setTimeout(() => {
        targetPage.classList.add('is-visible');
    }, 50);

    // If modifications page (simulator) is opened, resize canvas & reset drone
    if (targetId === 'model-shortcommings') {
        setTimeout(() => {
            resizeCanvas();
            resetDrone();
        }, 80);
    }

    // Close mobile nav drawer if open
    const navBar = document.getElementById('nav-bar');
    if (navBar) {
        navBar.classList.remove('open-nav');
    }

    // Smooth scroll to top of viewport
    window.scrollTo({ top: 0, behavior: 'smooth' });
}

// Mobile navigation drawer toggle
const menuToggle = document.getElementById('menu-toggle');
const navBar = document.getElementById('nav-bar');
if (menuToggle && navBar) {
    menuToggle.addEventListener('click', () => {
        navBar.classList.toggle('open-nav');
    });
}

// Mobile dropdown expand on click
const dropdownElements = document.querySelectorAll('.has-dropdown');
dropdownElements.forEach(d => {
    const link = d.querySelector('.nav-link');
    link.addEventListener('click', (e) => {
        if (window.innerWidth <= 1024) {
            e.preventDefault(); // Stop hash nav on mobile trigger click
            d.classList.toggle('mobile-dropdown-open');
        }
    });
});

// Setup scroll-triggered entrance observer
const pageSectionObserver = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
        if (entry.isIntersecting) {
            entry.target.classList.add('is-visible');
        }
    });
}, { threshold: 0.05 });

// Deep link routing on page load
window.addEventListener('load', () => {
    document.querySelectorAll('.page-section').forEach(section => {
        pageSectionObserver.observe(section);
    });

    const hash = window.location.hash.substring(1);
    if (hash && document.getElementById(hash)) {
        navigateToPage(hash);
    } else {
        navigateToPage('home');
    }
});


// ==========================================
// 2. Figure Replication Carousel Slider
// ==========================================

let currentSlide = 0;
const slides = document.querySelectorAll('.carousel-slide');
const indicators = document.querySelectorAll('.carousel-indicators .indicator');

function showSlide(index) {
    if (slides.length === 0) return;
    if (index >= slides.length) {
        currentSlide = 0;
    } else if (index < 0) {
        currentSlide = slides.length - 1;
    } else {
        currentSlide = index;
    }

    slides.forEach((slide, i) => {
        slide.classList.toggle('active-slide', i === currentSlide);
    });

    indicators.forEach((ind, i) => {
        ind.classList.toggle('active', i === currentSlide);
    });
}

function moveSlide(step) {
    showSlide(currentSlide + step);
}

function setSlide(index) {
    showSlide(index);
}


// ==========================================
// 3. Flight Simulation Telemetry & State
// ==========================================

const canvas = document.getElementById('simulation-canvas');
const ctx = canvas ? canvas.getContext('2d') : null;
const telemetryReadout = document.getElementById('telemetry-readout');

// Telemetry parameters
let activeScene = 'single'; // 'single' (1 obstacle) or 'three' (3 obstacles)
let activeController = 'itae'; // 'itae' (well-damped) or 'ise' (overshoot)
let activeThreshold = 1.0; // 1.0m (safe) or 0.5m (crash)

// Physics variables
let windForceX = 0.0;
let controlLag = 0.05; // feedback delay in seconds
let isPlaying = true;
let isCrashed = false;
let animationId = null;

const dt = 0.05; // integration time step (s)
const droneRadius = 10;
const targetRadius = 15;
const obstacleRadius = 25;

// Initial state values
let drone = {
    x: 80,
    y: 400,
    vx: 0,
    vy: 0,
    ax: 0,
    ay: 0,
    trail: []
};

let target = {
    x: 680,
    y: 80
};

let obstacles = [];

const singleSceneObstacles = [
    { x: 380, y: 240, r: obstacleRadius } // Central obstacle
];

const threeSceneObstacles = [
    { x: 300, y: 280, r: obstacleRadius }, // Obstacle 1
    { x: 440, y: 220, r: obstacleRadius }, // Obstacle 2
    { x: 560, y: 140, r: obstacleRadius }  // Obstacle 3
];

let commandQueue = [];

// ==========================================
// 4. Simulator UI Controls Sync
// ==========================================

const slideWind = document.getElementById('slide-wind');
const slideLag = document.getElementById('slide-lag');
const valWind = document.getElementById('val-wind');
const valLag = document.getElementById('val-lag');

const btnPlayPause = document.getElementById('btn-play-pause');
const btnResetSim = document.getElementById('btn-reset-sim');

function syncSliders() {
    if (slideWind) {
        windForceX = parseFloat(slideWind.value);
        valWind.textContent = windForceX > 0 ? `+${windForceX.toFixed(2)} (Right)` : windForceX < 0 ? `${windForceX.toFixed(2)} (Left)` : "0.00";
    }
    if (slideLag) {
        controlLag = parseFloat(slideLag.value);
        valLag.textContent = controlLag.toFixed(2) + "s";
    }
}

if (slideWind) slideWind.addEventListener('input', syncSliders);
if (slideLag) slideLag.addEventListener('input', syncSliders);

// Simulator select configurations
function setSimScene(scene) {
    activeScene = scene;
    document.getElementById('scene-single').classList.toggle('active-mode', scene === 'single');
    document.getElementById('scene-three').classList.toggle('active-mode', scene === 'three');
    resetDrone();
}

function setSimController(controller) {
    activeController = controller;
    document.getElementById('ctrl-itae').classList.toggle('active-mode', controller === 'itae');
    document.getElementById('ctrl-ise').classList.toggle('active-mode', controller === 'ise');
    resetDrone();
}

function setSimThreshold(threshold) {
    activeThreshold = threshold;
    document.getElementById('thresh-10').classList.toggle('active-mode', threshold === 1.0);
    document.getElementById('thresh-05').classList.toggle('active-mode', threshold === 0.5);
    resetDrone();
}

if (btnPlayPause) {
    btnPlayPause.addEventListener('click', () => {
        isPlaying = !isPlaying;
        btnPlayPause.textContent = isPlaying ? "Pause Flight" : "Resume Flight";
        btnPlayPause.classList.toggle('btn-primary', isPlaying);
        btnPlayPause.classList.toggle('btn-secondary', !isPlaying);
    });
}

if (btnResetSim) {
    btnResetSim.addEventListener('click', resetDrone);
}

// Resize canvas relative to layout width
function resizeCanvas() {
    if (!canvas) return;
    const rect = canvas.parentNode.getBoundingClientRect();
    canvas.width = rect.width;
    canvas.height = 480;
    
    // Reposition target relative to width
    target.x = canvas.width - 80;
    target.y = 80;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();


// ==========================================
// 5. Numerical Physics Engine (Euler Integration)
// ==========================================

function resetDrone() {
    drone.x = 80;
    drone.y = 400;
    drone.vx = 0;
    drone.vy = 0;
    drone.ax = 0;
    drone.ay = 0;
    drone.trail = [];
    commandQueue = [];
    isCrashed = false;
    
    // Load appropriate obstacles
    obstacles = activeScene === 'single' ? [...singleSceneObstacles] : [...threeSceneObstacles];
}

function computeForces(x, y) {
    // Controller behavior parameter scaling
    // ITAE has higher damping and accurate tracking
    // ISE has low damping and high overshoot (oscillatory behavior)
    const damping = activeController === 'itae' ? 0.20 : 0.06;
    const k_att = activeController === 'itae' ? 0.06 : 0.12; 

    // Target tracking vector
    let f_att_x = -k_att * (x - target.x);
    let f_att_y = -k_att * (y - target.y);

    const att_mag = Math.hypot(f_att_x, f_att_y);
    const max_att = 5.0;
    if (att_mag > max_att) {
        f_att_x = (f_att_x / att_mag) * max_att;
        f_att_y = (f_att_y / att_mag) * max_att;
    }

    let f_rep_x = 0;
    let f_rep_y = 0;
    let in_collision = false;

    // Safety threshold limits (1.0m vs 0.5m mapped to visual pixels)
    const d_0 = activeThreshold === 1.0 ? 115.0 : 45.0;
    const k_rep = activeThreshold === 1.0 ? 18.0 : 4.0;

    obstacles.forEach(obs => {
        const dx = x - obs.x;
        const dy = y - obs.y;
        const dist = Math.hypot(dx, dy);

        // Physical collision check (drone boundaries + obstacle boundary)
        if (dist <= (droneRadius + obs.r)) {
            in_collision = true;
        }

        // Repulsion field activation
        if (dist < d_0) {
            const d = Math.max(dist, 5.0);
            const rep_mag = k_rep * (1.0 / d - 1.0 / d_0) * (1.0 / (d * d)) * 1000;
            
            let force_x = rep_mag * (dx / d);
            let force_y = rep_mag * (dy / d);

            // Perpendicular steering angle alignment
            const dotProduct = (dx / d) * (f_att_x / max_att) + (dy / d) * (f_att_y / max_att);
            if (dotProduct < -0.7) {
                force_x += -force_y * 0.5;
                force_y += force_x * 0.5;
            }

            f_rep_x += force_x;
            f_rep_y += force_y;
        }
    });

    return {
        attX: f_att_x,
        attY: f_att_y,
        repX: f_rep_x,
        repY: f_rep_y,
        totalX: f_att_x + f_rep_x,
        totalY: f_att_y + f_rep_y,
        inCollision: in_collision,
        damping: damping
    };
}

function updatePhysics() {
    if (!isPlaying || isCrashed || !canvas) return;

    const forces = computeForces(drone.x, drone.y);

    if (forces.inCollision) {
        isCrashed = true;
        drone.vx = 0;
        drone.vy = 0;
        return;
    }

    // Process sensory delay latency queue
    const queueLimit = Math.max(Math.round(controlLag / dt), 1);
    commandQueue.push({ ax: forces.totalX, ay: forces.totalY });

    let appliedAccel = { ax: forces.totalX, ay: forces.totalY };
    if (commandQueue.length >= queueLimit) {
        appliedAccel = commandQueue.shift();
    }

    // Euler ODE integration
    const ax = appliedAccel.ax + windForceX - forces.damping * drone.vx;
    const ay = appliedAccel.ay - forces.damping * drone.vy;

    drone.vx += ax * dt;
    drone.vy += ay * dt;

    // Terminal velocity clamp
    const speed = Math.hypot(drone.vx, drone.vy);
    const maxSpeed = 12.0;
    if (speed > maxSpeed) {
        drone.vx = (drone.vx / speed) * maxSpeed;
        drone.vy = (drone.vy / speed) * maxSpeed;
    }

    drone.x += drone.vx * dt * 20;
    drone.y += drone.vy * dt * 20;

    // Boundary constraints check
    if (drone.x < 0 || drone.x > canvas.width || drone.y < 0 || drone.y > canvas.height) {
        resetDrone();
    }

    // Trajectory trail
    drone.trail.push({ x: drone.x, y: drone.y });
    if (drone.trail.length > 500) {
        drone.trail.shift();
    }

    // Check if goal reached
    const distToTarget = Math.hypot(drone.x - target.x, drone.y - target.y);
    if (distToTarget < targetRadius + 4) {
        drone.vx = 0;
        drone.vy = 0;
    }

    // Readout telemetry printout
    if (telemetryReadout) {
        const displayX = ((drone.x - 80) / 40).toFixed(2);
        const displayY = ((400 - drone.y) / 40).toFixed(2);
        const displaySpeed = (speed * 1.5).toFixed(2);
        telemetryReadout.innerHTML = `X: ${displayX}m | Y: ${displayY}m | Speed: ${displaySpeed}m/s`;
    }
}

// ==========================================
// 6. Simulation Rendering Engine
// ==========================================

function drawSimulation() {
    if (!ctx || !canvas) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Telemetry Radar Gridlines
    ctx.strokeStyle = 'rgba(0, 138, 252, 0.04)';
    ctx.lineWidth = 1;
    const gridSize = 40;
    for (let x = 0; x < canvas.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
    }
    for (let y = 0; y < canvas.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
    }

    // 2. Obstacles & Warning zones
    const d_0 = activeThreshold === 1.0 ? 115.0 : 45.0;
    obstacles.forEach(obs => {
        // Warning sensor boundary
        ctx.strokeStyle = 'rgba(253, 98, 98, 0.1)';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, d_0, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        // Radial obstacle body
        const grad = ctx.createRadialGradient(obs.x - 5, obs.y - 5, 2, obs.x, obs.y, obs.r);
        grad.addColorStop(0, '#ff9999');
        grad.addColorStop(0.7, '#FD6262');
        grad.addColorStop(1, '#8B0000');

        ctx.fillStyle = grad;
        ctx.shadowColor = 'rgba(253, 98, 98, 0.3)';
        ctx.shadowBlur = 10;
        ctx.beginPath();
        ctx.arc(obs.x, obs.y, obs.r, 0, Math.PI * 2);
        ctx.fill();
        ctx.shadowBlur = 0;
    });

    // 3. Goal Vector Target Crosshair
    ctx.strokeStyle = '#00ff66';
    ctx.lineWidth = 2;
    ctx.shadowColor = 'rgba(0, 255, 102, 0.4)';
    ctx.shadowBlur = 12;
    ctx.beginPath();
    ctx.arc(target.x, target.y, targetRadius, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = 'rgba(0, 255, 102, 0.12)';
    ctx.beginPath();
    ctx.arc(target.x, target.y, targetRadius - 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;

    // Crosshair ticks
    ctx.strokeStyle = 'rgba(0, 255, 102, 0.5)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(target.x - 22, target.y);
    ctx.lineTo(target.x + 22, target.y);
    ctx.moveTo(target.x, target.y - 22);
    ctx.lineTo(target.x, target.y + 22);
    ctx.stroke();

    // 4. Past flight trajectory line
    if (drone.trail.length > 1) {
        ctx.strokeStyle = isCrashed ? 'rgba(253, 98, 98, 0.35)' : 'rgba(0, 138, 252, 0.4)';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(drone.trail[0].x, drone.trail[0].y);
        for (let i = 1; i < drone.trail.length; i++) {
            ctx.lineTo(drone.trail[i].x, drone.trail[i].y);
        }
        ctx.stroke();
    }

    // 5. Force Vectors
    const forces = computeForces(drone.x, drone.y);
    if (!isCrashed) {
        // Target seeking vector (Green)
        drawVector(drone.x, drone.y, forces.attX * 25, forces.attY * 25, '#00ff66', 1.5);
        // Obstacle avoidance feedback vector (Red)
        if (Math.hypot(forces.repX, forces.repY) > 0.05) {
            drawVector(drone.x, drone.y, forces.repX * 15, forces.repY * 15, '#FD6262', 1.5);
        }
    }

    // 6. Quadcopter Body
    ctx.fillStyle = isCrashed ? '#FD6262' : '#00f0ff';
    ctx.shadowColor = isCrashed ? 'rgba(253, 98, 98, 0.9)' : 'rgba(0, 240, 255, 0.8)';
    ctx.shadowBlur = isCrashed ? 25 : 12;
    ctx.beginPath();
    ctx.arc(drone.x, drone.y, droneRadius, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;

    // Dynamic spinning rotors
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
    ctx.lineWidth = 1;
    const rotorOffset = 14;
    ctx.beginPath();
    ctx.arc(drone.x - rotorOffset, drone.y - rotorOffset, 7, 0, Math.PI * 2);
    ctx.arc(drone.x + rotorOffset, drone.y - rotorOffset, 7, 0, Math.PI * 2);
    ctx.arc(drone.x - rotorOffset, drone.y + rotorOffset, 7, 0, Math.PI * 2);
    ctx.arc(drone.x + rotorOffset, drone.y + rotorOffset, 7, 0, Math.PI * 2);
    ctx.stroke();

    // Collision warning tag
    if (isCrashed) {
        ctx.fillStyle = 'rgba(253, 98, 98, 0.15)';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        ctx.fillStyle = '#FD6262';
        ctx.font = 'bold 20px Space Grotesk';
        ctx.shadowColor = 'rgba(253, 98, 98, 0.5)';
        ctx.shadowBlur = 10;
        ctx.fillText("COLLISION DETECTED", canvas.width / 2 - 120, canvas.height / 2);
        ctx.shadowBlur = 0;
    }
}

function drawVector(startX, startY, vx, vy, color, width) {
    if (Math.hypot(vx, vy) < 2) return;
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = width;

    const endX = startX + vx;
    const endY = startY + vy;

    ctx.beginPath();
    ctx.moveTo(startX, startY);
    ctx.lineTo(endX, endY);
    ctx.stroke();

    const angle = Math.atan2(vy, vx);
    ctx.beginPath();
    ctx.moveTo(endX, endY);
    ctx.lineTo(endX - 8 * Math.cos(angle - Math.PI / 6), endY - 8 * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(endX - 8 * Math.cos(angle + Math.PI / 6), endY - 8 * Math.sin(angle + Math.PI / 6));
    ctx.fill();
}

function simLoop() {
    updatePhysics();
    drawSimulation();
    animationId = requestAnimationFrame(simLoop);
}

// Start flight telemetry
resetDrone();
syncSliders();
if (canvas) {
    simLoop();
}
