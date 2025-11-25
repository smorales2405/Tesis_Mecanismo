// ==========================================
// Control de Mecanismo Oscilante - App.js
// ==========================================

// Configuración
const CONFIG = {
    dataPollingInterval: 200,
    chartMaxPoints: 100
};

// Estado global
let chartsPaused = false;
let chartOy, chartTh4;
let startTime = Date.now();

// ==========================================
// Inicialización
// ==========================================
document.addEventListener('DOMContentLoaded', () => {
    initCharts();
    initEventListeners();
    startDataPolling();
    updateMotorAnimation();
    updateConnectionStatus(true); // Asumimos conectado al cargar
});

// ==========================================
// API - Endpoints HTTP
// ==========================================

// Función genérica para llamar endpoints
async function callEndpoint(endpoint, params = {}) {
    try {
        const queryString = new URLSearchParams(params).toString();
        const url = queryString ? `/${endpoint}?${queryString}` : `/${endpoint}`;
        const response = await fetch(url);
        const data = await response.json();
        console.log(`${endpoint}:`, data);
        return data;
    } catch (error) {
        console.error(`Error en ${endpoint}:`, error);
        updateConnectionStatus(false);
        return { error: error.message };
    }
}

// Motor 1 (Crank) - Velocidad
function setMotorSpeed(rpm) {
    return callEndpoint('motor1/speed', { rpm: rpm });
}

// Motor 2 (Actuador) - Posición fija
function setLinearPosition(cm) {
    return callEndpoint('motor2/position', { cm: cm });
}

// Motor 2 (Actuador) - Velocidad oscilante
function setLinearOscillation(speed, minLimit, maxLimit) {
    return callEndpoint('motor2/oscillate', { 
        speed: speed, 
        min: minLimit, 
        max: maxLimit 
    });
}

// Motor 2 - Detener oscilación
function stopLinearOscillation() {
    return callEndpoint('motor2/oscillate', { speed: 0 });
}

// Detener ambos motores (retorno a home)
function stopMotors() {
    return callEndpoint('stop');
}

// Parada de emergencia
function emergencyStop() {
    return callEndpoint('emergency');
}

// Reset encoders
function resetEncoders() {
    return callEndpoint('reset');
}

// Obtener datos actuales
async function getData() {
    try {
        const response = await fetch('/data');
        const data = await response.json();
        updateConnectionStatus(true);
        return data;
    } catch (error) {
        updateConnectionStatus(false);
        return null;
    }
}

// ==========================================
// Estado de conexión
// ==========================================
function updateConnectionStatus(connected) {
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.status-text');
    
    if (connected) {
        statusDot.classList.add('connected');
        statusText.textContent = 'Conectado';
    } else {
        statusDot.classList.remove('connected');
        statusText.textContent = 'Desconectado';
    }
}

// ==========================================
// Manejo de datos entrantes
// ==========================================
function handleIncomingData(data) {
    if (!data) return;
    
    // Actualizar valores en tiempo real
    if (data.O_y !== undefined) {
        document.getElementById('realtimeOy').textContent = data.O_y.toFixed(2);
        if (!chartsPaused) {
            addDataPoint(data.O_y, chartOy);
        }
    }
    
    if (data.th4 !== undefined) {
        document.getElementById('realtimeTh4').textContent = data.th4.toFixed(2);
        if (!chartsPaused) {
            addDataPoint(data.th4, chartTh4);
        }
    }
    
    // Actualizar posición del actuador
    if (data.position !== undefined) {
        updateLinearSlider(data.position);
        document.getElementById('currentPosition').textContent = data.position.toFixed(1) + ' cm';
    }
    
    // Actualizar estado
    if (data.state !== undefined) {
        document.getElementById('actuatorState').textContent = data.state;
    }
}

// ==========================================
// Polling de datos
// ==========================================
function startDataPolling() {
    setInterval(async () => {
        const data = await getData();
        handleIncomingData(data);
    }, CONFIG.dataPollingInterval);
}

// ==========================================
// Gráficas
// ==========================================
function initCharts() {
    const commonOptions = {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 0 },
        layout: {
            padding: { bottom: 5 }
        },
        scales: {
            x: {
                display: true,
                title: { display: false },
                ticks: { maxTicksLimit: 5, font: { size: 12 } }
            },
            y: {
                display: true,
                ticks: { font: { size: 12 } }
            }
        },
        plugins: {
            legend: { display: false }
        },
        elements: {
            point: { radius: 0 },
            line: { tension: 0.2, borderWidth: 2 }
        }
    };
    
    // Gráfica O_y
    const ctxOy = document.getElementById('chartOy').getContext('2d');
    chartOy = new Chart(ctxOy, {
        type: 'line',
        data: {
            labels: [],
            datasets: [{
                label: 'O_y (cm)',
                data: [],
                borderColor: '#5a67d8',
                backgroundColor: 'rgba(90, 103, 216, 0.1)',
                fill: true
            }]
        },
        options: {
            ...commonOptions,
            scales: {
                ...commonOptions.scales,
                y: {
                    ...commonOptions.scales.y,
                    min: 0,
                    max: 15,
                    title: {
                        display: true,
                        text: 'cm',
                        font: { size: 13, weight: 'bold' }
                    }
                }
            }
        }
    });
    
    // Gráfica th4
    const ctxTh4 = document.getElementById('chartTh4').getContext('2d');
    chartTh4 = new Chart(ctxTh4, {
        type: 'line',
        data: {
            labels: [],
            datasets: [{
                label: 'θ₄ (°)',
                data: [],
                borderColor: '#ed8936',
                backgroundColor: 'rgba(237, 137, 54, 0.1)',
                fill: true
            }]
        },
        options: {
            ...commonOptions,
            scales: {
                ...commonOptions.scales,
                y: {
                    ...commonOptions.scales.y,
                    min: -15,
                    max: 15,
                    title: {
                        display: true,
                        text: 'grados (°)',
                        font: { size: 13, weight: 'bold' }
                    }
                }
            }
        }
    });
}

function addDataPoint(value, chart) {
    const currentTime = ((Date.now() - startTime) / 1000).toFixed(1);
    
    // Añadir etiqueta de tiempo
    if (chart.data.labels.length > CONFIG.chartMaxPoints) {
        chart.data.labels.shift();
    }
    chart.data.labels.push(currentTime + 's');
    
    // Añadir dato
    if (chart.data.datasets[0].data.length > CONFIG.chartMaxPoints) {
        chart.data.datasets[0].data.shift();
    }
    chart.data.datasets[0].data.push(value);
    
    chart.update('none');
}

function clearCharts() {
    chartOy.data.labels = [];
    chartOy.data.datasets[0].data = [];
    chartOy.update();
    
    chartTh4.data.labels = [];
    chartTh4.data.datasets[0].data = [];
    chartTh4.update();
    
    startTime = Date.now();
}

// ==========================================
// Event Listeners
// ==========================================
function initEventListeners() {
    // ---- Motor del Crank (Motor 1) ----
    const crankSpeed = document.getElementById('crankSpeed');
    const crankDirection = document.getElementById('crankDirection');
    
    crankSpeed.addEventListener('input', (e) => {
        document.getElementById('crankSpeedValue').textContent = e.target.value + ' RPM';
        updateMotorAnimation();
    });
    
    crankDirection.addEventListener('change', updateMotorAnimation);
    
    document.getElementById('btnSendCrank').addEventListener('click', async () => {
        const speed = parseFloat(crankSpeed.value);
        const dir = parseInt(crankDirection.value);
        const rpm = speed * dir;
        await setMotorSpeed(rpm);
    });
    
    // ---- Modo del actuador lineal ----
    const linearMode = document.getElementById('linearMode');
    linearMode.addEventListener('change', (e) => {
        const mode = e.target.value;
        document.getElementById('modeIndicator').textContent = mode === 'fixed' ? 'Fijo' : 'Oscilante';
        
        if (mode === 'fixed') {
            document.getElementById('fixedControls').classList.remove('hidden');
            document.getElementById('oscillatingControls').classList.add('hidden');
        } else {
            document.getElementById('fixedControls').classList.add('hidden');
            document.getElementById('oscillatingControls').classList.remove('hidden');
        }
    });
    
    // ---- Controles modo Fijo ----
    const linearPosition = document.getElementById('linearPosition');
    linearPosition.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value).toFixed(1);
        document.getElementById('linearPosValue').textContent = val + ' cm';
        updateLinearSlider(parseFloat(val));
    });
    
    document.getElementById('btnSendPosition').addEventListener('click', async () => {
        const pos = parseFloat(linearPosition.value);
        await setLinearPosition(pos);
        document.getElementById('actuatorState').textContent = 'Moviendo...';
    });
    
    // ---- Controles modo Oscilante ----
    const oscMin = document.getElementById('oscMin');
    const oscMax = document.getElementById('oscMax');
    const oscSpeed = document.getElementById('oscSpeed');
    
    oscMin.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value).toFixed(1);
        document.getElementById('oscMinValue').textContent = val + ' cm';
        if (parseFloat(val) >= parseFloat(oscMax.value)) {
            oscMax.value = (parseFloat(val) + 0.5).toFixed(1);
            document.getElementById('oscMaxValue').textContent = oscMax.value + ' cm';
        }
    });
    
    oscMax.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value).toFixed(1);
        document.getElementById('oscMaxValue').textContent = val + ' cm';
        if (parseFloat(val) <= parseFloat(oscMin.value)) {
            oscMin.value = (parseFloat(val) - 0.5).toFixed(1);
            document.getElementById('oscMinValue').textContent = oscMin.value + ' cm';
        }
    });
    
    oscSpeed.addEventListener('input', (e) => {
        document.getElementById('oscSpeedValue').textContent = e.target.value + ' cm/s';
    });
    
    document.getElementById('btnStartOsc').addEventListener('click', async () => {
        const speed = parseFloat(oscSpeed.value);
        const min = parseFloat(oscMin.value);
        const max = parseFloat(oscMax.value);
        await setLinearOscillation(speed, min, max);
        document.getElementById('actuatorState').textContent = 'Oscilando';
    });
    
    document.getElementById('btnStopOsc').addEventListener('click', async () => {
        await stopLinearOscillation();
        document.getElementById('actuatorState').textContent = 'Detenido';
    });
    
    // ---- Botones generales ----
    document.getElementById('btnEmergency').addEventListener('click', async () => {
        await emergencyStop();
        document.getElementById('actuatorState').textContent = '⚠ EMERGENCIA';
        // Resetear visualización del motor
        document.getElementById('crankSpeed').value = 0;
        document.getElementById('crankSpeedValue').textContent = '0 RPM';
        updateMotorAnimation();
    });
    
    document.getElementById('btnStopMotors').addEventListener('click', async () => {
        await stopMotors();
        document.getElementById('actuatorState').textContent = 'Retornando...';
    });
    
    document.getElementById('btnResetEncoders').addEventListener('click', async () => {
        await resetEncoders();
        document.getElementById('actuatorState').textContent = 'Encoders reseteados';
    });
    
    // ---- Controles de gráficas ----
    document.getElementById('btnClearCharts').addEventListener('click', clearCharts);
    
    document.getElementById('btnPauseCharts').addEventListener('click', (e) => {
        chartsPaused = !chartsPaused;
        e.target.textContent = chartsPaused ? '▶ Reanudar' : '⏸ Pausar';
    });
}

// ==========================================
// Animaciones Visuales
// ==========================================
function updateMotorAnimation() {
    const speed = parseInt(document.getElementById('crankSpeed').value);
    const dir = parseInt(document.getElementById('crankDirection').value);
    const motor = document.getElementById('rotatingMotor');
    
    if (speed === 0) {
        motor.style.animationPlayState = 'paused';
    } else {
        const duration = 60 / speed;
        motor.style.animationDuration = duration + 's';
        motor.style.animationDirection = dir > 0 ? 'normal' : 'reverse';
        motor.style.animationPlayState = 'running';
    }
}

function updateLinearSlider(position) {
    const slider = document.getElementById('linearSlider');
    const track = slider.parentElement;
    const maxPosition = 14.3;
    
    // Calcular posición vertical (bottom)
    const trackHeight = track.offsetHeight - 40;
    const percentage = Math.max(0, Math.min(1, position / maxPosition));
    const bottomPos = 5 + (percentage * trackHeight);
    
    slider.style.bottom = bottomPos + 'px';
}
