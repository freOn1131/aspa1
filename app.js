// ELRS Antenna Tracker Real-time Monitoring Application

class AntennaTracker {
    constructor() {
        this.isRunning = false;
        this.currentMode = 'auto';
        this.startTime = Date.now();
        this.simulationTime = 0;
        
        // System state
        this.state = {
            drone: {
                azimuth: 180,
                elevation: 45,
                distance: 500,
                speed: 15
            },
            antenna: {
                azimuth: 175,
                elevation: 43,
                targetAzimuth: 180,
                targetElevation: 45
            },
            telemetry: {
                rssi1: -65,
                rssi2: -67,
                linkQuality: 95,
                lastUpdate: Date.now()
            }
        };
        
        // Trajectory parameters
        this.trajectory = {
            type: 'circle',
            radius: 45,
            centerAzimuth: 180,
            centerElevation: 45,
            angularSpeed: 10, // degrees per second
            elevationVariation: 20
        };
        
        // RSSI history for chart
        this.rssiHistory = {
            time: [],
            rssi1: [],
            rssi2: []
        };
        
        // Canvas contexts
        this.azimuthCanvas = document.getElementById('azimuthCanvas');
        this.azimuthCtx = this.azimuthCanvas.getContext('2d');
        this.elevationCanvas = document.getElementById('elevationCanvas');
        this.elevationCtx = this.elevationCanvas.getContext('2d');
        
        // Chart.js instance
        this.rssiChart = null;
        
        this.initializeChart();
        this.setupEventListeners();
        this.startSimulation();
    }
    
    initializeChart() {
        const ctx = document.getElementById('rssiChart').getContext('2d');
        
        this.rssiChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'RSSI1',
                    data: [],
                    borderColor: '#00bfff',
                    backgroundColor: 'rgba(0, 191, 255, 0.1)',
                    borderWidth: 2,
                    fill: false,
                    tension: 0.4
                }, {
                    label: 'RSSI2',
                    data: [],
                    borderColor: '#ff5459',
                    backgroundColor: 'rgba(255, 84, 89, 0.1)',
                    borderWidth: 2,
                    fill: false,
                    tension: 0.4
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                plugins: {
                    legend: {
                        labels: {
                            color: '#e0e0e0',
                            font: {
                                size: 12
                            }
                        }
                    }
                },
                scales: {
                    x: {
                        ticks: {
                            color: '#e0e0e0',
                            maxTicksLimit: 10
                        },
                        grid: {
                            color: 'rgba(224, 224, 224, 0.1)'
                        }
                    },
                    y: {
                        min: -100,
                        max: -40,
                        ticks: {
                            color: '#e0e0e0',
                            callback: function(value) {
                                return value + ' dBm';
                            }
                        },
                        grid: {
                            color: 'rgba(224, 224, 224, 0.1)'
                        }
                    }
                },
                animation: {
                    duration: 0
                }
            }
        });
    }
    
    setupEventListeners() {
        // Control buttons
        document.getElementById('startBtn').addEventListener('click', () => this.start());
        document.getElementById('stopBtn').addEventListener('click', () => this.stop());
        document.getElementById('resetBtn').addEventListener('click', () => this.reset());
        
        // Mode selector
        document.getElementById('modeSelect').addEventListener('change', (e) => {
            this.setMode(e.target.value);
        });
        
        // Manual controls
        document.getElementById('setAngleBtn').addEventListener('click', () => {
            if (this.currentMode === 'manual') {
                const azimuth = parseInt(document.getElementById('manualAzimuth').value);
                const elevation = parseInt(document.getElementById('manualElevation').value);
                this.setManualAngles(azimuth, elevation);
            }
        });
    }
    
    start() {
        this.isRunning = true;
        this.updateStatus();
        this.updateMode();
    }
    
    stop() {
        this.isRunning = false;
        this.updateStatus();
        this.updateMode();
    }
    
    reset() {
        this.simulationTime = 0;
        this.startTime = Date.now();
        this.state.drone = {
            azimuth: 180,
            elevation: 45,
            distance: 500,
            speed: 15
        };
        this.state.antenna = {
            azimuth: 175,
            elevation: 43,
            targetAzimuth: 180,
            targetElevation: 45
        };
        this.rssiHistory = {
            time: [],
            rssi1: [],
            rssi2: []
        };
        this.rssiChart.data.labels = [];
        this.rssiChart.data.datasets[0].data = [];
        this.rssiChart.data.datasets[1].data = [];
        this.rssiChart.update();
    }
    
    setMode(mode) {
        this.currentMode = mode;
        const manualControls = document.getElementById('manualControls');
        
        if (mode === 'manual') {
            manualControls.style.display = 'flex';
        } else {
            manualControls.style.display = 'none';
        }
        
        this.updateMode();
    }
    
    setManualAngles(azimuth, elevation) {
        this.state.antenna.targetAzimuth = azimuth;
        this.state.antenna.targetElevation = elevation;
    }
    
    updateMode() {
        let modeText = '';
        switch (this.currentMode) {
            case 'auto':
                modeText = this.isRunning ? 'СЛЕЖЕНИЕ' : 'ОСТАНОВЛЕНО';
                break;
            case 'manual':
                modeText = this.isRunning ? 'РУЧНОЙ' : 'ОСТАНОВЛЕНО';
                break;
            case 'search':
                modeText = this.isRunning ? 'ПОИСК' : 'ОСТАНОВЛЕНО';
                break;
        }
        document.getElementById('currentMode').textContent = modeText;
    }
    
    updateStatus() {
        const indicator = document.getElementById('statusIndicator');
        const statusText = document.getElementById('statusText');
        
        if (this.isRunning) {
            indicator.className = 'status-indicator connected';
            statusText.textContent = 'Подключено';
        } else {
            indicator.className = 'status-indicator disconnected';
            statusText.textContent = 'Остановлено';
        }
    }
    
    simulateDroneMovement() {
        if (!this.isRunning) return;
        
        // Circular trajectory with elevation variation
        const t = this.simulationTime;
        const angularPosition = (t * this.trajectory.angularSpeed) % 360;
        
        this.state.drone.azimuth = (this.trajectory.centerAzimuth + 
            this.trajectory.radius * Math.cos(angularPosition * Math.PI / 180)) % 360;
        
        // Add elevation variation
        const elevationOffset = this.trajectory.elevationVariation * 
            Math.sin(angularPosition * Math.PI / 90) * 0.3;
        this.state.drone.elevation = Math.max(10, 
            Math.min(80, this.trajectory.centerElevation + elevationOffset));
        
        // Vary distance slightly
        this.state.drone.distance = 500 + 100 * Math.sin(angularPosition * Math.PI / 180);
        
        if (this.state.drone.azimuth < 0) {
            this.state.drone.azimuth += 360;
        }
    }
    
    simulateAntennaTracking() {
        if (!this.isRunning) return;
        
        if (this.currentMode === 'auto') {
            // Auto tracking mode - follow the drone with some delay
            this.state.antenna.targetAzimuth = this.state.drone.azimuth;
            this.state.antenna.targetElevation = this.state.drone.elevation;
        } else if (this.currentMode === 'search') {
            // Search mode - slowly pan around
            const searchSpeed = 5; // degrees per second
            this.state.antenna.targetAzimuth = (this.simulationTime * searchSpeed) % 360;
            this.state.antenna.targetElevation = 30 + 20 * Math.sin(this.simulationTime * Math.PI / 10);
        }
        // Manual mode uses manually set targets
        
        // Simulate antenna movement with some inertia
        const trackingSpeed = 0.1; // How fast antenna tracks target
        
        let azimuthDiff = this.state.antenna.targetAzimuth - this.state.antenna.azimuth;
        if (azimuthDiff > 180) azimuthDiff -= 360;
        if (azimuthDiff < -180) azimuthDiff += 360;
        
        this.state.antenna.azimuth += azimuthDiff * trackingSpeed;
        if (this.state.antenna.azimuth < 0) this.state.antenna.azimuth += 360;
        if (this.state.antenna.azimuth >= 360) this.state.antenna.azimuth -= 360;
        
        const elevationDiff = this.state.antenna.targetElevation - this.state.antenna.elevation;
        this.state.antenna.elevation += elevationDiff * trackingSpeed;
        this.state.antenna.elevation = Math.max(0, Math.min(90, this.state.antenna.elevation));
    }
    
    simulateTelemetry() {
        if (!this.isRunning) return;
        
        // Calculate antenna pointing error
        let azimuthError = Math.abs(this.state.antenna.azimuth - this.state.drone.azimuth);
        if (azimuthError > 180) azimuthError = 360 - azimuthError;
        
        const elevationError = Math.abs(this.state.antenna.elevation - this.state.drone.elevation);
        const totalError = Math.sqrt(azimuthError * azimuthError + elevationError * elevationError);
        
        // RSSI decreases with pointing error and distance
        const baseRSSI1 = -50;
        const baseRSSI2 = -52;
        
        const errorPenalty = totalError * 0.5; // 0.5 dBm per degree of error
        const distancePenalty = (this.state.drone.distance - 400) * 0.02; // Distance effect
        
        this.state.telemetry.rssi1 = baseRSSI1 - errorPenalty - distancePenalty + 
            (Math.random() - 0.5) * 4; // Add some noise
        this.state.telemetry.rssi2 = baseRSSI2 - errorPenalty - distancePenalty + 
            (Math.random() - 0.5) * 4;
        
        // Link quality based on RSSI
        const avgRSSI = (this.state.telemetry.rssi1 + this.state.telemetry.rssi2) / 2;
        this.state.telemetry.linkQuality = Math.max(0, Math.min(100, 
            100 + (avgRSSI + 60) * 2)); // Scale RSSI to 0-100%
        
        this.state.telemetry.lastUpdate = Date.now();
    }
    
    updateRSSIHistory() {
        const now = new Date();
        const timeStr = now.toLocaleTimeString();
        
        this.rssiHistory.time.push(timeStr);
        this.rssiHistory.rssi1.push(this.state.telemetry.rssi1);
        this.rssiHistory.rssi2.push(this.state.telemetry.rssi2);
        
        // Keep only last 60 data points (1 minute at 1 second intervals)
        const maxPoints = 60;
        if (this.rssiHistory.time.length > maxPoints) {
            this.rssiHistory.time.shift();
            this.rssiHistory.rssi1.shift();
            this.rssiHistory.rssi2.shift();
        }
        
        // Update chart
        this.rssiChart.data.labels = this.rssiHistory.time;
        this.rssiChart.data.datasets[0].data = this.rssiHistory.rssi1;
        this.rssiChart.data.datasets[1].data = this.rssiHistory.rssi2;
        this.rssiChart.update('none');
    }
    
    drawAzimuthView() {
        const ctx = this.azimuthCtx;
        const canvas = this.azimuthCanvas;
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const radius = Math.min(centerX, centerY) - 20;
        
        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw compass circle
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
        ctx.stroke();
        
        // Draw compass grid lines
        ctx.strokeStyle = '#222';
        ctx.lineWidth = 1;
        for (let i = 0; i < 360; i += 30) {
            const angle = (i - 90) * Math.PI / 180;
            const x1 = centerX + Math.cos(angle) * (radius - 10);
            const y1 = centerY + Math.sin(angle) * (radius - 10);
            const x2 = centerX + Math.cos(angle) * radius;
            const y2 = centerY + Math.sin(angle) * radius;
            
            ctx.beginPath();
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
        }
        
        // Draw antenna direction (green arrow)
        const antennaAngle = (this.state.antenna.azimuth - 90) * Math.PI / 180;
        const antennaX = centerX + Math.cos(antennaAngle) * (radius * 0.8);
        const antennaY = centerY + Math.sin(antennaAngle) * (radius * 0.8);
        
        ctx.strokeStyle = '#00ff00';
        ctx.fillStyle = '#00ff00';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(antennaX, antennaY);
        ctx.stroke();
        
        // Arrow head for antenna
        const headLength = 15;
        const headAngle = Math.PI / 6;
        ctx.beginPath();
        ctx.moveTo(antennaX, antennaY);
        ctx.lineTo(
            antennaX - headLength * Math.cos(antennaAngle - headAngle),
            antennaY - headLength * Math.sin(antennaAngle - headAngle)
        );
        ctx.moveTo(antennaX, antennaY);
        ctx.lineTo(
            antennaX - headLength * Math.cos(antennaAngle + headAngle),
            antennaY - headLength * Math.sin(antennaAngle + headAngle)
        );
        ctx.stroke();
        
        // Draw antenna coverage sector (semi-transparent cone)
        const sectorAngle = 30 * Math.PI / 180; // 30 degree sector
        ctx.fillStyle = 'rgba(0, 255, 0, 0.1)';
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.arc(centerX, centerY, radius * 0.9, 
            antennaAngle - sectorAngle / 2, 
            antennaAngle + sectorAngle / 2);
        ctx.closePath();
        ctx.fill();
        
        // Draw drone position (blue dot)
        const droneAngle = (this.state.drone.azimuth - 90) * Math.PI / 180;
        const droneDistance = (this.state.drone.distance / 600) * radius * 0.7; // Scale distance
        const droneX = centerX + Math.cos(droneAngle) * droneDistance;
        const droneY = centerY + Math.sin(droneAngle) * droneDistance;
        
        ctx.fillStyle = '#00bfff';
        ctx.beginPath();
        ctx.arc(droneX, droneY, 6, 0, 2 * Math.PI);
        ctx.fill();
        
        // Draw drone trail
        ctx.strokeStyle = 'rgba(0, 191, 255, 0.3)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(centerX, centerY, droneDistance, 0, 2 * Math.PI);
        ctx.stroke();
        
        // Draw center point
        ctx.fillStyle = '#666';
        ctx.beginPath();
        ctx.arc(centerX, centerY, 3, 0, 2 * Math.PI);
        ctx.fill();
    }
    
    drawElevationView() {
        const ctx = this.elevationCtx;
        const canvas = this.elevationCanvas;
        const width = canvas.width;
        const height = canvas.height;
        
        // Clear canvas
        ctx.clearRect(0, 0, width, height);
        
        // Draw elevation scale
        ctx.strokeStyle = '#333';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(20, 10);
        ctx.lineTo(20, height - 10);
        ctx.stroke();
        
        // Draw scale marks
        ctx.strokeStyle = '#222';
        ctx.lineWidth = 1;
        for (let i = 0; i <= 90; i += 22.5) {
            const y = height - 10 - (i / 90) * (height - 20);
            ctx.beginPath();
            ctx.moveTo(15, y);
            ctx.lineTo(25, y);
            ctx.stroke();
        }
        
        // Draw antenna elevation (green line)
        const antennaY = height - 10 - (this.state.antenna.elevation / 90) * (height - 20);
        ctx.strokeStyle = '#00ff00';
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(30, antennaY);
        ctx.lineTo(width - 10, antennaY);
        ctx.stroke();
        
        // Draw drone elevation (blue circle)
        const droneY = height - 10 - (this.state.drone.elevation / 90) * (height - 20);
        ctx.fillStyle = '#00bfff';
        ctx.beginPath();
        ctx.arc(width / 2, droneY, 6, 0, 2 * Math.PI);
        ctx.fill();
    }
    
    updateUI() {
        // Update uptime
        const uptime = Math.floor((Date.now() - this.startTime) / 1000);
        const hours = Math.floor(uptime / 3600).toString().padStart(2, '0');
        const minutes = Math.floor((uptime % 3600) / 60).toString().padStart(2, '0');
        const seconds = (uptime % 60).toString().padStart(2, '0');
        document.getElementById('uptime').textContent = `${hours}:${minutes}:${seconds}`;
        
        // Update antenna angles
        document.getElementById('currentAzimuth').textContent = this.state.antenna.azimuth.toFixed(1) + '°';
        document.getElementById('targetAzimuth').textContent = this.state.antenna.targetAzimuth.toFixed(1) + '°';
        document.getElementById('currentElevation').textContent = this.state.antenna.elevation.toFixed(1) + '°';
        document.getElementById('targetElevation').textContent = this.state.antenna.targetElevation.toFixed(1) + '°';
        
        // Calculate and update tracking error
        let azimuthError = Math.abs(this.state.antenna.azimuth - this.state.antenna.targetAzimuth);
        if (azimuthError > 180) azimuthError = 360 - azimuthError;
        const elevationError = Math.abs(this.state.antenna.elevation - this.state.antenna.targetElevation);
        const totalError = Math.sqrt(azimuthError * azimuthError + elevationError * elevationError);
        document.getElementById('trackingError').textContent = totalError.toFixed(1) + '°';
        
        // Update drone data
        document.getElementById('droneAzimuth').textContent = this.state.drone.azimuth.toFixed(1) + '°';
        document.getElementById('droneElevation').textContent = this.state.drone.elevation.toFixed(1) + '°';
        document.getElementById('droneSpeed').textContent = this.state.drone.speed + ' м/с';
        
        // Update telemetry values
        document.getElementById('rssi1Value').textContent = this.state.telemetry.rssi1.toFixed(0) + ' dBm';
        document.getElementById('rssi2Value').textContent = this.state.telemetry.rssi2.toFixed(0) + ' dBm';
        document.getElementById('linkQuality').textContent = this.state.telemetry.linkQuality.toFixed(0) + '%';
        document.getElementById('distance').textContent = Math.round(this.state.drone.distance) + ' м';
    }
    
    startSimulation() {
        setInterval(() => {
            if (this.isRunning) {
                this.simulationTime += 0.1; // 100ms increments
            }
            
            this.simulateDroneMovement();
            this.simulateAntennaTracking();
            this.simulateTelemetry();
            
            this.drawAzimuthView();
            this.drawElevationView();
            this.updateUI();
        }, 100);
        
        // Update RSSI chart every second
        setInterval(() => {
            if (this.isRunning) {
                this.updateRSSIHistory();
            }
        }, 1000);
    }
}

// Initialize the application when the page loads
document.addEventListener('DOMContentLoaded', () => {
    const tracker = new AntennaTracker();
    
    // Auto-start the system
    tracker.start();
});