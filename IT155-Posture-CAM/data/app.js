/**
 * @file app.js
 * @brief Smart Posture Coach Web Dashboard JavaScript
 * @author Smart Posture Coach Team
 * @date 2025-11-17
 * @updated 2025-11-17 03:36:33 UTC
 * @user hyoono
 */

// API endpoints
const API_BASE = '';
const API_STATUS = `${API_BASE}/api/status`;
const API_POSTURE = `${API_BASE}/api/posture`;
const API_STATS = `${API_BASE}/api/stats`;
const API_PRIVACY = `${API_BASE}/api/privacy`;
const API_BREAK = `${API_BASE}/api/break`;
const API_SNOOZE = `${API_BASE}/api/snooze`;

// Update intervals
const UPDATE_INTERVAL_FAST = 1000;  // 1 second for posture data
const UPDATE_INTERVAL_SLOW = 5000;  // 5 seconds for system status

// State
let privacyMode = false;
let inBreak = false;
let alertsSnoozed = false;

// DOM Elements
const elements = {
    postureScore: document.getElementById('postureScore'),
    scoreProgress: document.getElementById('scoreProgress'),
    scorePercent: document.getElementById('scorePercent'),
    scoreIndicator: document.getElementById('scoreIndicator'),
    distance: document.getElementById('distance'),
    distanceIndicator: document.getElementById('distanceIndicator'),
    fps: document.getElementById('fps'),
    uptime: document.getElementById('uptime'),
    cameraStatus: document.getElementById('cameraStatus'),
    arduinoStatus: document.getElementById('arduinoStatus'),
    goodTime: document.getElementById('goodTime'),
    badTime: document.getElementById('badTime'),
    alertCount: document.getElementById('alertCount'),
    breakTime: document.getElementById('breakTime'),
    firmwareVersion: document.getElementById('firmwareVersion'),
    btnBreak: document.getElementById('btnBreak'),
    btnSnooze: document.getElementById('btnSnooze'),
    btnPrivacy: document.getElementById('btnPrivacy'),
    controlStatus: document.getElementById('controlStatus')
};

// Initialize
document.addEventListener('DOMContentLoaded', () => {
    console.log('Smart Posture Coach Dashboard Initialized');
    
    // Set up button handlers
    elements.btnBreak.addEventListener('click', toggleBreak);
    elements.btnSnooze.addEventListener('click', toggleSnooze);
    elements.btnPrivacy.addEventListener('click', togglePrivacy);
    
    // Start update loops
    updatePostureData();
    updateSystemStatus();
    updateSessionStats();
    
    setInterval(updatePostureData, UPDATE_INTERVAL_FAST);
    setInterval(updateSystemStatus, UPDATE_INTERVAL_SLOW);
    setInterval(updateSessionStats, UPDATE_INTERVAL_SLOW);
});

// Fetch posture data
async function updatePostureData() {
    try {
        const response = await fetch(API_POSTURE);
        const data = await response.json();
        
        // Update score
        const score = data.score || 0;
        elements.postureScore.textContent = score;
        elements.scoreProgress.style.width = `${score}%`;
        elements.scorePercent.textContent = `${score}%`;
        
        // Update score indicator
        if (score >= 80) {
            elements.scoreIndicator.className = 'status-indicator status-good';
        } else if (score >= 60) {
            elements.scoreIndicator.className = 'status-indicator status-warning';
        } else {
            elements.scoreIndicator.className = 'status-indicator status-bad';
        }
        
        // Update distance
        const distance = data.distance || 0;
        elements.distance.textContent = distance > 0 ? distance : '--';
        
        // Update distance indicator
        if (distance >= 50 && distance <= 70) {
            elements.distanceIndicator.className = 'status-indicator status-good';
        } else if (distance > 0) {
            elements.distanceIndicator.className = 'status-indicator status-warning';
        } else {
            elements.distanceIndicator.className = 'status-indicator status-offline';
        }
        
    } catch (error) {
        console.error('Error fetching posture data:', error);
        elements.postureScore.textContent = '--';
        elements.distance.textContent = '--';
    }
}

// Fetch system status
async function updateSystemStatus() {
    try {
        const response = await fetch(API_STATUS);
        const data = await response.json();
        
        // Update firmware version
        elements.firmwareVersion.textContent = data.version || 'Unknown';
        
        // Update FPS
        elements.fps.textContent = (data.fps || 0).toFixed(1);
        
        // Update uptime
        const uptime = data.uptime || 0;
        elements.uptime.textContent = formatUptime(uptime);
        
        // Update status indicators
        elements.cameraStatus.className = data.cameraOK ? 
            'status-indicator status-good' : 'status-indicator status-bad';
        elements.arduinoStatus.className = data.arduinoOK ? 
            'status-indicator status-good' : 'status-indicator status-bad';
        
        // Update button states
        privacyMode = data.privacyMode || false;
        inBreak = data.inBreak || false;
        alertsSnoozed = data.alertsSnoozed || false;
        
        updateButtonStates();
        
    } catch (error) {
        console.error('Error fetching system status:', error);
    }
}

// Fetch session statistics
async function updateSessionStats() {
    try {
        const response = await fetch(API_STATS);
        const data = await response.json();
        
        elements.goodTime.textContent = data.goodPostureTime || 0;
        elements.badTime.textContent = data.badPostureTime || 0;
        elements.alertCount.textContent = data.alertCount || 0;
        elements.breakTime.textContent = data.breakTime || 0;
        
    } catch (error) {
        console.error('Error fetching session stats:', error);
    }
}

// Toggle break mode
async function toggleBreak() {
    try {
        showControlStatus('Processing...', 'info');
        
        const formData = new FormData();
        formData.append('start', (!inBreak).toString());
        
        const response = await fetch(API_BREAK, {
            method: 'POST',
            body: formData
        });
        
        if (response.ok) {
            inBreak = !inBreak;
            updateButtonStates();
            showControlStatus(
                inBreak ? '✅ Break mode started' : '✅ Break mode ended',
                'success'
            );
        } else {
            showControlStatus('❌ Failed to toggle break', 'error');
        }
    } catch (error) {
        console.error('Error toggling break:', error);
        showControlStatus('❌ Connection error', 'error');
    }
}

// Toggle snooze
async function toggleSnooze() {
    try {
        showControlStatus('Processing...', 'info');
        
        const formData = new FormData();
        formData.append('enable', (!alertsSnoozed).toString());
        
        const response = await fetch(API_SNOOZE, {
            method: 'POST',
            body: formData
        });
        
        if (response.ok) {
            alertsSnoozed = !alertsSnoozed;
            updateButtonStates();
            showControlStatus(
                alertsSnoozed ? '🔕 Alerts snoozed' : '🔔 Alerts active',
                'success'
            );
        } else {
            showControlStatus('❌ Failed to toggle snooze', 'error');
        }
    } catch (error) {
        console.error('Error toggling snooze:', error);
        showControlStatus('❌ Connection error', 'error');
    }
}

// Toggle privacy mode
async function togglePrivacy() {
    try {
        showControlStatus('Processing...', 'info');
        
        const formData = new FormData();
        formData.append('enable', (!privacyMode).toString());
        
        const response = await fetch(API_PRIVACY, {
            method: 'POST',
            body: formData
        });
        
        if (response.ok) {
            privacyMode = !privacyMode;
            updateButtonStates();
            showControlStatus(
                privacyMode ? '🔒 Privacy mode enabled' : '👁️ Monitoring active',
                'success'
            );
        } else {
            showControlStatus('❌ Failed to toggle privacy', 'error');
        }
    } catch (error) {
        console.error('Error toggling privacy:', error);
        showControlStatus('❌ Connection error', 'error');
    }
}

// Update button states based on current modes
function updateButtonStates() {
    // Break button
    elements.btnBreak.textContent = inBreak ? 'End Break' : 'Start Break';
    elements.btnBreak.className = inBreak ? 'btn btn-warning' : 'btn btn-success';
    
    // Snooze button
    elements.btnSnooze.textContent = alertsSnoozed ? 'Resume Alerts' : 'Snooze Alerts';
    elements.btnSnooze.className = alertsSnoozed ? 'btn btn-success' : 'btn btn-warning';
    
    // Privacy button
    elements.btnPrivacy.textContent = privacyMode ? 'Disable Privacy' : 'Privacy Mode';
    elements.btnPrivacy.className = privacyMode ? 'btn btn-success' : 'btn btn-danger';
}

// Show control status message
function showControlStatus(message, type) {
    elements.controlStatus.textContent = message;
    elements.controlStatus.className = `alert alert-${type}`;
    elements.controlStatus.style.display = 'block';
    
    setTimeout(() => {
        elements.controlStatus.style.display = 'none';
    }, 3000);
}

// Format uptime (seconds to HH:MM:SS)
function formatUptime(seconds) {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}