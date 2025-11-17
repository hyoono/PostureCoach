/**
 * @file main.cpp
 * @brief Smart Posture & Ergonomics Camera Coach - ESP32-CAM FreeRTOS Firmware
 * @author Smart Posture Coach Team
 * @date 2025-11-17
 * @updated 2025-11-17 09:56:47 UTC
 * @user hyoono
 * 
 * ESP32-CAM - Master Controller with FreeRTOS Multi-tasking
 * 
 * VERSION: 2.1.0-FreeRTOS-Bridge (Final)
 * 
 * FEATURES:
 * - FreeRTOS multi-tasking on dual cores
 * - WiFi AP mode for system control
 * - UDP bridge communication with Wemos D1 (port 8888)
 * - OV2640 camera capture at 8-10 FPS
 * - Web server with REST API
 * - Real-time posture monitoring (simulated)
 * - Session statistics tracking
 * 
 * HARDWARE:
 * - ESP32-CAM (AI-Thinker) on USB programmer
 * - OV2640 Camera Module
 * - Built-in LED (GPIO4) for status
 * 
 * COMMUNICATION:
 * - WiFi AP: PostureCoach-AP (password: posture2025)
 * - IP Address: 192.168.4.1
 * - UDP Port: 8888 (Wemos bridge)
 * - Web Server: Port 80
 * 
 * UPLOAD:
 * 1. Connect GPIO0 to GND
 * 2. Press RESET button
 * 3. pio run -e esp32cam -t upload
 * 4. Disconnect GPIO0 from GND
 * 5. Press RESET button
 * 
 * MONITOR:
 * pio device monitor -e esp32cam -b 115200
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <esp_camera.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <WiFiUdp.h>

// ========== VERSION INFO ==========
#define FIRMWARE_VERSION "2.1.0-FreeRTOS-Bridge"
#define BUILD_DATE "2025-11-17"
#define BUILD_TIME "09:56:47"
#define BUILD_USER "hyoono"

// ========== CAMERA PINS (AI-THINKER ESP32-CAM) ==========
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ========== LED PIN ==========
#define LED_BUILTIN 4

// ========== FREERTOS CONFIGURATION ==========
#define TASK_STACK_SIZE_CAMERA    4096
#define TASK_STACK_SIZE_POSE      8192
#define TASK_STACK_SIZE_ARDUINO   4096
#define TASK_STACK_SIZE_DECISION  4096
#define TASK_STACK_SIZE_HEARTBEAT 2048

#define TASK_PRIORITY_CAMERA      3
#define TASK_PRIORITY_POSE        2
#define TASK_PRIORITY_ARDUINO     2
#define TASK_PRIORITY_DECISION    2
#define TASK_PRIORITY_HEARTBEAT   1

#define FRAME_QUEUE_SIZE     10
#define POSTURE_QUEUE_SIZE   5
#define ARDUINO_TX_QUEUE_SIZE 10
#define ARDUINO_RX_QUEUE_SIZE 10

// ========== WIFI CONFIGURATION ==========
const char* AP_SSID = "PostureCoach-AP";
const char* AP_PASSWORD = "posture2025";

// ========== UDP BRIDGE CONFIGURATION ==========
const uint16_t UDP_PORT = 8888;
const uint16_t UDP_LOCAL_PORT = 8888;

// ========== GLOBAL OBJECTS ==========
AsyncWebServer server(80);
WiFiUDP udpBridge;

// ========== FREERTOS HANDLES ==========
TaskHandle_t taskHandleCamera = NULL;
TaskHandle_t taskHandlePose = NULL;
TaskHandle_t taskHandleArduino = NULL;
TaskHandle_t taskHandleDecision = NULL;
TaskHandle_t taskHandleHeartbeat = NULL;

QueueHandle_t queueFrames = NULL;
QueueHandle_t queuePostureData = NULL;
QueueHandle_t queueArduinoTX = NULL;
QueueHandle_t queueArduinoRX = NULL;

SemaphoreHandle_t mutexSharedData = NULL;
SemaphoreHandle_t mutexUDP = NULL;

// ========== DATA STRUCTURES ==========

struct FrameData {
    camera_fb_t* fb;
    uint32_t timestamp;
};

struct PostureData {
    uint8_t score;
    bool isGood;
    bool userPresent;
    float confidence;
    uint32_t timestamp;
};

struct ArduinoTXMessage {
    char command[64];
    uint32_t timestamp;
};

struct ArduinoRXData {
    char message[128];
    uint32_t timestamp;
    IPAddress senderIP;
    uint16_t senderPort;
};

struct SystemState {
    bool cameraInitialized;
    bool bridgeConnected;
    bool arduinoConnected;
    bool wifiConnected;
    uint32_t uptime;
    uint32_t totalFrames;
    uint32_t droppedFrames;
    float averageFPS;
    uint32_t lastBridgeHeartbeat;
    uint32_t lastArduinoHeartbeat;
} systemState;

struct SharedPostureState {
    uint8_t currentScore;
    bool isGoodPosture;
    int16_t distance;
    bool userPresent;
    uint32_t lastUpdate;
} sharedPosture;

struct SessionStats {
    uint32_t goodPostureTime;
    uint32_t badPostureTime;
    uint32_t breakTime;
    uint16_t alertCount;
    uint8_t averageScore;
} sessionStats;

// ========== MODE FLAGS ==========
bool privacyMode = false;
bool alertsSnoozed = false;
bool inBreak = false;

// ========== FUNCTION PROTOTYPES ==========
bool initCamera();
bool initWiFi();
void initWebServer();
void printSystemInfo();

void taskCameraCapture(void* parameter);
void taskPoseProcessing(void* parameter);
void taskArduinoCommunication(void* parameter);
void taskDecisionEngine(void* parameter);
void taskHeartbeat(void* parameter);

void analyzePosture(camera_fb_t* fb, PostureData* result);
uint8_t calculatePostureScore(PostureData* data);

void sendToArduinoViaBridge(const char* cmd);
void processArduinoMessage(const char* msg);
void sendUDPMessage(const char* message);

void setupAPIRoutes();
void blinkLED(int times, int delayMs);
void printTaskStats();

// ========== SETUP ==========
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    delay(1000);
    Serial.println();
    Serial.println();
    
    printSystemInfo();
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Create mutexes
    mutexSharedData = xSemaphoreCreateMutex();
    mutexUDP = xSemaphoreCreateMutex();
    
    if (mutexSharedData == NULL || mutexUDP == NULL) {
        Serial.println("ERROR: Failed to create mutexes!");
        while (1) {
            blinkLED(5, 100);
            delay(1000);
        }
    }
    
    // Create queues
    queueFrames = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(FrameData));
    queuePostureData = xQueueCreate(POSTURE_QUEUE_SIZE, sizeof(PostureData));
    queueArduinoTX = xQueueCreate(ARDUINO_TX_QUEUE_SIZE, sizeof(ArduinoTXMessage));
    queueArduinoRX = xQueueCreate(ARDUINO_RX_QUEUE_SIZE, sizeof(ArduinoRXData));
    
    if (queueFrames == NULL || queuePostureData == NULL || 
        queueArduinoTX == NULL || queueArduinoRX == NULL) {
        Serial.println("ERROR: Failed to create queues!");
        while (1) {
            blinkLED(5, 100);
            delay(1000);
        }
    }
    
    Serial.println("FreeRTOS queues and mutexes created!");
    
    // Initialize camera
    Serial.println("Initializing camera...");
    if (!initCamera()) {
        Serial.println("ERROR: Camera init failed!");
        while (1) {
            blinkLED(3, 200);
            delay(1000);
        }
    }
    systemState.cameraInitialized = true;
    Serial.println("Camera initialized!");
    
    // Initialize WiFi
    Serial.println("Initializing WiFi...");
    if (!initWiFi()) {
        Serial.println("WARNING: WiFi init failed!");
    }
    
    // Initialize UDP
    Serial.println("Initializing UDP bridge...");
    udpBridge.begin(UDP_LOCAL_PORT);
    Serial.printf("UDP listening on port %d\n", UDP_LOCAL_PORT);
    
    // Initialize web server
    Serial.println("Initializing web server...");
    initWebServer();
    server.begin();
    Serial.println("Web server started!");
    
    // Initialize data structures
    memset(&sharedPosture, 0, sizeof(sharedPosture));
    memset(&sessionStats, 0, sizeof(sessionStats));
    
    // Create FreeRTOS tasks
    Serial.println("\nCreating FreeRTOS tasks...");
    
    xTaskCreatePinnedToCore(taskCameraCapture, "CameraTask", TASK_STACK_SIZE_CAMERA, NULL, TASK_PRIORITY_CAMERA, &taskHandleCamera, 1);
    xTaskCreatePinnedToCore(taskPoseProcessing, "PoseTask", TASK_STACK_SIZE_POSE, NULL, TASK_PRIORITY_POSE, &taskHandlePose, 1);
    xTaskCreatePinnedToCore(taskArduinoCommunication, "ArduinoTask", TASK_STACK_SIZE_ARDUINO, NULL, TASK_PRIORITY_ARDUINO, &taskHandleArduino, 0);
    xTaskCreatePinnedToCore(taskDecisionEngine, "DecisionTask", TASK_STACK_SIZE_DECISION, NULL, TASK_PRIORITY_DECISION, &taskHandleDecision, 1);
    xTaskCreatePinnedToCore(taskHeartbeat, "HeartbeatTask", TASK_STACK_SIZE_HEARTBEAT, NULL, TASK_PRIORITY_HEARTBEAT, &taskHandleHeartbeat, 0);
    
    Serial.println("All tasks created!");
    
    Serial.println("\n=== FreeRTOS Task Configuration ===");
    Serial.printf("Camera Task: Core 1, Priority %d, Stack %d bytes\n", TASK_PRIORITY_CAMERA, TASK_STACK_SIZE_CAMERA);
    Serial.printf("Pose Task: Core 1, Priority %d, Stack %d bytes\n", TASK_PRIORITY_POSE, TASK_STACK_SIZE_POSE);
    Serial.printf("Arduino Task (UDP): Core 0, Priority %d, Stack %d bytes\n", TASK_PRIORITY_ARDUINO, TASK_STACK_SIZE_ARDUINO);
    Serial.printf("Decision Task: Core 1, Priority %d, Stack %d bytes\n", TASK_PRIORITY_DECISION, TASK_STACK_SIZE_DECISION);
    Serial.printf("Heartbeat Task: Core 0, Priority %d, Stack %d bytes\n", TASK_PRIORITY_HEARTBEAT, TASK_STACK_SIZE_HEARTBEAT);
    
    blinkLED(5, 100);
    
    Serial.println("\n╔═════════════════════════════════════════╗");
    Serial.println("║         SYSTEM READY!                   ║");
    Serial.println("╚═════════════════════════════════════════╝");
    Serial.println("Communication: WiFi UDP via Wemos Bridge");
    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("=========================================\n");
}

// ========== MAIN LOOP ==========
void loop() {
    static uint32_t lastStatsTime = 0;
    uint32_t currentTime = millis();
    
    if (currentTime - lastStatsTime >= 30000) {
        printTaskStats();
        lastStatsTime = currentTime;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ========== FREERTOS TASK IMPLEMENTATIONS ==========

void taskCameraCapture(void* parameter) {
    Serial.println("[CameraTask] Started");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
            bool skip = privacyMode || inBreak;
            xSemaphoreGive(mutexSharedData);
            
            if (skip) {
                continue;
            }
        }
        
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb == NULL) {
            systemState.droppedFrames++;
            continue;
        }
        
        FrameData frameData;
        frameData.fb = fb;
        frameData.timestamp = millis();
        
        if (xQueueSend(queueFrames, &frameData, 0) != pdTRUE) {
            esp_camera_fb_return(fb);
            systemState.droppedFrames++;
        } else {
            systemState.totalFrames++;
        }
    }
}

void taskPoseProcessing(void* parameter) {
    Serial.println("[PoseTask] Started");
    
    FrameData frameData;
    PostureData postureResult;
    
    while (1) {
        if (xQueueReceive(queueFrames, &frameData, portMAX_DELAY) == pdTRUE) {
            analyzePosture(frameData.fb, &postureResult);
            esp_camera_fb_return(frameData.fb);
            
            if (xQueueSend(queuePostureData, &postureResult, 0) != pdTRUE) {
                // Queue full
            }
            
            if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
                sharedPosture.currentScore = postureResult.score;
                sharedPosture.isGoodPosture = postureResult.isGood;
                sharedPosture.userPresent = postureResult.userPresent;
                sharedPosture.lastUpdate = millis();
                xSemaphoreGive(mutexSharedData);
            }
        }
    }
}

void taskArduinoCommunication(void* parameter) {
    Serial.println("[ArduinoTask] Started (UDP Bridge Mode)");
    
    ArduinoTXMessage txMsg;
    char udpBuffer[256];
    
    Serial.println("[ArduinoTask] Waiting for Wemos bridge...");
    
    while (1) {
        // Send outgoing messages to Wemos
        if (xQueueReceive(queueArduinoTX, &txMsg, 0) == pdTRUE) {
            Serial.printf("[ArduinoTask] TX: %s\n", txMsg.command);
            sendUDPMessage(txMsg.command);
        }
        
        // Receive incoming messages from Wemos
        int packetSize = udpBridge.parsePacket();
        if (packetSize > 0) {
            int len = udpBridge.read(udpBuffer, sizeof(udpBuffer) - 1);
            if (len > 0) {
                udpBuffer[len] = '\0';
                
                IPAddress senderIP = udpBridge.remoteIP();
                uint16_t senderPort = udpBridge.remotePort();
                
                Serial.printf("[ArduinoTask] RX from %s:%d: %s\n", 
                    senderIP.toString().c_str(), senderPort, udpBuffer);
                
                systemState.lastBridgeHeartbeat = millis();
                systemState.bridgeConnected = true;
                
                if (strstr(udpBuffer, "DATA") || strstr(udpBuffer, "BTN") || 
                    strstr(udpBuffer, "READY") || strstr(udpBuffer, "HB")) {
                    systemState.lastArduinoHeartbeat = millis();
                    systemState.arduinoConnected = true;
                }
                
                ArduinoRXData rxData;
                strncpy(rxData.message, udpBuffer, sizeof(rxData.message) - 1);
                rxData.message[sizeof(rxData.message) - 1] = '\0';
                rxData.timestamp = millis();
                rxData.senderIP = senderIP;
                rxData.senderPort = senderPort;
                
                if (xQueueSend(queueArduinoRX, &rxData, 0) != pdTRUE) {
                    Serial.println("[ArduinoTask] RX queue full");
                }
            }
        }
        
        // Check connection timeouts
        if (systemState.bridgeConnected && (millis() - systemState.lastBridgeHeartbeat > 10000)) {
            Serial.println("[ArduinoTask] Bridge connection lost!");
            systemState.bridgeConnected = false;
        }
        
        if (systemState.arduinoConnected && (millis() - systemState.lastArduinoHeartbeat > 10000)) {
            Serial.println("[ArduinoTask] Arduino connection lost!");
            systemState.arduinoConnected = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void taskDecisionEngine(void* parameter) {
    Serial.println("[DecisionTask] Started");
    
    PostureData postureData;
    ArduinoRXData arduinoRX;
    
    uint32_t lastAlertTime = 0;
    const uint32_t ALERT_COOLDOWN_MS = 30000;
    
    while (1) {
        // Process posture data
        if (xQueueReceive(queuePostureData, &postureData, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (postureData.isGood) {
                sessionStats.goodPostureTime++;
            } else {
                sessionStats.badPostureTime++;
            }
            
            bool shouldAlert = false;
            if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
                shouldAlert = !postureData.isGood && 
                             !alertsSnoozed && 
                             !inBreak &&
                             (millis() - lastAlertTime > ALERT_COOLDOWN_MS);
                xSemaphoreGive(mutexSharedData);
            }
            
            if (shouldAlert && systemState.bridgeConnected) {
                char alertCmd[64];
                snprintf(alertCmd, sizeof(alertCmd), "<ALERT:POSTURE,%d>", postureData.score);
                sendToArduinoViaBridge(alertCmd);
                sessionStats.alertCount++;
                lastAlertTime = millis();
                
                Serial.printf("[DecisionTask] Alert sent: Score %d\n", postureData.score);
            }
        }
        
        // Process Arduino messages
        if (xQueueReceive(queueArduinoRX, &arduinoRX, 0) == pdTRUE) {
            processArduinoMessage(arduinoRX.message);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void taskHeartbeat(void* parameter) {
    Serial.println("[HeartbeatTask] Started");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5000);
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        systemState.uptime = millis() / 1000;
        
        static uint32_t lastFrameCount = 0;
        uint32_t framesDelta = systemState.totalFrames - lastFrameCount;
        systemState.averageFPS = framesDelta / 5.0;
        lastFrameCount = systemState.totalFrames;
        
        sendUDPMessage("<HB>");
        
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(LED_BUILTIN, LOW);
        
        Serial.printf("[Heartbeat] Uptime: %lu s, FPS: %.1f, Bridge:%s, Arduino:%s\n",
            systemState.uptime, systemState.averageFPS,
            systemState.bridgeConnected ? "OK" : "X",
            systemState.arduinoConnected ? "OK" : "X");
    }
}

// ========== INITIALIZATION FUNCTIONS ==========

void printSystemInfo() {
    Serial.println("╔═══════════════════════════════════════════════╗");
    Serial.println("║   SMART POSTURE & ERGONOMICS CAMERA COACH    ║");
    Serial.println("║    ESP32-CAM FreeRTOS + UDP Bridge           ║");
    Serial.println("╚═══════════════════════════════════════════════╝");
    Serial.printf("\nFirmware: %s\n", FIRMWARE_VERSION);
    Serial.printf("Build: %s %s\n", BUILD_DATE, BUILD_TIME);
    Serial.printf("User: %s\n", BUILD_USER);
    Serial.printf("Chip: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("CPU Freq: %d MHz (Dual Core)\n", ESP.getCpuFreqMHz());
    Serial.printf("Flash: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("Free Heap: %d KB\n", ESP.getFreeHeap() / 1024);
    Serial.printf("PSRAM: %d KB\n", ESP.getPsramSize() / 1024);
    Serial.println();
}

bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_VGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }
    
    sensor_t* s = esp_camera_sensor_get();
    if (s == NULL) {
        Serial.println("Failed to get camera sensor");
        return false;
    }
    
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
    
    return true;
}

bool initWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    
    Serial.println("AP Mode started");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    
    systemState.wifiConnected = true;
    return true;
}

void initWebServer() {
    setupAPIRoutes();
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        String html = "<!DOCTYPE html><html><head><title>Posture Coach</title></head>";
        html += "<body style='font-family:Arial;text-align:center;padding:50px;'>";
        html += "<h1>Smart Posture Coach</h1>";
        html += "<p>Firmware: " + String(FIRMWARE_VERSION) + "</p>";
        html += "<p>FreeRTOS + UDP Bridge Mode</p>";
        html += "<p><a href='/api/status'>System Status (JSON)</a></p>";
        html += "<p><a href='/api/posture'>Posture Data (JSON)</a></p>";
        html += "<p><a href='/api/stats'>Session Stats (JSON)</a></p>";
        html += "</body></html>";
        
        request->send(200, "text/html", html);
    });
}

// ========== POSTURE ANALYSIS ==========

void analyzePosture(camera_fb_t* fb, PostureData* result) {
    // Simulated posture analysis
    result->userPresent = true;
    result->confidence = 0.85;
    
    static uint8_t simulatedScore = 85;
    simulatedScore += random(-5, 6);
    if (simulatedScore > 100) simulatedScore = 100;
    if (simulatedScore < 50) simulatedScore = 50;
    
    result->score = simulatedScore;
    result->isGood = (simulatedScore >= 75);
    result->timestamp = millis();
}

uint8_t calculatePostureScore(PostureData* data) {
    return data->score;
}

// ========== ARDUINO COMMUNICATION ==========

void sendToArduinoViaBridge(const char* cmd) {
    ArduinoTXMessage msg;
    strncpy(msg.command, cmd, sizeof(msg.command) - 1);
    msg.command[sizeof(msg.command) - 1] = '\0';
    msg.timestamp = millis();
    
    xQueueSend(queueArduinoTX, &msg, pdMS_TO_TICKS(100));
}

void sendUDPMessage(const char* message) {
    if (xSemaphoreTake(mutexUDP, pdMS_TO_TICKS(100)) == pdTRUE) {
        IPAddress broadcastIP(192, 168, 4, 255);
        udpBridge.beginPacket(broadcastIP, UDP_PORT);
        udpBridge.write((const uint8_t*)message, strlen(message));
        udpBridge.endPacket();
        xSemaphoreGive(mutexUDP);
    }
}

void processArduinoMessage(const char* msg) {
    if (strstr(msg, "DATA:DIST")) {
        const char* comma = strchr(msg, ',');
        if (comma) {
            int16_t distance = atoi(comma + 1);
            if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
                sharedPosture.distance = distance;
                xSemaphoreGive(mutexSharedData);
            }
            Serial.printf("[Arduino] Distance: %d cm\n", distance);
        }
    } else if (strstr(msg, "DATA:BTN")) {
        const char* comma = strchr(msg, ',');
        if (comma) {
            int btnNum = atoi(comma + 1);
            Serial.printf("[Arduino] Button %d pressed\n", btnNum);
            
            if (btnNum == 1) {
                if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
                    inBreak = !inBreak;
                    Serial.printf("Break mode: %s\n", inBreak ? "ON" : "OFF");
                    xSemaphoreGive(mutexSharedData);
                }
            } else if (btnNum == 2) {
                if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
                    alertsSnoozed = !alertsSnoozed;
                    Serial.printf("Alerts snoozed: %s\n", alertsSnoozed ? "ON" : "OFF");
                    xSemaphoreGive(mutexSharedData);
                }
            } else if (btnNum == 3) {
                if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
                    privacyMode = !privacyMode;
                    Serial.printf("Privacy mode: %s\n", privacyMode ? "ON" : "OFF");
                    xSemaphoreGive(mutexSharedData);
                }
            }
        }
    } else if (strstr(msg, "READY")) {
        Serial.println("[Arduino] Arduino is READY!");
        systemState.arduinoConnected = true;
    }
}

// ========== WEB SERVER ROUTES ==========

void setupAPIRoutes() {
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;
        doc["version"] = FIRMWARE_VERSION;
        doc["uptime"] = systemState.uptime;
        doc["cameraOK"] = systemState.cameraInitialized;
        doc["bridgeOK"] = systemState.bridgeConnected;
        doc["arduinoOK"] = systemState.arduinoConnected;
        doc["wifiOK"] = systemState.wifiConnected;
        
        if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
            doc["privacyMode"] = privacyMode;
            doc["inBreak"] = inBreak;
            doc["alertsSnoozed"] = alertsSnoozed;
            xSemaphoreGive(mutexSharedData);
        }
        
        doc["fps"] = systemState.averageFPS;
        doc["totalFrames"] = systemState.totalFrames;
        doc["droppedFrames"] = systemState.droppedFrames;
        doc["freeHeap"] = ESP.getFreeHeap();
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    server.on("/api/posture", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;
        
        if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
            doc["score"] = sharedPosture.currentScore;
            doc["isGood"] = sharedPosture.isGoodPosture;
            doc["distance"] = sharedPosture.distance;
            doc["userPresent"] = sharedPosture.userPresent;
            doc["lastUpdate"] = sharedPosture.lastUpdate;
            xSemaphoreGive(mutexSharedData);
        }
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    server.on("/api/stats", HTTP_GET, [](AsyncWebServerRequest* request) {
        JsonDocument doc;
        doc["goodPostureTime"] = sessionStats.goodPostureTime;
        doc["badPostureTime"] = sessionStats.badPostureTime;
        doc["breakTime"] = sessionStats.breakTime;
        doc["alertCount"] = sessionStats.alertCount;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    server.on("/api/privacy", HTTP_POST, [](AsyncWebServerRequest* request) {
        if (request->hasParam("enable", true)) {
            bool enable = request->getParam("enable", true)->value() == "true";
            
            if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(100)) == pdTRUE) {
                privacyMode = enable;
                xSemaphoreGive(mutexSharedData);
            }
            
            if (systemState.bridgeConnected) {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "<PRIVACY:%s>", enable ? "ENABLE" : "DISABLE");
                sendToArduinoViaBridge(cmd);
            }
            
            request->send(200, "application/json", "{\"success\":true}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing parameter\"}");
        }
    });
    
    server.on("/api/break", HTTP_POST, [](AsyncWebServerRequest* request) {
        if (request->hasParam("start", true)) {
            bool start = request->getParam("start", true)->value() == "true";
            
            if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(100)) == pdTRUE) {
                inBreak = start;
                xSemaphoreGive(mutexSharedData);
            }
            
            if (systemState.bridgeConnected) {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "<BREAK:%s>", start ? "START" : "END");
                sendToArduinoViaBridge(cmd);
            }
            
            request->send(200, "application/json", "{\"success\":true}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing parameter\"}");
        }
    });
    
    server.on("/api/snooze", HTTP_POST, [](AsyncWebServerRequest* request) {
        if (request->hasParam("enable", true)) {
            bool enable = request->getParam("enable", true)->value() == "true";
            
            if (xSemaphoreTake(mutexSharedData, pdMS_TO_TICKS(100)) == pdTRUE) {
                alertsSnoozed = enable;
                xSemaphoreGive(mutexSharedData);
            }
            
            if (systemState.bridgeConnected) {
                char cmd[32];
                snprintf(cmd, sizeof(cmd), "<SNOOZE:%s>", enable ? "ENABLE" : "DISABLE");
                sendToArduinoViaBridge(cmd);
            }
            
            request->send(200, "application/json", "{\"success\":true}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing parameter\"}");
        }
    });
}

// ========== UTILITY FUNCTIONS ==========

void blinkLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delayMs);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delayMs);
    }
}

void printTaskStats() {
    Serial.println("\n=== System Statistics ===");
    Serial.printf("Free Heap: %d KB\n", ESP.getFreeHeap() / 1024);
    Serial.printf("Min Free Heap: %d KB\n", ESP.getMinFreeHeap() / 1024);
    Serial.printf("Free PSRAM: %d KB\n", ESP.getFreePsram() / 1024);
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    
    Serial.println("\nQueue Status:");
    Serial.printf("  Frame Queue: %d/%d\n", 
        uxQueueMessagesWaiting(queueFrames), FRAME_QUEUE_SIZE);
    Serial.printf("  Posture Queue: %d/%d\n", 
        uxQueueMessagesWaiting(queuePostureData), POSTURE_QUEUE_SIZE);
    Serial.printf("  Arduino TX: %d/%d\n", 
        uxQueueMessagesWaiting(queueArduinoTX), ARDUINO_TX_QUEUE_SIZE);
    Serial.printf("  Arduino RX: %d/%d\n", 
        uxQueueMessagesWaiting(queueArduinoRX), ARDUINO_RX_QUEUE_SIZE);
    
    Serial.printf("\nActive Tasks: %d\n", uxTaskGetNumberOfTasks());
    
    Serial.println("\nConnection Status:");
    Serial.printf("  WiFi: %s\n", systemState.wifiConnected ? "Connected" : "Disconnected");
    Serial.printf("  Wemos Bridge: %s\n", systemState.bridgeConnected ? "Connected" : "Disconnected");
    Serial.printf("  Arduino: %s\n", systemState.arduinoConnected ? "Connected" : "Disconnected");
    
    Serial.println("================================\n");
}