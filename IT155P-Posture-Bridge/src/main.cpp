/**
 * @file main.cpp
 * @brief Smart Posture Coach - Wemos D1 R2 WiFi-Serial Bridge
 * @author Smart Posture Coach Team
 * @date 2025-11-17
 * @updated 2025-11-17 09:52:41 UTC
 * @user hyoono
 * 
 * VERSION: 1.0.5-Bridge (Final)
 * 
 * FEATURES:
 * - WiFi client to ESP32-CAM AP
 * - SoftwareSerial to Arduino Uno
 * - UDP bridge (port 8888)
 * - Bidirectional message forwarding
 * - Web status dashboard
 * 
 * PINS:
 * D5 (GPIO14) - RX from Arduino D12
 * D6 (GPIO12) - TX to Arduino D7
 * 
 * WIRING:
 * Wemos D6 (TX) → Arduino D7 (RX)
 * Wemos D5 (RX) ← Arduino D12 (TX)
 * Wemos GND → Arduino GND
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>

#define FIRMWARE_VERSION "1.0.5-Bridge"
#define BUILD_DATE "2025-11-17"
#define BUILD_TIME "09:52:41"
#define BUILD_USER "hyoono"

// Pin definitions
#define ARDUINO_RX_PIN 14  // D5 (GPIO14) - Wemos RX ← Arduino TX
#define ARDUINO_TX_PIN 12  // D6 (GPIO12) - Wemos TX → Arduino RX
#define ARDUINO_BAUD 57600
#define STATUS_LED_PIN LED_BUILTIN

// WiFi configuration
const char* ESP32_SSID = "PostureCoach-AP";
const char* ESP32_PASSWORD = "posture2025";
const char* BRIDGE_SSID = "PostureBridge";
const char* BRIDGE_PASSWORD = "bridge2025";

// Network configuration
const char* ESP32_IP = "192.168.4.1";
const uint16_t UDP_PORT = 8888;

// Objects
SoftwareSerial arduinoSerial(ARDUINO_RX_PIN, ARDUINO_TX_PIN);
WiFiUDP udp;
ESP8266WebServer server(80);

// State
bool wifiConnected = false;
bool esp32Connected = false;
bool arduinoConnected = false;
bool isAPMode = false;
uint32_t lastESP32Ping = 0;
uint32_t lastArduinoPing = 0;
uint32_t messagesFromESP32 = 0;
uint32_t messagesFromArduino = 0;
uint32_t messagesDropped = 0;

// Buffers
#define BUFFER_SIZE 256
char wifiRxBuffer[BUFFER_SIZE];
char serialRxBuffer[BUFFER_SIZE];
uint16_t serialBufferIndex = 0;

// Function prototypes
void setupWiFi();
void setupSerial();
void setupWebServer();
void handleUDP();
void handleSerialData();
void forwardToArduino(const char* data, size_t length);
void forwardToESP32(const char* data, size_t length);
void sendUDP(const char* data, size_t length);
void checkConnections();
void updateStatusLED();
void printSystemInfo();
void printStats();

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n\n\n\n");
    printSystemInfo();
    
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
    
    setupSerial();
    setupWiFi();
    
    Serial.println("Initializing UDP...");
    udp.begin(UDP_PORT);
    Serial.print("UDP listening on port ");
    Serial.println(UDP_PORT);
    Serial.println();
    
    setupWebServer();
    
    for (int i = 0; i < 5; i++) {
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
    }
    
    Serial.println("╔═════════════════════════════════════════╗");
    Serial.println("║        BRIDGE READY!                    ║");
    Serial.println("╚═════════════════════════════════════════╝");
    Serial.print("WiFi: ");
    Serial.println(wifiConnected ? "Connected" : (isAPMode ? "AP Mode" : "Failed"));
    Serial.println("Arduino: Waiting for data...");
    Serial.println("=========================================\n");
}

void loop() {
    static uint32_t lastCheck = 0;
    static uint32_t lastStats = 0;
    uint32_t now = millis();
    
    handleUDP();
    handleSerialData();
    server.handleClient();
    
    if (now - lastCheck >= 1000) {
        checkConnections();
        updateStatusLED();
        lastCheck = now;
    }
    
    if (now - lastStats >= 30000) {
        printStats();
        lastStats = now;
    }
    
    yield();
}

void printSystemInfo() {
    Serial.println("╔═══════════════════════════════════════════════╗");
    Serial.println("║    SMART POSTURE COACH - WiFi BRIDGE         ║");
    Serial.println("║         Wemos D1 R2 Firmware                 ║");
    Serial.println("╚═══════════════════════════════════════════════╝");
    Serial.print("\nFirmware: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.print("Build: ");
    Serial.print(BUILD_DATE);
    Serial.print(" ");
    Serial.println(BUILD_TIME);
    Serial.print("User: ");
    Serial.println(BUILD_USER);
    Serial.print("Chip ID: ");
    Serial.println(ESP.getChipId(), HEX);
    Serial.print("Flash: ");
    Serial.print(ESP.getFlashChipSize() / 1024);
    Serial.println(" KB");
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");
    Serial.println();
}

void setupSerial() {
    Serial.println("Setting up Arduino UART...");
    Serial.println("  RX: D5 (GPIO14) ← Arduino D12");
    Serial.println("  TX: D6 (GPIO12) → Arduino D7");
    Serial.print("  Baud: ");
    Serial.println(ARDUINO_BAUD);
    
    arduinoSerial.begin(ARDUINO_BAUD);
    delay(100);
    
    Serial.println("[OK] Arduino UART ready\n");
}

void setupWiFi() {
    Serial.println("Setting up WiFi...");
    Serial.print("Connecting to: ");
    Serial.println(ESP32_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ESP32_SSID, ESP32_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        isAPMode = false;
        
        Serial.println("[OK] WiFi connected!");
        Serial.print("  IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("  Gateway: ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("  Signal: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("[FAIL] WiFi connection failed");
        Serial.println("  Starting AP mode...");
        
        WiFi.mode(WIFI_AP);
        WiFi.softAP(BRIDGE_SSID, BRIDGE_PASSWORD);
        
        isAPMode = true;
        wifiConnected = false;
        
        Serial.println("[OK] Bridge AP started");
        Serial.print("  SSID: ");
        Serial.println(BRIDGE_SSID);
        Serial.print("  IP: ");
        Serial.println(WiFi.softAPIP());
    }
    Serial.println();
}

void setupWebServer() {
    Serial.println("Starting web server...");
    
    server.on("/", HTTP_GET, []() {
        String html = "<!DOCTYPE html><html><head>";
        html += "<title>Posture Bridge</title>";
        html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
        html += "<style>body{font-family:Arial;padding:20px;background:#f0f0f0;}";
        html += "h1{color:#333;}.status{background:#fff;padding:15px;margin:10px 0;border-radius:5px;}";
        html += ".ok{color:#28a745;font-weight:bold;}.error{color:#dc3545;font-weight:bold;}</style>";
        html += "</head><body><h1>🌉 Posture Bridge</h1>";
        html += "<div class='status'><h2>Status</h2>";
        html += "<p>Firmware: <b>" + String(FIRMWARE_VERSION) + "</b></p>";
        html += "<p>Uptime: <b>" + String(millis() / 1000) + " s</b></p>";
        html += "<p>WiFi: <span class='" + String(wifiConnected ? "ok" : "error") + "'>";
        html += wifiConnected ? "Connected" : (isAPMode ? "AP Mode" : "Disconnected");
        html += "</span></p>";
        html += "<p>ESP32: <span class='" + String(esp32Connected ? "ok" : "error") + "'>";
        html += esp32Connected ? "Connected" : "Disconnected";
        html += "</span></p>";
        html += "<p>Arduino: <span class='" + String(arduinoConnected ? "ok" : "error") + "'>";
        html += arduinoConnected ? "Connected" : "Disconnected";
        html += "</span></p></div>";
        html += "<div class='status'><h2>Statistics</h2>";
        html += "<p>From ESP32: <b>" + String(messagesFromESP32) + "</b></p>";
        html += "<p>From Arduino: <b>" + String(messagesFromArduino) + "</b></p>";
        html += "<p>Dropped: <b>" + String(messagesDropped) + "</b></p></div>";
        html += "<p><a href='/status'>JSON Status</a></p>";
        html += "</body></html>";
        
        server.send(200, "text/html", html);
    });
    
    server.on("/status", HTTP_GET, []() {
        String json = "{";
        json += "\"version\":\"" + String(FIRMWARE_VERSION) + "\",";
        json += "\"uptime\":" + String(millis() / 1000) + ",";
        json += "\"wifi\":" + String(wifiConnected ? "true" : "false") + ",";
        json += "\"esp32\":" + String(esp32Connected ? "true" : "false") + ",";
        json += "\"arduino\":" + String(arduinoConnected ? "true" : "false") + ",";
        json += "\"messagesFromESP32\":" + String(messagesFromESP32) + ",";
        json += "\"messagesFromArduino\":" + String(messagesFromArduino) + ",";
        json += "\"messagesDropped\":" + String(messagesDropped);
        json += "}";
        
        server.send(200, "application/json", json);
    });
    
    server.begin();
    Serial.println("[OK] Web server started on port 80\n");
}

void handleUDP() {
    int packetSize = udp.parsePacket();
    
    if (packetSize > 0) {
        int len = udp.read(wifiRxBuffer, sizeof(wifiRxBuffer) - 1);
        if (len > 0) {
            wifiRxBuffer[len] = '\0';
            
            Serial.print("[WiFi→Arduino] ");
            Serial.println(wifiRxBuffer);
            
            forwardToArduino(wifiRxBuffer, len);
            
            lastESP32Ping = millis();
            messagesFromESP32++;
            esp32Connected = true;
        }
    }
}

void handleSerialData() {
    while (arduinoSerial.available() > 0) {
        char c = arduinoSerial.read();
        
        if (c == '\n' || c == '\r' || c == '>') {
            if (serialBufferIndex > 0) {
                serialRxBuffer[serialBufferIndex] = '\0';
                
                Serial.print("[Arduino→WiFi] ");
                Serial.println(serialRxBuffer);
                
                forwardToESP32(serialRxBuffer, serialBufferIndex);
                
                lastArduinoPing = millis();
                messagesFromArduino++;
                arduinoConnected = true;
                
                serialBufferIndex = 0;
            }
        } else if (serialBufferIndex < sizeof(serialRxBuffer) - 1) {
            serialRxBuffer[serialBufferIndex++] = c;
        } else {
            Serial.println("[ERROR] Serial buffer overflow");
            serialBufferIndex = 0;
            messagesDropped++;
        }
    }
}

void forwardToArduino(const char* data, size_t length) {
    arduinoSerial.write(data, length);
    arduinoSerial.write('\n');
    arduinoSerial.flush();
}

void forwardToESP32(const char* data, size_t length) {
    if (!wifiConnected) {
        Serial.println("[ERROR] WiFi not connected");
        messagesDropped++;
        return;
    }
    sendUDP(data, length);
}

void sendUDP(const char* data, size_t length) {
    udp.beginPacket(ESP32_IP, UDP_PORT);
    udp.write((const uint8_t*)data, length);
    udp.endPacket();
}

void checkConnections() {
    uint32_t now = millis();
    
    // Check WiFi
    if (WiFi.status() != WL_CONNECTED && !isAPMode) {
        if (wifiConnected) {
            Serial.println("[WARN] WiFi connection lost");
        }
        wifiConnected = false;
    } else if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
    }
    
    // Check ESP32 timeout
    if (esp32Connected && (now - lastESP32Ping > 10000)) {
        Serial.println("[WARN] ESP32 connection timeout");
        esp32Connected = false;
    }
    
    // Check Arduino timeout
    if (arduinoConnected && (now - lastArduinoPing > 10000)) {
        Serial.println("[WARN] Arduino connection timeout");
        arduinoConnected = false;
    }
}

void updateStatusLED() {
    static uint32_t lastBlink = 0;
    static bool ledState = false;
    uint32_t now = millis();
    
    uint16_t blinkInterval;
    
    if (wifiConnected && esp32Connected && arduinoConnected) {
        blinkInterval = 2000;  // Slow - all good
    } else if (wifiConnected) {
        blinkInterval = 500;   // Medium - WiFi only
    } else {
        blinkInterval = 200;   // Fast - no WiFi
    }
    
    if (now - lastBlink >= blinkInterval) {
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState ? LOW : HIGH);
        lastBlink = now;
    }
}

void printStats() {
    Serial.println("\n========== Bridge Statistics ==========");
    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" s");
    Serial.print("WiFi: ");
    Serial.println(wifiConnected ? "Connected" : "Disconnected");
    Serial.print("ESP32: ");
    Serial.println(esp32Connected ? "Connected" : "Disconnected");
    Serial.print("Arduino: ");
    Serial.println(arduinoConnected ? "Connected" : "Disconnected");
    Serial.print("Messages from ESP32: ");
    Serial.println(messagesFromESP32);
    Serial.print("Messages from Arduino: ");
    Serial.println(messagesFromArduino);
    Serial.print("Dropped: ");
    Serial.println(messagesDropped);
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    Serial.println("======================================\n");
}