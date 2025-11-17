/**
 * @file main.cpp
 * @brief Smart Posture & Ergonomics Camera Coach - Arduino Main Controller
 * @author Smart Posture Coach Team
 * @date 2025-11-17
 * @updated 2025-11-17 09:52:41 UTC
 * @user hyoono
 * 
 * VERSION: 1.0.11 (Stable - Final)
 * 
 * FEATURES:
 * - HC-SR04 ultrasonic distance sensor
 * - 3 push buttons (Break, Snooze, Privacy)
 * - RGB LED feedback (9 colors)
 * - Piezo buzzer alerts (7 patterns)
 * - Dual serial: USB + Wemos communication
 * 
 * PINS:
 * D0/D1  - USB Serial Monitor
 * D2     - HC-SR04 Trigger
 * D3     - HC-SR04 Echo
 * D4     - Button 1 (Break)
 * D5     - Button 2 (Snooze)
 * D6     - Button 3 (Privacy)
 * D7     - Wemos RX (Arduino reads from Wemos)
 * D8     - Buzzer
 * D9     - LED Red
 * D10    - LED Green
 * D11    - LED Blue
 * D12    - Wemos TX (Arduino sends to Wemos)
 * 
 * WIRING:
 * Arduino D12 → Wemos D5 (RX)
 * Arduino D7 ← Wemos D6 (TX)
 * Arduino GND → Wemos GND
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

#define FIRMWARE_VERSION "1.0.11"
#define BUILD_DATE "2025-11-17"
#define BUILD_TIME "09:52:41"
#define BUILD_USER "hyoono"

// Pin definitions
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define BUTTON_1_PIN 4
#define BUTTON_2_PIN 5
#define BUTTON_3_PIN 6
#define WEMOS_RX_PIN 7
#define BUZZER_PIN 8
#define LED_RED_PIN 9
#define LED_GREEN_PIN 10
#define LED_BLUE_PIN 11
#define WEMOS_TX_PIN 12

// Timing constants
#define DISTANCE_MEASURE_INTERVAL 2000
#define DISTANCE_REPORT_INTERVAL 2000
#define BUTTON_DEBOUNCE_DELAY 50
#define BUTTON_LONG_PRESS_TIME 3000
#define HEARTBEAT_INTERVAL 5000

// Distance thresholds
#define OPTIMAL_DISTANCE_MIN 50
#define OPTIMAL_DISTANCE_MAX 70
#define MIN_VALID_DISTANCE 10
#define MAX_VALID_DISTANCE 400

// LED colors
enum LedColor {
    LED_OFF = 0,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_YELLOW,
    LED_ORANGE,
    LED_PURPLE,
    LED_CYAN,
    LED_WHITE
};

// Buzzer patterns
enum BuzzerPattern {
    BUZZ_URGENT,
    BUZZ_WARNING,
    BUZZ_DISTANCE,
    BUZZ_SUCCESS,
    BUZZ_BREAK_REMINDER,
    BUZZ_BUTTON_CLICK,
    BUZZ_CANCEL
};

// Button state structure
struct ButtonState {
    uint8_t pin;
    bool lastState;
    bool currentState;
    uint32_t lastDebounceTime;
    uint32_t pressStartTime;
    bool longPressTriggered;
};

// Button instances
ButtonState button1 = {BUTTON_1_PIN, HIGH, HIGH, 0, 0, false};
ButtonState button2 = {BUTTON_2_PIN, HIGH, HIGH, 0, 0, false};
ButtonState button3 = {BUTTON_3_PIN, HIGH, HIGH, 0, 0, false};

// System state
bool breakMode = false;
bool snoozeMode = false;
bool privacyMode = false;
LedColor currentLedColor = LED_GREEN;
int lastDistance = 0;
uint32_t lastDistanceMeasure = 0;
uint32_t lastDistanceReport = 0;
uint32_t lastHeartbeat = 0;

// SoftwareSerial for Wemos
SoftwareSerial wemosSerial(WEMOS_RX_PIN, WEMOS_TX_PIN);

// Function prototypes
void printSystemInfo();
void initializeHardware();
void testHardware();
int getDistance();
void processDistance(int distance);
void updateButton(ButtonState* btn);
void handleButton1();
void handleButton2();
void handleButton3();
void setLED(LedColor color);
void setLEDRGB(uint8_t r, uint8_t g, uint8_t b);
void playBuzzer(BuzzerPattern pattern);
void playTone(uint8_t pin, uint16_t frequency, uint32_t duration);
void sendToWemos(const char* msg);
void processSerialMonitorCommand();
void processWemosMessage();
void processCommand(String msg);

// ========== SETUP ==========
void setup() {
    // USB Serial - MUST be first
    Serial.begin(57600);
    delay(3000);  // Wait for Serial to stabilize
    
    Serial.println("\n\n\n\n\n");
    Serial.println("╔═══════════════════════════════════════════════╗");
    Serial.println("║   SMART POSTURE & ERGONOMICS CAMERA COACH    ║");
    Serial.println("║           Arduino Main Firmware              ║");
    Serial.println("╚═══════════════════════════════════════════════╝");
    Serial.println();
    
    printSystemInfo();
    
    // Wemos Serial
    wemosSerial.begin(57600);
    delay(100);
    Serial.println("[OK] SoftwareSerial started (D7/D12)");
    
    initializeHardware();
    
    Serial.println("\nTesting hardware...");
    testHardware();
    
    Serial.println("\nSending READY to Wemos...");
    sendToWemos("<READY>");
    
    Serial.println("\n=== SYSTEM READY ===");
    Serial.println("Type commands like: <LED:RED>");
    Serial.println("Or press physical buttons");
    Serial.println("====================\n");
}

// ========== MAIN LOOP ==========
void loop() {
    uint32_t now = millis();
    
    // Update buttons
    updateButton(&button1);
    updateButton(&button2);
    updateButton(&button3);
    
    // Measure distance
    if (now - lastDistanceMeasure >= DISTANCE_MEASURE_INTERVAL) {
        int dist = getDistance();
        if (dist > 0) {
            lastDistance = dist;
            processDistance(dist);
        }
        lastDistanceMeasure = now;
    }
    
    // Report distance to Wemos
    if (now - lastDistanceReport >= DISTANCE_REPORT_INTERVAL) {
        if (lastDistance > 0) {
            char msg[32];
            snprintf(msg, sizeof(msg), "<DATA:DIST,%d>", lastDistance);
            sendToWemos(msg);
        }
        lastDistanceReport = now;
    }
    
    // Send heartbeat
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        sendToWemos("<HB>");
        lastHeartbeat = now;
    }
    
    // Read from USB Serial Monitor
    if (Serial.available()) {
        processSerialMonitorCommand();
    }
    
    // Read from Wemos
    if (wemosSerial.available()) {
        processWemosMessage();
    }
    
    delay(10);
}

// ========== INITIALIZATION ==========

void printSystemInfo() {
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.print("Build: ");
    Serial.print(BUILD_DATE);
    Serial.print(" ");
    Serial.println(BUILD_TIME);
    Serial.print("User: ");
    Serial.println(BUILD_USER);
    Serial.println("Board: Arduino Uno");
    Serial.println();
}

void initializeHardware() {
    Serial.println("Initializing hardware...");
    
    // Distance sensor
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    
    // Buttons
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    
    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    // RGB LED
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    setLED(LED_OFF);
    
    Serial.println("[OK] Hardware initialized");
}

void testHardware() {
    int testDist = getDistance();
    if (testDist > 0 && testDist < MAX_VALID_DISTANCE) {
        Serial.print("[OK] Ultrasonic: ");
        Serial.print(testDist);
        Serial.println(" cm");
    } else {
        Serial.println("[WARN] Ultrasonic: Check wiring");
    }
    
    Serial.println("[OK] RGB LED: Testing");
    setLED(LED_RED);
    delay(200);
    setLED(LED_GREEN);
    delay(200);
    setLED(LED_BLUE);
    delay(200);
    setLED(LED_GREEN);
    
    Serial.println("[OK] Buzzer: Testing");
    playBuzzer(BUZZ_BUTTON_CLICK);
}

// ========== DISTANCE SENSOR ==========

int getDistance() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return -1;
    
    return duration * 0.034 / 2;
}

void processDistance(int distance) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm");
    
    if (distance < OPTIMAL_DISTANCE_MIN) {
        Serial.println(" (TOO CLOSE)");
        if (!breakMode && !privacyMode) {
            setLED(LED_ORANGE);
            playBuzzer(BUZZ_DISTANCE);
        }
    } else if (distance > OPTIMAL_DISTANCE_MAX) {
        Serial.println(" (TOO FAR)");
        if (!breakMode && !privacyMode) {
            setLED(LED_ORANGE);
            playBuzzer(BUZZ_DISTANCE);
        }
    } else {
        Serial.println(" (OPTIMAL)");
        if (!breakMode && !privacyMode && !snoozeMode) {
            if (currentLedColor != LED_CYAN && currentLedColor != LED_PURPLE && currentLedColor != LED_YELLOW) {
                setLED(LED_GREEN);
            }
        }
    }
}

// ========== BUTTON HANDLING ==========

void updateButton(ButtonState* btn) {
    bool reading = digitalRead(btn->pin);
    
    if (reading != btn->lastState) {
        btn->lastDebounceTime = millis();
    }
    
    if ((millis() - btn->lastDebounceTime) > BUTTON_DEBOUNCE_DELAY) {
        if (reading != btn->currentState) {
            btn->currentState = reading;
            
            if (btn->currentState == LOW) {
                btn->pressStartTime = millis();
                btn->longPressTriggered = false;
            } else {
                uint32_t pressDuration = millis() - btn->pressStartTime;
                
                if (!btn->longPressTriggered && pressDuration < BUTTON_LONG_PRESS_TIME) {
                    if (btn->pin == BUTTON_1_PIN) handleButton1();
                    else if (btn->pin == BUTTON_2_PIN) handleButton2();
                    else if (btn->pin == BUTTON_3_PIN) handleButton3();
                }
            }
        }
        
        if (btn->currentState == LOW && !btn->longPressTriggered) {
            uint32_t pressDuration = millis() - btn->pressStartTime;
            if (pressDuration >= BUTTON_LONG_PRESS_TIME) {
                btn->longPressTriggered = true;
                Serial.print("[BTN");
                Serial.print(btn->pin == BUTTON_1_PIN ? 1 : (btn->pin == BUTTON_2_PIN ? 2 : 3));
                Serial.println("] Long-press");
                playBuzzer(BUZZ_SUCCESS);
            }
        }
    }
    
    btn->lastState = reading;
}

void handleButton1() {
    breakMode = !breakMode;
    Serial.print("[BTN1] Break ");
    Serial.println(breakMode ? "ON" : "OFF");
    
    if (breakMode) {
        setLED(LED_CYAN);
        playBuzzer(BUZZ_BREAK_REMINDER);
    } else {
        setLED(LED_GREEN);
        playBuzzer(BUZZ_SUCCESS);
    }
    
    sendToWemos("<DATA:BTN,1>");
}

void handleButton2() {
    snoozeMode = !snoozeMode;
    Serial.print("[BTN2] Snooze ");
    Serial.println(snoozeMode ? "ON" : "OFF");
    
    if (snoozeMode) {
        setLED(LED_YELLOW);
        playBuzzer(BUZZ_BUTTON_CLICK);
    } else {
        setLED(LED_GREEN);
        playBuzzer(BUZZ_BUTTON_CLICK);
    }
    
    sendToWemos("<DATA:BTN,2>");
}

void handleButton3() {
    privacyMode = !privacyMode;
    Serial.print("[BTN3] Privacy ");
    Serial.println(privacyMode ? "ON" : "OFF");
    
    if (privacyMode) {
        setLED(LED_PURPLE);
        playBuzzer(BUZZ_BUTTON_CLICK);
    } else {
        setLED(LED_GREEN);
        playBuzzer(BUZZ_BUTTON_CLICK);
    }
    
    sendToWemos("<DATA:BTN,3>");
}

// ========== LED CONTROL ==========

void setLED(LedColor color) {
    currentLedColor = color;
    
    switch (color) {
        case LED_OFF: setLEDRGB(0, 0, 0); break;
        case LED_RED: setLEDRGB(255, 0, 0); break;
        case LED_GREEN: setLEDRGB(0, 255, 0); break;
        case LED_BLUE: setLEDRGB(0, 0, 255); break;
        case LED_YELLOW: setLEDRGB(255, 255, 0); break;
        case LED_ORANGE: setLEDRGB(255, 128, 0); break;
        case LED_PURPLE: setLEDRGB(128, 0, 255); break;
        case LED_CYAN: setLEDRGB(0, 255, 255); break;
        case LED_WHITE: setLEDRGB(255, 255, 255); break;
    }
}

void setLEDRGB(uint8_t r, uint8_t g, uint8_t b) {
    analogWrite(LED_RED_PIN, r);
    analogWrite(LED_GREEN_PIN, g);
    analogWrite(LED_BLUE_PIN, b);
}

// ========== BUZZER CONTROL ==========

void playBuzzer(BuzzerPattern pattern) {
    switch (pattern) {
        case BUZZ_URGENT:
            for (int i = 0; i < 3; i++) {
                playTone(BUZZER_PIN, 2000, 300);
                delay(400);
            }
            break;
        case BUZZ_WARNING:
            for (int i = 0; i < 2; i++) {
                playTone(BUZZER_PIN, 1500, 200);
                delay(250);
            }
            break;
        case BUZZ_DISTANCE:
            playTone(BUZZER_PIN, 1000, 100);
            break;
        case BUZZ_SUCCESS:
            playTone(BUZZER_PIN, 1000, 100);
            delay(120);
            playTone(BUZZER_PIN, 1500, 100);
            break;
        case BUZZ_BREAK_REMINDER:
            playTone(BUZZER_PIN, 1000, 200);
            delay(250);
            playTone(BUZZER_PIN, 1200, 200);
            delay(250);
            playTone(BUZZER_PIN, 1500, 300);
            break;
        case BUZZ_BUTTON_CLICK:
            playTone(BUZZER_PIN, 2000, 50);
            break;
        case BUZZ_CANCEL:
            playTone(BUZZER_PIN, 1500, 100);
            delay(120);
            playTone(BUZZER_PIN, 1000, 100);
            break;
    }
}

void playTone(uint8_t pin, uint16_t frequency, uint32_t duration) {
    unsigned long period = 1000000L / frequency;
    unsigned long elapsed = 0;
    
    while (elapsed < duration * 1000) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(period / 2);
        digitalWrite(pin, LOW);
        delayMicroseconds(period / 2);
        elapsed += period;
    }
}

// ========== COMMUNICATION ==========

void sendToWemos(const char* msg) {
    wemosSerial.println(msg);
    Serial.print("[→Wemos] ");
    Serial.println(msg);
}

void processSerialMonitorCommand() {
    static String buffer = "";
    static unsigned long lastChar = 0;
    
    while (Serial.available()) {
        char c = Serial.read();
        
        if (millis() - lastChar > 100 && buffer.length() > 0) {
            Serial.print("[TIMEOUT] ");
            Serial.println(buffer);
            buffer = "";
        }
        lastChar = millis();
        
        if (c == '<') {
            buffer = "<";
        } else if (c == '>') {
            if (buffer.length() > 0) {
                buffer += ">";
                Serial.print("[USB] ");
                Serial.println(buffer);
                processCommand(buffer);
                buffer = "";
            }
        } else if (buffer.length() > 0) {
            buffer += c;
            if (buffer.length() > 100) {
                Serial.println("[ERROR] Too long");
                buffer = "";
            }
        }
    }
}

void processWemosMessage() {
    static String buffer = "";
    static unsigned long lastChar = 0;
    
    while (wemosSerial.available()) {
        char c = wemosSerial.read();
        
        if (millis() - lastChar > 100 && buffer.length() > 0) {
            Serial.print("[TIMEOUT] ");
            Serial.println(buffer);
            buffer = "";
        }
        lastChar = millis();
        
        if (c == '<') {
            buffer = "<";
        } else if (c == '>') {
            if (buffer.length() > 0) {
                buffer += ">";
                Serial.print("[Wemos] ");
                Serial.println(buffer);
                processCommand(buffer);
                buffer = "";
            }
        } else if (buffer.length() > 0) {
            buffer += c;
            if (buffer.length() > 100) {
                Serial.println("[ERROR] Too long");
                buffer = "";
            }
        }
    }
}

void processCommand(String msg) {
    msg.replace("<", "");
    msg.replace(">", "");
    
    Serial.print("[CMD] ");
    Serial.println(msg);
    
    if (msg.startsWith("LED:")) {
        String color = msg.substring(4);
        if (color == "RED") { setLED(LED_RED); Serial.println("→ LED RED"); }
        else if (color == "GREEN") { setLED(LED_GREEN); Serial.println("→ LED GREEN"); }
        else if (color == "BLUE") { setLED(LED_BLUE); Serial.println("→ LED BLUE"); }
        else if (color == "YELLOW") { setLED(LED_YELLOW); Serial.println("→ LED YELLOW"); }
        else if (color == "ORANGE") { setLED(LED_ORANGE); Serial.println("→ LED ORANGE"); }
        else if (color == "PURPLE") { setLED(LED_PURPLE); Serial.println("→ LED PURPLE"); }
        else if (color == "CYAN") { setLED(LED_CYAN); Serial.println("→ LED CYAN"); }
        else if (color == "WHITE") { setLED(LED_WHITE); Serial.println("→ LED WHITE"); }
        else if (color == "OFF") { setLED(LED_OFF); Serial.println("→ LED OFF"); }
    }
    else if (msg.startsWith("BUZZER:")) {
        String pattern = msg.substring(7);
        Serial.print("→ BUZZER ");
        Serial.println(pattern);
        
        if (pattern == "URGENT") playBuzzer(BUZZ_URGENT);
        else if (pattern == "WARNING") playBuzzer(BUZZ_WARNING);
        else if (pattern == "DISTANCE") playBuzzer(BUZZ_DISTANCE);
        else if (pattern == "SUCCESS") playBuzzer(BUZZ_SUCCESS);
        else if (pattern == "BREAK") playBuzzer(BUZZ_BREAK_REMINDER);
        else if (pattern == "CLICK") playBuzzer(BUZZ_BUTTON_CLICK);
    }
    else if (msg.startsWith("ALERT:POSTURE")) {
        int comma = msg.indexOf(',');
        if (comma > 0) {
            int score = msg.substring(comma + 1).toInt();
            Serial.print("→ ALERT Score:");
            Serial.println(score);
            
            if (score < 50) {
                setLED(LED_RED);
                playBuzzer(BUZZ_URGENT);
            } else if (score < 75) {
                setLED(LED_ORANGE);
                playBuzzer(BUZZ_WARNING);
            }
        }
    }
    else if (msg.startsWith("PRIVACY:")) {
        if (msg.indexOf("ENABLE") > 0) {
            privacyMode = true;
            setLED(LED_PURPLE);
            Serial.println("→ PRIVACY ON");
        } else {
            privacyMode = false;
            setLED(LED_GREEN);
            Serial.println("→ PRIVACY OFF");
        }
    }
    else if (msg.startsWith("BREAK:")) {
        if (msg.indexOf("START") > 0) {
            breakMode = true;
            setLED(LED_CYAN);
            playBuzzer(BUZZ_BREAK_REMINDER);
            Serial.println("→ BREAK START");
        } else {
            breakMode = false;
            setLED(LED_GREEN);
            Serial.println("→ BREAK END");
        }
    }
    else if (msg.startsWith("SNOOZE:")) {
        if (msg.indexOf("ENABLE") > 0) {
            snoozeMode = true;
            setLED(LED_YELLOW);
            Serial.println("→ SNOOZE ON");
        } else {
            snoozeMode = false;
            setLED(LED_GREEN);
            Serial.println("→ SNOOZE OFF");
        }
    }
    else if (msg == "HB") {
        // Silent heartbeat
    }
    else {
        Serial.print("→ UNKNOWN: ");
        Serial.println(msg);
    }
}