/**
 * @file SerialProtocol.h
 * @brief Serial Communication Protocol (ESP32-CAM Version)
 * @author Smart Posture Coach Team
 * @date 2025-11-17
 * @updated 2025-11-17 03:39:06 UTC
 * @user hyoono
 * 
 * ESP32 version - uses Serial2 for Arduino communication
 */

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <HardwareSerial.h>  // ADD THIS for ESP32

// ========== PROTOCOL CONSTANTS ==========
#define SERIAL_BAUD_RATE 115200
#define MAX_MESSAGE_LENGTH 64
#define MESSAGE_START_CHAR '<'
#define MESSAGE_END_CHAR '>'
#define MESSAGE_DELIMITER ':'
#define PARAM_DELIMITER ','
#define LINE_DELIMITER '|'

// Message timeout
#define MESSAGE_TIMEOUT_MS 100
#define HEARTBEAT_INTERVAL_MS 5000

// ========== ESP32-CAM UART PINS ==========
// Serial2: RX=GPIO13, TX=GPIO12
#define ARDUINO_RX_PIN 13  // ESP32 RX <- Arduino TX
#define ARDUINO_TX_PIN 12  // ESP32 TX -> Arduino RX

// ========== MESSAGE TYPES ==========
enum MessageType : uint8_t {
    MSG_UNKNOWN = 0,
    MSG_LED,
    MSG_BUZZER,
    MSG_DISPLAY,
    MSG_BREAK,
    MSG_ALERT,
    MSG_STATUS,
    MSG_SNOOZE,
    MSG_PRIVACY,
    MSG_DATA,
    MSG_ACK,
    MSG_READY,
    MSG_HEARTBEAT
};

// ========== LED COLORS ==========
enum LedColor : uint8_t {
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

// ========== BUZZER PATTERNS ==========
enum BuzzerPatternType : uint8_t {
    BUZZ_URGENT = 0,
    BUZZ_WARNING,
    BUZZ_DISTANCE,
    BUZZ_SUCCESS,
    BUZZ_BREAK_REMINDER,
    BUZZ_BUTTON_CLICK,
    BUZZ_CANCEL
};

// ========== MESSAGE STRUCTURE ==========
struct SerialMessage {
    MessageType type;
    char command[12];
    char params[3][24];
    uint8_t paramCount;
    bool isValid;
};

// ========== SERIAL PROTOCOL CLASS (ESP32 VERSION) ==========
class SerialProtocol {
private:
    HardwareSerial* serial;  // Use Serial2 on ESP32
    char receiveBuffer[MAX_MESSAGE_LENGTH];
    uint8_t bufferIndex;
    bool receivingMessage;
    uint32_t lastHeartbeat;
    uint32_t lastReceiveTime;
    
    // Helper functions
    void clearBuffer();
    void parseMessage(const char* message, SerialMessage* msg);
    MessageType getMessageType(const char* command);
    
public:
    SerialProtocol();
    
    // Initialize with ESP32 Serial2
    void begin(uint32_t baudRate = SERIAL_BAUD_RATE);
    
    // Receive messages
    bool available();
    bool readMessage(SerialMessage* msg);
    
    // Send commands to Arduino
    void sendLedCommand(LedColor color);
    void sendBuzzerCommand(BuzzerPatternType pattern);
    void sendDisplayCommand(const char* text);
    void sendBreakCommand(bool start);
    void sendAlertCommand(const char* type, int value);
    void sendStatusRequest();
    void sendSnoozeCommand(bool enable);
    void sendPrivacyCommand(bool enable);
    
    // Utility
    bool checkTimeout();
    void printMessage(const SerialMessage* msg);
};

// ========== COLOR NAME TO ENUM ==========
LedColor stringToLedColor(const char* colorName);
const char* ledColorToString(LedColor color);

// ========== PATTERN NAME TO ENUM ==========
BuzzerPatternType stringToBuzzerPattern(const char* patternName);
const char* buzzerPatternToString(BuzzerPatternType pattern);

#endif // SERIAL_PROTOCOL_H