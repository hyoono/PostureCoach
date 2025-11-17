/**
 * @file SerialProtocol.h
 * @brief Serial Communication Protocol for Arduino-ESP32 Communication
 * @author Smart Posture Coach Team
 * @date 2025-11-17
 * @updated 2025-11-17 02:21:54 UTC
 * @user hyoono
 * 
 * OPTIMIZED FOR LOW RAM: Reduced buffer sizes to work with OLED
 */

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

// ========== PROTOCOL CONSTANTS (OPTIMIZED FOR RAM) ==========
#define SERIAL_BAUD_RATE 115200
#define MAX_MESSAGE_LENGTH 64      // Reduced from 128
#define MESSAGE_START_CHAR '<'
#define MESSAGE_END_CHAR '>'
#define MESSAGE_DELIMITER ':'
#define PARAM_DELIMITER ','
#define LINE_DELIMITER '|'

// Message timeout
#define MESSAGE_TIMEOUT_MS 100
#define HEARTBEAT_INTERVAL_MS 5000

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

// ========== MESSAGE STRUCTURE (OPTIMIZED) ==========
struct SerialMessage {
    MessageType type;
    char command[12];        // Reduced from 16
    char params[3][24];      // Reduced from 5x32 to 3x24
    uint8_t paramCount;
    bool isValid;
};

// ========== SERIAL PROTOCOL CLASS ==========
class SerialProtocol {
private:
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
    
    // Initialize serial communication
    void begin(uint32_t baudRate = SERIAL_BAUD_RATE);
    
    // Receive messages
    bool available();
    bool readMessage(SerialMessage* msg);
    
    // Send messages to ESP32
    void sendDistanceData(int16_t distance);
    void sendButtonPress(uint8_t buttonNumber, bool isDouble = false);
    void sendStatus(bool privacy, bool snooze, bool inBreak);
    void sendAck(bool success, const char* errorMsg = nullptr);
    void sendReady();
    void sendHeartbeat();
    
    // Utility
    bool checkTimeout();
    void printMessage(const SerialMessage* msg);
};

// ========== COLOR NAME TO ENUM ==========
LedColor stringToLedColor(const char* colorName);

// ========== PATTERN NAME TO ENUM ==========
BuzzerPatternType stringToBuzzerPattern(const char* patternName);

#endif // SERIAL_PROTOCOL_H