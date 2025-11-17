/**
 * @file SerialProtocol.cpp
 * @brief Serial Communication Protocol Implementation (ESP32 Version)
 * @author Smart Posture Coach Team
 * @date 2025-11-17
 * @updated 2025-11-17 03:39:06 UTC
 * @user hyoono
 * 
 * ESP32-CAM version using Serial2 (GPIO12=TX, GPIO13=RX)
 */

#include "SerialProtocol.h"
#include <HardwareSerial.h>  // ADD THIS

// ========== CONSTRUCTOR ==========
SerialProtocol::SerialProtocol() {
    serial = &Serial2;  // Use Serial2 on ESP32
    bufferIndex = 0;
    receivingMessage = false;
    lastHeartbeat = 0;
    lastReceiveTime = 0;
    memset(receiveBuffer, 0, MAX_MESSAGE_LENGTH);
}

// ========== PUBLIC METHODS ==========

void SerialProtocol::begin(uint32_t baudRate) {
    // Initialize Serial2 with specified pins
    serial->begin(baudRate, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);
    delay(100);
    clearBuffer();
    lastHeartbeat = millis();
    
    // Use Serial for debug output (ESP32 has both Serial and Serial2)
    if (Serial) {  // Check if Serial is available
        Serial.printf("[SerialProtocol] Initialized on Serial2 (RX=%d, TX=%d) at %lu baud\n", 
            ARDUINO_RX_PIN, ARDUINO_TX_PIN, (unsigned long)baudRate);
    }
}

bool SerialProtocol::available() {
    while (serial->available() > 0) {
        char c = serial->read();
        
        if (c == MESSAGE_START_CHAR) {
            receivingMessage = true;
            bufferIndex = 0;
            clearBuffer();
            lastReceiveTime = millis();
        } else if (c == MESSAGE_END_CHAR && receivingMessage) {
            receiveBuffer[bufferIndex] = '\0';
            receivingMessage = false;
            return true;
        } else if (receivingMessage) {
            if (bufferIndex < MAX_MESSAGE_LENGTH - 1) {
                receiveBuffer[bufferIndex++] = c;
            } else {
                clearBuffer();
                receivingMessage = false;
            }
        }
        
        if (receivingMessage && (millis() - lastReceiveTime > MESSAGE_TIMEOUT_MS)) {
            clearBuffer();
            receivingMessage = false;
        }
    }
    
    return false;
}

bool SerialProtocol::readMessage(SerialMessage* msg) {
    if (!msg) return false;
    
    parseMessage(receiveBuffer, msg);
    clearBuffer();
    
    return msg->isValid;
}

// ========== SEND COMMANDS TO ARDUINO ==========

void SerialProtocol::sendLedCommand(LedColor color) {
    serial->print(MESSAGE_START_CHAR);
    serial->print("LED:");
    serial->print(ledColorToString(color));
    serial->println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendBuzzerCommand(BuzzerPatternType pattern) {
    serial->print(MESSAGE_START_CHAR);
    serial->print("BUZZER:");
    serial->print(buzzerPatternToString(pattern));
    serial->println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendDisplayCommand(const char* text) {
    serial->print(MESSAGE_START_CHAR);
    serial->print("DISPLAY:");
    serial->print(text);
    serial->println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendBreakCommand(bool start) {
    serial->print(MESSAGE_START_CHAR);
    serial->print("BREAK:");
    serial->print(start ? "START" : "END");
    serial->println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendAlertCommand(const char* type, int value) {
    serial->print(MESSAGE_START_CHAR);
    serial->print("ALERT:");
    serial->print(type);
    serial->print(PARAM_DELIMITER);
    serial->print(value);
    serial->println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendStatusRequest() {
    serial->print(MESSAGE_START_CHAR);
    serial->print("STATUS:REQUEST");
    serial->println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendSnoozeCommand(bool enable) {
    serial->print(MESSAGE_START_CHAR);
    serial->print("SNOOZE:");
    serial->print(enable ? "ENABLE" : "DISABLE");
    serial->println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendPrivacyCommand(bool enable) {
    serial->print(MESSAGE_START_CHAR);
    serial->print("PRIVACY:");
    serial->print(enable ? "ENABLE" : "DISABLE");
    serial->println(MESSAGE_END_CHAR);
}

bool SerialProtocol::checkTimeout() {
    return receivingMessage && (millis() - lastReceiveTime > MESSAGE_TIMEOUT_MS);
}

void SerialProtocol::printMessage(const SerialMessage* msg) {
    if (!msg || !msg->isValid) {
        if (Serial) {
            Serial.println("[SerialProtocol] Invalid message");
        }
        return;
    }
    
    if (Serial) {
        Serial.print("[SerialProtocol] Type: ");
        Serial.print(msg->type);
        Serial.print(", Command: ");
        Serial.print(msg->command);
        Serial.print(", Params: ");
        for (uint8_t i = 0; i < msg->paramCount; i++) {
            Serial.print(msg->params[i]);
            if (i < msg->paramCount - 1) Serial.print(", ");
        }
        Serial.println();
    }
}

// ========== PRIVATE METHODS ==========

void SerialProtocol::clearBuffer() {
    bufferIndex = 0;
    memset(receiveBuffer, 0, MAX_MESSAGE_LENGTH);
}

void SerialProtocol::parseMessage(const char* message, SerialMessage* msg) {
    if (!message || !msg) return;
    
    msg->isValid = false;
    msg->paramCount = 0;
    msg->type = MSG_UNKNOWN;
    memset(msg->command, 0, sizeof(msg->command));
    for (uint8_t i = 0; i < 3; i++) {
        memset(msg->params[i], 0, sizeof(msg->params[i]));
    }
    
    char buffer[MAX_MESSAGE_LENGTH];
    strncpy(buffer, message, MAX_MESSAGE_LENGTH - 1);
    buffer[MAX_MESSAGE_LENGTH - 1] = '\0';
    
    char* token = strtok(buffer, ":");
    if (!token) return;
    
    strncpy(msg->command, token, sizeof(msg->command) - 1);
    msg->type = getMessageType(msg->command);
    
    while ((token = strtok(NULL, ",")) != NULL && msg->paramCount < 3) {
        strncpy(msg->params[msg->paramCount], token, sizeof(msg->params[0]) - 1);
        msg->paramCount++;
    }
    
    msg->isValid = true;
}

MessageType SerialProtocol::getMessageType(const char* command) {
    if (strcmp(command, "LED") == 0) return MSG_LED;
    if (strcmp(command, "BUZZER") == 0) return MSG_BUZZER;
    if (strcmp(command, "DISPLAY") == 0) return MSG_DISPLAY;
    if (strcmp(command, "BREAK") == 0) return MSG_BREAK;
    if (strcmp(command, "ALERT") == 0) return MSG_ALERT;
    if (strcmp(command, "STATUS") == 0) return MSG_STATUS;
    if (strcmp(command, "SNOOZE") == 0) return MSG_SNOOZE;
    if (strcmp(command, "PRIVACY") == 0) return MSG_PRIVACY;
    if (strcmp(command, "DATA") == 0) return MSG_DATA;
    if (strcmp(command, "ACK") == 0) return MSG_ACK;
    if (strcmp(command, "READY") == 0) return MSG_READY;
    if (strcmp(command, "HB") == 0) return MSG_HEARTBEAT;
    
    return MSG_UNKNOWN;
}

// ========== UTILITY FUNCTIONS ==========

LedColor stringToLedColor(const char* colorName) {
    if (strcmp(colorName, "OFF") == 0) return LED_OFF;
    if (strcmp(colorName, "RED") == 0) return LED_RED;
    if (strcmp(colorName, "GREEN") == 0) return LED_GREEN;
    if (strcmp(colorName, "BLUE") == 0) return LED_BLUE;
    if (strcmp(colorName, "YELLOW") == 0) return LED_YELLOW;
    if (strcmp(colorName, "ORANGE") == 0) return LED_ORANGE;
    if (strcmp(colorName, "PURPLE") == 0) return LED_PURPLE;
    if (strcmp(colorName, "CYAN") == 0) return LED_CYAN;
    if (strcmp(colorName, "WHITE") == 0) return LED_WHITE;
    return LED_OFF;
}

const char* ledColorToString(LedColor color) {
    switch (color) {
        case LED_OFF: return "OFF";
        case LED_RED: return "RED";
        case LED_GREEN: return "GREEN";
        case LED_BLUE: return "BLUE";
        case LED_YELLOW: return "YELLOW";
        case LED_ORANGE: return "ORANGE";
        case LED_PURPLE: return "PURPLE";
        case LED_CYAN: return "CYAN";
        case LED_WHITE: return "WHITE";
        default: return "OFF";
    }
}

BuzzerPatternType stringToBuzzerPattern(const char* patternName) {
    if (strcmp(patternName, "URGENT") == 0) return BUZZ_URGENT;
    if (strcmp(patternName, "WARNING") == 0) return BUZZ_WARNING;
    if (strcmp(patternName, "DISTANCE") == 0) return BUZZ_DISTANCE;
    if (strcmp(patternName, "SUCCESS") == 0) return BUZZ_SUCCESS;
    if (strcmp(patternName, "BREAK") == 0) return BUZZ_BREAK_REMINDER;
    if (strcmp(patternName, "CLICK") == 0) return BUZZ_BUTTON_CLICK;
    if (strcmp(patternName, "CANCEL") == 0) return BUZZ_CANCEL;
    return BUZZ_SUCCESS;
}

const char* buzzerPatternToString(BuzzerPatternType pattern) {
    switch (pattern) {
        case BUZZ_URGENT: return "URGENT";
        case BUZZ_WARNING: return "WARNING";
        case BUZZ_DISTANCE: return "DISTANCE";
        case BUZZ_SUCCESS: return "SUCCESS";
        case BUZZ_BREAK_REMINDER: return "BREAK";
        case BUZZ_BUTTON_CLICK: return "CLICK";
        case BUZZ_CANCEL: return "CANCEL";
        default: return "SUCCESS";
    }
}