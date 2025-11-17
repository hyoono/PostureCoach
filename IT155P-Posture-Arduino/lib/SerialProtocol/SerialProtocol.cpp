/**
 * @file SerialProtocol.cpp
 * @brief Serial Communication Protocol Implementation (RAM Optimized)
 * @author Smart Posture Coach Team
 * @date 2025-11-17
 * @updated 2025-11-17 02:21:54 UTC
 * @user hyoono
 */

#include "SerialProtocol.h"

// ========== CONSTRUCTOR ==========
SerialProtocol::SerialProtocol() {
    bufferIndex = 0;
    receivingMessage = false;
    lastHeartbeat = 0;
    lastReceiveTime = 0;
    memset(receiveBuffer, 0, MAX_MESSAGE_LENGTH);
}

// ========== PUBLIC METHODS ==========

void SerialProtocol::begin(uint32_t baudRate) {
    Serial.begin(baudRate);
    // Don't wait for Serial on Uno (no native USB)
    delay(50);
    clearBuffer();
    lastHeartbeat = millis();
}

bool SerialProtocol::available() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
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

void SerialProtocol::sendDistanceData(int16_t distance) {
    Serial.print(MESSAGE_START_CHAR);
    Serial.print(F("DATA:DIST,"));
    Serial.print(distance);
    Serial.println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendButtonPress(uint8_t buttonNumber, bool isDouble) {
    Serial.print(MESSAGE_START_CHAR);
    if (isDouble) {
        Serial.print(F("DATA:BTNDBL,"));
    } else {
        Serial.print(F("DATA:BTN,"));
    }
    Serial.print(buttonNumber);
    Serial.println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendStatus(bool privacy, bool snooze, bool inBreak) {
    Serial.print(MESSAGE_START_CHAR);
    Serial.print(F("STATUS:"));
    Serial.print(privacy ? '1' : '0');
    Serial.print(PARAM_DELIMITER);
    Serial.print(snooze ? '1' : '0');
    Serial.print(PARAM_DELIMITER);
    Serial.print(inBreak ? '1' : '0');
    Serial.println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendAck(bool success, const char* errorMsg) {
    Serial.print(MESSAGE_START_CHAR);
    Serial.print(F("ACK:"));
    if (success) {
        Serial.print(F("OK"));
    } else {
        Serial.print(F("ERROR"));
        if (errorMsg) {
            Serial.print(PARAM_DELIMITER);
            Serial.print(errorMsg);
        }
    }
    Serial.println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendReady() {
    Serial.print(MESSAGE_START_CHAR);
    Serial.print(F("READY"));
    Serial.println(MESSAGE_END_CHAR);
}

void SerialProtocol::sendHeartbeat() {
    uint32_t currentTime = millis();
    if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        Serial.print(MESSAGE_START_CHAR);
        Serial.print(F("HB"));
        Serial.println(MESSAGE_END_CHAR);
        lastHeartbeat = currentTime;
    }
}

bool SerialProtocol::checkTimeout() {
    return receivingMessage && (millis() - lastReceiveTime > MESSAGE_TIMEOUT_MS);
}

void SerialProtocol::printMessage(const SerialMessage* msg) {
    if (!msg || !msg->isValid) {
        Serial.println(F("[SERIAL] Invalid message"));
        return;
    }
    
    Serial.print(F("[SERIAL] Type: "));
    Serial.print(msg->type);
    Serial.print(F(", Command: "));
    Serial.print(msg->command);
    Serial.print(F(", Params: "));
    for (uint8_t i = 0; i < msg->paramCount; i++) {
        Serial.print(msg->params[i]);
        if (i < msg->paramCount - 1) Serial.print(F(", "));
    }
    Serial.println();
}

// ========== PRIVATE METHODS ==========

void SerialProtocol::clearBuffer() {
    bufferIndex = 0;
    memset(receiveBuffer, 0, MAX_MESSAGE_LENGTH);
}

void SerialProtocol::parseMessage(const char* message, SerialMessage* msg) {
    if (!message || !msg) return;
    
    // Initialize message
    msg->isValid = false;
    msg->paramCount = 0;
    msg->type = MSG_UNKNOWN;
    memset(msg->command, 0, sizeof(msg->command));
    for (uint8_t i = 0; i < 3; i++) {  // Changed from 5 to 3
        memset(msg->params[i], 0, sizeof(msg->params[i]));
    }
    
    // Copy message for parsing
    char buffer[MAX_MESSAGE_LENGTH];
    strncpy(buffer, message, MAX_MESSAGE_LENGTH - 1);
    buffer[MAX_MESSAGE_LENGTH - 1] = '\0';
    
    // Parse command
    char* token = strtok(buffer, ":");
    if (!token) return;
    
    strncpy(msg->command, token, sizeof(msg->command) - 1);
    msg->type = getMessageType(msg->command);
    
    // Parse parameters (max 3 now)
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