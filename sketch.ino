//motor actuation code from server side remaining 
// odomety code from server side remaining for reciving payload 
//add redenduncy for wifi and websocket connection

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <Adafruit_Sensor.h>

// ---------------------
// Odometry / Position Tracking Variables
// ---------------------
float x = 0, y = 0; // X, Y coordinates in cm
float heading = 0;           // Relative heading in degrees
float headingOffset = 0;     // Initial reading offset
bool headingInitialized = false;

// ---------------------
// Function: resetEncoderAndPosition
// ---------------------
void resetEncoderAndPosition() {
    noInterrupts();
    encoderCountA = 0;
    encoderCountB = 0;
    interrupts();
    
    x = 0;
    y = 0;
    heading = 0;
    headingInitialized = false;
    
    Serial.println("Encoder and Position Reset");
}
#include <Adafruit_HMC5883_U.h>

// ---------------------
// Encoder Pin Definitions (Left Encoder = 34, Right Encoder = 35)
// ---------------------
#define ENCODER_A 34  // Left wheel encoder
#define ENCODER_B 35  // Right wheel encoder

volatile long encoderCountA = 0;  // Left encoder pulse count
volatile long encoderCountB = 0;  // Right encoder pulse count

// ---------------------
// Wheel & Encoder Constants
// ---------------------
float wheelDiameter = 6.5;                           // in cm
float wheelCircumference = PI * wheelDiameter;       // cm per revolution
const int encoderPulsesPerRev = 20;                  // Pulses per revolution

// ---------------------
// Encoder Interrupt Service Routines
// ---------------------
void IRAM_ATTR encoderISR_A() {
    encoderCountA++; // Left encoder: count pulses
}

void IRAM_ATTR encoderISR_B() {
    encoderCountB++; // Right encoder: count pulses
}

// ---------------------
// Function: updatePosition
// ---------------------
void updatePosition() {
    static unsigned long lastUpdateTime = millis();
    unsigned long currentTime = millis();

    if (currentTime - lastUpdateTime >= 100) { // Update every 100ms
        noInterrupts();
        long pulsesLeft = encoderCountA;
        long pulsesRight = encoderCountB;
        encoderCountA = 0;
        encoderCountB = 0;
        interrupts();

        float distanceLeft = (pulsesLeft / (float)encoderPulsesPerRev) * wheelCircumference;
        float distanceRight = (pulsesRight / (float)encoderPulsesPerRev) * wheelCircumference;

        // Average distance traveled by both wheels
        float distanceTraveled = (distanceLeft + distanceRight) / 2.0;

        // Convert heading angle to radians
        float headingRad = heading * (PI / 180.0);

        // Only update X, Y if moving forward or backward
        if (movementDirection != 0) {
            x += movementDirection * distanceTraveled * cos(headingRad);
            y += movementDirection * distanceTraveled * sin(headingRad);
        }

        // Debugging Output
        Serial.printf("Heading: %.2f° | Distance: %.2f cm | X: %.2f cm | Y: %.2f cm\n", heading, distanceTraveled, x, y);

        lastUpdateTime = currentTime;
    }
}

// ---------------------
// Function: resetEncoderAndPosition
// ---------------------

WebSocketsClient webSocket;

// WiFi
const char* ssid = "Rishi17";
const char* password = "11111111";

// Use this for SSL
#define USE_SSL true

// ---------------------
// Compass Sensor Setup
// ---------------------
Adafruit_HMC5883_Unified hmc5883 = Adafruit_HMC5883_Unified(12345);

// ---------------------
// Odometry / Position Tracking Variables
// ---------------------
float heading = 0;           // Relative heading in degrees
float headingOffset = 0;     // Initial reading offset
bool headingInitialized = false;

// ---------------------
// Motor Driver Pin Connections
// ---------------------
int enA = 4, in1 = 18, in2 = 19;
int enB = 5, in3 = 25, in4 = 26;

// ---------------------
// Motor Speed
// ---------------------
int motorSpeed = 200;

// ---------------------
// Movement Direction
// ---------------------
int movementDirection = 0; // +1 for forward, -1 for backward, 0 during turns

// ---------------------
// Movement Functions
// ---------------------
void moveForward() {
    Serial.println("Moving Forward...");
    movementDirection = 1;
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
}

void moveBackward() {
    Serial.println("Moving Backward...");
    movementDirection = -1;
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
}

void turnLeft() {
    Serial.println("Turning Left...");
    movementDirection = 0;  // No linear update during turn
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
}

void turnRight() {
    Serial.println("Turning Right...");
    movementDirection = 0;
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
}

void stopMotor() {
    Serial.println("Stopping...");
    movementDirection = 0;
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
}

// ---------------------
// Function: updateHeading
// ---------------------
void updateHeading() {
        sensors_event_t event;
        hmc5883.getEvent(&event);
        
        float rawHeading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
        if (rawHeading < 0) rawHeading += 360;
        
        if (!headingInitialized) {
                headingOffset = rawHeading;  // Set initial offset
                heading = 0;                 // Initial relative heading = 0°
                headingInitialized = true;
        } else {
                heading = rawHeading - headingOffset;
                if (heading < 0) heading += 360;
        }
}

void setup() {
        Serial.begin(115200);
        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                Serial.print(".");
        }

        Serial.println("\n✅ WiFi Connected");

        // Use beginSSL if wss://
        webSocket.beginSSL("esp32-websocket-server33.onrender.com", 443, "/ws");

        webSocket.onEvent(webSocketEvent);
        webSocket.setReconnectInterval(5000);

        // Compass Initialization
        if (!hmc5883.begin()) {
                Serial.println("[ERROR] Compass not detected!");
                while (1);
        }

        // Motor Pins Setup
        pinMode(enA, OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pinMode(enB, OUTPUT);
        pinMode(in3, OUTPUT);
        pinMode(in4, OUTPUT);

        stopMotor(); // Ensure motors are stopped at startup
}

void loop() {
        webSocket.loop();

        static unsigned long lastSent = 0;
        if (millis() - lastSent > 1000) {
                updateHeading(); // Update compass heading
                sendData();
                lastSent = millis();
        }
}

void sendData() {
    StaticJsonDocument<200> doc;
    doc["event"] = "update";

    JsonObject payload = doc.createNestedObject("payload");
    payload["heading"] = 90; // Static heading value as per the format
    payload["X"] = x;        // Send X coordinate from odometry
    payload["Y"] = y;        // Send Y coordinate from odometry

    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
    Serial.println("📤 Sent: " + json);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.println("✅ WebSocket Connected");
            break;

        case WStype_DISCONNECTED:
            Serial.println("❌ WebSocket Disconnected");
            break;

        case WStype_TEXT: {
            Serial.print("📩 Received: ");
            Serial.println((char*)payload);

            // Parse the received JSON payload
            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, payload);

            if (error) {
                Serial.print("❌ JSON Parse Error: ");
                Serial.println(error.c_str());
                return;
            }

            // Check if the event is "command"
            const char* event = doc["event"];
            if (strcmp(event, "command") == 0) {
                JsonObject command = doc["payload"];

                // Execute the corresponding command based on the flags
                if (command["forward"] == true) {
                    moveForward();
                } else if (command["reverse"] == true) {
                    moveBackward();
                } else if (command["left"] == true) {
                    turnLeft();
                } else if (command["right"] == true) {
                    turnRight();
                } else if (command["stop"] == true) {
                    stopMotor();
                } else {
                    Serial.println("❌ Invalid Command");
                }
            } else {
                Serial.println("❌ Unsupported Event");
            }
            break;
        }

        default:
            break;
    }
}
