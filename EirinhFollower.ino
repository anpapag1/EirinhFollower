#include <QTRSensors.h>
#include <pidautotuner.h>
#include <EEPROM.h>

// Button and motor pin definitions
#define LED_BUILTIN 2
#define BUTTON_PIN 17
#define AIN1 19
#define AIN2 18
#define BIN1 21
#define BIN2 22
#define SLEEP 23

// Sensor configuration
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint8_t sensorPins[SensorCount] = {13, 27, 26, 25, 33, 32, 35, 34};
int blackLineValue = 4090;

// PID variables
double Kp = 10;
double Ki = 0;
double Kd = 100;
int maxSpeed = 255;
int baseSpeed = 50;

PIDAutotuner tuner = PIDAutotuner();

// Robot state variables
uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
bool debugMode = false;

// EEPROM storage addresses
#define EEPROM_ADDR_KP 0
#define EEPROM_ADDR_KI 4
#define EEPROM_ADDR_KD 8
bool calibrationLoaded = false;

void setup() {
    // Initialize pins
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED to indicate setup is running
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(SLEEP, OUTPUT);

    // Configure sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorPins, SensorCount);

    tuner.setTargetInputValue(35);
    tuner.setLoopInterval(1000); // Set loop interval to 1000 microseconds (1 ms)
    tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

    // Load calibration data from EEPROM
    loadCalibration();

    Serial.begin(500000);
    delay(500);
    processCommand("h"); // Display help on startup
    digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED to indicate setup is complete
}

void loop() {
    // Process serial commands
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }

    // read button state and start or calibrate PID if long pressed
    if (digitalRead(BUTTON_PIN) == LOW) {
        unsigned long buttonPressTime = millis();
        while (digitalRead(BUTTON_PIN) == LOW) {}
        if (millis() - buttonPressTime > 2000) {
            startupSequence(3,3);
            calibratePid();
        }
        else {
            startupSequence(3,3);
            digitalWrite(SLEEP, HIGH);
        }
    }

    position = readLine(position, blackLineValue);
    PID_Linefollow(position);
    if (debugMode) Serial.println("");
}

void processCommand(String command) {
    // Process incoming commands
    if (command.length() < 1) return;

    char cmd = command.charAt(0);
    // Serial.println(command);
    float value = 0;

    if (command.length() > 1) {
        value = command.substring(1).toFloat();
    }

    switch (cmd) {
        case 'p': // Set proportional gain
        if (value) Kp = value; // Only update if a valid value is provided
        Serial.println("Kp set to " + String(Kp, 4));
        break;
        case 'i': // Set integral gain
        if (value) Ki = value;
        Serial.println("Ki set to " + String(Ki, 4));
        break;
        case 'd': // Set derivative gain
        if (value) Kd = value;
        Serial.println("Kd set to " + String(Kd, 4));
        break;
        case 's': // Set base speed
        if (value) baseSpeed = (int)value;
        Serial.println("Base speed set to " + String(baseSpeed));
        break;
        case 't': // Toggle robot state
        digitalWrite(SLEEP, !digitalRead(SLEEP));
        Serial.println("Robot " + String(digitalRead(SLEEP) ? "enabled" : "disabled"));
        break;
        case 'g': // Toggle debug mode
        debugMode = !debugMode;
        Serial.println("Debug mode " + String(debugMode ? "enabled" : "disabled"));
        break;
        case 'c': // Calibrate PID Values
        calibratePid();
        break;
        case 'r': // Reset calibration to defaults
        resetCalibration();
        break;
        case 'h': // Display help
        Serial.println("Commands:");
        Serial.println("  p <value> - Set proportional gain");
        Serial.println("  i <value> - Set integral gain");
        Serial.println("  d <value> - Set derivative gain");
        Serial.println("  s <value> - Set base speed");
        Serial.println("  t - Toggle robot state");
        Serial.println("  g - Toggle debug mode");
        Serial.println("  c - Calibrate PID values");
        Serial.println("  r - Reset calibration to defaults");
        Serial.println("  h - Display help");
        break;
    }
}

void PID_Linefollow(int position) {
    error = SensorCount / 2 * 10 - position;

    // Calculate PID values
    P = error;
    I = I + error;
    D = error - previousError;

    float PIDvalue = Kp * P + Ki * I + Kd * D;
    previousError = error;

    motor_drive(PIDvalue);
}

int readLine(int lastposition, int blackLineValue) {
    // Read sensor values and calculate position
    qtr.read(sensorValues);

    int position = 0;
    int count = 0;
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > blackLineValue) {
        count++;
        position += i * 10;
        }
    }

    if (count == SensorCount) {
        digitalWrite(SLEEP, LOW);
        position = 0;
    } else if (count != 0) {
        position = position / count;
    } else {
        if (lastposition > SensorCount / 2) {
        position = SensorCount * 10;
        } else {
        position = 0;
        }
    }
    // debug
    if (debugMode) {
        Serial.print("\t");
        for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > blackLineValue) {
            Serial.print("▢");
        } else {
            Serial.print("_");
        }
        }
        Serial.print("\tposition: ");
        Serial.print(position);
    }

    return position;
}

void motor_drive(int PIDvalue) {
    // Calculate motor speeds based on PID output
    int right = constrain(baseSpeed + PIDvalue, -255, 255);
    int left = constrain(baseSpeed - PIDvalue, -255, 255);

    // Control motor movement
    if (right > 0) {
    analogWrite(AIN1, right);
    analogWrite(AIN2, 0);
    } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, abs(right));
    }

    if (left > 0) {
    analogWrite(BIN1, left);
    analogWrite(BIN2, 0);
    } else {
    analogWrite(BIN1, 0);
    analogWrite(BIN2, abs(left));
    }
    if (debugMode) Serial.print("\tleft:" + String(left) + "\tright: " + String(right));
}

void calibratePid(){
    Serial.println("Starting PID autotuning...");
    tuner.setOutputRange(-maxSpeed/2, maxSpeed/2); // Set output range to match motor control input
    tuner.startTuningLoop(micros());
    digitalWrite(SLEEP, HIGH);
    long microseconds;
    while (!tuner.isFinished()) {

        // This loop must run at the same speed as the PID control loop being tuned
        long prevMicroseconds = microseconds;
        microseconds = micros();

        // Get input value here (temperature, encoder position, velocity, etc)
        double input = readLine(position, 4090);

        // Call tunePID() with the input value and current time in microseconds
        double output = tuner.tunePID(input, microseconds);

        // Set the output - tunePid() will return values within the range configured
        // by setOutputRange(). Don't change the value or the tuning results will be
        // incorrect.
        motor_drive(output);
        // This loop must run at the same speed as the PID control loop being tuned
        while (micros() - microseconds < 1) delayMicroseconds(1);
        if (digitalRead(SLEEP) == LOW) {
            Serial.println("Tuning stopped - robot disabled");
            return;
        }
    }

    // Turn the output off here.
    digitalWrite(SLEEP, LOW);

    // Blink built-in LED to indicate tuning is complete
    for (int i = 0; i < 2; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

    // Get PID gains - set your PID controller's gains to these
    double kp = tuner.getKp();
    double ki = tuner.getKi();
    double kd = tuner.getKd();
    Serial.println("Tuning complete! Kp: " + String(kp, 4) + " Ki: " + String(ki, 4) + " Kd: " + String(kd, 4));

    // Save calibration data to EEPROM
    saveCalibration();
}

void saveCalibration() {
    EEPROM.put(EEPROM_ADDR_KP, Kp);
    EEPROM.put(EEPROM_ADDR_KI, Ki);
    EEPROM.put(EEPROM_ADDR_KD, Kd);
    Serial.println("Calibration saved to EEPROM");
}

void loadCalibration() {
    if (EEPROM.read(EEPROM_ADDR_CALIBRATED) == 1) {
        EEPROM.get(EEPROM_ADDR_KP, Kp);
        EEPROM.get(EEPROM_ADDR_KI, Ki);
        EEPROM.get(EEPROM_ADDR_KD, Kd);

        Serial.println("Calibration loaded from EEPROM:");
        Serial.println("  Kp: " + String(Kp, 4));
        Serial.println("  Ki: " + String(Ki, 4));
        Serial.println("  Kd: " + String(Kd, 4));
    } else {
        Serial.println("No calibration data found in EEPROM, using defaults");
    }
}

void resetCalibration() {
    Kp = 10;
    Ki = 0;
    Kd = 100;
    Serial.println("Calibration reset to defaults");
}

void startupSequence(int blinks,int sec) {
    Serial.println("Starting in " + String(sec) + " seconds...");
    int delays = (sec * 1000) / (blinks * 4); // Calculate delay based on number of blinks and total time
    unsigned long now = millis();
    // Blink built-in LED to indicate startup in different delays according to the time until start
    for (int i = 0; i <= blinks; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(i*delays);
        digitalWrite(LED_BUILTIN, LOW);
        delay(i*delays);
    }
    Serial.println("Go!");
}