#include <PID_v1.h>

// Motor Control Pins
#define MOTOR_PWM 9  // PWM output (ENA - controls speed)
#define MOTOR_IN1 5  // Motor Direction (IN1)
#define MOTOR_IN2 6  // Motor Direction (IN2)

// Encoder Pins
#define ENCODER_A 2  // Encoder Signal A
#define ENCODER_B 3  // Encoder Signal B

// PID Variables
double setpoint = 50;  // Desired RPM
double input, output;
double Kp = 1.665, Ki = 1.328, Kd = 0.52361;  // PID tuning parameters

// Create PID Controller
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Encoder Variables
volatile int encoderCount = 0;
unsigned long lastTime = 0;
double rpm = 0;
const int PPR = 540;  // Pulses per revolution for your encoder

// Moving Average Buffer for RPM Smoothing
double rpmBuffer[5] = {0};  
int bufferIndex = 0;

// Interrupt Service Routine for Encoder
void encoderISR() {
    encoderCount++;  // Increment count every time encoder A changes
}

// Function to Calculate RPM with Moving Average Filtering
void calculateRPM() {
    unsigned long currentTime = millis();
    double elapsedTime = (currentTime - lastTime) / 1000.0;  // Convert ms to seconds

    if (elapsedTime > 0 && encoderCount > 0) {  
        double newRPM = (encoderCount * 60.0) / (PPR * elapsedTime);  // Corrected PPR = 540

        // Apply Moving Average Filtering
        rpmBuffer[bufferIndex] = newRPM;
        bufferIndex = (bufferIndex + 1) % 5;  // Keep buffer index in range
        
        double sum = 0;
        for (int i = 0; i < 5; i++) sum += rpmBuffer[i];
        rpm = sum / 5;  // Compute average
    } else {
        rpm = 0;
    }

    encoderCount = 0;  // Reset count after reading
    lastTime = currentTime;
}

void setup() {
    Serial.begin(2000000);

    // Setup Encoder Interrupt
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

    // Setup Motor Control
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);

    // Initialize PID
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(0, 255);  // PWM range
}

// Variables to control update timing
unsigned long lastPIDUpdate = 0;
unsigned long lastPrint = 0;

void loop() {
    unsigned long now = millis();

    // Update setpoint with smooth sinusoidal reference (oscillates between 10-90 RPM)
    setpoint = 50 + 40 * sin(now / 5000.0);

    // Update RPM every 200ms
    if (now - lastTime >= 200) {
        calculateRPM();
        input = rpm;
    }

    // Compute PID every 100ms
    if (now - lastPIDUpdate >= 100) {
        motorPID.Compute();
        lastPIDUpdate = now;
    }

    // Apply PID output with smooth motor control
    if (output > 0) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, output);
    } else if (output < 0) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        analogWrite(MOTOR_PWM, abs(output));
    } else {
        // Stop motor smoothly
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, 0);
    }

    // Print Debug Info Every 300ms
    if (now - lastPrint >= 300) {
        Serial.print("Setpoint: ");
        Serial.print(setpoint);
        Serial.print(" RPM | Measured: ");
        Serial.print(rpm);
        Serial.print(" RPM | PWM: ");
        Serial.println(output);
        lastPrint = now;
    }
}
