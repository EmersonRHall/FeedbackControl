#include <PID_v1.h>

// Motor Control Pins
#define MOTOR_PWM 9  // PWM output (ENA - controls speed)
#define MOTOR_IN1 5  // Motor Direction (IN1)
#define MOTOR_IN2 6  // Motor Direction (IN2)

// Encoder Pins
#define ENCODER_A 2  // Encoder Signal A
#define ENCODER_B 3  // Encoder Signal B

// PID Variables
double setpoint = 0;  
double input, output;
double Kp = 1.665, Ki = 1.328, Kd = 0.52361;

// Motor Specifications
const double MAX_SPEED = 130;  
double high_speed = 1.5 * MAX_SPEED;
double low_speed = 0.5 * MAX_SPEED;

// Back-Calculation Anti-Windup Parameter
double Tt = Ki;  // Suggested tracking time constant Tt = Ti

// Create PID Controller
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Encoder Variables
volatile int encoderCount = 0;
unsigned long lastTime = 0;
double rpm = 0;
const int PPR = 540;

// Moving Average Buffer for RPM Smoothing
double rpmBuffer[5] = {0};  
int bufferIndex = 0;

// Timing Variables for Step Trajectory
unsigned long phaseStartTime = 0;
int phase = 0;  
unsigned long lastPrint = 0;  
unsigned long lastPIDUpdate = 0;

// Interrupt Service Routine for Encoder
void encoderISR() {
    encoderCount++;
}

// Function to Calculate RPM
void calculateRPM() {
    unsigned long currentTime = millis();
    double elapsedTime = (currentTime - lastTime) / 1000.0;

    if (elapsedTime > 0 && encoderCount > 0) {  
        double newRPM = (encoderCount * 60.0) / (PPR * elapsedTime);  

        rpmBuffer[bufferIndex] = newRPM;
        bufferIndex = (bufferIndex + 1) % 5;  
        
        double sum = 0;
        for (int i = 0; i < 5; i++) sum += rpmBuffer[i];
        rpm = sum / 5;  
    } else {
        rpm = 0;
    }

    encoderCount = 0;
    lastTime = currentTime;
}

// Function to Update the Reference Trajectory
void updateSetpoint() {
    unsigned long now = millis();
    if (phase == 0) {  
        setpoint = 0;
        Serial.println("Phase 0: Motor stopped.");
        phaseStartTime = now;
        phase = 1;
    } 
    else if (phase == 1 && now - phaseStartTime >= 2000) {  
        setpoint = high_speed;
        Serial.println("Phase 1: Increasing to 1.5x max speed.");
        phaseStartTime = now;
        phase = 2;
    } 
    else if (phase == 2 && now - phaseStartTime >= 12000) {  
        setpoint = low_speed;
        Serial.println("Phase 2: Decreasing to 0.5x max speed.");
        phaseStartTime = now;
        phase = 3;
    } 
    else if (phase == 3) {  
        setpoint = low_speed;
    }
}

void setup() {
    Serial.begin(2000000);
    delay(1000);  

    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);

    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(0, 255);  

    Serial.println("Initializing...");
    phaseStartTime = millis();
}

void loop() {
    unsigned long now = millis();

    updateSetpoint();

    if (now - lastTime >= 200) {
        calculateRPM();
        input = rpm;
    }

    if (now - lastPIDUpdate >= 100) {
        motorPID.Compute();

        // Store the computed (unsaturated) output
        double unsat_output = output;

        // Apply actuator saturation
        if (output > 255) {
            output = 255;
        } else if (output < 0) {
            output = 0;
        }

        // Correct the integral action using back-calculation
        static double integral_term = 0;
        double anti_windup_correction = (unsat_output - output) / Tt;
        integral_term += anti_windup_correction;

        // Apply correction to the PID input
        input += integral_term;

        lastPIDUpdate = now;
    }

    // Apply the adjusted PID output
    if (output > 0) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, output);
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, 0);
    }

    if (now - lastPrint >= 300) {
        Serial.print("Phase: ");
        Serial.print(phase);
        Serial.print(" | Setpoint: ");
        Serial.print(setpoint);
        Serial.print(" RPM | Measured: ");
        Serial.print(rpm);
        Serial.print(" RPM | PWM: ");
        Serial.println(output);
        lastPrint = now;
    }
}

