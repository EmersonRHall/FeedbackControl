#include <PID_v1.h>

// Motor Control Pins
#define MOTOR_PWM 9  
#define MOTOR_IN1 5  
#define MOTOR_IN2 6  

// Encoder Pins
#define ENCODER_A 2  
#define ENCODER_B 3  

// PID Variables
double setpoint = 50;  
double input, output;
double Kp = 1.665, Ki = 1.328, Kd = 0.52361;  // PID tuning parameters

// Custom Derivative Filter Variable (N)
double N = 20;  // Change this to test different values (e.g., 5, 10, 15, 20)
double lastDerivative = 0;

// Create PID Controller
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Encoder Variables
volatile int encoderCount = 0;
unsigned long lastTime = 0;
double rpm = 0;
const int PPR = 540;  

// Moving Average Buffer
double rpmBuffer[5] = {0};  
int bufferIndex = 0;

// Interrupt Service Routine
void encoderISR() {
    encoderCount++;
}

// Improved RPM Calculation
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

// Custom Derivative Filtering Function
double filteredDerivative(double currentValue, double lastValue) {
    double alpha = 1.0 / N;  // N determines the strength of filtering
    return (alpha * (currentValue - lastValue)) + ((1 - alpha) * lastDerivative);
}

void setup() {
    Serial.begin(2000000);

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
}

// Timing variables
unsigned long lastPIDUpdate = 0;
unsigned long lastPrint = 0;

void loop() {
    unsigned long now = millis();

    // Update setpoint dynamically
    setpoint = 50 + 40 * sin(now / 5000.0);

    // Faster RPM updates every 100ms
    if (now - lastTime >= 100) {  
        calculateRPM();
        input = rpm;
    }

    // Compute PID every 100ms with filtered derivative
    if (now - lastPIDUpdate >= 100) {
        // Apply filtered derivative approximation
        double derivative = filteredDerivative(input, rpm);
        lastDerivative = derivative;  // Store previous derivative

        // Adjust PID controller gains dynamically based on N
        motorPID.SetTunings(Kp, Ki, Kd * (1.0 / (1.0 + N * 0.1)));  // Modify Kd with N effect

        motorPID.Compute();
        lastPIDUpdate = now;
    }

    // Apply PID output
    if (output > 0) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, output);
    } else if (output < 0) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        analogWrite(MOTOR_PWM, abs(output));
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, 0);
    }

    // Print Debug Info Every 300ms
    if (now - lastPrint >= 300) {
        Serial.print("N: ");
        Serial.print(N);
        Serial.print(" | Setpoint: ");
        Serial.print(setpoint);
        Serial.print(" RPM | Measured: ");
        Serial.print(rpm);
        Serial.print(" RPM | PWM: ");
        Serial.println(output);
        lastPrint = now;
    }
}
