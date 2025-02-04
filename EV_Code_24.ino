// Based on example code from Yahboom Motor Encoder (https://github.com/YahboomTechnology/Motor-with-Encoder)

// Motor Control Pins
const int motor_c_ENA = 6;   // Motor A PWM
const int motor_c_IN1 = 10;  // Motor A direction
const int motor_c_IN2 = 9;   // Motor A direction

const int motor_c_ENB = 5;   // Motor B PWM
const int motor_c_IN3 = 8;   // Motor B direction
const int motor_c_IN4 = 7;   // Motor B direction

// Encoder Pins
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3

// Button Pin
#define BUTTON_PIN 12

// Configuration
const float wheel_diameter_meters = 0.073; // 2-7/8 inches in meters
const int pulses_per_revolution = 100;
const float wheel_circumference = wheel_diameter_meters * 3.14159;

volatile long pulse_count = 0; 
long pulse_count_interval = 0; // Pulses counted in the current interval
int max_speed = 255;
int min_speed = 100;
int slow_down_threshold = 1000;
long target_distance_pulses = 0;

// RPM Calculation Variables
unsigned long previous_time = 0;      // Time at the start of the interval
const unsigned long interval = 1000;  // Interval duration in milliseconds (1 second)
float current_RPM = 0.0;              // Current RPM

void setup() {
    // Motor A Pin Setup
    pinMode(motor_c_ENA, OUTPUT);
    pinMode(motor_c_IN1, OUTPUT);
    pinMode(motor_c_IN2, OUTPUT);

    // Motor B Pin Setup
    pinMode(motor_c_ENB, OUTPUT);
    pinMode(motor_c_IN3, OUTPUT);
    pinMode(motor_c_IN4, OUTPUT);

    // Encoder Pin Setup
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), readEncoder, FALLING);

    // Button Pin Setup
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.begin(9600);

    // Set target distance (example: 7 meters)
    target_distance_pulses = metersToPulses(7.0); // Corrected from 7000 to 7.0 meters

    previous_time = millis(); // Initialize the previous_time
}

void loop() {
    // Wait for button press
    if (digitalRead(BUTTON_PIN) == LOW) { // Changed to LOW assuming button press connects to GND
        stopMotors();
        Serial.println("Motors Stopped by Button Press");
        while (1); // Halt further execution
    }

    // Calculate remaining pulses
    long remaining_pulses = target_distance_pulses - pulse_count;
    int current_speed = max_speed;

    // Adjust speed based on remaining distance
    if (remaining_pulses <= slow_down_threshold) {
        current_speed = min_speed + (remaining_pulses * (max_speed - min_speed)) / slow_down_threshold;
        current_speed = constrain(current_speed, min_speed, max_speed);
    }

    // Move Motors Forward
    digitalWrite(motor_c_IN1, HIGH);
    digitalWrite(motor_c_IN2, LOW);
    analogWrite(motor_c_ENA, current_speed);

    digitalWrite(motor_c_IN3, HIGH);
    digitalWrite(motor_c_IN4, LOW);
    analogWrite(motor_c_ENB, current_speed);

    // Stop Motors when target is reached
    if (pulse_count >= target_distance_pulses) {
        stopMotors();
        Serial.println("Target Reached");
        while (1); // Halt further execution
    }

    // RPM Calculation
    unsigned long current_time = millis();
    if (current_time - previous_time >= interval) {
        noInterrupts(); // Temporarily disable interrupts to safely copy pulse counts
        pulse_count_interval = pulse_count;
        pulse_count = 0;
        interrupts(); // Re-enable interrupts

        // Calculate RPM
        current_RPM = ((float)pulse_count_interval / pulses_per_revolution) * (60000.0 / interval);
        
        previous_time += interval; // Update the previous_time for the next interval
    }

    // Debugging Output
    Serial.print("Pulses: ");
    Serial.print(pulse_count_interval);
    Serial.print(" | Speed: ");
    Serial.print(current_speed);
    Serial.print(" | RPM: ");
    Serial.println(current_RPM);

    delay(100); // Adjust delay as needed
}

void readEncoder() {
    pulse_count++;
}

void stopMotors() {
    // Stop Motor A
    analogWrite(motor_c_ENA, 0);
    digitalWrite(motor_c_IN1, LOW);
    digitalWrite(motor_c_IN2, LOW);

    // Stop Motor B
    analogWrite(motor_c_ENB, 0);
    digitalWrite(motor_c_IN3, LOW);
    digitalWrite(motor_c_IN4, LOW);
}

long metersToPulses(float distance_meters) {
    return (distance_meters / wheel_circumference) * pulses_per_revolution;
}
