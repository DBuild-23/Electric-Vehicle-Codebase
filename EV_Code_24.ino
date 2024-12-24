// Source from: https://github.com/YahboomTechnology/Motor-with-Encoder

// Motor Control Pins
int motor_c_ENA = 9;  // Speed Control
int motor_c_IN1 = 6; // Direction Control
int motor_c_IN2 = 7;

// Encoder Pins
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3

// target distance configuration
const long target_distance_pulses = 3057; 
long pulse_count = 0; 

// speed sontrol variables
int max_speed = 255;
int min_speed = 100;
int slow_down_threshold = 1000; 

#include <MsTimer2.h>

void setup() {
    // Motor Pin Setup
    pinMode(motor_c_ENA, OUTPUT);
    pinMode(motor_c_IN1, OUTPUT);
    pinMode(motor_c_IN2, OUTPUT);

    // Encoder Pin Setup
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), readEncoder, FALLING);

    Serial.begin(9600);
}

void loop() {
    // remaining distance
    long remaining_pulses = target_distance_pulses - pulse_count;

    // Dynamically adjust speed based on remaining distance
    int current_speed = max_speed;
    if (remaining_pulses <= slow_down_threshold) {
        current_speed = map(remaining_pulses, 0, slow_down_threshold, min_speed, max_speed);
        current_speed = constrain(current_speed, min_speed, max_speed);
    }

    // Move forward
    digitalWrite(motor_c_IN1, HIGH);
    digitalWrite(motor_c_IN2, LOW);
    analogWrite(motor_c_ENA, current_speed);

    // Stop the motor when target is reached
    if (pulse_count >= target_distance_pulses) {
        stopMotor();
        Serial.println("Target Reached");
        while (1); 
    }

    // Print
    Serial.print("Pulses: ");
    Serial.print(pulse_count);
    Serial.print(" | Speed: ");
    Serial.println(current_speed);
    delay(100);
}

void readEncoder() {
    // Update pulse count on encoder signal
    pulse_count++;
}

void stopMotor() {
    // Stop the motor immediately
    analogWrite(motor_c_ENA, 0);
    digitalWrite(motor_c_IN1, LOW);
    digitalWrite(motor_c_IN2, LOW);
}
