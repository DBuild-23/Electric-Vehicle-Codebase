// Some code in this file is inspired by the Yahboom Motor with Encoder project
// Source: https://github.com/YahboomTechnology/Motor-with-Encoder

// Motor Control Pins
int motor_c_ENA = 9;  // Motor A Speed Control
int motor_c_IN1 = 6;  // Motor A Direction Control
int motor_c_IN2 = 7;  // Motor A Direction Control
int motor_c_ENB = 10; // Motor B Speed Control
int motor_c_IN3 = 4;  // Motor B Direction Control
int motor_c_IN4 = 5;  // Motor B Direction Control

// Encoder Pins
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3

const float wheel_diameter_meters = 0.073; // 2-7/8 inches in meters
const int pulses_per_revolution = 100;
const float wheel_circumference = wheel_diameter_meters * 3.14159;

long pulse_count = 0; 
int max_speed = 255;
int min_speed = 100;
int slow_down_threshold = 1000;
long target_distance_pulses = 0;

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

    Serial.begin(9600);

    // Set target distance (example: 7 meters)
    target_distance_pulses = metersToPulses(7);
}

void loop() {
    long remaining_pulses = target_distance_pulses - pulse_count;
    int current_speed = max_speed;

    if (remaining_pulses <= slow_down_threshold) {
        current_speed = min_speed + (remaining_pulses * (max_speed - min_speed)) / slow_down_threshold;
        if (current_speed > max_speed) {
            current_speed = max_speed;
        } else if (current_speed < min_speed) {
            current_speed = min_speed;
        }
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
        while (1);
    }

    // Outputs
    Serial.print("Pulses: ");
    Serial.print(pulse_count);
    Serial.print(" | Speed: ");
    Serial.println(current_speed);
    delay(100);
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
