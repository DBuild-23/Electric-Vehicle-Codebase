// Some code in this file is inspired by the Yahboom Motor with Encoder project
// Source: https://github.com/YahboomTechnology/Motor-with-Encoder

// Motor Control Pins
int motor_c_ENA = 9;  // Speed Control
int motor_c_ENB = 10;  
int motor_c_IN1 = 6; // Direction Control
int motor_c_IN2 = 7;

// Encoder Pins
#define ENCODER_A_PIN 2
#define ENCODER_B_PIN 3
#define STBY 8

const long target_distance_pulses = 3057; 
long pulse_count = 0; 

// speed control variables
int max_speed = 255;
int min_speed = 100;
int slow_down_threshold = 1000; 

void setup() {
    // Motor Standby Settings
    pinMode(STBY, OUTPUT)
    digitalWrite(STBY, 1)

    // Motor Pin Setup
    pinMode(motor_c_ENA, OUTPUT);
    pinMode(motor_c_ENB, OUTPUT);
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
        current_speed = min_speed + (remaining_pulses * (max_speed - min_speed)) / slow_down_threshold;
        if (current_speed > max_speed) {
            current_speed = max_speed;
        } else if (current_speed < min_speed) {
            current_speed = min_speed;
        }
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








const int BUTTON_PIN = 7;
int lastState = HIGH;
int currentState == LOW;

void setup() 
{
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() 
{
  lastState = currentState;
  currentState = digitalRead(BUTTON_PIN);

  if (lastState == HIGH && currentState == LOW)
  {
    run();
    return;
  }
}

//Looped code for moving car! (Can be anything we want to run though)
void run()
{
  //Driving code
}