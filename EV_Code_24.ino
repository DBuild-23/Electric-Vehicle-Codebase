#include <PinChangeInterrupt.h>

// -----------------------------
// L298N Motor Control Pins
// -----------------------------
const int motor_c_ENA = 6;   // Motor A PWM
const int motor_c_IN1 = 10;  // Motor A direction
const int motor_c_IN2 = 9;   // Motor A direction

const int motor_c_ENB = 5;  // Motor B PWM
const int motor_c_IN3 = 8;  // Motor B direction
const int motor_c_IN4 = 7;  // Motor B direction

// -----------------------------
// Encoder Pins (Pin Change Interrupt)
// -----------------------------
#define ENCODER_A1_PIN 2
#define ENCODER_B1_PIN 11

#define ENCODER_A2_PIN 3
#define ENCODER_B2_PIN 4

// -----------------------------
// Encoder Pulse Counters
// -----------------------------
volatile long totalPulsesA = 0;  // total pulses for distance
volatile long totalPulsesB = 0;

// We'll also track pulses in each sample interval:
volatile long intervalPulsesA = 0;
volatile long intervalPulsesB = 0;

// -----------------------------
// Wheel / Encoder Parameters
// -----------------------------
const float wheel_diameter_meters = 0.073;  // ~2.875" in meters
const int pulses_per_revolution = 100;      // Adjust if needed
const float wheel_circumference = wheel_diameter_meters * 3.14159;

// -----------------------------
// PID Settings
// -----------------------------
float Kp = 2.0;  // Proportional gain
float Ki = 0.5;  // Integral gain
float Kd = 0.1;  // Derivative gain

float integratorA = 0, integratorB = 0;
float lastErrorA = 0, lastErrorB = 0;

// -----------------------------
// Control / Distance Parameters
// -----------------------------
int max_speed = 255;            // Max PWM
int min_speed = 0;              // Min PWM
float target_speed_pps = 50.0;  // Target speed in pulses per second
long target_distance_pulses = 0;

// We sample speeds every X ms:
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL_MS = 50;  // 50ms = 20 updates/sec

// -----------------------------
// BUTTON ON PIN 12
// -----------------------------
const int buttonPin = 12;
bool runPID = false;  // Flag: are we currently running?

// Also track if we've reached the distance
bool reachedTarget = false;

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(motor_c_ENA, OUTPUT);
  pinMode(motor_c_IN1, OUTPUT);
  pinMode(motor_c_IN2, OUTPUT);

  pinMode(motor_c_ENB, OUTPUT);
  pinMode(motor_c_IN3, OUTPUT);
  pinMode(motor_c_IN4, OUTPUT);

  // Direction forward by default:
  digitalWrite(motor_c_IN1, HIGH);
  digitalWrite(motor_c_IN2, LOW);
  digitalWrite(motor_c_IN3, HIGH);
  digitalWrite(motor_c_IN4, LOW);

  // Encoder pins
  pinMode(ENCODER_A1_PIN, INPUT);
  pinMode(ENCODER_B1_PIN, INPUT);
  pinMode(ENCODER_A2_PIN, INPUT);
  pinMode(ENCODER_B2_PIN, INPUT);

  // Attach pin change interrupts
  attachPCINT(digitalPinToPCINT(ENCODER_A1_PIN), encoderA_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(ENCODER_B1_PIN), encoderA_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(ENCODER_A2_PIN), encoderB_ISR, CHANGE);
  attachPCINT(digitalPinToPCINT(ENCODER_B2_PIN), encoderB_ISR, CHANGE);

  // Button
  pinMode(buttonPin, INPUT_PULLUP);
  // (Now the button should be wired so that pressing it connects pin 12 to GND.)

  // Distance target
  target_distance_pulses = metersToPulses(1.0);

  // Ensure motors are stopped
  stopMotors();

  Serial.println("Setup complete. Press the button to start the movement...");
}

// -----------------------------
// LOOP
// -----------------------------
void loop() {
  // 1) Check if button is pressed (active LOW on pin 12)
  if (!runPID) {
    // If not currently running, see if user just pressed the button
    if (digitalRead(buttonPin) == LOW) {
      // Debounce wait
      delay(50);
      if (digitalRead(buttonPin) == LOW) {
        // Clear any stale data and start running
        resetMotion();
        runPID = true;
        Serial.println("Button pressed. Starting motion...");
      }
    }
  }
  // 2) If we are running PID, do the normal speed/distance logic
  if (runPID && !reachedTarget) {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
      lastUpdateTime = currentTime;

      // Copy pulse counts atomically
      noInterrupts();
      long pulsesA = intervalPulsesA;
      long pulsesB = intervalPulsesB;
      intervalPulsesA = 0;
      intervalPulsesB = 0;
      long totalA = totalPulsesA;
      long totalB = totalPulsesB;
      interrupts();

      // Compute actual speeds in pulses/sec
      float actualSpeedA = (float)pulsesA / (UPDATE_INTERVAL_MS / 1000.0);
      float actualSpeedB = (float)pulsesB / (UPDATE_INTERVAL_MS / 1000.0);

      // Check distance
      float avgDistance = 0.5 * (totalA + totalB);
      if (avgDistance >= target_distance_pulses) {
        stopMotors();
        reachedTarget = true;
        Serial.println("Target Distance Reached!");
      } else {
        // PID for each motor
        // Motor A
        float errorA = target_speed_pps - actualSpeedA;
        integratorA += errorA * (UPDATE_INTERVAL_MS / 1000.0);
        float derivativeA = (errorA - lastErrorA) / (UPDATE_INTERVAL_MS / 1000.0);
        float outputA = Kp * errorA + Ki * integratorA + Kd * derivativeA;
        lastErrorA = errorA;
        int pwmA = constrain((int)outputA, min_speed, max_speed);

        // Motor B
        float errorB = target_speed_pps - actualSpeedB;
        integratorB += errorB * (UPDATE_INTERVAL_MS / 1000.0);
        float derivativeB = (errorB - lastErrorB) / (UPDATE_INTERVAL_MS / 1000.0);
        float outputB = Kp * errorB + Ki * integratorB + Kd * derivativeB;
        lastErrorB = errorB;
        int pwmB = constrain((int)outputB, min_speed, max_speed);

        // Write to motors
        analogWrite(motor_c_ENA, pwmA);
        analogWrite(motor_c_ENB, pwmB);

        // Debug
        Serial.print("DistA=");
        Serial.print(totalA);
        Serial.print(" DistB=");
        Serial.print(totalB);
        Serial.print(" SpdA=");
        Serial.print(actualSpeedA);
        Serial.print(" SpdB=");
        Serial.print(actualSpeedB);
        Serial.print(" PWM_A=");
        Serial.print(pwmA);
        Serial.print(" PWM_B=");
        Serial.print(pwmB);
        Serial.print(" errA=");
        Serial.print(errorA);
        Serial.print(" errB=");
        Serial.print(errorB);
        Serial.println();
      }
    }
  }

  // 3) If we've reached the target, motors stay off until reset
  //    Optionally, you could watch for another button press to start again.
}

// -----------------------------
// FUNCTIONS
// -----------------------------
void encoderA_ISR() {
  bool A = digitalRead(ENCODER_A1_PIN);
  bool B = digitalRead(ENCODER_B1_PIN);

  if (A == B) {
    totalPulsesA++;
    intervalPulsesA++;
  } else {
    totalPulsesA--;
    intervalPulsesA--;
  }
}

void encoderB_ISR() {
  bool A2 = digitalRead(ENCODER_A2_PIN);
  bool B2 = digitalRead(ENCODER_B2_PIN);

  if (A2 == B2) {
    totalPulsesB++;
    intervalPulsesB++;
  } else {
    totalPulsesB--;
    intervalPulsesB--;
  }
}

// Stop both motors
void stopMotors() {
  analogWrite(motor_c_ENA, 0);
  digitalWrite(motor_c_IN1, LOW);
  digitalWrite(motor_c_IN2, LOW);

  analogWrite(motor_c_ENB, 0);
  digitalWrite(motor_c_IN3, LOW);
  digitalWrite(motor_c_IN4, LOW);
}

// Reset counters & integrators so we can run fresh
void resetMotion() {
  noInterrupts();
  totalPulsesA = 0;
  totalPulsesB = 0;
  intervalPulsesA = 0;
  intervalPulsesB = 0;
  integratorA = 0;
  integratorB = 0;
  lastErrorA = 0;
  lastErrorB = 0;
  reachedTarget = false;
  interrupts();
  stopMotors();  // ensure motors are at 0 before we start
}

long metersToPulses(float distance_meters) {
  return (long)((distance_meters / wheel_circumference) * pulses_per_revolution);
}
