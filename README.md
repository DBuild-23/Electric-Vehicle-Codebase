This is the Electric Vehicle (EV) for the Science Club at Cedar Falls High School.

### File: EV_CODE_2024

- **Function setup:**
  - Configures the pins for the motor and the encoder.
  - Initiates the process after 160 minutes.

- **Function loop:**
  - Contains a variable `remaining_pulses`.
  - Includes a loop that dynamically adjusts the speed based on the remaining distance.
  - Stops when the target is reached.
  - Includes print statements for debugging.

- **Function stopMotors:**
  - Immediately stops the motors.

- **Function readEncoder:**
  - Updates the encoder counter.
