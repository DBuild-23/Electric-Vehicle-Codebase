This is the Electric Vehicle (EV) for the Science Club at Cedar Falls High School.

## Pin Configuration

- **Motor Control Pins**:
  - `motor_c_ENA`: Pin 9 (PWM for speed control)
  - `motor_c_IN1`: Pin 6 (Direction control)
  - `motor_c_IN2`: Pin 7 (Direction control)
- **Encoder Pins**:
  - `ENCODER_A_PIN`: Pin 2 (Interrupt-enabled pin for encoder A signal)
  - `ENCODER_B_PIN`: Pin 3 (Pin for encoder B signal, not used for direction in this code)

## How It Works

1. **Setup**:
   - Initializes motor and encoder pins.
   - Configures an interrupt on the encoder A pin to track pulses.
2. **Dynamic Speed Control**:
   - Calculates remaining pulses to the target distance.
   - Adjusts speed proportionally as the vehicle nears the target.
3. **Stopping**:
   - Halts the motor once the pulse count matches the target.
4. **Logging**:
   - Outputs current pulse count and speed.

## Configuration

- **Target Distance**: Defined in `target_distance_pulses`. Calculate pulses using:

  ```
  target_distance_pulses = (distance_in_meters / wheel_circumference) * pulses_per_revolution;
  ```

  - Wheel Circumference: 0.073m (from 2-7/8 inch diameter)
  - Pulses Per Revolution: 100 (based on motor specs)

- **Speed Parameters**:

  - `max_speed`: 255 (maximum PWM duty cycle)
  - `min_speed`: 100 (minimum speed for reliable movement)
  - `slow_down_threshold`: 1000 pulses (distance to start slowing down, can be changed)

Based on example code from Yahboom Motor Encoder ([https://github.com/YahboomTechnology/Motor-with-Encoder](https://github.com/YahboomTechnology/Motor-with-Encoder)).

