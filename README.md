# SweepBot: Floor Mopper Module (Arduino Uno)

This module implements the floor-mopping stage for SweepBot. After navigation (sound-following and obstacle avoidance) brings the robot to the target area, this code:
- Wets the floor by running a small DC water pump.
- Spins a mop motor via the L298N driver.
- Optionally lifts/lowers the mop head using an SG90 servo.
- Uses an ultrasonic interlock to pause if an obstacle is too close.


## Hardware

- Arduino Uno R3
- L298N motor driver (one DC channel dedicated to mop motor)
- DC mop motor
- HC-SR04 ultrasonic sensor
- SG90 micro servo (optional)
- DC water pump (5–12V) + reservoir
- Logic-level N-MOSFET (e.g., AO3400, IRLZ44N) or 5V relay module to switch the pump
- Flyback diode across pump if using MOSFET
- Separate motor power (e.g., 2–3x 18650 with proper BMS) and Arduino 5V logic supply
- Common ground between motor power (driver/mosfet) and Arduino GND

## Wiring (default pins)

Update SweepBot_Mopper.ino if wiring differs.

- HC-SR04:
  - TRIG -> D8
  - ECHO -> D9
  - Vcc -> 5V
  - GND -> GND

- L298N (mop motor on one channel):
  - EN (ENA/ENB) -> D5 (PWM)
  - IN1 -> D6
  - IN2 -> D7
  - L298N 12V -> Battery motor rail
  - L298N GND -> Battery GND + Arduino GND (common)
  - Motor -> L298N outputs for chosen channel

- Pump (via MOSFET or relay):
  - Arduino D4 -> MOSFET gate (through 220Ω–1kΩ) or Relay IN
  - Pump + -> Battery +
  - Pump - -> MOSFET drain (or Relay NO)
  - MOSFET source -> GND
  - Diode across pump (1N4007): cathode to +, anode to -

- Servo (optional):
  - Signal -> D10
  - +5V -> 5V (use separate 5V regulator capable of ≥500mA; avoid drawing heavy servo current from Uno 5V)
  - GND -> GND

- Start trigger:
  - D2 expects HIGH to start mopping. Connect from navigation MCU or a switch which pulls D2 HIGH when at destination.
  - If using a push button to GND with INPUT_PULLUP, invert logic as described in the code comments.

- Status LED: uses onboard LED at D13.

## Parameters

Adjust in the top of the sketch:
- MOP_SPEED_PWM: 0–255, mop motor speed (default 180).
- PUMP_ON_TIME_MS: initial wetting time (default 3,000ms).
- MOP_PASS_TIME_MS: mop spin duration (default 15,000ms).
- SAFETY_STOP_DIST_CM: obstacle threshold (default 12cm).

## Safety and Power Notes

- Do not power motors or pump from the Uno 5V pin.
- Use a proper battery pack and regulator(s), with adequate current capacity.
- Always share a common ground between Arduino and motor/pump driver circuits.
- Add flyback diodes for inductive loads.
- Test with wheels lifted off the ground first.

## Build and Upload

1. Open SweepBot_Mopper.ino in Arduino IDE.
2. Select Board: Arduino Uno, correct Port.
3. Upload.

## Operation

- Keep the trigger LOW (or inactive) during navigation.
- When the robot reaches the destination, drive the trigger line HIGH.
- The sequence:
  1) Servo lowers mop (if enabled).
  2) Pump wets the floor for PUMP_ON_TIME_MS.
  3) Pump stops; mop motor spins for MOP_PASS_TIME_MS.
  4) Servo lifts mop; state goes to DONE until trigger is released.
- If an obstacle is detected closer than SAFETY_STOP_DIST_CM at any time, the system halts, lifts the mop, and waits until the path is clear and the trigger is released.

## Integrating With Your Existing Code

- Replace isMopStartRequested() to read your actual navigation “arrived” flag.
- If sharing the ultrasonic sensor with obstacle avoidance, ensure time-multiplexing to avoid interference.
- If L298N is already fully used by drivetrain, use a small separate motor driver (e.g., L9110S or a second L298N channel) for the mop motor.

## Troubleshooting

- Pump or mop not running: check battery voltage under load and grounds in common.
- Random resets: add decoupling (100µF near drivers), separate 5V regulator for servo, and shield signal wires.
- Ultrasonic false positives: add a small median filter or increase SAFETY_STOP_DIST_CM.
- Burnt boards previously: avoid drawing motor power from Arduino; separate power domains and ensure proper wiring and heat dissipation.
