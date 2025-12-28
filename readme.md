I need Arduino .ino code for an ESP32 Dev Module.

**PROJECT:** Bluetooth-Controlled Car with Obstacle Avoidance for a Defined Track Race.
**MAIN LOGIC:** The car must receive movement commands via Bluetooth and immediately stop if an ultrasonic sensor detects a barrier closer than a defined STOPPING_DISTANCE_CM.

**COMPONENTS & PINOUT:**
- **L298N Motor Driver:** Connected to ESP32 GPIOs for two DC motors.
  - Left Motor: IN1=GPIO16, IN2=GPIO17, EnableA=GPIO4 (PWM for speed).
  - Right Motor: IN3=GPIO18, IN4=GPIO19, EnableB=GPIO5 (PWM for speed).
- **HC-SR04 Ultrasonic Sensor:** Trigger=GPIO13, Echo=GPIO12.
- **Active Buzzer:** GPIO14. Sounds audible alarm when obstacle detected (No resistor needed - 3.3V safe).
- **Bluetooth:** Uses built-in BluetoothSerial library. The device name should be "IoT_RaceCar".
- **Power:** L298N logic power (5V) from ESP32's 5V pin. Motor power from separate 7.4V battery. Sensor VCC to ESP32 5V.

**REQUIRED BEHAVIOR:**
1. **Bluetooth Control:** Continuously listen for single-character commands from a Serial Bluetooth Terminal app (e.g., 'F','B','L','R','S' for Forward, Back, Left, Right, Stop).
2. **Obstacle Priority:** Before executing any movement command, and during movement, continuously check the distance from the HC-SR04.
   - **If distance < STOPPING_DISTANCE_CM (e.g., 20cm):** Immediately stop both motors (regardless of Bluetooth command), activate buzzer alarm, and send "OBSTACLE! STOPPED" message back via Bluetooth.
   - **If distance >= STOPPING_DISTANCE_CM:** Execute the last valid Bluetooth movement command and deactivate buzzer.
3. **Motor Functions:** Create simple, non-blocking functions for car actions: `moveForward()`, `moveBackward()`, `turnLeft()`, `turnRight()`, `stopCar()`.
4. **Buzzer Functions:** Create functions `activateBuzzer()` and `deactivateBuzzer()` to control audible warning.

**LIBRARIES TO USE:**
- `BluetoothSerial.h` for communication.
- Use `pulseIn()` for ultrasonic reading. Handle the sensor reading in a non-blocking way or with a short timeout to avoid locking up the main loop.

**CODE STRUCTURE REQUEST:**
- Begin with the necessary `#include` and `#define` for all pins and constants (including buzzer pin).
- Declare global BluetoothSerial and motor/sensor/buzzer variables.
- In `setup()`, initialize Serial Monitor (for debugging), Bluetooth with the name "IoT_RaceCar", set all motor control pins as OUTPUTs, set the ultrasonic pins, set buzzer pin as OUTPUT, and ensure buzzer starts OFF.
- In `loop()`, follow this exact high-level logic flow:
    1. Check Bluetooth for a new command and store it if received.
    2. Read the current distance from the ultrasonic sensor.
    3. **IF** (distance < STOPPING_DISTANCE_CM) **THEN** call `stopCar()`, call `activateBuzzer()`, and notify via Bluetooth.
    4. **ELSE** call `deactivateBuzzer()` and execute the motor command corresponding to the last stored Bluetooth character.
- Implement the motor control functions (`stopCar`, `moveForward`, etc.) that set the correct HIGH/LOW states on IN1-IN4 and PWM values on Enable pins.
- Implement buzzer control functions (`activateBuzzer`, `deactivateBuzzer`) using digitalWrite.

**ADDITIONAL NOTES:**
- The code must be efficient and avoid using `delay()` in the main loop to keep the car responsive.
- Include brief comments explaining the logic at each major step.

Please generate the complete Arduino .ino code based on these specifications.