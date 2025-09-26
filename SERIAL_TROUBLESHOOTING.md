# Serial Output Troubleshooting Guide
# ===================================
# Diagnosing why no serial output is appearing

## ISSUE: No Serial Output Appearing

### 🔍 DIAGNOSTIC STEPS:

## STEP 1: CHECK USB SERIAL (Setup Messages)

### What to Monitor:
- **USB Serial (Serial)** - Setup and diagnostic messages
- **Port**: Usually shows as "Teensy" or "Arduino" in device manager
- **Baud**: 115200

### Expected USB Serial Output:
```
=== RoboCup Robot Starting ===
Serial7 initialized at 115200 baud
Initializing sensors...
Initializing motors...
Initializing IMU...
Setting up task scheduler...
Debug mode enabled - adding debug tasks
Debug tasks added
Enabling all tasks...
Waiting for servo to reach CLOSED position...
....
Servo positioned!
Getting initial IMU heading...
=== Setup complete, starting main loop ===
Debug output will appear on Serial7 (hardware UART)
USB Serial debug messages will continue here
Main loop running - check Serial7 for sensor data
```

### If USB Serial Shows Nothing:
1. **Wrong Port**: Check device manager for correct COM port
2. **Driver Issue**: Install Teensy USB drivers
3. **Power Issue**: Robot not powered or USB connection loose
4. **Code Upload Failed**: Robot still running old code

## STEP 2: CHECK WHERE ROBOT HANGS

### Possible Hang Points:

#### A) Servo Initialization Loop:
If you see dots `....` but no "Servo positioned!":
- **Issue**: Servo not reaching CLOSED position
- **Solution**: Check servo wiring, power supply, servo horn alignment

#### B) Sensor Initialization:
If it hangs after "Initializing sensors...":
- **Issue**: I2C sensors not responding
- **Solution**: Check I2C wiring (pins 18/19), sensor power

#### C) IMU Initialization:
If it hangs after "Initializing IMU...":
- **Issue**: IMU not responding
- **Solution**: Check IMU wiring and power

## STEP 3: CHECK SERIAL7 OUTPUT (Sensor Data)

### Hardware Requirements:
- **Serial7 uses different pins than USB**
- **Need physical wire connection** to Serial7 TX pin
- **Baud Rate**: 115200

### Find Serial7 Pins on Teensy 4.0:
```
Teensy 4.0 Serial7 Pins:
- TX7: Pin 29
- RX7: Pin 30
```

### Connection Options:

#### Option A: USB-to-Serial Adapter
```
USB-Serial Adapter → Teensy 4.0
GND                → GND
RX                 → Pin 29 (TX7)
(Don't connect TX unless you need bidirectional)
```

#### Option B: Second Arduino/Teensy as Serial Bridge
```cpp
// On second device, simple serial bridge:
void setup() {
    Serial.begin(115200);  // USB
    Serial1.begin(115200); // Connect to Teensy Serial7
}
void loop() {
    if (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
```

## STEP 4: QUICK FIXES TO TRY

### Fix 1: Add USB Serial Output to Print Functions
Add this to make debug output go to both USB and Serial7:

```cpp
// In sensors.cpp - modify sensors_tof_print():
void sensors_tof_print(void){
    // ... existing code ...
    
    // Also output to USB Serial for debugging
    if (printRaw) {
        for (uint8_t i = 0; i < verticalSensorCount; i++) {
            for (uint8_t j = 0; j < horizontalSensorCount; j++) {
                Serial.printf("%5d,%5d %5.2f,%5.2f ", 
                    tof_sensor_values[i][j][0], tof_sensor_values[i][j][1],
                    tof_sensor_amb_peak[i][j][0], tof_sensor_amb_peak[i][j][1]);
            }
            Serial.println();
        }
    }
}
```

### Fix 2: Bypass Servo Hang
Comment out the servo wait if it's hanging:
```cpp
// Comment this out temporarily:
// while (!motors_smart_servo_controller(CLOSED)) {
//     Serial.print(".");
//     delay(100);
//     continue;
// }
```

### Fix 3: Simple Test Output
Add this to main loop for basic output test:
```cpp
void loop() {
    static unsigned long lastTest = 0;
    taskManager.execute();
    
    if (millis() - lastTest > 1000) {
        Serial.println("USB: Main loop running");
        Serial7.println("Serial7: Test output");
        lastTest = millis();
    }
}
```

## STEP 5: MONITOR METHODS

### Method 1: PlatformIO Monitor (USB Serial)
```bash
# This monitors USB Serial (for setup messages)
platformio device monitor --port [USB_PORT] --baud 115200
```

### Method 2: Separate Serial Monitor (Serial7)
```bash
# This monitors Serial7 (for sensor data) - needs hardware connection
platformio device monitor --port [SERIAL7_PORT] --baud 115200
```

### Method 3: VS Code Serial Monitor
1. Open Command Palette (Ctrl+Shift+P)
2. "PlatformIO: Serial Monitor"
3. Select correct port

## COMMON ISSUES & SOLUTIONS:

### Issue 1: "Main loop running" but no sensor data
- **Cause**: Serial7 not connected or wrong port
- **Solution**: Wire Serial7 TX (pin 29) to USB-serial adapter

### Issue 2: Hangs on servo initialization
- **Cause**: Servo not reaching target position
- **Solution**: Check servo power, wiring, mechanical binding

### Issue 3: I2C sensor failures
- **Cause**: TOF sensors not responding
- **Solution**: Check I2C wiring (pins 18/19), sensor addresses

### Issue 4: No USB output at all
- **Cause**: Wrong port or driver issue
- **Solution**: Check Device Manager, reinstall Teensy drivers

### Issue 5: Garbled output
- **Cause**: Wrong baud rate
- **Solution**: Ensure both code and monitor use 115200 baud

### Issue 6: Output stops after startup
- **Cause**: Task scheduler not running or sensors failing
- **Solution**: Check for infinite loops in task functions

## QUICK DIAGNOSTIC CHECKLIST:

- [ ] USB Serial shows startup messages
- [ ] Robot completes setup without hanging
- [ ] Serial7 hardware connection established
- [ ] Correct baud rate (115200) on both ends
- [ ] Debug mode enabled (`debug = true`)
- [ ] Debug tasks uncommented in main.cpp
- [ ] Sensor initialization successful
- [ ] Task scheduler running

## EMERGENCY FALLBACK:

If all else fails, modify the code to output everything to USB Serial:
1. Replace all `Serial7.` with `Serial.` in sensor print functions
2. This will put all output on USB Serial for easier debugging
3. Remember to change back for final version

This should help you identify exactly where the issue is occurring! 🔧