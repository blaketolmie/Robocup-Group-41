# RoboCup Serial Debug Output Guide
# =================================
# How to enable and monitor all debug data over serial

## STEP 1: ENABLE DEBUG MODE

### Code Changes Made:
1. **Set debug flag to true** in `src/telem.cpp`:
   ```cpp
   bool debug = true;  // Changed from false
   ```

2. **Uncomment debug tasks** in `src/main.cpp`:
   ```cpp
   if (debug) {
       taskManager.addTask(Tsensors_tof_print);     // ✅ Enabled
       taskManager.addTask(Tmotors_print);          // ✅ Enabled  
       taskManager.addTask(Ttelem_read);            // ✅ Enabled
       taskManager.addTask(Tsensors_navigation_print); // ✅ Enabled
   }
   ```

## STEP 2: BUILD AND UPLOAD

### PlatformIO Commands:
```bash
# Clean build
platformio run --target fullclean --environment teensy40

# Build with debug enabled
platformio run --environment teensy40

# Upload to Teensy
platformio run --target upload --environment teensy40
```

## STEP 3: MONITOR SERIAL OUTPUT

### Hardware Connection:
- **Serial Port**: Hardware UART (Serial7)
- **Baud Rate**: 115200
- **Pins**: Check your Teensy 4.0 pinout for Serial7 TX/RX
- **Connection**: Use USB-to-serial adapter or direct UART connection

### PlatformIO Serial Monitor:
```bash
# Start serial monitor
platformio device monitor --baud 115200 --port [YOUR_PORT]

# Example for Windows:
platformio device monitor --baud 115200 --port COM3

# Example for Linux/Mac:
platformio device monitor --baud 115200 --port /dev/ttyUSB0
```

### VS Code Terminal Method:
1. Open VS Code terminal
2. Run: `platformio device monitor`
3. PlatformIO will auto-detect the port and baud rate

## STEP 4: WHAT YOU'LL SEE

### 🔍 TOF Sensor Data (every 80ms):
```
Format: [Distance1],[Distance2] [AmbientPeak1],[AmbientPeak2]

Sample Output:
  245,  251  2.45,  2.51    # Sensor [0][0] - both readings + ambient light
  312,  298  3.12,  2.98    # Sensor [0][1] 
  187,  201  1.87,  2.01    # Sensor [0][2]
  456,  442  4.56,  4.42    # Sensor [1][0]
  523,  509  5.23,  5.09    # Sensor [1][1]
  234,  267  2.34,  2.67    # Sensor [1][2]
```

### 🚗 Motor Data (every 50ms):
```
Format: Left :[current]->[target] Right :[current]->[target]

Sample Output:
Left :  1520->  1600 Right :  1480->  1400
Left :  1525->  1600 Right :  1475->  1400
Left :  1530->  1600 Right :  1470->  1400
```

### 📐 Ultrasonic Navigation Data (every 50ms):
```
Format: Front L: [dist], R: [dist] Back L: [dist], R: [dist]

Sample Output:
Front L:  245, R:  267
Back L:  198, R:  201
Front L:  243, R:  265
Back L:  199, R:  203
```

### 🎨 Color Sensor Data (when triggered):
```
Sample Output:
R: 45.67, G: 78.23, B: 23.45
colour: 1
R: 42.34, G: 81.56, B: 21.78
colour: 2
```

### ⚠️ System Messages:
```
Error Messages:
"Failed to detect and initialize sensor L0 0"
"Failed to detect and initialize sensor L1 1,2"
"Data not ready"
"No TCS34725 found ... check your connections"

Navigation Messages:
"IMU Watchdog Triggered"
"Search Watchdog Triggered"
```

## STEP 5: DATA INTERPRETATION

### 📊 TOF Sensor Array Layout:
```
Sensor Layout (2x3 array):
[0][0]  [0][1]  [0][2]
[1][0]  [1][1]  [1][2]

Each sensor reports:
- Two distance readings (mm)
- Two ambient light levels
```

### 🚗 Motor Values:
```
Motor Control:
- 1500 = Stop (neutral)
- 1000-1499 = Reverse (lower = faster reverse)
- 1501-2000 = Forward (higher = faster forward)
- Current = actual servo position
- Target = desired servo position
```

### 📐 Ultrasonic Sensors:
```
Sensor Positions:
- Front Left/Right: Forward obstacle detection
- Back Left/Right: Rear obstacle detection
- Values in millimeters (mm)
- Range: typically 20mm to 4000mm
```

## STEP 6: ADVANCED MONITORING

### Serial Command Interface:
With `Ttelem_read` enabled, you can send commands via serial:
```
# Send commands through serial monitor
# Commands are processed in telem.cpp execute() function
```

### Performance Monitoring:
To enable task timing analysis, uncomment the performance monitoring code in main.cpp:
```cpp
// Uncomment the alternative loop() function for timing data
```

## TROUBLESHOOTING

### No Serial Output:
1. **Check connections**: Verify Serial7 TX/RX pins
2. **Verify baud rate**: Must be 115200
3. **Check debug flag**: Ensure `debug = true`
4. **Port permissions**: May need sudo on Linux/Mac

### Garbled Output:
1. **Wrong baud rate**: Set to 115200
2. **Wiring issues**: Check TX/RX not swapped
3. **Ground connection**: Ensure common ground

### Missing Data:
1. **Tasks not added**: Verify debug tasks are uncommented
2. **Sensor failures**: Check error messages for failed sensors
3. **System overload**: High-frequency output may cause timing issues

## PERFORMANCE IMPACT

### ⚠️ Debug Mode Effects:
- **Increased CPU usage**: Serial printing takes time
- **Potential timing issues**: May affect real-time performance
- **Memory usage**: Buffers for serial output
- **I/O load**: High-frequency serial transmission

### 🎯 Recommended Usage:
- **Development/Testing**: Enable for debugging
- **Competition**: Disable for maximum performance
- **Monitoring**: Use for system health checks
- **Tuning**: Enable to adjust PID parameters

## QUICK ENABLE/DISABLE

### To Enable All Debug:
```cpp
// In telem.cpp
bool debug = true;

// In main.cpp - uncomment all debug tasks
taskManager.addTask(Tsensors_tof_print);
taskManager.addTask(Tmotors_print);
taskManager.addTask(Ttelem_read);
taskManager.addTask(Tsensors_navigation_print);
```

### To Disable All Debug:
```cpp
// In telem.cpp  
bool debug = false;

// Debug tasks are automatically excluded when debug = false
```

This setup will give you comprehensive real-time telemetry from your RoboCup robot! 🤖📊