# RoboCup Task Scheduling & Serial Output Analysis
# =================================================
# Date: September 25, 2025
# Analysis of TaskScheduler system and Serial7 telemetry output

## TASK SCHEDULING SYSTEM

### 🔧 Scheduler Architecture:
The system uses the **TaskScheduler library** with a single main scheduler called `taskManager`.

```cpp
Scheduler taskManager;
```

### 📋 Active Tasks (Production Mode):
When `debug = false`, only essential tasks run:

| Task Name | Interval (ms) | Function | Purpose |
|-----------|---------------|----------|---------|
| `Tsensors_tof_read` | 80 | `sensors_tof_read()` | Read TOF distance sensors |
| `Tmotors_PID_drive` | 20 | `motors_PID_drive()` | Motor PID control loop |
| `Tmotors_robot_servo_controller` | 100 | `motors_robot_servo_controller()` | Servo position control |
| `Tnavigate_logic` | 20 | `navigate_logic()` | Main navigation decisions |
| `Tsensors_ultrasonic_read` | 40 | `sensors_ultrasonic_read()` | Read ultrasonic sensors |
| `TIMU_update_watchdog` | 150 | `IMU_update_watchdog()` | IMU timeout monitoring |

### 📋 Debug Tasks (Debug Mode):
When `debug = true`, additional telemetry tasks are enabled:

| Task Name | Interval (ms) | Function | Purpose |
|-----------|---------------|----------|---------|
| `Tsensors_tof_print` | 80 | `sensors_tof_print()` | Print TOF sensor data |
| ~~`Tmotors_print`~~ | 50 | `motors_print()` | Print motor speeds (commented out) |
| ~~`Ttelem_read`~~ | 20 | `telem_read()` | Read serial commands (commented out) |
| ~~`Tsensors_navigation_print`~~ | 50 | `sensors_navigation_print()` | Print navigation data (commented out) |

**Note:** Most debug tasks are currently commented out in the code.

## EXECUTION FLOW

### 🚀 Setup Sequence:
1. **Initialize subsystems:**
   ```cpp
   telem_serial_init();    // Serial7 at 115200 baud
   sensors_init();         // TOF, ultrasonic, color sensors
   motors_init();          // Motors, encoders, servos  
   IMU_init();            // Inertial measurement unit
   ```

2. **Configure scheduler:**
   ```cpp
   taskManager.init();
   // Add production tasks
   // Add debug tasks (if debug mode)
   taskManager.enableAll();
   ```

3. **Initial servo positioning:**
   ```cpp
   while (!motors_smart_servo_controller(CLOSED)) {
       continue; // Wait for servo to reach closed position
   }
   ```

### 🔄 Main Loop:
```cpp
void loop() {
    taskManager.execute(); // Run all scheduled tasks
}
```

## SERIAL OUTPUT (Serial7 - Hardware UART)

### 📡 Serial Configuration:
- **Port:** Serial7 (Hardware UART on Teensy pins)
- **Baud Rate:** 115200
- **Purpose:** Telemetry and debugging output

### 📊 TOF Sensor Data Output (Debug Mode):
When `sensors_tof_print()` runs every 80ms:

```
Format: [Distance1],[Distance2] [AmbientPeak1],[AmbientPeak2]

Example Output:
  245,  251  2.45,  2.51  // Sensor [0][0] - both readings + ambient
  312,  298  3.12,  2.98  // Sensor [0][1] 
  187,  201  1.87,  2.01  // Sensor [0][2]
  456,  442  4.56,  4.42  // Sensor [1][0]
  523,  509  5.23,  5.09  // Sensor [1][1]
  234,  267  2.34,  2.67  // Sensor [1][2]
```

**Data Structure:**
- **6 TOF sensors** in 2x3 array (`verticalSensorCount=2`, `horizontalSensorCount=3`)
- **2 readings per sensor** (`tof_sensor_values[i][j][0]` and `[1]`)
- **Ambient light data** (`tof_sensor_amb_peak[i][j][0]` and `[1]`)

### 🚗 Motor Data Output (Commented Out):
When `motors_print()` would run every 50ms:
```
Format: Left :[current]->[target] Right :[current]->[target]

Example Output:
Left :  1520->  1600 Right :  1480->  1400
```

### 📐 Ultrasonic Sensor Data:
When `sensors_navigation_print()` runs:
```
Format: Front L: [dist], R: [dist] Back L: [dist], R: [dist]

Example Output:  
Front L:  245, R:  267
Back L:  198, R:  201
```

### 🎨 Color Sensor Data:
When color detection occurs:
```
Format: R: [red], G: [green], B: [blue]
        colour: [detected_type]

Example Output:
R: 45.67, G: 78.23, B: 23.45
colour: 1
```

### ⚠️ Error Messages:
```
"Failed to detect and initialize sensor L0 [index]"
"Failed to detect and initialize sensor L1 [i,j]" 
"Data not ready"
"No TCS34725 found ... check your connections"
"IMU Watchdog Triggered"
"Search Watchdog Triggered"
```

## TASK TIMING ANALYSIS

### ⏱️ Performance Monitoring:
The system includes commented-out task timing code that tracks:
- **Minimum execution time** per task
- **Maximum execution time** per task  
- **Average execution time** per task
- **Execution count** per task

### 📈 Task Frequency Analysis:
Based on the intervals, here's the execution frequency:

| Task | Frequency (Hz) | CPU Load Estimate |
|------|----------------|------------------|
| `motors_PID_drive` | 50 Hz | High (critical) |
| `telem_read` | 50 Hz | Low |
| `navigate_logic` | 50 Hz | Medium |
| `sensors_ultrasonic_read` | 25 Hz | Medium |
| `sensors_tof_read` | 12.5 Hz | High (I2C intensive) |
| `motors_robot_servo_controller` | 10 Hz | Low |
| `IMU_update_watchdog` | 6.7 Hz | Low |

## COMMUNICATION PROTOCOLS

### 📡 Serial Command Interface:
The `telem_read()` function (currently disabled) can process incoming serial commands for:
- Real-time parameter adjustment
- Debug mode toggling
- Manual control override
- System status queries

### 🔗 I2C Bus Usage:
High-frequency I2C communication for:
- TOF sensor readings (6 sensors + 1 short range)
- SX1509 I/O expander control
- IMU data acquisition
- Color sensor readings

## CURRENT SYSTEM STATE

### ✅ Active Features:
- Core navigation and motor control
- TOF distance sensing
- Ultrasonic ranging  
- IMU monitoring
- Servo control

### 🚧 Disabled Features:
- Most serial debug output (commented out)
- Serial command processing
- Detailed motor telemetry
- Navigation data printing
- Task performance monitoring

The system is configured for **production operation** with minimal debug output to maximize performance.