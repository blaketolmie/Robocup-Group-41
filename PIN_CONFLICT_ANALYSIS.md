# Pin Conflict Analysis Report
# ================================
# Date: September 25, 2025
# Report: Investigation of Pin 2-7 Conflicts in RoboCup Project

## EXECUTIVE SUMMARY
✅ **PIN CONFLICT IS RESOLVED** - No hardware conflict exists!

The warning in PIN_MAPPING.md about pins 2-7 having overlapping assignments is **INCORRECT**. The hardware design cleverly uses an I/O expander to avoid any pin conflicts.

## DETAILED ANALYSIS

### 🔍 The Apparent Conflict
The PIN_MAPPING.md file suggests that pins 2-7 are assigned to both:
- Encoders (pins 2,3,4,5)
- TOF sensor XSHUT control (pins 1,2,3,4,5,6,7)

### 🎯 The Actual Solution
After examining the source code, here's what actually happens:

#### **ENCODERS - Direct Teensy Pins:**
```cpp
// From motors.h:
enum PinAssignments {
  encoder1PinA = 2,    // Left motor encoder A
  encoder1PinB = 3,    // Left motor encoder B
  encoder2PinA = 4,    // Right motor encoder A
  encoder2PinB = 5,    // Right motor encoder B
};

// From motors.cpp:
attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, CHANGE);  // Teensy pin 2
attachInterrupt(digitalPinToInterrupt(4), doEncoder2A, CHANGE);  // Teensy pin 4
```

#### **TOF SENSORS - I/O Expander Pins:**
```cpp
// From sensors.cpp:
SX1509 TOF_io; // I/O expander object
TOF_io.begin(TOF_SX1509_ADDRESS); // Address 0x3F

// TOF control uses I/O expander pins, NOT Teensy pins:
TOF_io.pinMode(xshutPinsL1[i][j], OUTPUT);     // SX1509 pin control
TOF_io.digitalWrite(xshutPinsL1[i][j], LOW);   // SX1509 pin control

// The xshutPinsL1 array refers to SX1509 pins 2,3,4,5,6,7
// NOT Teensy pins 2,3,4,5,6,7!
```

## HARDWARE ARCHITECTURE

### 📋 Actual Pin Usage:

**Teensy 4.0 Direct Pins:**
- Pin 2: encoder1PinA (Left motor encoder A - interrupt)
- Pin 3: encoder1PinB (Left motor encoder B)
- Pin 4: encoder2PinA (Right motor encoder A - interrupt)  
- Pin 5: encoder2PinB (Right motor encoder B)
- Pin 18: SDA (I2C data to SX1509)
- Pin 19: SCL (I2C clock to SX1509)

**SX1509 I/O Expander Pins (accessed via I2C):**
- SX1509 Pin 1: xshutPinsL0[0] (Short range TOF shutdown)
- SX1509 Pin 2: xshutPinsL1[0][0] (Long range TOF shutdown)
- SX1509 Pin 3: xshutPinsL1[0][1] (Long range TOF shutdown)
- SX1509 Pin 4: xshutPinsL1[0][2] (Long range TOF shutdown)
- SX1509 Pin 5: xshutPinsL1[1][0] (Long range TOF shutdown)
- SX1509 Pin 6: xshutPinsL1[1][1] (Long range TOF shutdown)
- SX1509 Pin 7: xshutPinsL1[1][2] (Long range TOF shutdown)

## KEY EVIDENCE

### 🔬 Code Evidence:
1. **I/O Expander Declaration:** `SX1509 TOF_io;` in sensors.cpp
2. **I/O Expander Initialization:** `TOF_io.begin(TOF_SX1509_ADDRESS);`
3. **Expander Pin Control:** All TOF operations use `TOF_io.pinMode()` and `TOF_io.digitalWrite()`
4. **Direct Pin Control:** Encoders use direct `attachInterrupt(digitalPinToInterrupt(2), ...)`

### 🛠️ Hardware Design:
- **SX1509 I/O Expander** provides 16 additional GPIO pins
- Connected to Teensy via I2C (pins 18/19)
- TOF sensors controlled through expander pins
- Encoders use direct Teensy pins for fast interrupt response

## RECOMMENDATIONS

### ✅ Immediate Actions:
1. **Update PIN_MAPPING.md** to correctly reflect the I/O expander usage
2. **Remove the conflict warning** as it's based on incorrect assumptions
3. **Document the SX1509 expander** in the pin mapping

### 📝 Documentation Improvements:
1. Clearly separate "Teensy Direct Pins" from "SX1509 Expander Pins"
2. Add I/O expander section with pin mappings
3. Explain the I2C connection between Teensy and SX1509

## CONCLUSION

**NO ACTION REQUIRED** - The hardware design is correct and functional.

The apparent pin conflict was a documentation error, not a hardware problem. The system uses:
- **Teensy pins 2,3,4,5** for high-speed encoder interrupts (direct connection)
- **SX1509 pins 1-7** for TOF sensor control (via I2C expander)

This is an elegant solution that:
- ✅ Avoids pin conflicts
- ✅ Provides sufficient I/O pins
- ✅ Keeps critical interrupts on direct pins
- ✅ Uses I2C expansion for non-critical control signals

**The system is working as designed!** 🎉