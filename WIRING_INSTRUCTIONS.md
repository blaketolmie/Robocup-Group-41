# RoboCup Robot Wiring Instructions
# ==================================
# Date: September 25, 2025
# Hardware: Teensy 4.0 + SX1509 I/O Expander + Robot Components

## REQUIRED HARDWARE
- Teensy 4.0 microcontroller
- SX1509 I/O Expander breakout board
- 2x DC Motors with encoders
- 2x Main servo (gripper/collection)
- 4x HC-SR04 Ultrasonic sensors
- 6x VL53L1X TOF sensors (long range array)
- 1x VL53L0X TOF sensor (short range)
- 2x Inductive proximity sensor
- 1x Blue start button
- Power supply (7-12V for VIN, or 5V via USB)




## POWER CONNECTIONS

### Teensy 4.0 Power:
```
Power Source → Teensy Pin
├── 7-12V DC   → VIN (primary power)
├── 5V USB     → USB connector (programming/debug)
├── Ground     → GND (multiple GND pins available)
└── 3.3V Out   → Use for 3.3V sensors
```

**⚠️ Important:** Teensy 4.0 operates at 3.3V logic levels but pins are 5V tolerant for inputs.

## I2C BUS CONNECTIONS

### Primary I2C Bus (for sensors):
```
Teensy Pin 18 (SDA) → Connect to:
├── SX1509 I/O Expander SDA
├── All VL53L1X TOF sensors SDA
├── VL53L0X TOF sensor SDA
└── Any other I2C sensors

Teensy Pin 19 (SCL) → Connect to:
├── SX1509 I/O Expander SCL  
├── All VL53L1X TOF sensors SCL
├── VL53L0X TOF sensor SCL
└── Any other I2C sensors
```

**Addresses:**
- SX1509 I/O Expander: 0x3F
- Additional SX1509 (if used): 0x3E

## MOTOR SYSTEM CONNECTIONS

### DC Motors (ESC/Servo Control):
```
Left Motor ESC:
├── Signal Wire    → Teensy Pin 0 (LEFT_MOTOR_PIN)
├── Power (Red)    → 5V or Battery+
└── Ground (Brown) → GND

Right Motor ESC:
├── Signal Wire    → Teensy Pin 1 (RIGHT_MOTOR_PIN)  
├── Power (Red)    → 5V or Battery+
└── Ground (Brown) → GND
```

### Motor Encoders:
```
Left Motor Encoder:
├── Channel A → Teensy Pin 2 (encoder1PinA) [INTERRUPT PIN]
├── Channel B → Teensy Pin 3 (encoder1PinB)
├── VCC       → 3.3V or 5V
└── GND       → GND

Right Motor Encoder:
├── Channel A → Teensy Pin 4 (encoder2PinA) [INTERRUPT PIN]
├── Channel B → Teensy Pin 5 (encoder2PinB)
├── VCC       → 3.3V or 5V
└── GND       → GND
```

### Main Servo (Gripper/Collection):
```
Main Servo:
├── Signal (Orange) → Teensy Pin 8 (ROBOT_SERVO_PIN)
├── Power (Red)     → 5V
└── Ground (Brown)  → GND

Test Servo (Development):
├── Signal (Orange) → Teensy Pin 9 (SERVO_PIN)
├── Power (Red)     → 5V  
└── Ground (Brown)  → GND
```

## SENSOR CONNECTIONS

### Ultrasonic Sensors (HC-SR04):
```
Ultrasonic Sensor A:
├── VCC  → 5V
├── GND  → GND
├── Trig → Teensy Pin 32 (AtrigPin)
└── Echo → Teensy Pin 33 (AechoPin) [INTERRUPT PIN]

Ultrasonic Sensor B:
├── VCC  → 5V
├── GND  → GND
├── Trig → Teensy Pin 30 (BtrigPin)
└── Echo → Teensy Pin 31 (BechoPin) [INTERRUPT PIN]

Ultrasonic C and D are on CON76 - need to manually do these connections.
Ultrasonic Sensor C:
├── VCC  → 5V
├── GND  → GND
├── Trig → Teensy Pin 22 (CtrigPin)
└── Echo → Teensy Pin 23 (CechoPin) [INTERRUPT PIN]

Ultrasonic Sensor D:
├── VCC  → 5V
├── GND  → GND
├── Trig → Teensy Pin 20 (DtrigPin)
└── Echo → Teensy Pin 21 (DechoPin) [INTERRUPT PIN]
```

### TOF Sensors (Time of Flight):
```
All TOF Sensors (VL53L1X Long Range + VL53L0X Short Range):
├── VCC   → 3.3V (NOT 5V!)
├── GND   → GND
├── SDA   → I2C Bus SDA (Teensy Pin 18)
├── SCL   → I2C Bus SCL (Teensy Pin 19)
└── XSHUT → SX1509 I/O Expander pins (see below)
```

### TOF Sensor XSHUT Control (via SX1509):
```
Short Range TOF (VL53L0X):
└── XSHUT → SX1509 Pin 1

Long Range TOF Array (VL53L1X) - 6 sensors:
├── Sensor [0][0] XSHUT → SX1509 Pin 2
├── Sensor [0][1] XSHUT → SX1509 Pin 3  
├── Sensor [0][2] XSHUT → SX1509 Pin 4
├── Sensor [1][0] XSHUT → SX1509 Pin 5
├── Sensor [1][1] XSHUT → SX1509 Pin 6
└── Sensor [1][2] XSHUT → SX1509 Pin 7
```

## INPUT/CONTROL CONNECTIONS

### User Interface:
```
Blue Start Button:
├── One Terminal → Teensy Pin 24 (BLUE_BUTTON_PIN) [INTERRUPT PIN]
├── Other Terminal → GND
└── Pull-up resistor (10kΩ) → 3.3V (or use internal pull-up)

Inductive Proximity Sensor:
├── Signal → Teensy Pin 25 (INDUCTIVE_PIN) [INTERRUPT PIN]
├── VCC    → 5V or 12V (check sensor specs)
└── GND    → GND
```

## SX1509 I/O EXPANDER SETUP

### SX1509 Connections:
```
SX1509 I/O Expander:
├── VCC → 3.3V
├── GND → GND  
├── SDA → Teensy Pin 18 (I2C Data)
├── SCL → Teensy Pin 19 (I2C Clock)
└── ADDR → Configure for address 0x3F
```

### SX1509 Pin Assignments:
```
SX1509 Pin 1 → VL53L0X XSHUT (Short range TOF)
SX1509 Pin 2 → VL53L1X[0][0] XSHUT (Long range TOF)
SX1509 Pin 3 → VL53L1X[0][1] XSHUT (Long range TOF)
SX1509 Pin 4 → VL53L1X[0][2] XSHUT (Long range TOF)  
SX1509 Pin 5 → VL53L1X[1][0] XSHUT (Long range TOF)
SX1509 Pin 6 → VL53L1X[1][1] XSHUT (Long range TOF)
SX1509 Pin 7 → VL53L1X[1][2] XSHUT (Long range TOF)
SX1509 Pins 8-15 → Available for expansion
```

## WIRING CHECKLIST

### Before Powering On:
- [ ] All ground connections secure
- [ ] No short circuits between power and ground
- [ ] 3.3V components not connected to 5V
- [ ] I2C pull-up resistors in place (usually on breakout boards)
- [ ] All interrupt pins properly connected
- [ ] Motor ESC calibration completed
- [ ] Servo horns properly aligned

### Power-On Sequence:
1. Connect Teensy via USB (programming power)
2. Upload and test basic code
3. Connect external power (7-12V to VIN)
4. Test each subsystem individually
5. Run full system integration test

## TROUBLESHOOTING TIPS

### Common Issues:
- **I2C not working:** Check SDA/SCL connections and pull-up resistors
- **Encoders not counting:** Verify interrupt pin connections
- **Servos jittering:** Check power supply capacity
- **TOF sensors not responding:** Verify 3.3V power and XSHUT control
- **Ultrasonic timeout:** Check trigger/echo pin connections

### Voltage Levels:
- **Teensy I/O:** 3.3V (5V tolerant inputs)
- **TOF Sensors:** 3.3V ONLY
- **Ultrasonic:** 5V preferred
- **Motors/Servos:** 5V-6V typical
- **Logic Signals:** 3.3V from Teensy, 5V tolerant

## WIRE COLOR CONVENTIONS

### Suggested Color Coding:
```
Power:
├── Red    → +5V / VIN
├── Black  → GND
└── Orange → +3.3V

Signals:
├── Yellow → Digital I/O
├── Green  → I2C SDA
├── Blue   → I2C SCL
├── White  → PWM/Servo signals
└── Purple → Interrupt signals
```

**⚠️ SAFETY NOTES:**
- Double-check all connections before applying power
- Use appropriate gauge wire for motor currents
- Ensure proper voltage levels for each component
- Keep I2C wires short and use twisted pairs if possible
- Provide adequate power supply capacity for all motors and servos