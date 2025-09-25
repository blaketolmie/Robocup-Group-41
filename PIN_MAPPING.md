# Teensy 4.0 Pin Mapping for RoboCup Group 41
# ============================================
# Last Updated: August 29, 2025
# This file documents all pin assignments for the robocup robot

## MOTOR CONTROL PINS
# Pin 0  - LEFT_MOTOR_PIN     - Left DC motor control (PWM/Servo signal)
# Pin 1  - RIGHT_MOTOR_PIN    - Right DC motor control (PWM/Servo signal)
# Pin 8  - ROBOT_SERVO_PIN    - Main robot servo control

## ENCODER PINS (Interrupt capable)
# Pin 2  - encoder1PinA       - Left motor encoder channel A (interrupt)
# Pin 3  - encoder1PinB       - Left motor encoder channel B
# Pin 4  - encoder2PinA       - Right motor encoder channel A (interrupt)
# Pin 5  - encoder2PinB       - Right motor encoder channel B

## TOF (Time of Flight) SENSOR XSHUT PINS
# Long Range TOF Array (L1):
# Pin 2  - xshutPinsL1[0][0]  - TOF sensor shutdown control
# Pin 3  - xshutPinsL1[0][1]  - TOF sensor shutdown control
# Pin 4  - xshutPinsL1[0][2]  - TOF sensor shutdown control
# Pin 5  - xshutPinsL1[1][0]  - TOF sensor shutdown control
# Pin 6  - xshutPinsL1[1][1]  - TOF sensor shutdown control
# Pin 7  - xshutPinsL1[1][2]  - TOF sensor shutdown control
# Short Range TOF (L0):
# Pin 1  - xshutPinsL0[0]     - TOF sensor shutdown control

## ULTRASONIC SENSOR PINS
# Ultrasonic Sensor A:
# Pin 32 - AtrigPin           - Ultrasonic A trigger pin
# Pin 33 - AechoPin           - Ultrasonic A echo pin (interrupt)

# Ultrasonic Sensor B:
# Pin 30 - BtrigPin           - Ultrasonic B trigger pin
# Pin 31 - BechoPin           - Ultrasonic B echo pin (interrupt)

# Ultrasonic Sensor C:
# Pin 22 - CtrigPin           - Ultrasonic C trigger pin
# Pin 23 - CechoPin           - Ultrasonic C echo pin (interrupt)

# Ultrasonic Sensor D:
# Pin 20 - DtrigPin           - Ultrasonic D trigger pin
# Pin 21 - DechoPin           - Ultrasonic D echo pin (interrupt)

## INPUT/CONTROL PINS  
# Pin 24 - BLUE_BUTTON_PIN    - Blue start/stop button (interrupt)
# Pin 25 - INDUCTIVE_PIN      - Inductive proximity sensor (interrupt)

## TEST/DEVELOPMENT PINS
# Pin 9  - SERVO_PIN          - Test servo for gripper/collection system

## PIN CONFLICT WARNINGS
# ⚠️  CRITICAL: Pins 2-7 have overlapping assignments!
#     - Pins 2,3,4,5 are used for BOTH encoders AND TOF sensors
#     - This will cause hardware conflicts
#     - Need to reassign either encoder pins or TOF XSHUT pins

## INTERRUPT CAPABLE PINS (Teensy 4.0)
# All digital pins on Teensy 4.0 can be used for interrupts
# Currently using interrupts on:
# - Pin 2  (encoder1PinA)
# - Pin 4  (encoder2PinA)  
# - Pin 21 (DechoPin)
# - Pin 23 (CechoPin)
# - Pin 24 (BLUE_BUTTON_PIN)
# - Pin 25 (INDUCTIVE_PIN)
# - Pin 31 (BechoPin)
# - Pin 33 (AechoPin)

## I2C PINS (For TOF sensors)
# Pin 18 - SDA (I2C Data)
# Pin 19 - SCL (I2C Clock)

## SERIAL/DEBUG PINS
# Pin 0  - RX1 (Serial1 receive) - Also used for LEFT_MOTOR_PIN
# Pin 1  - TX1 (Serial1 transmit) - Also used for RIGHT_MOTOR_PIN
# USB    - Serial (main debug output via USB)

## POWER PINS
# VIN    - Input voltage (7-12V recommended)
# 3.3V   - 3.3V regulated output
# 5V     - 5V output (when powered via USB or VIN)
# GND    - Ground (multiple pins available)

## NOTES:
# 1. Teensy 4.0 runs at 3.3V logic levels
# 2. All pins are 5V tolerant when used as inputs
# 3. PWM available on most pins
# 4. Some pins have overlapping assignments - review needed
# 5. Consider using different pins for encoders to avoid TOF conflicts