# Parking Lot Entry System - Arduino Mega

## Required Arduino Folder Layout
Keep the sketch in a folder with the same name as the .ino file:

ParkingLotSystem/
- ParkingLotSystem.ino
- README.md
- src/
- include/
- lib/

Important:
- The folder name and sketch file name must match exactly: ParkingLotSystem <-> ParkingLotSystem.ino
- Open the sketch by selecting ParkingLotSystem.ino from inside the ParkingLotSystem folder
- Store reusable helper files in src/, headers in include/, and custom libraries in lib/

## Project Overview
An automated parking lot entry system that detects vehicles, prompts for user input, controls a gate servo, and provides LCD feedback.

## Current Pin Assignments
- **Ultrasonic Sensor (WIRED)**
  - Trigger: D6
  - Echo: D7

## Pin Assignments (To Configure)
- **Servo (Gate Control)**: D9 (PWM)
- **Push Button (User Input)**: D8
- **Green LED (Entry Allowed)**: D10
- **Red LED (Gate Closed)**: D11
- **LCD I2C**: SDA (D20), SCL (D21) on Mega

## Assignment Requirements
1. ✓ Vehicle Detection (Ultrasonic)
2. ✓ User Input (Push Button)
3. Gate Control (Servo Motor)
4. Vehicle Passage Detection (PIR or IR break-beam)
5. LCD Display (16x2 I2C)
6. LEDs (Green & Red)
7. State Machine Logic
8. Optional: SD card, buzzer, menus

## Next Steps
1. Wire remaining components to assigned pins
2. Add LCD library and servo library
3. Fill in TODO sections
4. Test each sensor/actuator individually
5. Integrate full system

## Notes
- Arduino Mega has plenty of PWM pins for servo and future expansion
- I2C LCD uses default address 0x27 (verify with I2C scanner if issues)
- Ultrasonic sensor is distance-based; consider PIR or IR beam for vehicle passage confirmation
