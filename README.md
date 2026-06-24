# ParkingLotSystem

## Final Board Programs
- `ParkingLotSystem.ino`: Arduino Mega main controller for entry gate, exit gate, LCD, buzzer, LEDs, and Uno R4 status output.
- `UnoR4WiFi_MatrixReceiver_CopyPaste.txt`: final copy/paste sketch for the Uno R4 WiFi LED matrix receiver.
- `boards/uno-r4-wifi/UnoR4WiFi_MatrixReceiver.ino`: tracked repo copy of the same Uno R4 receiver sketch.

## Final Mega Pin Map
- Ultrasonic trigger: D7
- Ultrasonic echo: D6
- Touch sensor: D9
- Entry limit switch: D11
- Buzzer mute button: D8
- Joystick switch: D43
- Exit limit switch: D44
- Buzzer: D2
- LED red: D3
- LED yellow: D4
- LED green: D5
- Entry servo: D10
- Exit servo: D45
- LCD I2C: SDA D20, SCL D21
- Uno R4 status link: Mega `Serial2`

## Final Uno R4 Role
- Receives newline-delimited exit status messages from the Mega on hardware serial.
- Displays `O5..O1`, `O0`, `C5..C1`, and `C0` on the LED matrix.
- Shows the sad face only when communication is lost.

## Submission Notes
- Keep the sketch inside the `ParkingLotSystem` folder so the Arduino IDE sees `ParkingLotSystem.ino` correctly.
- The Mega and Uno R4 programs are intentionally kept separate for submission.
- Backup `.bak` files in the repo are history snapshots and are not required for upload.
