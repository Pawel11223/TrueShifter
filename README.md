# TrueShifter

TrueShifter is a dual-MCU bridge that translates bHaptics BLE protocols into TrueGear BLE protocols in real-time. It allows TrueGear haptic vests to be recognized as a bHaptics TactSuit X40.

This enables native haptic feedback in any VR game supported by bhaptics. 

## Web Installer

You can flash the pre-compiled firmware directly to your ESP32 boards using a Chromium-based browser (Chrome, Edge). No drivers or IDE installation required.

**[Launch TrueShifter Web Installer](https://twoj-nick.github.io/TrueShifter/)**

## Hardware Requirements

* 2x ESP32-WROOM-32 development boards
* Jumper wires

## Wiring Diagram

The two boards communicate via hardware UART. Connect the pins as shown below:

| Board 1 | Connection | Board 2 | Description |
| :--- | :---: | :--- | :--- |
| **GND** | ↔ | **GND** | Common ground (Required for UART) |
| **VIN (or 5V)** | ↔ | **VIN (or 5V)** | Power sharing (Allows powering both via one USB) |
| **GPIO 17 (TX2)** | → | **GPIO 16 (RX2)** | Haptic data stream (1 to 2) |
| **GPIO 16 (RX2)** | ← | **GPIO 17 (TX2)** | Battery telemetry data (2 to 1) |

## Hardware Test

Board 2 features a built-in hardware test. To verify the BLE connection with the TrueGear vest without using a PC or VR headset, short **GPIO 22** and **GPIO 23** together. This will trigger a continuous full-body vibration on the vest.


## Acknowledgments

This project relies heavily on the incredible reverse-engineering work done by the **[SenseShift](https://github.com/senseshift/)** project.

## License

This project is licensed under the [GPL-3.0 License](https://www.gnu.org/licenses/gpl-3.0.en.html), in accordance with the licensing terms of the original SenseShift project.
