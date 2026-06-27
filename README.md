# TrueShifter

TrueShifter is a dual-MCU bridge that translates bHaptics BLE protocols into TrueGear protocols in real time. It allows TrueGear haptic vests to be recognized natively as a TactSuit X40 within the ecosystem.

This enables native haptic feedback without requiring background services, driver installations, or game modifications.

## Supported Platforms
* **PCVR** (via bHaptics Player on Windows)
* **Meta Quest Standalone** 
* **Pico Standalone** 
* **Mobile** (bHaptics Player for Android / iOS)

## Web Installer
You can flash the pre-compiled firmware directly to your ESP32 boards using a Chromium-based browser (Chrome, Edge). No drivers or IDE installation required.

**[Launch TrueShifter Web Installer](https://pawel11223.github.io/TrueShifter/)**

## Hardware Requirements
* 2x **ESP32-WROOM-32** development boards
* Jumper wires
* 5V USB Power source

> **⚠️ Board Compatibility:**  
> The pre-compiled web installer uses GPIO 16 & 17. Due to hardware differences, the following boards are **NOT supported** out of the box:
> * **ESP32-WROVER** - *Will crash (Kernel Panic)*. These boards use pins 16/17 for PSRAM.
> * **ESP8266** (NodeMCU, Wemos D1) - Hardware does not support Bluetooth BLE.
> * **ESP32-S2** - Hardware does not support Bluetooth BLE.
> * **Mini/Micro boards** (e.g., ESP32-C3 SuperMini) - Lack physical pins 16/17. *(You can still use them if you compile the project from source and reassign the UART pins manually).*

## Wiring Diagram
The two boards communicate via hardware UART. Connect the pins as shown below:

| Board 1 (Protocol Translator) | Connection | Board 2 (TrueGear Bridge) | Description |
| :--- | :---: | :--- | :--- |
| **GND** | ↔ | **GND** | Common ground (Crucial for UART) |
| **VIN (or 5V)** | ↔ | **VIN (or 5V)** | Power sharing (Powers both via one USB) |
| **GPIO 17 (TX2)** | → | **GPIO 16 (RX2)** | Haptic data stream (1 to 2) |
| **GPIO 16 (RX2)** | ← | **GPIO 17 (TX2)** | Haptic data stream (2 to 1) |


## Acknowledgments
This project relies heavily on the incredible reverse-engineering work done by the **[SenseShift](https://github.com/senseshift/)** project.

## Disclaimer
TrueShifter is an independent, community-driven open-source project. It is not affiliated with, endorsed by, certified by, or in any way officially connected with bHaptics Inc., TrueGear, Meta Platforms, Inc., or Pico Interactive.

All product names, logos, brands, trademarks, and registered trademarks mentioned in this project are the property of their respective owners. Their use in this repository is strictly for identification and educational purposes (interoperability) and does not imply any association.

## License
This project is licensed under the [GPL-3.0 License](https://www.gnu.org/licenses/gpl-3.0.en.html), in accordance with the licensing terms of the original SenseShift project.

---

### Support
If you find this project useful, consider grabbing me a coffee. Any support is greatly appreciated and helps keep the project going! 
<br>[Support via PayPal](https://www.paypal.com/paypalme/weavr1)
