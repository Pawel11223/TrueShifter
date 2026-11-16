[![Discord](https://img.shields.io/discord/1495376023070441482?style=for-the-badge&logo=discord)](https://discord.gg/rcnSCGr6tx) &nbsp;&nbsp;&nbsp;&nbsp;
[![Buy Me a Coffee](https://img.shields.io/badge/Buy%20me%20a%20coffee-support%20for%20more%20projects-FFDD00?style=for-the-badge&logo=buymeacoffee&logoColor=black)](https://buymeacoffee.com/weavr)



# TrueShifter V3

TrueShifter is an open-source hardware bridge that translates bHaptics BLE protocols into TrueGear protocols in real time. By utilizing a **single microcontroller** acting concurrently as both a BLE Peripheral and BLE Central, it allows TrueGear haptic vests to be recognized natively as a TactSuit X40.

This enables native haptic feedback without requiring background services, driver installations, or game modifications on your PC or headset.

## Supported Platforms

* **Meta Quest Standalone** 
* **Pico Standalone** 
* **PCVR** (via bHaptics Player on Windows)
* **Mobile** (bHaptics Player for Android / iOS)

## Web Installer

You can flash the pre-compiled firmware directly to your ESP32/nRF52 board using a Chromium-based browser (Chrome, Edge). No drivers, IDE installation, or coding required. Simply select your board model and click install.

**[Launch TrueShifter Web Installer](https://pawel11223.github.io/TrueShifter/)**

## Hardware Requirements

* **1x ESP32/nRF52 Microcontroller** (See compatibility below)
* A standard USB cable & 5V power source (e.g., a PC USB port, phone charger, etc.)

|             | Info:                           | List of supported boards:                | Work with:              |
|-------------|---------------------------------|------------------------------------------|-------------------------|
| V2.1        | Vest emulation                  | ESP32 Classic <br> ESP32 C3 <br> ESP32 S3 <br> nRF52840 | android <br> quest/pico <br> PCVR |
| V3          | Vest emulation <br> EMS arm bands emulation  | nRF52840                                 | PCVR                    |
| unsupported |                                 | ESP8266 <br> ESP32 S2                         |                         |

## Acknowledgments

This project relies heavily on the incredible reverse-engineering work done by the **[SenseShift](https://github.com/senseshift/)** project.

### Disclaimer

TrueShifter is an independent, community-driven open-source project. It is not affiliated with, endorsed by, certified by, or in any way officially connected with bHaptics, TrueGear, Meta, or Pico.

All product names, logos, brands, trademarks, and registered trademarks mentioned in this project are the property of their respective owners. Their use in this repository is strictly for identification and educational purposes (interoperability) and does not imply any association.

### License

This project is licensed under the [GPL-3.0 License](https://www.gnu.org/licenses/gpl-3.0.en.html), in accordance with the licensing terms of the original SenseShift project.
