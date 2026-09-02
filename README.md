# TrueShifter V2

TrueShifter is an open-source hardware bridge that translates bHaptics BLE protocols into TrueGear protocols in real time. By utilizing a **single microcontroller** acting concurrently as both a BLE Peripheral and BLE Central, it allows TrueGear haptic vests to be recognized natively as a TactSuit X40.

This enables native haptic feedback without requiring background services, driver installations, or game modifications on your PC or headset.

## Supported Platforms
* **Meta Quest Standalone** 
* **Pico Standalone** 
* **PCVR** (via bHaptics Player on Windows)
* **Mobile** (bHaptics Player for Android / iOS)

## Web Installer
You can flash the pre-compiled firmware directly to your ESP32 board using a Chromium-based browser (Chrome, Edge). No drivers, IDE installation, or coding required. Simply select your board model and click install.

**[Launch TrueShifter Web Installer](https://pawel11223.github.io/TrueShifter/)**

## Hardware Requirements
* **1x ESP32 Microcontroller** (See compatibility below)
* A standard USB cable & 5V power source (e.g., a PC USB port, phone charger, etc.)

> **✅ Supported Boards (Version 2.0.0+):**
> * **ESP32 Classic** (WROOM / NodeMCU-32S)
> * **ESP32-C3** (e.g., C3 SuperMini - highly recommended for its compact size)
> * **ESP32-S3**
> * **ESP32 WROVER**
>
> **❌ Unsupported Boards:**
> * **ESP8266** (NodeMCU, Wemos D1) 
> * **ESP32-S2** 
> * **ESP32-P4** 
> * **ESP32-H Series** 

## Acknowledgments
This project relies heavily on the incredible reverse-engineering work done by the **[SenseShift](https://github.com/senseshift/)** project.

## Community

Join our Discord server for support, discussions, bug reports, and updates: [**WeaVR Discord**](https://discord.gg/rcnSCGr6tx)

## Support

If you find this project useful, consider grabbing me a coffee. Any support is greatly appreciated and helps keep the project going!

<a href="https://buymeacoffee.com/weavr" target="_blank">
  <img src="https://img.buymeacoffee.com/button-api/?text=Buy%20me%20a%20coffee&emoji=%E2%98%95&slug=weavr&button_colour=FFDD00&font_colour=000000&font_family=Cookie&outline_colour=000000&coffee_colour=ffffff" alt="Buy Me a Coffee">
</a>

### Disclaimer

TrueShifter is an independent, community-driven open-source project. It is not affiliated with, endorsed by, certified by, or in any way officially connected with bHaptics, TrueGear, Meta, or Pico.

All product names, logos, brands, trademarks, and registered trademarks mentioned in this project are the property of their respective owners. Their use in this repository is strictly for identification and educational purposes (interoperability) and does not imply any association.

### License

This project is licensed under the [GPL-3.0 License](https://www.gnu.org/licenses/gpl-3.0.en.html), in accordance with the licensing terms of the original SenseShift project.
