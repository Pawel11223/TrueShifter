# TrueShifter V2.1 (nRF)

nRF Connect SDK / Zephyr firmware for TrueShifter.

| Item | Value |
| --- | --- |
| Target board | promicro_nrf52840/nrf52840/uf2 |
| Board name | nice!nano v2 (nRF52840) |
| nRF Connect SDK | v3.4.1 |
| Zephyr | 4.4.2 |

## Building

```sh
west build --build-dir build . --pristine --board promicro_nrf52840/nrf52840/uf2 --no-sysbuild -- -DCONFIG_DEBUG_THREAD_INFO=y -DCONF_FILE=prj.conf
```

Flash artifact: `build/zephyr/TrueShifterV21.uf2` (also `.bin` / `.hex` / `.elf`). Flash by copying the UF2 file to the bootloader's USB mass-storage drive (double-tap reset).

## Layout

- `src/main.c` — application source (single file)
- `prj.conf` — Zephyr / Kconfig configuration
- `CMakeLists.txt` — application build description
