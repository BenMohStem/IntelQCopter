# PlatformIO configuration reference: RP2040 and RP2350

This guide covers the RP2040 and RP2350 microcontroller families in PlatformIO, with emphasis on the Raspberry Pi Pico/Pico W/Pico 2/Pico 2 W and the `raspberrypi` platform integration used by the Arduino-Pico core.

> **Scope of “complete”:** PlatformIO configuration is extensible. A platform, framework, custom board JSON, or extra script can introduce arbitrary keys, so there is no closed list of every possible `board_build.*` spelling. This reference lists the RP-family keys and options documented or consumed by the current `platform-raspberrypi` builder and Earle Philhower Arduino-Pico integration, plus the common PlatformIO keys used to build, upload, monitor, and debug those boards. It does not attempt to enumerate every vendor board name in the community platform.

## 1. The configuration layers

```ini
[platformio]
default_envs = pico2_arm

[env:pico2_arm]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico2
framework = arduino
board_build.mcu = rp2350
```

- `[platformio]` holds project-wide PlatformIO settings.
- `[env:pico2_arm]` defines one named build environment. `pico2_arm` is just a label.
- `platform` selects the build-platform package and its board/toolchain integration.
- `board` selects a board manifest. That manifest supplies defaults such as MCU, clock, RAM and flash capacity.
- `framework` selects the APIs/runtime built for the board.
- `board_build.NAME` overrides a board manifest's `build.NAME` property. It does **not** mean that every spelling is meaningful: the selected platform/framework must read that property.
- `board_upload.NAME` overrides a board manifest's `upload.NAME` property.
- `build_flags` passes compiler/preprocessor flags; it is separate from board identity and framework selection.

PlatformIO reference: [project configuration](https://docs.platformio.org/en/latest/projectconf/).

## 2. RP2040 and RP2350 board values

These are the common Pico-family IDs in the community Raspberry Pi platform. Other RP2040/RP2350 vendor boards are available, and their IDs depend on the installed platform version. Ask PlatformIO for its installed list with `pio boards raspberrypi`.

| `board = ...` | Hardware | MCU default | Notes |
|---|---|---|---|
| `board = pico` | Raspberry Pi Pico | RP2040 | Also used by the upstream PlatformIO platform; Arduino core choice depends on platform. |
| `board = rpipico` | Raspberry Pi Pico | RP2040 | Arduino-Pico-specific board ID; selects the Earle core by default. |
| `board = rpipicow` | Raspberry Pi Pico W | RP2040 | W variant includes wireless hardware. |
| `board = rpipico2` | Raspberry Pi Pico 2 | RP2350 ARM | Board manifest defaults to ARM; `board_build.mcu = rp2350` makes that intent explicit. |
| `board = rpipico2w` | Raspberry Pi Pico 2 W | RP2350 ARM | W variant includes wireless hardware. |

The upstream PlatformIO `raspberrypi` board page documents `pico`; the community platform includes `rpipico*` IDs. Board IDs are platform-specific, so do not assume every ID works with every `platform` URL. [Upstream Pico board](https://docs.platformio.org/en/latest/boards/raspberrypi/pico.html), [community platform examples](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/examples/picosdk-blink/platformio.ini), [Pico 2 board manifest](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/boards/rpipico2.json).

## 3. Framework values

The community platform currently declares these framework names:

| `framework = ...` | Meaning |
|---|---|
| `framework = arduino` | Arduino API: `setup()`, `loop()`, `Serial`, Arduino libraries. The platform/core selection determines which Arduino core implementation is used. |
| `framework = picosdk` | Raspberry Pi Pico SDK. Usually a C/C++ `main()` entry point and Pico SDK APIs. |
| `framework = mbed-ce` | Mbed CE framework, when supported by the selected board and toolchain. |

Use `picosdk`, not `sdk`, for the framework name in this platform. `platform` and `framework` are separate settings: the platform supplies integration; the framework supplies the programming environment. [Community `platform.json`](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/platform.json).

## 4. MCU and CPU architecture

| `board_build.mcu = ...` | Target |
|---|---|
| `board_build.mcu = rp2040` | RP2040 target, such as Pico/Pico W. Usually already supplied by the board manifest. |
| `board_build.mcu = rp2350` | RP2350 ARM Cortex-M33 target. It is the default for Pico 2 in the community board definition, but can be stated explicitly. |
| `board_build.mcu = rp2350-riscv` | RP2350 Hazard3 RISC-V target. This switches the toolchain and architecture selection. |

For study, you can include `board_build.mcu = rp2350` even though the Pico 2 manifest already selects it. The Arduino-Pico guide documents the RISC-V override; the platform builder selects the RISC-V toolchain when this value is set. [Arduino-Pico architecture guidance](https://github.com/earlephilhower/arduino-pico/blob/master/docs/platformio.rst#selecting-the-cpu-architecture), [platform builder](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/builder/main.py).

## 5. RP-family `board_build.*` keys

These are the documented or actively consumed `board_build.*` properties for the Raspberry Pi platform and Arduino-Pico integration.

| Key | What it does | Applicability / caution |
|---|---|---|
| `board_build.mcu = ...` | Selects the MCU/compiler target. | RP2040, RP2350 ARM, or RP2350 RISC-V as above. |
| `board_build.core = earlephilhower` | Selects Earle Philhower's Arduino-Pico core when the chosen board can use multiple Arduino cores. | Relevant to `framework = arduino`; the Arduino-Pico guide identifies `board = pico` and `board = nanorp2040connect` as needing this switch. Arduino-Pico-only IDs such as `rpipico` do not need it; `rpipico2` already declares this core. It does not select the PlatformIO platform by itself. |
| `board_build.filesystem_size = 0m` | Reserves zero flash bytes for the Arduino-Pico LittleFS filesystem. | Details in section 6. Use a nonzero value only if the firmware will build/use a filesystem image. |
| `board_build.f_cpu = 133000000L` | Overrides the board's CPU-clock value passed to the framework. | The board has a default; this line does not itself guarantee stable overclocking. Use a supported clock and verify timing/peripherals. |
| `board_build.generate_disassembly = yes` | Asks the community platform builder to create disassembly listing files. | Builder-specific diagnostic option; use `yes`, `true`, or `on` to enable. |
| `board_build.arduino.earlephilhower.boot2_source = ...` | Chooses Arduino-Pico's second-stage bootloader source for a compatible flash chip. | Normally leave the board default. Only change for a custom board/flash configuration; use a valid source filename from the core's `boot2/rp2040` set. |
| `board_build.arduino.earlephilhower.usb_manufacturer = ...` | Sets the USB manufacturer string. | Arduino-Pico USB descriptor customization. |
| `board_build.arduino.earlephilhower.usb_product = ...` | Sets the USB product string. | Arduino-Pico USB descriptor customization. |
| `board_build.arduino.earlephilhower.usb_vid = 0x....` | Overrides the USB vendor ID. | Choose an ID you own or are authorized to use; do not impersonate another vendor. |
| `board_build.arduino.earlephilhower.usb_pid = 0x....` | Overrides the USB product ID. | Must be consistent with your USB identity and deployment. |
| `board_build.picosdk.board_header = pimoroni_plasma2350.h` | Selects the Pico SDK board header for SDK pin/board definitions. | Pico SDK / platform-specific; use a header available in the installed SDK. A community platform example shows this override. |

### Common `board_build.*` spellings that need RP-specific care

PlatformIO allows board-manifest overrides using `board_{OBJECT.PATH}`. That means these familiar spellings can parse, but their behavior depends on whether this RP platform/framework reads the overridden property.

| Key | RP2040/RP2350 interpretation |
|---|---|
| `board_build.f_flash = ...` | A generic PlatformIO board override. It is not the Arduino-Pico RP flash-capacity setting; don't use it to claim a different flash chip or capacity. |
| `board_build.flash_mode = ...` | Common on other platforms, but not the documented way to select RP2040/RP2350 external flash behavior. Boot2/core and board definitions handle that. |
| `board_build.flash_size = ...` | See the correction below. It does not set this RP builder's physical-flash capacity. |
| `board_build.ldscript = ...` | Generic linker-script override. Only use with a framework/build that explicitly consumes it; Pico SDK projects commonly define memory layout through their SDK/CMake setup. |
| `board_build.variant = ...` | Overrides a board variant property if the selected framework consumes it. Changing it can silently select incorrect pin mappings; use a matching board definition instead. |
| `board_build.extra_flags = ...` | Overrides the board's extra compiler definitions when consumed. It can replace board-provided flags, so `build_flags` is usually safer for adding a project define. |

The generic override syntax is documented by PlatformIO, but that does not make each option meaningful for every platform. [PlatformIO platform/board options](https://docs.platformio.org/en/latest/projectconf/section_env_general.html).

### `board_build.flash_size` — important correction

`board_build.flash_size = 2MB` is **not the documented flash-capacity control for this RP platform**. The current community builder reads the board's `upload.maximum_size` as the board's total flash capacity, then calculates the app/filesystem layout from that value and `board_build.filesystem_size`. It does not read `build.flash_size` to determine capacity. Consequently, `board_build.flash_size` may be accepted as a generic board-property override but have no effect on the RP flash layout.

For normal Pico boards, keep the flash size from the board definition. For a custom board with a different physical flash chip, use a correct custom board manifest (including its `upload.maximum_size`) or a verified `board_upload.maximum_size` override supported by the installed PlatformIO version. That value must describe the actual hardware. Do not copy ESP32 examples that use `board_build.flash_size`; ESP32 platform behavior does not establish RP platform behavior. [RP builder flash-size calculation](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/builder/main.py#L50-L95), [Arduino-Pico flash-size guidance](https://github.com/earlephilhower/arduino-pico/blob/master/docs/platformio.rst#flash-size).

The matching override spelling for the manifest's `upload.maximum_size` property is `board_upload.maximum_size`, not `board_build.flash_size`. In the RP builder this field is used as total flash capacity for its layout math; only change it for a real custom board with known flash hardware. The board manifests also carry `upload.maximum_ram_size`, which describes on-chip RAM and should not be changed to advertise more RAM than exists. The RP2350 PSRAM override is separately named `board_upload.psram_length` (section 8).

Other arbitrary `board_build.SOMETHING` spellings are possible because PlatformIO maps them to board properties, but they only do useful work if the selected platform/framework consumes that property. They are not automatically valid settings just because the INI parser accepts them.

## 6. Detailed explanation: `board_build.filesystem_size`

```ini
board_build.filesystem_size = 0m
```

This controls the amount of **on-board flash reserved for the Arduino-Pico LittleFS filesystem**. It is not the MCU's RAM size, not the physical flash-chip size, and not the firmware's maximum clock.

Examples:

```ini
; Reserve no filesystem area; leave nearly all flash for the program/EEPROM layout.
board_build.filesystem_size = 0m

; Reserve 512 KiB for LittleFS.
board_build.filesystem_size = 512k

; Reserve 0.5 MiB for LittleFS (same size).
board_build.filesystem_size = 0.5m

; Reserve 1 MiB for LittleFS.
board_build.filesystem_size = 1m
```

The Arduino-Pico platform integration accepts byte, kilobyte, and megabyte suffixes (`b`, `k`/`kb`, `m`/`mb`) and fractional sizes. The platform computes filesystem start/end addresses from the board's declared total flash and the requested filesystem size. It also reserves flash for EEPROM emulation. The checked builder source currently subtracts 4096 bytes for this reserve; the Arduino-Pico board generator has architecture-specific flash-menu accounting, so the exact available app limit can vary with core/platform revisions. Use the size report from your resolved build instead of hand-calculating for another revision. [Arduino-Pico filesystem documentation](https://github.com/earlephilhower/arduino-pico/blob/master/docs/platformio.rst#flash-size), [builder calculation](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/builder/main.py#L50-L95), [board generator](https://github.com/earlephilhower/arduino-pico/blob/master/tools/makeboards.py).

With `0m`, there is no usable LittleFS partition: do not expect `LittleFS.begin()` or `pio run -t uploadfs` to work as though space had been reserved. EEPROM emulation is separate and may still consume its reserved flash area. If the program needs to store files, choose a nonzero filesystem size and check that the application still fits.

## 7. Detailed explanation: `board_build.core = earlephilhower`

```ini
framework = arduino
board_build.core = earlephilhower
```

`framework = arduino` says “build an Arduino program.” `board_build.core = earlephilhower` selects which Arduino core implementation to use when the selected board definition offers a choice.

- On the upstream `raspberrypi` platform, `board = pico` normally selects Arduino Mbed. Add `board_build.core = earlephilhower` to select Earle Philhower's Arduino-Pico core through the community platform.
- With the Arduino-Pico-only ID `board = rpipico`, `rpipicow`, `rpipico2`, or `rpipico2w`, the Earle core is already the board default. The line is redundant, though it can document intent.
- This line does **not** turn `framework = picosdk` into Arduino; it applies to an Arduino framework build.
- The platform still matters. For the documented Earle core setup, use `platform = https://github.com/maxgerhardt/platform-raspberrypi.git`.

The guide explicitly describes the `pico` core switch and says it is unnecessary for Arduino-Pico-only board IDs. [Arduino-Pico core selection](https://github.com/earlephilhower/arduino-pico/blob/master/docs/platformio.rst#selecting-the-new-core).

## 8. RP2350 PSRAM options

Only configure these for an RP2350 board that physically has PSRAM. Standard Raspberry Pi Pico 2 and Pico 2 W boards do not gain PSRAM from these settings.

```ini
; Tell the core/platform the actual PSRAM capacity, in bytes.
board_upload.psram_length = 1048576

; If required by the specific board, define its PSRAM chip-select GPIO.
build_flags = -DRP2350_PSRAM_CS=47
```

Documented capacity examples are `1048576` (1 MiB), `2097152` (2 MiB), and `4194304` (4 MiB). The CS pin is a compile-time definition because it is board wiring, not a generic flash/board setting. Many supported boards provide their PSRAM pin definition already, so do not duplicate it without checking that board's manifest and variant. [Arduino-Pico PSRAM configuration](https://github.com/earlephilhower/arduino-pico/blob/master/docs/platformio.rst#psram-size).

## 9. Other common PlatformIO keys for RP boards

These are not all `board_build.*` keys; they configure general build, upload, monitor, or debug behavior.

| Key | Brief meaning / common example |
|---|---|
| `default_envs = pico2_arm` | Default environment to build. |
| `extends = common` | Inherit values from another section/environment. |
| `platform_packages = ...` | Pin/replace a framework or tool package, for example a chosen Arduino-Pico Git revision. Advanced; pin revisions for repeatable builds. |
| `build_flags = ...` | Compiler definitions/options; see section 10. |
| `build_unflags = ...` | Remove compiler flags inherited from defaults. |
| `build_type = debug` | Select a debug-oriented build configuration where supported. |
| `build_src_filter = ...` | Include/exclude source files from the build. |
| `lib_deps = ...` | Declare external libraries. |
| `lib_ignore = ...` | Exclude named libraries from dependency selection. |
| `upload_protocol = picotool` | Select upload method. Documented methods include `picotool`, `mbed`, `picoprobe`, `cmsis-dap`, `jlink`, `raspberrypi-swd`, and `blackmagic` where tool/board setup supports them. |
| `upload_port = ...` | Select an upload device/port when needed. `picotool` can discover BOOTSEL devices. |
| `upload_flags = ...` | Extra uploader arguments. |
| `debug_tool = cmsis-dap` | Select a compatible SWD/debug probe. |
| `debug_port = ...` | Debug adapter connection endpoint where relevant. |
| `debug_speed = 5000` | Debug adapter speed in kHz for supported tools. |
| `debug_init_break = ...` | Initial debugger breakpoint. |
| `debug_extra_cmds = ...` | Extra debugger commands. |
| `monitor_port = ...` | Serial/USB serial port to monitor. |
| `monitor_speed = 115200` | UART serial baud rate; it does not set USB speed or CPU frequency. |
| `monitor_filters = ...` | Transform/filter serial monitor output. |
| `monitor_rts = ...`, `monitor_dtr = ...` | Set serial control lines where the port/driver supports them. |

The exact upload/debug methods depend on the selected platform version and physical probe. The Arduino-Pico guide lists supported debug probes and notes that supported debug tools can also be used as upload protocols. [Arduino-Pico debugging](https://github.com/earlephilhower/arduino-pico/blob/master/docs/platformio.rst#debugging).

## 10. Arduino-Pico `build_flags` options

The following are compiler flags/macros, not board properties. Add them to `build_flags` when needed; multiple lines under one `build_flags =` block accumulate.

| Option | Effect |
|---|---|
| `-DDEBUG_RP2040_PORT=Serial` | Select debug output port; alternatives include `Serial1` and `Serial2`. |
| `-DDEBUG_RP2040_CORE` | Enable core debug logging. |
| `-DDEBUG_RP2040_SPI` | Enable SPI debug logging. |
| `-DDEBUG_RP2040_WIRE` | Enable I2C/Wire debug logging. |
| `-DPIO_FRAMEWORK_ARDUINO_ENABLE_EXCEPTIONS` | Enable C++ exceptions (disabled by default in the documented core setup). |
| `-fstack-protector` | Enable compiler stack protection. |
| `-DPIO_FRAMEWORK_ARDUINO_ENABLE_RTTI` | Enable C++ RTTI. |
| `-DUSE_TINYUSB` | Select the Adafruit TinyUSB stack. |
| `-DPIO_FRAMEWORK_ARDUINO_NO_USB` | Disable USB stack integration. |
| `-DPIO_FRAMEWORK_ARDUINO_ENABLE_IPV6` | Enable IPv6 support in the lwIP configuration. |
| `-DPIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH` | Enable Bluetooth Classic/BLE support on supported wireless boards. |
| `-DPIO_FRAMEWORK_ARDUINO_ENABLE_FREERTOS` | Enable the core's FreeRTOS support. |
| `-DRP2350_PSRAM_CS=47` | Define the PSRAM chip-select GPIO for boards needing an explicit setting. |

Do not enable USB/Bluetooth/FreeRTOS features blindly: they change code size, RAM use, and sometimes runtime behavior. [Arduino-Pico PlatformIO options](https://github.com/earlephilhower/arduino-pico/blob/master/docs/platformio.rst).

## 11. Study configurations

### Pico 1, Arduino-Pico core, explicitly name the RP2040 target

```ini
[platformio]
default_envs = pico1_arduino

[env:pico1_arduino]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico
framework = arduino
board_build.mcu = rp2040
board_build.filesystem_size = 0m
monitor_speed = 115200
```

### Pico 2, Arduino-Pico core, explicitly name ARM and no filesystem

```ini
[platformio]
default_envs = pico2_arm

[env:pico2_arm]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico2
framework = arduino
board_build.mcu = rp2350
board_build.filesystem_size = 0m
monitor_speed = 115200
```

### Pico 2, Pico SDK, explicitly name ARM

```ini
[env:pico2_sdk_arm]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico2
framework = picosdk
board_build.mcu = rp2350
```

### Pico 2, Pico SDK, RISC-V

```ini
[env:pico2_sdk_riscv]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico2
framework = picosdk
board_build.mcu = rp2350-riscv
```

## 12. Key takeaways

1. `board` identifies a board manifest; `board_build.mcu` selects the compiler target when needed.
2. `framework` selects Arduino, Pico SDK, or Mbed CE APIs.
3. `board_build.core = earlephilhower` chooses the Arduino core only for boards where that choice is available; it is redundant on Arduino-Pico-only IDs.
4. `board_build.filesystem_size = 0m` means no LittleFS flash reservation. It does not change physical flash capacity.
5. Do not use `board_build.flash_size` as an RP flash-size control in this platform. The builder uses the board's upload flash capacity and `board_build.filesystem_size` for its layout.
6. Keep the board's real hardware capacity and pin wiring authoritative; configuration cannot add flash, PSRAM, wireless, or a different MCU to a board.

## Sources checked

- [Arduino-Pico: PlatformIO integration](https://github.com/earlephilhower/arduino-pico/blob/master/docs/platformio.rst)
- [Arduino-Pico board definitions generator](https://github.com/earlephilhower/arduino-pico/blob/master/tools/makeboards.py)
- [Community Raspberry Pi platform builder](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/builder/main.py)
- [Community Raspberry Pi platform manifest](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/platform.json)
- [Community Pico 1 manifest](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/boards/rpipico.json)
- [Community Pico 2 manifest](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/boards/rpipico2.json)
- [Community Raspberry Pi platform Pico SDK examples](https://github.com/maxgerhardt/platform-raspberrypi/blob/develop/examples/picosdk-blink/platformio.ini)
- [PlatformIO Pico board documentation](https://docs.platformio.org/en/latest/boards/raspberrypi/pico.html)
