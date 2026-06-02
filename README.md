# EGFC GEMS Lander Firmware

PlatformIO firmware for a Teensy 4.1 GEMS lander controller. The firmware controls an SRS RGA over `Serial4`, controls a turbopump over USB host serial, logs RGA mass-scan data to the built-in SD card, and can communicate with the surface over USB serial or Ethernet/UDP.

## Project Layout

- `src/GEMS_Lander.ino`: main setup, loop, surface command handling, run/stop sequencing, SD logging, status messages.
- `src/RGA.ino`: RGA status, noise-floor, and mass-scan helpers.
- `src/Turbo.ino`: USB host setup, turbopump start/stop/speed/status helpers.
- `platformio.ini`: Teensy 4.1 PlatformIO build configuration.

## Hardware and Defaults

- Board: Teensy 4.1
- Debug/surface serial: `Serial` at `9600`
- RGA serial: `Serial4` at `28800`, `SERIAL_8N1`
- Turbopump serial: USB host serial at `9600`
- SD card: `BUILTIN_SDCARD`
- Default turbopump speed: `1200 Hz`
- RGA noise floor: `2`
- RGA masses: `2, 15, 16, 18, 28, 30, 32, 33, 34, 40, 44`
- Ethernet is disabled by default. Uncomment `#define USE_ETHERNET` in `src/GEMS_Lander.ino` to use UDP.

## Build and Upload

Install PlatformIO, then run from the repository root:

```sh
pio run -e teensy41
pio run -e teensy41 -t upload
pio device monitor
```

## Surface Commands

Commands are ASCII and terminated with carriage return (`\r`).

| Command | Action |
| --- | --- |
| `?` | Query current status code. |
| `T<unix>` | Set RTC/system time from Unix time. |
| `!Z10` | Start turbopump only. |
| `!Z11` | Start full system: turbopump, then RGA when turbo is ready. |
| `!Z12` | Start RGA only if the turbopump is already ready. |
| `!Z20` | Safely stop turbopump: stop acquisition, verify RGA filament is off, then stop turbo. |
| `!Z21` | Stop system. |
| `!Z22` | Stop system. |
| `!ZFS` | Turn off RGA filament only. |
| `!RS####` | Set turbopump target speed in Hz, for example `!RS1200`. |

Status query responses use `?N`. Data rows use:

```text
R:<timestamp>,<mass>,<current>
```

## Serial Data Output

The USB serial port runs at `9600`. It carries human-readable boot/debug messages plus these machine-readable records:

| Prefix | Format | Meaning |
| --- | --- | --- |
| `?` | `?N` | Current state/on-off response. |
| `!:` | `!:<timestamp>,<payload>` | Status event or detailed status report. |
| `R:` | `R:<timestamp>,<mass>,<current>` | One RGA mass reading. Also written to the SD data file. |

Timestamps are ISO-8601-style UTC strings from the Teensy RTC, for example:

```text
R:2026-06-02T14:30:00Z,28,12345
```

Simple status events use a numeric payload, for example:

```text
!:2026-06-02T14:30:00Z,5
```

Detailed status rows are sent when `StatusMsg(3)` runs. In serial builds, the payload includes turbopump error code, actual speed, drive power, drive voltage, electronics temperature, pump-bottom temperature, motor temperature, and RGA filament status.

## Running the System

1. Connect the RGA to `Serial4`, the turbopump controller to Teensy USB host serial, and insert the SD card.
2. Power the system and open the serial monitor at `9600`.
3. Confirm boot output shows RTC, RGA initialization, SD initialization, and `Surface ready`.
4. If needed, set time with `T<unix>`.
5. Start the full measurement sequence with `!Z11`.
6. The firmware sets turbopump speed, starts the pump, checks for readiness, turns on the RGA filament, then begins mass scans.
7. During measurement, RGA rows are printed, written to SD, and sent over UDP if Ethernet is enabled.
8. Stop with `!Z20`, `!Z21`, or `!Z22`. `!Z20` explicitly verifies the RGA filament is off before stopping the turbopump.

## Notes

- Several startup, shutdown, RGA, and turbopump operations are blocking in the current firmware.
- The active SD file is named `gems_YYYY-MM-DD-HH-MM.txt`.
- Data files rotate every 4th hour when the minute equals `10`.
