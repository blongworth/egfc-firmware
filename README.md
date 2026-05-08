# EGFC GEMS Lander Firmware

Firmware for the eelgrass flux chamber GEMS lander controller. The project is a PlatformIO Arduino build for a Teensy 4.1 that controls the RGA, controls and monitors the turbopump, logs RGA measurements to SD, and exchanges commands/status/data with the surface system.

## TODO

* valve control
* seaphox/scallop interface
* chamber switching, flushing logic and status logging

## Summary

The firmware runs a cooperative main loop. Long-running activities are split into service functions so RGA acquisition, turbopump supervision, surface communication, USB host servicing, SD file rotation, and status reporting can continue without blocking each other during normal measurement cycles.

Current hardware-facing features:

- RGA control and nonblocking scan-cycle acquisition.
- Turbopump target-speed control, startup, shutdown, and health checks.
- SD logging with timed file rotation.
- Ethernet/UDP surface communication by default.
- Native PlatformIO tests for command parsing and state/status mapping.

ADV and valve-control code have been removed.

## Project Layout

- `src/lander.cpp`: top-level firmware orchestration, Arduino `setup()`/`loop()`, state transitions, startup/shutdown sequencing, surface command handling, logging calls, and status reporting.
- `include/LanderConfig.h`: firmware configuration constants, including RGA mass list, timeouts, turbopump thresholds, SD settings, and Ethernet addresses/ports.
- `include/lander.h`: small shared application header for the main firmware file.
- `lib/RGA`: `RGAController`, RGA configuration, and `RGACycleData`/`RGAMassReading` structs.
- `lib/TurboPump`: `TurboPumpController`, turbopump configuration, command response, and status structs.
- `lib/LanderCore`: Arduino-independent state and surface-command parsing code.
- `lib/SurfaceLink`: surface transport wrapper. Builds for UDP when `USE_ETHERNET` is defined, otherwise uses a serial stream.
- `lib/DataLogger`: SD initialization, file creation/rotation, and line-oriented writes.
- `test/test_lander_core`: native Unity tests for protocol parsing and state/status behavior.

## Configuration

Primary runtime settings are in `include/LanderConfig.h`.

Important values:

- RGA masses: `2, 15, 16, 18, 28, 30, 32, 33, 34, 40, 44`
- RGA serial port: `Serial4` at `28800`, configured in `src/lander.cpp` through `RGA_SERIAL`
- Turbopump USB serial baud: `9600`
- Default turbopump target speed: `1200 Hz`
- Turbopump max speed used for command scaling: `1500 Hz`
- Turbopump ready condition: target minus `50 Hz`, error code `0`, drive power below `15 W`
- SD chip select: `BUILTIN_SDCARD`
- Data file rotation: every 4th hour at minute 10
- Ethernet local IP/port: `111.111.111.111:8000`
- Ethernet destination IP/port: `111.111.111.222:8002`

Ethernet is enabled by the `-DUSE_ETHERNET` build flag in `platformio.ini`. Remove that build flag to compile `SurfaceLink` for serial surface communication instead.

## Build, Test, and Upload

Install PlatformIO, then run commands from the repository root.

Build the Teensy firmware:

```sh
pio run -e teensy41
```

Run native tests:

```sh
pio test -e native
```

Upload to a connected Teensy 4.1:

```sh
pio run -e teensy41 -t upload
```

Open a serial monitor for debug output:

```sh
pio device monitor
```

## Operating Model

On boot, the firmware:

1. Starts USB serial debug output at `9600`.
2. Initializes `Serial4` for the RGA.
3. Synchronizes from the Teensy RTC if available.
4. Starts USB host support for the turbopump serial interface.
5. Configures the RGA and ensures the RGA filament is off.
6. Initializes the SD card and opens a data file.
7. Starts Ethernet and requests time from the surface system when Ethernet is enabled.
8. Enters the `Idle` state and waits for surface commands.

The loop continuously services USB, surface messages, turbopump startup/shutdown state machines, SD file rotation, RGA acquisition when measuring, and optional loop-rate debug output.

## Surface Command Protocol

Surface commands are ASCII messages terminated with carriage return (`\r`). UDP builds read commands from the configured local UDP port. Serial builds read commands from the configured surface serial stream.

Supported commands:

| Command | Action |
| --- | --- |
| `?` | Query current state code. |
| `T<unix>` | Set Teensy RTC/system time to a Unix timestamp. Timestamp must be greater than `1735689600`. |
| `!Z10` | Start turbopump only. |
| `!Z11` | Start turbopump, then start RGA when the turbopump is ready. |
| `!Z12` | Start RGA only if the turbopump is already ready. |
| `!Z21` | Stop system. |
| `!Z22` | Stop system. |
| `!ZFS` | Stop RGA filament only. |
| `!RS####` | Set turbopump target speed in Hz. The speed field must be exactly four digits, for example `!RS1200`. |

Malformed commands are rejected and reported on the debug serial port.

## State and Status Messages

State query responses use `?N`, where `N` is:

| Code | State |
| --- | --- |
| `0` | Idle |
| `1` | Starting or turbo running |
| `2` | Measuring |
| `3` | Stopping |
| `4` | Checking whether RGA can start |
| `5` | Error |

The firmware also sends on/off notifications as `?0` or `?1` when state changes or time is synchronized. `?1` means the lander is starting, turbo running, checking RGA readiness, or measuring.

Status payloads use:

```text
!:<timestamp>,<payload>\r
```

For detailed status payloads, fields are:

```text
errorCode,actualSpeedHz,drivePowerW,driveVoltageV,electronicsTempC,pumpBottomTempC,motorTempC,filamentStatus
```

If turbopump or RGA status cannot be read, unavailable fields are reported as `-1`.

Some startup/shutdown status events are retained as numeric payloads for surface compatibility, for example:

```text
!:2026-05-07T12:00:00Z,5\r
```

## RGA Data Format

Each completed RGA scan cycle is stored in an `RGACycleData` struct. Each mass reading is stored as an `RGAMassReading` with the requested mass, current value, validity flag, timeout flag, and timing metadata.

Logged and transmitted RGA rows use:

```text
R:<timestamp>,<mass>,<current>
```

If a mass read times out:

```text
R:<timestamp>,<mass>,timeout
```

The active SD file is named:

```text
gems_YYYY-MM-DD-HH-MM.txt
```

The logger creates a new file at boot and rotates files at the configured rotation time.

## Normal Operation

1. Power the system with the RGA, turbopump USB serial interface, Ethernet, and SD card connected.
2. Open the debug serial monitor and confirm boot messages show RTC/RGA/SD/Ethernet initialization.
3. When Ethernet is enabled, the firmware sends `$` to request time. Reply with a valid `T<unix>` command if the RTC is not already correct.
4. Send `!Z11` to start a full measurement sequence.
5. The firmware sets the turbopump target speed, starts the pump, and polls status at the configured interval until the pump is ready.
6. Once the turbopump is ready, the firmware starts the RGA filament and enters `Measuring`.
7. During measurement, the RGA scan cycle runs nonblocking. Completed readings are written to SD and sent to the surface.
8. Send `!Z21` or `!Z22` to stop. The firmware turns off the RGA filament, stops the turbopump, polls until the pump speed reaches zero or shutdown timeout expires, then returns to `Idle`.

## Safety and Fault Handling

- RGA acquisition timeouts are recorded per mass reading instead of blocking the loop indefinitely.
- Turbopump startup has a timeout. If the pump does not become ready in time, the firmware begins shutdown.
- During measurement, repeated bad turbopump readiness checks trigger shutdown.
- Detailed status reports use `-1` for unavailable values instead of reusing stale readings.

## Notes for Development

- Keep hardware-independent behavior in `lib/LanderCore` where possible so it can be covered by native tests.
- Keep hardware modules behind config structs and data structs. This makes new tests and future hardware swaps easier.
- Prefer adding new surface commands to `SurfaceCommandType` and `parseSurfaceCommand()` with corresponding native tests.
- Run both `pio test -e native` and `pio run -e teensy41` before deploying firmware changes.
