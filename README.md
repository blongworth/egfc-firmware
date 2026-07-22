# EGFC GEMS Lander Firmware

Firmware for the eelgrass flux chamber GEMS lander controller. The firmware controls an SRS RGA over `Serial4`, a turbopump over USB host serial, a SCALUP sonde over `Serial3`, a PWM/RPM pump, and two H-bridge-driven valves. It logs RGA, SCALUP, valve, and pump records to the built-in SD card and can communicate with the surface over USB serial or Ethernet/UDP.

## TODO

* remove blocking code (RGA DAQ, turbo shutdown)
* move user-tunable constants into a configuration file or persistent settings store
* Field config
* Electron Multiplier mode
* Total Pressure
* Autostart
* Minimum chamber time
* Integration with surface teensy via enet

## Project Layout

- `src/main.cpp`: main setup, loop, surface command handling, run/stop sequencing, SD logging, status messages, and experiment coordination.
- `src/RGA.cpp` and `src/RGA.h`: RGA serial module with status, noise-floor, and mass-scan helpers.
- `src/SCALUP.cpp` and `src/SCALUP.h`: SCALUP sonde serial parser with the most recent parsed reading.
- `src/PwmRpm.cpp` and `src/PwmRpm.h`: PWM output and RPM pulse-count readback helper.
- `src/Turbo.cpp` and `src/Turbo.h`: turbopump USB host module with start/stop/speed/status helpers.
- `src/Valve.cpp` and `src/Valve.h`: timed dual-valve H-bridge module with chamber/flush methods, commanded-position state, and shared `SLP` control.
- `platformio.ini`: Teensy 4.1 PlatformIO build configuration.

## Hardware and Defaults

- Board: Teensy 4.1
- Debug/surface serial: `Serial` at `9600`
- RGA serial: `Serial4` at `28800`, `SERIAL_8N1`
- SCALUP serial: `Serial3` at `28800`, `SERIAL_8N1`
- Turbopump serial: USB host serial at `9600`
- SD card: `BUILTIN_SDCARD`
- Default turbopump speed: `1200 Hz`
- Pump PWM output: pin `7`, default PWM frequency `20000 Hz`, 8-bit resolution
- Pump RPM readback: pin `8`, `INPUT_PULLUP`, rising-edge interrupt, 1 pulse/rev, 1 second RPM calculation interval
- Pump status log interval: `10000 ms`
- RGA noise floor: `2`
- RGA masses: `2, 15, 16, 18, 28, 30, 32, 33, 34, 40, 44`
- Ethernet is disabled by default. Build the `teensy41_ethernet` PlatformIO environment to use UDP.
- Valve pins are chamber A `2`, chamber B `3`, shared `SLP` `4`, flush A `5`, and flush B `6`.
- Valve timing: move time `10000 ms`, chamber toggle interval `20000 ms`, experiment interval `60000 ms`, flush interval `30000 ms` per chamber.
- Oxygen flush limits use the latest SCALUP dissolved oxygen reading: minimum `2.0 mg/L`, maximum `12.0 mg/L`.
- SCALUP raw serial echo is currently enabled for debugging.

## Build and Upload

Install PlatformIO, then run from the repository root:

```sh
pio run -e teensy41
pio run -e teensy41 -t upload
pio device monitor
```

## Commands

Commands are short ASCII strings with no spaces and are terminated with carriage return (`\r`).

| Command | Action |
| --- | --- |
| `?` | Query current readable status. |
| `TSTAT` | Query detailed turbopump status. |
| `PSTAT` | Query pump PWM/RPM status. |
| `OFF` | Safe stop all: stop acquisition, verify RGA filament is off, then stop turbo. |
| `TON` | Start turbopump only. |
| `TOFF` | Stop acquisition, then stop turbo only if RGA is off. |
| `RON` | Start RGA only if the turbopump is ready. |
| `ROFF` | Stop acquisition and turn off the RGA filament, leaving turbo running if ready. |
| `AON` | Start acquisition if RGA is ready. |
| `AOFF` | Stop acquisition and leave RGA ready. |
| `RUN` | Full start: turbopump, ready dwell, RGA, then acquisition. |
| `RDY` | Full start to RGA ready, without acquisition. |
| `SPD####` | Set turbopump target speed in Hz, for example `SPD1200`. |
| `PMP#` | Set pump PWM duty cycle percent from `0` to `100`, for example `PMP50` or `PMP12.5`. |
| `TIME<unix>` | Set RTC/system time from Unix time. |
| `CLR` | Clear error state. |

Legacy aliases are still accepted:

| Alias | Command |
| --- | --- |
| `!Z10` | `TON` |
| `!Z11` | `RUN` |
| `!Z12` | `RON` |
| `!Z20`, `!Z21`, `!Z22` | `OFF` |
| `!ZFS` | `ROFF` |
| `!RS####` | `SPD####` |
| `T<unix>` | `TIME<unix>` |

Status responses use:

```text
S,<state>,SPD=<target>,TURBO=<ready|not ready>,RGA=<on|off>
TS,ERR=<error>,SPD=<actual>,PWR=<watts>,V=<volts>,ETEMP=<degC>,BTEMP=<degC>,MTEMP=<degC>,RGA=<filament>
PS,PWM=<duty_percent>,RPM=<rpm>
```

Immediate commands return `OK,<command>` when complete. Transition commands return `ACK,<command>` when accepted and `DONE,<command>` when the target state is reached. Errors use `ERR,<command>,<message>`.

Transition commands are `TON`, `RUN`, `RDY`, and `OFF`. `OFF` can interrupt another active transition. Other transition commands return `ERR,<command>,Busy` while a transition is active.

Readable states are `Off`, `Turbo starting`, `Turbo ready`, `RGA starting`, `RGA ready`, `Acquiring`, `Stopping`, and `Error`.

Data rows use:

```text
R:<timestamp>,<mass>,<current>
```

## Serial Data Output

The USB serial port runs at `9600`. It carries human-readable boot/debug messages plus these machine-readable records:

| Prefix | Format | Meaning |
| --- | --- | --- |
| `S,` | `S,<state>,SPD=<target>,TURBO=<ready|not ready>,RGA=<on|off>` | Current readable status response. |
| `TS,` | `TS,ERR=<error>,SPD=<actual>,PWR=<watts>,V=<volts>,ETEMP=<degC>,BTEMP=<degC>,MTEMP=<degC>,RGA=<filament>` | Detailed turbopump status response. |
| `PS,` | `PS,PWM=<duty_percent>,RPM=<rpm>` | Pump status response. |
| `V:` | `V:<timestamp>,<event>,CHAMBER=<state>,FLUSH=<state>` | Valve change event. Also written to the SD data file. |
| `P:` | `P:<rtc_timestamp>,<scalup_timestamp>,<temp_degC>,<sal_PSU>,<oxygen_mg_L>,<pH>` | SCALUP sonde reading. Also written to the SD data file. |
| `PM:` | `PM:<timestamp>,<duty_percent>,<rpm>` | Pump status row every 10 seconds. Also written to the SD data file. |
| `OK,` | `OK,<command>` | Immediate command completed. |
| `ACK,` | `ACK,<command>` | Transition command accepted. |
| `DONE,` | `DONE,<command>` | Transition command reached its target state. |
| `ERR,` | `ERR,<command>,<message>` | Command rejected. |
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

1. Connect cables
  * 15-pin surface power and data
  * 8-pin SCALUP sonde (connector "A")
  * 8-pin pump/valve (connector "B")
  * 6-pin USB (optional, currently some connection issues)
2. Power the system and open the serial monitor at `9600`.
3. Confirm boot output shows RTC, RGA initialization, SD initialization, and `Surface ready`.
4. If needed, set time with `TIME<unix>` (`T<unix>` is still accepted).
5. Start the full measurement sequence with `RUN` (`!Z11` is still accepted).
6. The firmware sets turbopump speed, starts the turbopump, checks for readiness, turns on the RGA filament, then begins mass scans.
7. During acquisition, the valve experiment starts with flush recirculating and chamber A selected, toggles the chamber valve on the configured interval, then flushes chamber A and chamber B before starting the next experiment.
8. RGA, SCALUP, valve, and pump rows are printed, written to SD, and sent over UDP if Ethernet is enabled.
9. Stop with `OFF` (`!Z20`, `!Z21`, and `!Z22` are still accepted). This stops acquisition, verifies the RGA filament is off, then stops the turbopump.

## Notes

- If time permits, start the turbo manually with `TON` and allow to run as long as possible (~1h) before starting the RGA (`RON`) and aquisition (`AON`). This is better for the RGA and aquisition stability.
- Several startup, shutdown, RGA, and turbopump operations are blocking in the current firmware.
- The active SD file is named `gems_YYYY-MM-DD-HH-MM.txt`.
- Data files rotate every 4th hour when the minute equals `10`.
- Pump RPM accuracy depends on the pump tach signal, pullup/level shifting, interrupt edge, and `pulsesPerRevolution` setting in `PwmRpm::Config`.
