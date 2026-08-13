# EGFC Lander Firmware

Firmware for the eelgrass flux chamber lander controller. The firmware controls an SRS RGA, a turbopump, a SCALUP sonde, a PWM/RPM pump, and two H-bridge-driven valves. It logs RGA, SCALUP, valve, and pump records to the built-in SD card and can communicate with the surface over USB serial or Ethernet/UDP.

## Project Layout

- `src/main.cpp`: main setup, loop, surface command handling, run/stop sequencing, SD logging, status messages, and experiment coordination.
- `src/Config.h`: user-tunable hardware pins, serial settings, timing values, thresholds, and network settings.
- `src/RuntimeConfig.cpp` and `src/RuntimeConfig.h`: runtime config storage, command allowlist, parsing, and response formatting.
- `src/ConfigStore.cpp` and `src/ConfigStore.h`: EEPROM persistence for runtime config.
- `src/RGA.cpp` and `src/RGA.h`: RGA serial module with status, noise-floor, and mass-scan helpers.
- `src/SCALUP.cpp` and `src/SCALUP.h`: SCALUP sonde serial parser with the most recent parsed reading.
- `src/PwmRpm.cpp` and `src/PwmRpm.h`: PWM output and RPM pulse-count readback helper.
- `src/Turbo.cpp` and `src/Turbo.h`: turbopump USB host module with start/stop/speed/status helpers.
- `src/Valve.cpp` and `src/Valve.h`: timed dual-valve H-bridge module with chamber/flush methods, commanded-position state, and shared `SLP` control.
- `platformio.ini`: Teensy 4.1 PlatformIO build configuration.

## Hardware and Defaults

- Board: Teensy 4.1
- Main firmware configuration is in `src/Config.h`
- Loop-rate logging is disabled by default with `ENABLE_LOOP_RATE_LOG = 0`; set it to `1` to print loop frequency once per second.
- Full-system autostart is disabled by default with `AUTOSTART_ON_BOOT = false`; it starts the normal `RUN` sequence immediately after boot setup completes.
- RGA serial: `Serial4` at `28800`, `SERIAL_8N1`
- SCALUP serial: `Serial3` at `28800`, `SERIAL_8N1`
- Turbopump serial: USB host serial
- SD card: `BUILTIN_SDCARD`
- Default turbopump speed: `1200 Hz`
- Pump PWM output: pin `7`, default duty `100%`, startup disabled with `PUMP_ON_AT_STARTUP = false`, default PWM frequency `20000 Hz`, 8-bit resolution
- Pump RPM readback: pin `8`, `INPUT_PULLUP`, rising-edge interrupt, 1 pulse/rev, 1 second RPM calculation interval
- Pump RPM is included in the detailed `!:` status row.
- RGA noise floor: `2`
- RGA masses: `2, 15, 16, 18, 28, 30, 32, 33, 34, 40, 44`
- RGA electron multiplier command bias: `1400 V` (`HV1400`); off command uses `HV0`
- RGA electron multiplier at startup: disabled by default with `RGA_ELECTRON_MULTIPLIER_ON_AT_STARTUP = false`
- RGA electron multiplier total pressure limit: disabled by default with `RGA_ELECTRON_MULTIPLIER_MAX_TP_A = 0.0`; set a positive ion-current threshold in amps to require `TP?` below that value before enabling the multiplier
- RGA filament-off dwell before turbopump shutdown defaults to `60000 ms` with `RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS`.
- RGA-ready dwell before acquisition is controlled by `RGA_READY_BEFORE_ACQUISITION_MS`.
- Ethernet is enabled in the default PlatformIO build. The `teensy41_ethernet` environment uses UDP while keeping USB serial commands enabled.
- Valve pins are chamber A `2`, chamber B `3`, shared `SLP` `4`, flush A `5`, and flush B `6`.
- Valve timing: move time `10000 ms`, preflush interval `20000 ms`, chamber toggle interval `20000 ms`, minimum experiment interval before oxygen checks `30000 ms`, maximum experiment interval `60000 ms`, flush interval `30000 ms` per chamber.
- Oxygen flush limits use the latest SCALUP dissolved oxygen reading: minimum `2.0 mg/L`, maximum `12.0 mg/L`.
- SCALUP raw serial echo may be enabled for debugging.

## Build and Upload

Install PlatformIO, then run from the repository root:

```sh
pio run
pio run -t upload
pio device monitor
```

or use the PlatformIO VSCode extension

The default environment is `teensy41_ethernet`, which enables communication with the surface via Ethernet. To build without Ethernet, use `pio run -e teensy41`.

## Commands

Commands are short ASCII strings with no spaces and are terminated with carriage return (`\r`) or newline (`\n`). In Ethernet builds, commands are accepted from either USB serial or one UDP datagram per command.

| Command | Action |
| --- | --- |
| `?` | Query current readable status. |
| `TSTAT` | Query detailed turbopump status. |
| `CFG?` | Print all runtime settings. |
| `CFG,<KEY>?` | Print one runtime setting. |
| `CFG,<KEY>=<VALUE>` | Override an allowed runtime setting for the current power cycle. Rejected while acquiring, acquisition-starting, or busy. |
| `CFGS` | Save current runtime settings to Teensy EEPROM. Rejected while acquiring, acquisition-starting, or busy. |
| `CFGL` | Load runtime settings from Teensy EEPROM. Rejected while acquiring, acquisition-starting, or busy. |
| `CFGD` | Clear saved EEPROM settings and restore `Config.h` defaults. Rejected while acquiring, acquisition-starting, or busy. |
| `TP` | Query raw RGA total pressure integer from `TP?`. Rejected while RGA mass acquisition is active. |
| `ST` | Query RGA stored total-pressure sensitivity factor in `mA/Torr`. Rejected while RGA mass acquisition is active. |
| `RERR` | Query the RGA STATUS error byte with `ER?`. Rejected while RGA mass acquisition is active. |
| `RCLR` | Clear/update RGA error bytes by querying `EC?`, `ED?`, `EF?`, `EM?`, `EP?`, and `EQ?`, then report `ER?`. Rejected while acquiring. |
| `EMON` | Turn on the RGA electron multiplier using the configured bias voltage. Requires filament on and CDEM option present. Rejected while acquiring. |
| `EMOFF` | Turn off the RGA electron multiplier. Rejected while acquiring. |
| `PSTAT` | Query pump PWM/RPM status. |
| `VSTAT` | Query current valve positions, valve motion state, and pump PWM/RPM status. |
| `PON` | Turn pump PWM output on at the configured/current duty setting. |
| `POFF` | Turn pump PWM output off. |
| `FON` | Start manual chamber flushing: set flush valve to `Fl`, start on `C1`, then alternate `C1`/`C2` every `FLUSH_INTERVAL_MS`. Rejected while acquiring or busy. |
| `FOFF` | Stop manual chamber flushing or startup valve exercise and set flush valve to `Re`. |
| `OFF` | Safe stop all: stop acquisition, verify RGA filament is off, then stop turbo. |
| `TON` | Start turbopump only. |
| `TOFF` | Stop acquisition, then stop turbo only if RGA is off. |
| `RON` | Start RGA only if the turbopump is ready. |
| `ROFF` | Stop acquisition and turn off the RGA filament, leaving turbo running if ready. |
| `AON` | Start acquisition immediately if RGA is ready. |
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
CFG,<KEY>=<VALUE>
TS,ERR=<error>,SPD=<actual>,PWR=<watts>,V=<volts>,ETEMP=<degC>,BTEMP=<degC>,MTEMP=<degC>,RGA=<filament>,TP=<raw_total_pressure_current|NA>
TP,<raw_total_pressure_current>
ST,<total_pressure_sensitivity_mA_per_Torr>
RE,STATUS=<status_byte>
PS,STATE=<on|off>,PWM=<duty_percent>,RPM=<rpm>
VS,CHAMBER=<C1|C2|Unknown>,FLUSH=<Re|Fl|Unknown>,VALVES=<idle|moving>,PUMP=<on|off>,PWM=<duty_percent>,RPM=<rpm>
```

Immediate commands return `OK,<command>` when complete. Transition commands return `ACK,<command>` when accepted and `DONE,<command>` when the target state is reached. Errors use `ERR,<command>,<message>`.

Transition commands are `TON`, `RUN`, `RDY`, and `OFF`. `OFF` can interrupt another active transition. Other transition commands return `ERR,<command>,Busy` while a transition is active.

Readable states are `Off`, `Turbo starting`, `Turbo ready`, `RGA starting`, `RGA ready`, `Acquisition starting`, `Acquiring`, `Stopping`, and `Error`.

On boot, valid saved EEPROM settings override the compiled `src/Config.h` defaults. Serial `CFG` writes change the current RAM settings only until `CFGS` is sent. Command-settable keys are:

```text
AUTOSTART_ON_BOOT
RGA_MASSES
RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS
RGA_READY_BEFORE_ACQUISITION_MS
TURBO_READY_BEFORE_RGA_MS
CHAMBER_VALVE_TOGGLE_INTERVAL_MS
MIN_EXPERIMENT_INTERVAL_MS
MAX_EXPERIMENT_INTERVAL_MS
OXYGEN_MIN_MG_L
OXYGEN_MAX_MG_L
```

`PUMP_ON_AT_STARTUP` can be queried with `CFG,PUMP_ON_AT_STARTUP?`, but is read-only over serial because pump startup still uses the compiled boot setting.

Data rows use:

```text
R:<timestamp>,<mass>,<current>
```

## Serial Data Output

The USB serial port carries human-readable boot/debug messages plus these machine-readable records:

| Prefix | Format | Meaning |
| --- | --- | --- |
| `S,` | `S,<state>,SPD=<target>,TURBO=<ready/not ready>,RGA=<on/off>` | Current readable status response. |
| `CFG,` | `CFG,<KEY>=<VALUE>` | Runtime config response. |
| `TS,` | `TS,ERR=<error>,SPD=<actual>,PWR=<watts>,V=<volts>,ETEMP=<degC>,BTEMP=<degC>,MTEMP=<degC>,RGA=<filament>,TP=<raw_total_pressure_current/NA>` | Detailed turbopump/RGA status response. |
| `TP,` | `TP,<raw_total_pressure_current>` | Raw 4-byte signed integer from the RGA `TP?` command. Multiply by `1e-16` for amps. |
| `ST,` | `ST,<total_pressure_sensitivity_mA_per_Torr>` | RGA stored total-pressure sensitivity factor response from the RGA `ST?` command. |
| `RE,` | `RE,STATUS=<status_byte>` | RGA error status response. |
| `PS,` | `PS,STATE=<on/off>,PWM=<duty_percent>,RPM=<rpm>` | Pump status response. |
| `VS,` | `VS,CHAMBER=<C1/C2/Unknown>,FLUSH=<Re/Fl/Unknown>,VALVES=<idle/moving>,PUMP=<on/off>,PWM=<duty_percent>,RPM=<rpm>` | Valve and pump status response. |
| `V:` | `V:<timestamp>,<C1/C2>,<Re/Fl>` | Valve position after a change. Also written to the SD data file. |
| `P:` | `P:<rtc_timestamp>,<scalup_timestamp>,<temp_degC>,<sal_PSU>,<pressure_mbar>,<oxygen_mg_L>,<pH>` | SCALUP sonde reading. Also written to the SD data file. |
| `OK,` | `OK,<command>` | Immediate command completed. |
| `ACK,` | `ACK,<command>` | Transition command accepted. |
| `DONE,` | `DONE,<command>` | Transition command reached its target state. |
| `ERR,` | `ERR,<command>,<message>` | Command rejected. |
| `!:` | `!:<timestamp>,<payload>` | Status event or detailed status report. For payload `3`, the row is `!:<timestamp>,<turbo_error>,<turbo_speed_Hz>,<turbo_power_W>,<turbo_voltage>,<turbo_electronics_temp_C>,<turbo_bottom_temp_C>,<turbo_motor_temp_C>,<rga_filament>,<raw_total_pressure_current/NA>,<pump_rpm>`. |
| `R:` | `R:<timestamp>,<mass>,<current>` | One RGA mass reading. Also written to the SD data file. |

Timestamps are ISO-8601-style UTC strings from the Teensy RTC, for example:

```text
R:2026-06-02T14:30:00Z,28,12345
```

Simple status events use a numeric payload, for example:

```text
!:2026-06-02T14:30:00Z,5
```

Detailed status rows are sent when `StatusMsg(3)` runs. The payload includes turbopump error code, actual speed, drive power, drive voltage, electronics temperature, pump-bottom temperature, motor temperature, RGA filament status, raw RGA total pressure current, and pump RPM. Multiply the raw total pressure value by `1e-16` for amps.

## Running the System

1. Connect cables
  * 15-pin surface power and data
  * 8-pin SCALUP sonde (connector "A")
  * 8-pin pump/valve (connector "P")
  * 6-pin USB (optional, currently some connection issues)
2. Power the system and open the serial monitor at `9600`.
3. Confirm boot output shows RTC, RGA initialization, SD initialization, and `Surface ready`.
4. If needed, set time with `TIME<unix>` (`T<unix>` is still accepted).
5. Start the full measurement sequence with `RUN` (`!Z11` is still accepted), or set `AUTOSTART_ON_BOOT = true` to start automatically after boot setup.
6. The firmware sets turbopump speed, starts the turbopump, checks for readiness, turns on the RGA filament, waits `RGA_READY_BEFORE_ACQUISITION_MS`, then begins mass scans. This dwell applies to `RUN`, including boot autostart.
7. If `PUMP_ON_AT_STARTUP` is true, preflush alternates staggered chamber and flush valve changes before acquisition starts.
8. During acquisition, the valve experiment starts with flush recirculating and chamber A selected, toggles the chamber valve on the configured interval, then flushes chamber A and chamber B before starting the next experiment.
9. RGA, SCALUP, valve, and pump rows are printed, written to SD, and sent over UDP if Ethernet is enabled.
10. Stop with `OFF` (`!Z20`, `!Z21`, and `!Z22` are still accepted). This stops acquisition, verifies the RGA filament is off, waits `RGA_FILAMENT_OFF_BEFORE_TURBO_STOP_MS`, then stops the turbopump.

## Notes

- If time permits, start the turbo manually with `TON` and allow to run as long as possible (~1h) before starting the RGA (`RON`) and aquisition (`AON`). This is better for the RGA and aquisition stability.
- RGA mass acquisition is non-blocking during `Acquiring`, but several startup, shutdown, RGA setup, and turbopump operations are still blocking.
- Before enabling the electron multiplier, the firmware checks that acquisition is not active, the filament is on, the CDEM option query `MO?` returns `1`, and, if configured, total pressure current from `TP?` is below `RGA_ELECTRON_MULTIPLIER_MAX_TP_A`. Operationally, also verify chamber pressure is safely in the multiplier range, the configured HV/gain calibration is appropriate, and the RGA has no active error status.
- If `RGA_ELECTRON_MULTIPLIER_ON_AT_STARTUP` is true, RGA startup fails unless the electron multiplier turns on successfully.
- The active SD file is named `gems_YYYY-MM-DD-HH-MM.txt`.
- Data files rotate every 4th hour when the minute equals `10`.
- Pump RPM accuracy depends on the pump tach signal, pullup/level shifting, interrupt edge, and `pulsesPerRevolution` setting in `PwmRpm::Config`.
