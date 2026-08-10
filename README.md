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
- `src/Config.h`: user-tunable hardware pins, serial settings, timing values, thresholds, and network settings.
- `src/RGA.cpp` and `src/RGA.h`: RGA serial module with status, noise-floor, and mass-scan helpers.
- `src/SCALUP.cpp` and `src/SCALUP.h`: SCALUP sonde serial parser with the most recent parsed reading.
- `src/PwmRpm.cpp` and `src/PwmRpm.h`: PWM output and RPM pulse-count readback helper.
- `src/Turbo.cpp` and `src/Turbo.h`: turbopump USB host module with start/stop/speed/status helpers.
- `src/Valve.cpp` and `src/Valve.h`: timed dual-valve H-bridge module with chamber/flush methods, commanded-position state, and shared `SLP` control.
- `platformio.ini`: Teensy 4.1 PlatformIO build configuration.

## Hardware and Defaults

- Board: Teensy 4.1
- Debug/surface serial: `Serial` at `9600`
- Main firmware configuration is in `src/Config.h`
- Loop-rate logging is disabled by default with `ENABLE_LOOP_RATE_LOG = 0`; set it to `1` to print loop frequency once per second.
- RGA serial: `Serial4` at `28800`, `SERIAL_8N1`
- SCALUP serial: `Serial3` at `28800`, `SERIAL_8N1`
- Turbopump serial: USB host serial at `9600`
- SD card: `BUILTIN_SDCARD`
- Default turbopump speed: `1200 Hz`
- Pump PWM output: pin `7`, default duty `100%`, startup enabled with `PUMP_ON_AT_STARTUP = true`, default PWM frequency `20000 Hz`, 8-bit resolution
- Pump RPM readback: pin `8`, `INPUT_PULLUP`, rising-edge interrupt, 1 pulse/rev, 1 second RPM calculation interval
- Pump status log interval: `10000 ms`
- RGA noise floor: `2`
- RGA masses: `2, 15, 16, 18, 28, 30, 32, 33, 34, 40, 44`
- RGA electron multiplier command bias: `1400 V` (`HV1400`); off command uses `HV0`
- RGA electron multiplier at startup: disabled by default with `RGA_ELECTRON_MULTIPLIER_ON_AT_STARTUP = false`
- RGA electron multiplier total pressure limit: disabled by default with `RGA_ELECTRON_MULTIPLIER_MAX_TP_A = 0.0`; set a positive ion-current threshold in amps to require `TP?` below that value before enabling the multiplier
- Ethernet is disabled by default. Build the `teensy41_ethernet` PlatformIO environment to use UDP.
- Valve pins are chamber A `2`, chamber B `3`, shared `SLP` `4`, flush A `5`, and flush B `6`.
- Valve timing: move time `10000 ms`, chamber toggle interval `20000 ms`, minimum experiment interval before oxygen checks `30000 ms`, maximum experiment interval `60000 ms`, flush interval `30000 ms` per chamber.
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
| `TP` | Query raw RGA total pressure integer from `TP?`. Rejected while RGA mass acquisition is active. |
| `ST` | Query RGA stored total-pressure sensitivity factor in `mA/Torr`. Rejected while RGA mass acquisition is active. |
| `RERR` | Query the RGA STATUS error byte with `ER?`. Rejected while RGA mass acquisition is active. |
| `RCLR` | Clear/update RGA error bytes by querying `EC?`, `ED?`, `EF?`, `EM?`, `EP?`, and `EQ?`, then report `ER?`. Rejected while acquiring. |
| `EMON` | Turn on the RGA electron multiplier using the configured bias voltage. Requires filament on and CDEM option present. Rejected while acquiring. |
| `EMOFF` | Turn off the RGA electron multiplier. Rejected while acquiring. |
| `PSTAT` | Query pump PWM/RPM status. |
| `PON` | Turn pump PWM output on at the configured/current duty setting. |
| `POFF` | Turn pump PWM output off. |
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
TS,ERR=<error>,SPD=<actual>,PWR=<watts>,V=<volts>,ETEMP=<degC>,BTEMP=<degC>,MTEMP=<degC>,RGA=<filament>,TP=<raw_total_pressure_current|NA>
TP,<raw_total_pressure_current>
ST,<total_pressure_sensitivity_mA_per_Torr>
RE,STATUS=<status_byte>
PS,STATE=<on|off>,PWM=<duty_percent>,RPM=<rpm>
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
| `TS,` | `TS,ERR=<error>,SPD=<actual>,PWR=<watts>,V=<volts>,ETEMP=<degC>,BTEMP=<degC>,MTEMP=<degC>,RGA=<filament>,TP=<raw_total_pressure_current|NA>` | Detailed turbopump/RGA status response. |
| `TP,` | `TP,<raw_total_pressure_current>` | Raw 4-byte signed integer from the RGA `TP?` command. Multiply by `1e-16` for amps. |
| `ST,` | `ST,<total_pressure_sensitivity_mA_per_Torr>` | RGA stored total-pressure sensitivity factor response from the RGA `ST?` command. |
| `RE,` | `RE,STATUS=<status_byte>` | RGA error status response. |
| `PS,` | `PS,STATE=<on|off>,PWM=<duty_percent>,RPM=<rpm>` | Pump status response. |
| `V:` | `V:<timestamp>,<C1|C2>,<Re|Fl>` | Valve position after a change. Also written to the SD data file. |
| `P:` | `P:<rtc_timestamp>,<scalup_timestamp>,<temp_degC>,<sal_PSU>,<oxygen_mg_L>,<pH>` | SCALUP sonde reading. Also written to the SD data file. |
| `PM:` | `PM:<timestamp>,<duty_percent>,<rpm>` | Pump status row every 10 seconds. Also written to the SD data file. |
| `OK,` | `OK,<command>` | Immediate command completed. |
| `ACK,` | `ACK,<command>` | Transition command accepted. |
| `DONE,` | `DONE,<command>` | Transition command reached its target state. |
| `ERR,` | `ERR,<command>,<message>` | Command rejected. |
| `!:` | `!:<timestamp>,<payload>` | Status event or detailed status report. For payload `3`, the row is `!:<timestamp>,<turbo_error>,<turbo_speed_Hz>,<turbo_power_W>,<turbo_voltage>,<turbo_electronics_temp_C>,<turbo_bottom_temp_C>,<turbo_motor_temp_C>,<rga_filament>,<raw_total_pressure_current|NA>`. |
| `R:` | `R:<timestamp>,<mass>,<current>` | One RGA mass reading. Also written to the SD data file. |

Timestamps are ISO-8601-style UTC strings from the Teensy RTC, for example:

```text
R:2026-06-02T14:30:00Z,28,12345
```

Simple status events use a numeric payload, for example:

```text
!:2026-06-02T14:30:00Z,5
```

Detailed status rows are sent when `StatusMsg(3)` runs. The payload includes turbopump error code, actual speed, drive power, drive voltage, electronics temperature, pump-bottom temperature, motor temperature, RGA filament status, and raw RGA total pressure current. Multiply the raw total pressure value by `1e-16` for amps.

## Running the System

1. Connect cables
  * 15-pin surface power and data
  * 8-pin SCALUP sonde (connector "A")
  * 8-pin pump/valve (connector "P")
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
- RGA mass acquisition is non-blocking during `Acquiring`, but several startup, shutdown, RGA setup, and turbopump operations are still blocking.
- Before enabling the electron multiplier, the firmware checks that acquisition is not active, the filament is on, the CDEM option query `MO?` returns `1`, and, if configured, total pressure current from `TP?` is below `RGA_ELECTRON_MULTIPLIER_MAX_TP_A`. Operationally, also verify chamber pressure is safely in the multiplier range, the configured HV/gain calibration is appropriate, and the RGA has no active error status.
- If `RGA_ELECTRON_MULTIPLIER_ON_AT_STARTUP` is true, RGA startup fails unless the electron multiplier turns on successfully.
- The active SD file is named `gems_YYYY-MM-DD-HH-MM.txt`.
- Data files rotate every 4th hour when the minute equals `10`.
- Pump RPM accuracy depends on the pump tach signal, pullup/level shifting, interrupt edge, and `pulsesPerRevolution` setting in `PwmRpm::Config`.
