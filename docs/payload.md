# Payload MCU contract

Reference firmware: Arduino Nano/ATmega328P, servo D9, closed switch D2, package
present switch D3. Switches close to GND; inputs use pull-ups. Servo power must
come from an appropriately rated supply, sharing signal ground. Configure
angles and pins on the bench before connecting the mechanism.

USB serial 115200 baud, ASCII LF terminated, at most 160 bytes. Every frame is
`TEXT*HHHH\n`, where HHHH is uppercase CRC-16/CCITT-FALSE (poly 0x1021,
initial 0xffff) of TEXT. Host sends PING at 10 Hz.

Commands: `PING`, `STOP`, `RELEASE <boot_counter> <32_lowercase_hex_mission_uuid>`.
State: `STATE <boot_counter> <closed> <present> <busy> <fault> <last_uuid_or_dash>`.
Boolean fields are exactly 0 or 1. The MCU publishes at 10 Hz.

Release requires a current boot counter, recent heartbeat, closed switch,
present sensor, no active motion/fault. The token is saved to EEPROM before
motion. A repeated token reports current state without repeating movement.
After present becomes false, the servo stows and the closed switch confirms
completion. Motion times out at 8 s; host action times out at 10 s. Missing
heartbeat for 0.5 s or STOP detaches the servo and latches fault.

Boot never moves the servo or resumes an interrupted token. The boot counter
changes, and the host fails the active action if it observes that change.
Serial telemetry loss fails the action; no automatic retry/reconnect release.
Faults need inspection and manual reset/recovery. Detaching PWM does not supply
a mechanical brake: design the latch so loss of power cannot release a load.

The ROS node independently requires fresh landed/disarmed telemetry for two
seconds before accepting a release action and throughout actuation. Success
requires absence of package plus closed/stowed feedback. This is a reference
sensor arrangement; no inference of package absence from servo angle alone.
