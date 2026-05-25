# Novy Undercover Cooktop CAN Bus Protocol

## Physical Layer

- **Protocol**: CAN 2.0A (standard 11-bit identifiers)
- **Baud rate**: 125 kbps
- **Connectors**: RJ45 (8P8C) with standard T-568B wiring
- **Pinout**:
  - Pin 1: CAN_H
  - Pin 2: CAN_L
  - Pins 3, 4: GND
  - Pin 5: controller presence signal. Actively driven to
    3.3V by the controller when running. Asserted on
    power-on and held high while the system is active.
    Required for normal operation — system shows errors
    if pin 5 is low or disconnected. ~15kΩ pull-down to
    GND on the bus side.
  - Pin 6: unknown, not required for operation
  - Pins 7, 8: +5V DC power (always on)

  Note: the pinout does not follow twisted-pair assignments.
  Pins are used sequentially, ignoring cable pairing.
- **Signal**: standard CAN differential, ~2V swing, ~2.2V
  common-mode bias. Idle state is recessive.

## System Architecture

Four devices on a single CAN bus:

- **Controller** — capacitive button + RF bridge to wireless
  remote. Initiates startup, sends commands, polls status.
- **PSU** — 2-phase power supply. Provides 5V on the RJ45.
- **Coil A** — induction coil serving the **left** zone.
- **Coil B** — induction coil serving **middle** and **right**
  zones. Both coils have 2 RJ45 ports for daisy-chaining.

## CAN ID Scheme

Each coil has a set of CAN IDs. The base offsets identify the
coil:

| Offset | Device | Notes                        |
|--------|--------|------------------------------|
| `x09`  | Coil A | Single coil, left zone       |
| `x13`  | Coil B | Double-wide, middle + right  |
| `x19`  | PSU    | Power supply                 |

IDs are formed as prefix + offset. Lower IDs have higher CAN
priority.

| Prefix  | Sender     | Role              | DLC   |
|---------|------------|-------------------|-------|
| `0x480` | Coil       | Pan detection     | 3-6   |
| `0x500` | Coil       | Ready / ack       | 0     |
| `0x580` | Any        | Ready / ack       | 0     |
| `0x600` | Coil       | Command ack       | 4     |
| `0x680` | Controller | Commands          | 4-5   |
| `0x700` | Coil       | Status response   | 5-8   |
| `0x780` | Controller | Status query      | 3     |

System-wide IDs:

| ID      | Sender     | Role                |
|---------|------------|---------------------|
| `0x680` | Controller | System init         |
| `0x681` | Controller | Controller announce |

## Message Formats

### Status Query / Response

The controller periodically polls each coil. It sends a 3-byte
query on `0x78x`; the coil responds on `0x70x` with the same
first 3 bytes plus response data.

```
Controller → 0x789  XX YY 00          (query)
Coil A     → 0x709  XX YY 00 01 ...   (response)
```

Examples:

```
Controller → 0x789  02 03 00
Coil A     → 0x709  02 03 00 01 00 1E       temperature = 0x1E = 30°C

Controller → 0x789  03 04 00
Coil A     → 0x709  03 04 00 01 00 2D       surface temp = 0x2D = 45°C

Controller → 0x789  02 05 00
Coil A     → 0x709  02 05 00 01 01          power delivery active

Controller → 0x789  00 03 00
Coil A     → 0x709  00 03 00 01 A5 51 D1 AD serial number
```

The first two bytes select the register:

| Byte 0 | Byte 1 | Response len | Meaning                     |
|--------|--------|--------------|-----------------------------|
| `00`   | `03`   | 4            | Device serial number        |
| `00`   | `04`   | 4            | Device capabilities         |
| `02`   | `03`   | 1            | Temperature sensor 1 (°C)   |
| `02`   | `04`   | 4            | Unknown (always 0)          |
| `02`   | `05`   | 1            | Power delivery state        |
| `02`   | `09`   | 2            | Unknown (constant 0xAAD8)   |
| `03`   | `02`   | 1            | Ambient / PCB temp (°C)     |
| `03`   | `03`   | 4            | Unknown (always 0)          |
| `03`   | `04`   | 1            | Surface temperature (°C)    |
| `03`   | `05`   | 1            | Unknown (always 0)          |
| `04`   | `02`   | 1            | Ambient temp, coil B zone 2 |
| `04`   | `03`   | 4            | Unknown, coil B zone 2      |
| `04`   | `04`   | 1            | Surface temp, coil B zone 2 |
| `04`   | `05`   | 1            | Unknown, coil B zone 2      |

The controller cycles through these registers once per second
per coil:

```
Controller → 0x789: 02 05, 02 04, 02 03          @+0ms
Controller → 0x789: 03 02, 03 03, 03 05, 03 04   @+250ms
Controller → 0x793: 03 02, 03 03, 03 05, 03 04   @+375ms
Controller → 0x793: 04 02, 04 03, 04 05, 04 04   @+500ms
(repeat at +1000ms)
```

Each query is immediately followed by the coil's response.
Byte 0 groups related registers: coil A is queried with
byte 0 = `02` and `03`, coil B with `02`, `03`, and `04`.
The extra group (`04`) is likely coil B's second zone.

### Serial Number (`00 03`)

```
Coil A → 0x709  00 03 00 01 A5 51 D1 AD
Coil B → 0x713  00 03 00 01 FE 23 6A 62
PSU    → 0x719  00 03 00 01 E8 97 67 B3
```

Bytes 4-7 are a unique 32-bit device identifier.

### Device Config (`00 04`)

```
Coil A → 0x709  00 04 00 01 00 02 00 00
Coil B → 0x713  00 04 00 01 00 02 00 00
PSU    → 0x719  00 04 00 01 00 02 01 00
```

Byte 6 = `01` for the PSU, `00` for coils.

### Temperature

Reported as single bytes in degrees Celsius.

- Register `03 02` / `04 02`: ambient / PCB temperature.
  Changes slowly (~1°C over a session).
- Register `03 04` / `04 04`: surface temperature. Rises
  during heating: 0 → 4 → 7 → 28 → 45°C observed over
  30 seconds of increasing power.

### Power Delivery State (`02 05`)

```
Coil A → 0x709  02 05 00 01 00    idle / no pan
Coil A → 0x709  02 05 00 01 01    delivering power / pan present
```

### Pan Detection (`0x489` / `0x493`)

Appears to be sent by the coil when pan state changes — in the
capture, these messages only appear at times consistent with
pan placement/removal, not at regular intervals. Uses the same
byte pattern (`02 05 XX`) as the power delivery state register,
but on the `0x480` prefix instead of `0x700`.

```
Byte 0: 0x02
Byte 1: 0x05
Byte 2: pan state
```

| Value | Meaning         |
|-------|-----------------|
| `00`  | No pan detected |
| `01`  | Pan on zone 1   |
| `02`  | Pan on zone 2   |

For coil A (`0x489`): zone 1 = left burner.
For coil B (`0x493`): zone 1 = middle, zone 2 = right.

### Power Level Control

Sent by the controller to set the power level. Each command is
immediately acknowledged by the coil.

```
Controller → 0x689  01 04 00 XX    set coil A power level
Coil A     → 0x609  01 04 00 01    acknowledged
```

The coil always responds with `01` in byte 3 regardless of
the level set.

Power level mapping:

| User level | Byte 3 |
|------------|--------|
| Off        | `0x61` |
| 1          | `0x01` |
| 2          | `0x02` |
| 3          | `0x03` |
| 4          | `0x04` |
| 5          | `0x05` |
| 6          | `0x06` |
| 7          | `0x08` |
| 8          | `0x0A` |
| 9          | `0x0E` |
| 10 / boost | `0xF1` |
| Shutdown   | `0x00` |

Levels 7-10 are non-linear (skip intermediate values). The
coils may accept the skipped values (0x07, 0x09, etc.) for
finer-grained control — untested.

### Control Sub-Commands

The second byte in control messages selects the sub-command:

| Byte 1 | DLC | Sender     | Meaning            |
|--------|-----|------------|--------------------|
| `0x01` | 4   | Controller | Init / reset       |
| `0x02` | 4   | Controller | PSU power off (?)       |
| `0x03` | 4   | Controller | PSU prepare shutdown (?)|
| `0x04` | 4   | Controller | Power level, zone 1 |
| `0x05` | 4   | Controller | Power level, zone 2 (coil B only) |
| `0x06` | 5   | Controller | Configuration (bytes 3-4 unknown, `0E 74` observed) |

The coil echoes the sub-command back on `0x60x` with byte 3 =
`01` (ack) or `00` (during shutdown).

## Startup Sequence

All initiated by the controller. Coils respond with ACKs.

```
Controller → 0x680  00 01 00 06          system init
Controller → 0x581  (empty)              controller ready
Coil A     → 0x509  (empty)              coil A ready
Coil B     → 0x513  (empty)              coil B ready
PSU        → 0x519  (empty)              PSU ready

Controller → 0x789  00 03 00             query coil A serial
Coil A     → 0x709  00 03 00 01 ...      serial response
Controller → 0x789  00 04 00             query coil A config
Coil A     → 0x709  00 04 00 01 ...      config response
             (same for coil B via 0x793/0x713, PSU via 0x799/0x719)

Controller → 0x681  00 0B 00 00          controller announce
Coil A     → 0x609  00 0B 00 01          ack
Coil B     → 0x613  00 0B 00 01          ack
PSU        → 0x619  00 0B 00 01          ack

Controller → 0x689  01 01 00 01          init coil A
Coil A     → 0x609  01 01 00 01          ack
Controller → 0x689  01 04 00 61          coil A → standby
Coil A     → 0x609  01 04 00 01          ack
Controller → 0x693  01 01 00 01          init coil B
Coil B     → 0x613  01 01 00 01          ack
Controller → 0x693  01 04 00 61          coil B zone 1 → standby
Coil B     → 0x613  01 04 00 01          ack
Controller → 0x693  01 05 00 61          coil B zone 2 → standby
Coil B     → 0x613  01 05 00 01          ack

Controller → 0x689  01 06 00 0E 74       configure coil A
Coil A     → 0x609  01 06 00 01          ack
Controller → 0x693  01 06 00 0E 74       configure coil B
Coil B     → 0x613  01 06 00 01          ack

Controller → 0x681  00 0B 00 00          announce (startup done)
             (periodic status polling begins)
```

## Shutdown Sequence

```
Controller → 0x689  01 04 00 00          coil A power off
Coil A     → 0x609  01 04 00 01          ack
Controller → 0x693  01 04 00 00          coil B power off
Coil B     → 0x613  01 04 00 01          ack
Controller → 0x693  01 05 00 00          coil B zone 2 off
Coil B     → 0x613  01 05 00 01          ack

             (final status queries)

Controller → 0x689  01 01 00 00          coil A reset
Coil A     → 0x609  01 01 00 01          ack
Controller → 0x693  01 01 00 00          coil B reset
Coil B     → 0x613  01 01 00 01          ack
Controller → 0x699  01 03 00 02          PSU shutdown
PSU        → 0x619  01 03 00 01          ack
Controller → 0x699  01 02 00 00          PSU shutdown
PSU        → 0x619  01 02 00 01          ack

Controller → 0x681  00 0B 00 00          final announce
             (bus goes silent)
```

## Hardware for Interfacing

To replace the controller with a microcontroller:

- **MCU**: ESP32-C6 (built-in TWAI/CAN controller)
- **Transceiver**: SN65HVD230 (3.3V CAN) or MCP2562 (5V with
  VIO pin for 3.3V logic)
- **Connection**: CANH/CANL → RJ45 pins 1/2,
  5V from RJ45 pins 7/8 → MCU VIN, GND → RJ45 pin 4,
  RJ45 pin 5 (PSU ready signal) → GPIO input
- **Bypass cap**: 100nF ceramic between transceiver VCC and GND
- **Termination**: 120Ω between CANH and CANL if the existing
  bus lacks termination (try without first)

The MCU must:

1. Send the startup sequence (init, enumerate, config, announce)
2. Poll status registers periodically (~1s cycle)
3. Send power level commands when knob position changes
4. Send the shutdown sequence when all knobs return to zero
5. Listen for pan detection and status responses
