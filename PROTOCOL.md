# Firefly Firmata 2 — wire protocol

Version: **2** (the major number of the firmware's reported `ver=`) · updated 2026-07-16
Scope: the serial line protocol between a host (Grasshopper running the Firefly plugin,
or anything else that speaks it) and a board running the Firefly Firmata 2. This document
is the source of truth; the firmware in this repository and the host-side parser in the
Firefly plugin both implement *it*, never each other.

## Goals

- **One universal firmware, configured at runtime.** No sketch regeneration, no
  per-project variants. The host discovers what a board can do (`caps?`) and
  configures it over the wire (`cfg`).
- **Human-readable text.** Every exchange must make sense typed by hand in a serial
  monitor (or Firefly's own Serial Read/Write components). Didactic value is a
  design requirement, not a nicety.
- **Nothing unsolicited until asked.** An idle, unconfigured board is silent — no
  hard-coded 10 ms broadcast loop. Data flows only after a `report` subscription.
- **Verifiable configuration.** Every command is acknowledged, so the host knows
  its configuration landed — and can re-push it after a board reset.

Non-goals (v2.0): binary framing, baud negotiation, network transports, I2C/SPI
passthrough, per-pin debouncing.

## Transport & framing

- Serial, **115200 baud, fixed** (8N1). Only classic AVR boards (Uno, Mega,
  Leonardo/Yun) have a physical baud rate; the native-USB boards (Due, R4, ESP32,
  Pico) ignore the number. Negotiation is deliberately omitted.
- Messages are ASCII lines terminated by `\n` (`\r\n` accepted, `\r` ignored).
- Everything is **lowercase**. Tokens are separated by single spaces. Values are
  decimal integers. Named arguments use `key=value` tokens.
- Maximum line length: **256 bytes** including the terminator, both directions.
  A longer command is answered with `err`. (Sized for a Mega-scale report list;
  the buffer is the board's main RAM cost, so the ceiling is deliberate.)
- Long payloads never need long lines: a board may split one report sample across
  several `!r` lines, and a host may split a large write across several `set`
  lines — both formats are keyed, so every line stands alone. Only the `report`
  command is an atom, and 256 bytes covers it.
- Blank lines and unrecognised lines are ignored by the board (never fatal).

## Message taxonomy

Three kinds of lines, distinguishable at a glance:

| Kind | Direction | Shape |
|---|---|---|
| Command | host → board | `verb [args…]` |
| Reply | board → host | zero or more payload lines, then exactly one status line: `ok verb …`, `wrn verb … reason`, or `err verb reason…` |
| Event | board → host | starts with `!` — sent unsolicited |

Reply statuses map onto Grasshopper's runtime-message levels:

- `ok` — the command was applied exactly as asked.
- `wrn` — **the command was applied**, but not exactly as asked (value clamped,
  cadence floored). Same machine-readable shape as the `ok` — verb, then
  *effective* values — with a reason token at the end. A host that doesn't care
  about warnings may treat `wrn` identically to `ok`.
- `err` — nothing was applied.

Rules:

- **Every command produces exactly one status line**, echoing the verb. Replies
  come in command order, so a pipelining host matches them by sequence.
- Payload lines (e.g. `cap …`) always precede their status line.
- Events may interleave between a command and its reply (reports don't pause for
  acks); a host parser must tolerate that.
- Warnings are reserved for the board *deviating from a request or struggling at
  runtime* — they are not a logging channel.

## Session lifecycle

1. Host opens the port. On AVR boards, DTR toggling resets the MCU (~2 s bootloader
   delay); native-USB boards may or may not reset.
2. On boot the board announces itself: `!ready fw=firefly2 ver=2.0.0 board=uno`.
3. Host sends `hello` to confirm identity, `caps?` to learn the pin map, then a
   series of `cfg` commands and a `report` subscription.
4. **Self-healing:** the host treats `!ready` at any moment as "the board rebooted,
   all configuration is lost" and re-pushes step 3. Grasshopper — not the board —
   is the source of truth for configuration; replugging a board heals itself.
5. `reset` returns the board to its boot state (all pins unconfigured, reports
   stopped) without a hardware reset.

## Commands

### `hello`
Identity check. Reply mirrors the `!ready` fields plus detail:

    ok hello fw=firefly2 ver=2.0.0 board=uno mcu=atmega328p volts=5.0

`ver` is the firmware version, and its **major number is the wire-protocol
version** — the compatibility gate. A host refuses to operate a board whose major
it doesn't implement (this document specifies major **2**). Minor/patch mark
firmware releases — bug fixes, new modules — that leave the grammar intact; hosts
display them (support asks for them) but never gate on them. The major digit is
reserved for grammar breaks: a firmware rewrite that keeps the wire format stays
within its major.

`volts` is the logic level — the host surfaces it so a 3.3 V board (Due, ESP32,
Pico) is never a surprise. Key=value field order is not significant.

### `caps?`
One payload line per configurable resource, then `ok caps <count>`:

    cap d2 modes=din,dout
    cap d3 modes=din,dout,pwm,servo pwm=8
    cap a0 modes=ain,din,dout ain=10
    cap dac0 modes=dac dac=12
    cap m1 modes=stepper
    ok caps 5

- Pin names are the board's own labels: `d<n>`, `a<n>`, `dac<n>`. Analog pins are
  usable digitally (`cfg a0 din`) where hardware allows — the name identifies the
  physical pin, the mode defines its use.
- Resolution is given in bits per mode where it varies (`ain=10`, `pwm=8`,
  `dac=12`). Value ranges are always `0 … 2^bits − 1`; servos are degrees `0–180`.
- `m<n>` are **motor slots** — firmware resources, not pins. The pins a motor uses
  are bound at `cfg` time. The slot count is a firmware/RAM budget: 2 on the small
  AVR boards (Uno, Leonardo, Yun), 4 elsewhere.
- Hosts must never hardcode a board's pin map; `caps?` is the only authority.

### `cfg <target> <mode> [args]`
Configure a pin or motor slot. Reconfiguring is always allowed; the last `cfg`
wins. (Pin *conflicts* — two components claiming one pin — are a host-side error,
detected in Grasshopper before anything is sent.)

    cfg d2 din pullup         → ok cfg d2 din pullup
    cfg d3 servo              → ok cfg d3 servo
    cfg a0 ain                → ok cfg a0 ain
    cfg d13 servo             → err cfg d13 servo unsupported
    cfg m1 stepper step=d2 dir=d3 speed=800 accel=1600
                              → ok cfg m1 stepper step=d2 dir=d3 speed=800 accel=1600

Pin modes: `din` (arg `pullup` optional), `dout`, `ain`, `pwm`, `servo`, `dac`.

Motor mode: `stepper`, in three driver flavours selected by `type=`:

    cfg m1 stepper step=d2 dir=d3 speed=800 accel=1600        (type=stepdir, the default)
    cfg m1 stepper type=full4 pins=d2,d3,d4,d5 speed=300      (4-wire, full-step)
    cfg m1 stepper type=half4 pins=d2,d3,d4,d5 speed=600      (4-wire, half-step)

- `stepdir` drives a step/dir driver board (A4988, DRV8825, TMC…) via `step=` and
  `dir=` pins. One *step* here is whatever the driver's microstep switches make it.
- `full4`/`half4` drive a 4-wire unipolar/darlington stage (ULN2003 + 28BYJ-48 kit
  steppers) directly via `pins=` — four coil pins in IN1…IN4 order. Full-step is
  4 states per cycle (more torque), half-step 8 (smoother, twice the steps/rev).
  Coils stay energised when the motor is at rest (holding torque).
- `speed=` (steps/s, peak) and `accel=` (steps/s², trapezoidal ramp) are optional,
  default 1000. The firmware clamps `speed` to what the board can generate and
  answers `wrn … clamped` — the echoed value is the effective peak rate.

Configuring a motor claims its pins; the board rejects a `cfg` whose pins are held
by a motor (`err cfg d2 dout claimed m1`), and a motor `cfg` whose pins are held by
*another* motor (`err cfg m1 stepper claimed m2`).

Re-`cfg`ing a motor slot with the **same type and pins is a retune**: only
`speed`/`accel` change, and position, target, coil phase and any motion in flight
are all preserved — the ramp re-plans and the motion continues under the new
limits (a live speed change slows or quickens a move, it never kills it). Changing
the type or any pin is a full reconfigure: previous pins are released, motion
halts silently, and the position resets to 0.

### `report <list> every <ms> [delta <n>]` · `report none`
Subscribe to input reporting. **One subscription per session** — each `report`
replaces the previous one; `report none` stops reporting.

    report a0,a1,d2 every 20            → ok report a0,a1,d2 every 20
    report a0 every 20 delta 4          → ok report a0 every 20 delta 4
    report a0 every 5                   → wrn report a0 every 10 floored
    report none                         → ok report none

- `every <ms>`: sampling cadence, minimum 10. A smaller value is floored to 10
  and the subscription goes live — flagged with a `wrn` echoing the effective
  cadence.
- `delta <n>` (optional): send a report only when some listed value changed since
  the last report — by at least `n` for analog channels; digital channels report
  any change. Without `delta`, every sample is sent.
- Listed pins must already be configured (`ain`/`din`), else `err`.

Reports arrive as `!r` events with `key=value` pairs for the subscribed pins:

    !r a0=512 a1=97 d2=1

Keyed, not positional — a line is meaningful on its own, robust to subscription
changes mid-stream, and readable in a monitor.

### `set <pin>=<value> [<pin>=<value> …]`
Write one or more configured output pins in a single line:

    set d3=90                 → ok set d3=90
    set d5=1 d6=200 dac0=2048 → ok set d5=1 d6=200 dac0=2048
    set d3=200                → wrn set d3=180 clamped

- Values out of range are **clamped** and applied — live slider data shouldn't
  fault a session — but the reply is a `wrn` echoing the *effective* values, so
  nothing is hidden.
- Writing an unconfigured or non-output pin: `err set d5 unconfigured`.

### `move <motor> <steps>` · `moveto <motor> <pos>` · `stop <motor>`
Non-blocking stepper motion (AccelStepper semantics): `move` is relative,
`moveto` targets an absolute position, `stop` decelerates to a halt. The `ok`
acknowledges *acceptance*; completion arrives later as an event:

    move m1 2000              → ok move m1 2000
    …                           (motion runs in the background)
                                !done m1 pos=2000

A new `move`/`moveto` while moving retargets the motion (no queue — the ramp
adjusts, overshooting and coming back if the new target is behind the braking
distance). `stop` also yields `!done` with the position where the motor came to
rest. A motion command that lands exactly on the current position replies `ok`
and emits `!done` immediately. Position is a signed step count, zeroed when the
motor is configured; open-loop steppers know nothing of the physical position —
zero is wherever the motor sat at `cfg` time.

Motion commands for an unconfigured slot answer `err move m1 unconfigured`.
`reset` (and therefore every configuration re-push) halts motors *silently* — no
`!done` — because the host initiated it and knows; hosts clear their motion state
when a session re-establishes.

### `zero <motor>`
Homing: declares the motor's current position to be 0 — the open-loop answer to
"this physical pose is my reference point". Only meaningful at rest; a moving
motor is refused (it is ambiguous where its in-flight motion should now end):

    zero m1                   → ok zero m1
    zero m1     (while moving)→ err zero m1 moving

The calibration flow is: drive the shaft onto the reference mark (jog or slider),
let it come to rest, `zero` it. Position is otherwise zeroed only when the motor
is (re)configured with new pins/type or after `reset`.

### `reset`
Return to boot state: all pins unconfigured, reports stopped, motors halted.
Replies `ok reset` (this is a command reply, not a reboot — no `!ready` follows).

## Events

| Event | Meaning |
|---|---|
| `!ready fw=… ver=… board=…` | Board (re)booted. Host must re-push configuration. |
| `!r k=v …` | Report sample (see `report`). |
| `!done m<n> pos=<p>` | A stepper finished or was stopped. |
| `!wrn reason…` | Runtime trouble not tied to a command, e.g. `!wrn report overrun` when samples are being dropped. |
| `!err …` | Reserved: asynchronous fault not tied to a command. |

`!wrn` is emitted **once when a condition begins** (repeated at most every few
seconds while it persists), never per occurrence — a warning per dropped sample
would flood the very wire that is already saturated. Hosts map `!wrn`/`wrn` to
Grasshopper warnings and `err`/`!err` to errors on the owning component.

## Worked example — Uno vs Due

Uno (5 V, 10-bit ADC, no DAC):

    !ready fw=firefly2 ver=2.0.0 board=uno
    hello
    ok hello fw=firefly2 ver=2.0.0 board=uno mcu=atmega328p volts=5.0
    caps?
    cap d2 modes=din,dout
    cap d3 modes=din,dout,pwm,servo pwm=8
    …
    cap a0 modes=ain,din,dout ain=10
    …
    cap m1 modes=stepper
    cap m2 modes=stepper
    ok caps 22
    cfg a0 ain
    ok cfg a0 ain
    cfg d3 servo
    ok cfg d3 servo
    report a0 every 20
    ok report a0 every 20
    !r a0=512
    set d3=90
    ok set d3=90
    !r a0=514

Due (3.3 V, 12-bit ADC, two true DACs) — same grammar, different answers:

    ok hello fw=firefly2 ver=2.0.0 board=due mcu=sam3x8e volts=3.3
    cap a0 modes=ain,din,dout ain=12
    cap dac0 modes=dac dac=12
    cfg dac0 dac
    ok cfg dac0 dac
    set dac0=2048
    ok set dac0=2048

## Limits & timing

- Board→host bandwidth budget: at 115200 baud (~11.5 KB/s), a 100 Hz subscription
  must keep its `!r` lines under ~115 bytes — larger samples need a slower
  cadence. The 10 ms `every` floor exists for this reason. Firmware drops a
  sample rather than buffering unboundedly, and announces it
  (`!wrn report overrun`, once per condition).
- Firmware drains its entire RX buffer each loop iteration and never blocks in a
  handler (lesson carried over from the v1 sketch fixes).
- Step pulses are generated from the main loop (`micros()`-scheduled, no timer
  interrupts), so the peak step rate is a board property, not a promise: the
  firmware clamps `speed=` to its ceiling (2000 steps/s on the 16 MHz AVRs, 8000
  on the 32-bit boards) and answers `wrn … clamped`. Heavy serial traffic can
  stretch individual step intervals — motion stays correct in step *count*,
  merely slower.
- Hosts should treat a missing reply after ~1 s as a dead/foreign device, not
  retry forever.

## Open items

- **Report keepalive:** with `delta`, a quiet board is indistinguishable from a
  dead one. Proposal: optional `max <ms>` ("send a full report at least this
  often") — not yet committed.
- **Stepper state query** (`state? m1` — position/moving) for hosts that join a
  session late.
- Whether `caps?` should include the firmware's motor-slot count on boards where
  RAM, not firmware, is the limit.
- Exact `board=`/`mcu=` identifier tables for the supported boards (uno, mega,
  leonardo, yun, due, unor4, redboard-esp32, pico2).
