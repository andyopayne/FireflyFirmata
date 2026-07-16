# Firefly Firmata

Arduino firmware for [Firefly](https://www.fireflyexperiments.com), the Grasshopper plugin
that connects Rhino to microcontrollers, sensors, and actuators. Flash one of these sketches
to your board and Firefly's components can talk to it over USB serial.

This repository holds two sketches:

| Sketch | Protocol | Use with |
|---|---|---|
| **Firefly_Firmata_2** | Firmata 2 — a runtime-configured text protocol (major version 2) | The Firefly 2 component family: Connect Board, Analog/Digital Read and Write, Servo Write, Stepper Write, Stepper Jog |
| **Firefly_Firmata_Legacy** | The original fixed-pin CSV protocol, unchanged since 2015 | Classic Firefly components (Uno Read / Uno Write), including their legacy versions in Firefly 2 |

If you are starting fresh, use **Firefly_Firmata_2**.

## Quick start

1. Open the sketch in the [Arduino IDE](https://www.arduino.cc/en/software)
   (each sketch lives in a folder named after itself, as the IDE requires).
2. Select your board and port (*Tools → Board*, *Tools → Port*).
3. Upload, then close the IDE (or its Serial Monitor) so the port is free.
4. In Grasshopper, drop Firefly's **Connect Board** component and pick the port.

If you installed the Firefly plugin through Rhino's package manager, you don't need to
download anything from here: the plugin bundles both sketches and copies them into your
Arduino sketchbook (`Documents/Arduino/`) the first time Grasshopper loads.

## About Firefly Firmata 2

One universal sketch for every supported board — no editing, no per-project variants.
The host discovers what your board can do and configures pins over the wire at runtime.
Everything is human-readable text at 115200 baud; open a serial monitor
(newline line endings) and try it yourself:

```
hello
ok hello fw=firefly2 ver=2.0.0 board=uno mcu=atmega328p volts=5.0
caps?
cap a0 modes=ain,din,dout ain=10
cap d3 modes=din,dout,pwm,servo pwm=8
...
cfg d13 dout
ok cfg d13 dout
set d13=1
ok set d13=1
```

That last command just turned on the board's LED.

Highlights:

- **Capability discovery** — `caps?` reports every pin and what it supports; the host
  never hardcodes a pin map.
- **Subscription reporting** — an idle board is silent; data flows only after a
  `report` command, at the cadence you ask for, optionally only on change.
- **Servos and steppers** — non-blocking stepper motion with trapezoidal
  acceleration ramps, for step/dir driver boards (A4988, DRV8825, EasyDriver) and
  4-wire darlington stages (ULN2003 + 28BYJ-48). No stepper library required.
- **Self-healing** — if the board resets, the host notices and reconfigures it.

Boards: Arduino Uno, Mega, Leonardo/Yun, Due, Uno R4, and boards using the RP2040/RP2350
(Pico) and ESP32 cores. The sketch compiles everywhere from a single file; the capability
query tells the host what each board actually offers.

The full wire grammar — every command, reply, and event, with worked examples — is
specified in [PROTOCOL.md](PROTOCOL.md). If you want to drive the firmware from your own
software (Python, Processing, anything with a serial port), that document is all you need.

## License

MIT — see [LICENSE](LICENSE). The sketches are meant to be read, edited, and learned
from; that's why the protocol is text.
