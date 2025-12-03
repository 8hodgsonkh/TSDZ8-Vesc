Welcome page
TSDZ8 VESC Firmware (Hazza Edition)
Firmware source can be found under branches.

A cursed science project disguised as an ebike controller.

⚡ What This Repo Is

This is my custom fork of the VESC firmware for the Go-FOC S100 / VESC 100/250, modified to run more sane on my TSDZ8 mid-drive, i got tired of faulty original controllers and had a vesc lying around.

This is not your normal typical firmware.
This is tinker-fuel — expect experiments, PAS logic rewrites, custom logic rewrites, and things that may or may not scream when compiled.

✅ 1. What This Project Is Building
1.1 Current-control throttle that behaves on a chain-driven mid-drive

– Smooth low-speed torque.
– No “chain slap → BANG BANG → nylon gear screaming REEEEE.”
– Adaptive ramping.
– Predictable launches.

Status: ✔ Fully implemented
Notes: Uses haz_throttle_process().

1.2 PAS sensor support (generic 3-wire cadence + throttle + UART)

– PAS and throttle run simultaneously.
– PAS only active when throttle = idle.
– Cadence → ERPM mapping.
– Interrupt-based timing on single sensor mode. quad mode still polling.
– Clean override logic.

Status: ✔ Fully implemented
Future: Integrate native TSDZ8 torque+cadence sensor if hardware feasibility works out.

1.3 Explore native TSDZ8 torque sensor

– Original torque sensor requires specialised signal conditioning.
– I have a small PCB from a dead controller that may provide the missing analog stages.

Status: ❌ Not implemented (hardware dependent)

1.4 Mid-drive-optimised FOC logic (“Hazza Mid-Drive Tuning”)

Chain slack kills normal FOC. This logic:

– Detects backlash (ERPM collapse, Iq overshoot, angle stall, mod saturation).
– Softens PI gains on impact.
– Slew-limits torque re-application.
– Gradually recovers stiffness.
– Eliminates bang-dead-bang FOC instability.

Status: ✔ Implemented (tuning ongoing)
Flag: Enable with #define HAZZA_MIDDRIVE_TUNING 1

1.5 External speed sensor support

Goal:
Use leftover pins  to support the TSDZ8 wheel speed sensor.

Status: ❌ Not implemented (pin constraints)
Past attempts:
– Tried using MCU temp input → failed
– ADC2 with pull-up hack works for PAS, may be repurposed later

1.6 Street/Off-road mode via power button (boot-time toggle)

– Street mode: throttle ERPM cap + PAS ERPM cap for UK legality
– Off-road mode: full power
– Uses power button like a TF2 Spycicle, gives some time immunity to P.C. pyro checking your bike
– Tap at boot = disguise as Heavy
– Hold for ~15 sec = Scout speed

Status: ✔ Fully implemented
Notes: Now protected so it can’t soft-brick the controller again.

1.7 Custom VESC Tool fork (future)

Goal: expose all Hazza variables cleanly:

– PAS tuneables → PAS App page
– Throttle ramp tuneables → ADC App page
– Mid-drive FOC tuneables → new “Hazza Mid-Drive Tuning” dropdown
– Street/off-road config (but NOT speed limits) → UART tab
– Maintain full compatibility with stock VESC Tool
– Add new serialization fields only at the end to avoid breaking existing layouts
– Ensure stock VESC Tool can still write configs without trashing Hazza options

Status: 🔄 In planning
Android build: planning

🔥 2. What Has Already Been Implemented (Technical Breakdown)
2.1 Custom Throttle Logic (haz_throttle_process)

✔ 12 Hz low-pass filter to remove jitter
✔ Normalizes request against batt + phase current limits
✔ Low-duty torque scaling (prevents instant chain snap at 0–8% duty)
✔ Launch-boost zone (<12% duty & <250 ERPM)
✔ Asymmetric ramping:

Ramp-up 10–30 A/s adaptive

Ramp-down 40 A/s hard
✔ Regen bypasses fancy logic
✔ Smooth, predictable starts

2.2 PAS Logic (Generic Cadence Sensor)

✔ Runs on PPM pin via interrupts
✔ Accurate cadence → ERPM mapping
✔ Throttle override
✔ PAS only active when throttle idle
✔ Zero busy loops
✔ Safe for high RPM cadence sensors
✔ Works with Bluetooth UART active simultaneously

2.3 Mid-Drive Safe FOC Logic

Implemented under the compile-time flag:

#define HAZZA_MIDDRIVE_TUNING 1

Core behaviour:

✔ Detects chain slap via:
– ERPM collapse
– Iq overshoot
– Modulation saturation
– Angle stall
– Iq jump events
✔ Switches through 3 states:
– IDLE → normal
– ACTIVE → torque heavily limited
– RECOVERING → gradual ramp-out
✔ Integrator bleed to prevent torque rebound
✔ PI gains reduced in ACTIVE
✔ PI gains interpolated in RECOVERING
✔ Holds halls authoritative
✔ Prevents observer from breaking traction control

🧩 3. Future Work Roadmap

Reduce Iq slew rates to match mid-drive power levels (40–70 A/s).

Integrate tuneables into custom VESC Tool.


Support multiple hardware targets (not only MakerX GO-FOC S100).

Add native speed sensor (TSDZ8)

Investigate torque-sensor feasibility




GO-FOC S100-Specific Tweaks

Things like:

board pin mappings

ADC behavior

hardware flags

whatever else the S100 needs to behave with a mid-drive instead of a hub motor

🧩 Why TSDZ8 + VESC Firmware?

To be clear:
I didn’t jump ship because the original TSDZ8+OSF setup was weak.
I actually had my own custom fork of OSF(thanks to mstrens for porting it from TSDZ2), tuned hard, stable, and genuinely strong — but at the end of the day it was still boxed in by the MCU.
Great firmware on a tiny chip only gets you so far.

The real reason I ended up here is way less dramatic:

my original TSDZ8 controller died,

every cheap aliexpress replacement controller I tried after that was failing because of questionable build quality

after my 3rd replacement I got tired of playing “Will today’s controller die?”

VESC fixes that in the most overkill way possible:

stable hardware

transparent motor control

open source support

PAS logic I can rebuild from scratch


The maker x GO-FOC S100 is cheap, mod-friendly, and powerful enough to treat a mid-drive like a science project — so it just made sense to go all-in.

Perfect excuse to experiment, learn, and make the TSDZ8 behave exactly the way I want.

Expect more modules as PAS logic evolves.

🛠️ How to Build (S100 / VESC 100/250)


For now, you need:

arm-none-eabi-gcc

make

the usual VESC toolchain

And then:

make go_foc_s100(replace with your hardware configuration)


Flash bin using vesc tool.


⚠️ Warnings

This firmware can:

behave weird

blow up if misconfigured

spin your cranks like a demon

make your mid-drive forget it’s not a motorcycle

Don’t run this on anyone else’s bike.
Test lightly.
Expect chaos until stable.

🧩 Credits

VESC Project (Benjamin + contributors)

Go-FOC S100 hardware folks

mstrens (OSF 860C / TSDZ8 OSF fork) — genuinely helped me understand the hardware side of the TSDZ8, especially how the motor, sensors, and current paths actually work. The OSF project is the reason I even knew what I was poking at when I started rewriting things here.

MakerX — for the GO-FOC S100 hardware and the hwconf files this whole thing stands on.

My own questionable ideas

💬 Contact

If you want to talk mid-drive firmware, or you’re modding a TSDZ8 with VESC too,
I’m usually somewhere online breaking things.
