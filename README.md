Welcome page
TSDZ8 VESC Firmware (Hazza Edition)
Firmware source can be found under branches.

A cursed science project disguised as an ebike controller.

⚡ What This Repo Is

This is my custom fork of the VESC firmware for the Go-FOC S100 / VESC 100/250, modified to run more sane on my TSDZ8 mid-drive, i got tired of faulty original controllers and had a vesc lying around.

This is not your normal typical firmware.
This is tinker-fuel — expect experiments, PAS logic rewrites, custom logic rewrites, and things that may or may not scream when compiled.

🔧 What I’m Building Here
1. Current control throttle that works smoothly without the *CHAIN SLAP BANG NYLON GEAR GO REEEE* ✅implmented
2. PAS That can be used along side throttle + uart for bluetooth module
3. Plans to maybe try and get the original pas+torque sensor to work
4. Try and add support for the original speed sensor


what I’m implementing and have already done:

*Throttle tweaks.

When the selected control mode is current-based, the value runs through haz_throttle_process, which is our custom “current envelope”:
A 12 Hz one-pole low-pass smooths any bike jitter before we even think about torque.
We track both absolute battery current (l_in) and phase current (l_current_max). The normalized request is clamped so you can never ask for more than either limit.
At very low duty (the first ~8 % of electrical rotation) we scale the request down, which stops the “chain slap” spike when you crack the throttle from a dead stop.
There’s a launch boost window (below ~12 % duty and ~250 erpm) that enforces a small minimum torque so the bike actually pulls away instead of chattering—once duty or rpm climb, the boost fades out automatically.
Ramp-up/down is asymmetric: we accelerate between 10–30 A/s based on how far you’ve opened the throttle, but the controller never increases faster than the available headroom (phase/batt actuals are compared against their limits and fall back to a gentler 10 A/s if you’re already saturating). Roll-off is capped at 40 A/s so letting go feels immediate without a jerk.
Regen/brake commands (negative throttle) bypass the fancy stuff; we simply decay the drive command to zero and let the standard brake current path take over.

This is currently implemented and may be tuned and/or better yet, have its variables tied to an option currently not being used in vesc tool to tweak after compile time.

*Add pas support (currently working with a generic 3 wire cadence sensor, maybe add native tsdz8 sensor in near future)

cadence sensing on the PPM input

interrupt-based timing for accurate pulse spacing

a cadence-driven ERPM target (motor matches your legs)

throttle always overrides PAS

PAS only active when throttle is idle

zero CPU-wasting loops

Basically:
The motor mirrors your pedaling speed.
If I spin faster, it spins faster.
If I stop, it stops.
If I twist throttle, PAS shuts the hell up.


Throttle remains king in this firmware.
PAS never overrides or limits it.

3. S100-Specific Tweaks

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
