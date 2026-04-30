# VESC firmware

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Travis CI Status](https://travis-ci.com/vedderb/bldc.svg?branch=master)](https://travis-ci.com/vedderb/bldc)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/75e90ffbd46841a3a7be2a9f7a94c242)](https://www.codacy.com/app/vedderb/bldc?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=vedderb/bldc&amp;utm_campaign=Badge_Grade)
[![Contributors](https://img.shields.io/github/contributors/vedderb/bldc.svg)](https://github.com/vedderb/bldc/graphs/contributors)
[![Watchers](https://img.shields.io/github/watchers/vedderb/bldc.svg)](https://github.com/vedderb/bldc/watchers)
[![Stars](https://img.shields.io/github/stars/vedderb/bldc.svg)](https://github.com/vedderb/bldc/stargazers)
[![Forks](https://img.shields.io/github/forks/vedderb/bldc.svg)](https://github.com/vedderb/bldc/network/members)

An open source motor controller firmware.

This is the source code for the VESC DC/BLDC/FOC controller. Read more at
[https://vesc-project.com/](https://vesc-project.com/)

---

## Hazza's TSDZ8 Mid-Drive Fork

Custom fork for a TSDZ8 mid-drive e-bike motor on a MakerX GO-FOC S100 D100S controller.

### What's different

**Direct Torque UART** — The TSDZ8's XMC1302 torque sensor controller sends raw torque ADC
values over UART at 500Hz directly to the VESC, bypassing the original display protocol entirely.
The XMC handles calibration (32-sample average + 10 margin at boot). The VESC takes its output
at face value — zero means zero, no second offset subtraction.

**Katana Smoothing** — Adaptive noise filter for the torque signal, inspired by
[Casainho's OpenSource Firmware (OSF)](https://github.com/OpenSourceEBike/TSDZ2-Smart-EBike)
for the TSDZ2/TSDZ8 series motors. OSF's extensive work on torque sensor signal processing
and the TSDZ platform in general was invaluable — Katana builds on those foundations with a
ring-buffer median approach that cuts through noise spikes while preserving genuine torque
transients. Configurable via `haz_torque_smoothing` (0 = passthrough, 0.3 = light, 0.7+ = heavy).

**PAS Integration** — Cadence-based pedal assist with idle timeout detection, cadence
deceleration ramp-down, and hybrid duty mode for smooth power delivery through a 38:1
internal gearbox.

**ESP32-S3 BLE Display** — Companion 7" touchscreen display connected via BLE through the
NRF52 on the S100 board. Selective telemetry, torque test screen with dual-line graphing
(raw torque + current command), voice event system with swappable voice packs, and full
ride analytics.

### Credits

- **Benjamin Vedder** — VESC firmware and the entire VESC ecosystem
- **Casainho & OSF contributors** — OpenSource Firmware for TSDZ2/TSDZ8, torque sensor
  signal processing research, and the original firmware that made these motors hackable
- **MakerX** — GO-FOC S100 D100S hardware

---

## Supported boards

All of them!

Check the supported boards by typing `make`

```
[Firmware]
     fw   - Build firmware for default target
                            supported boards are: 100_250 100_250_no_limits 100_500...
```

There are also many other options that can be changed in [conf_general.h](conf_general.h).

## Prerequisites

### On Ubuntu (Linux)/macOS
- Tools: `git`, `wget`, and `make`
- Additional Linux requirements: `libgl-dev` and `libxcb-xinerama0`
- Helpful Ubuntu commands:
```bash
sudo apt install git build-essential libgl-dev libxcb-xinerama0 wget git-gui
```
- Helpful macOS tools: 

```bash
brew install stlink
brew install openocd
```

### On Windows
- Chocolately: https://chocolatey.org/install
- Git: https://git-scm.com/download/win. Make sure to click any boxes to add Git to your Environment (aka PATH)

## Install Dev environment and build

### On Ubuntu (Linux)/MacOS
Open up a terminal
1.  `git clone http://github.com/vedderb/bldc.git`
2.  `cd bldc`
3.  Continue with [On all platforms](#on-all-platforms)

### On Windows

1.  Open up a Windows Powershell terminal (Resist the urge to run Powershell as administrator, that will break things)
2.  Type `choco install make`
3.  `git clone http://github.com/vedderb/bldc`
4.  `cd bldc`
5.  Continue with [On all platforms](#on-all-platforms)

### On all platforms

1.  `git checkout origin/master`
2.  `make arm_sdk_install`
3.  `make` <-- Pick out the name of your target device from the supported boards list. For instance, I have a Trampa **VESC 100/250**, so my target is `100_250`
4.   `make 100_250` <-- This will build the **VESC 100/250** firmware and place it into the `bldc/builds/100_250/` directory

## Other tools

**Linux Optional - Add udev rules to use the stlink v2 programmer without being root**
```bash
wget vedder.se/Temp/49-stlinkv2.rules
sudo mv 49-stlinkv2.rules /etc/udev/rules.d/
sudo udevadm trigger
```

## IDE
### Prerequisites
#### On macOS/Linux

- `python3`, and `pip`

#### On Windows
- Python 3: https://www.python.org/downloads/. Make sure to click the box to add Python3 to your Environment.

### All platforms

1.  `pip install aqtinstall`
2.  `make qt_install`
3.  Open Qt Creator IDE installed in `tools/Qt/Tools/QtCreator/bin/qtcreator`
4.  With Qt Creator, open the vesc firmware Qt Creator project, named vesc.pro. You will find it in `Project/Qt Creator/vesc.pro`
5.  The IDE is configured by default to build 100_250 firmware, this can be changed in the bottom of the left panel, there you will find all hardware variants supported by VESC

## Upload to VESC
### Method 1 - Flash it using an STLink SWD debugger

1.  Build and flash the [bootloader](https://github.com/vedderb/bldc-bootloader) first
2.  Then `_flash` to the target of your choice. So for instance, for the VESC 100/250: 
```bash
make 100_250_flash
```

### Method 2 - Upload Firmware via VESC tool through USB

1.  Clone and build the firmware in **.bin** format as in the above Build instructions

In VESC tool

2.  Connect to the VESC
3.  Navigate to the Firmware tab on the left side menu 
4.  Click on Custom file tab
5.  Click on the folder icon to select the built firmware in .bin format (e.g. `build/100_250/100_250.bin`)

##### [ Reminder : It is normal to see VESC disconnects during the firmware upload process ]  
#####  **[ Warning : DO NOT DISCONNECT POWER/USB to VESC during the upload process, or you will risk bricking your VESC ]**  
#####  **[ Warning : ONLY DISCONNECT your VESC 10s after the upload loading bar completed and "FW Upload DONE" ]**

6.  Press the upload firmware button (downward arrow) on the bottom right to start upload the selected firmware.
7.  Wait for **10s** after the loading bar completed (Warning: unplug sooner will risk bricking your VESC)
8.  The VESC will disconnect itself after new firmware is uploaded.

## In case you bricked your VESC
you will need to upload a new working firmware to the VESC.  
However, to upload a firmware to a bricked VESC, you have to use a SWD Debugger.


## Contribute

Head to the [forums](https://vesc-project.com/forum) to get involved and improve this project.
Join the [Discord](https://discord.gg/JgvV5NwYts) for real-time support and chat

## Tags

Every firmware release has a tag. They are created as follows:

```bash
git tag -a [version] [commit] -m "VESC Firmware Version [version]"
git push --tags
```

## License

The software is released under the GNU General Public License version 3.0
