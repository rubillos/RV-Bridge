# RV-Bridge: HomeKit to RV-C Bridge

![RV-Bridge](/docs/images/box_wire_scale.jpeg)

---
## Features

* Connects to RV-C network in many recent model RVs
    * (RV-C is a subset of CAN-Bus running at 250kbps)
* Uses ESP32 with a CAN-Bus interface
* Connects lights, fans, and thermostats to HomeKit
* Plugs into unused CAN-Bus socket inside RV control panel.
* STL files for 3D printed case are included.

---
## Background

At the start of the pandemic we realized that international travel was going to be off the table for a significant duration. We decided it was time to see more of the US West so we bought a class-A motorhome which we call The Penguin Express. We've put [almost 30k miles on it so far](http://rickandrandy.com/?rvlife).

![Penguin Express](/docs/images/Penguin_Express.jpg)

In addition to a bunch of 3D printed upgrades (we travel with a Prusa MK3S on board) I've done some arduino powered electronics work: a water valve for a reverse osmosis water filter with an LCD control panel, a GPS based clock for the bedroom that knows the exact timezone boundaries so it never needs to be set, and a gps based altimeter and tire pressure monitor with a 7" color display for the dash.

The RV's lights, fans, and climate are all controlled through a [Firefly Integrations](https://fireflyint.com) [Vegatouch Spectrum](https://www.vegatouch.com) multiplex system. A 10" LCD panel is used to control everything, in addition to wireless keypads around the RV. There is also a bluetooth module that connects to an iOS app for controlling via an iPhone.

|   |   |
| --- | --- |
| ![Firefly Main](/docs/images/Firefly_main.jpeg) | ![Firefly Lights](/docs/images/Firefly_lights.jpeg) |
| ![Firefly Climate](/docs/images/Firefly_climate.jpeg) | ![Firefly Fans](/docs/images/Firefly_fans.jpeg) |

It's a great system, but the app is a bit slow to load and can only be used in proximity to the RV. I've always wondered if there was a way to control it all via HomeKit and the Home app.

Recently I came across some documentation for the bus protocol that's used by the Spectrum system: it's called RV-C and is a subset of CAN-Bus, as well as the open source project [CoachProxyOS](https://github.com/linuxkidd/coachproxy-os) that documents getting a Raspberry Pi set up to host a web page for controlling an RV's network.

I also recently started playing with [HomeSpan](https://github.com/HomeSpan/HomeSpan), a library for implementing HomeKit accessories on an ESP-32 microcontroller, for a HomeKit doorbell project.

This project is the result of putting these pieces together.

---
## Current Project State

* Homespan pairing works, devices show up in Home app.
* CAN-Bus receiving works.
* CAN-Bus messages route correctly to HomeKit devices.
* Correct RV-C messages are being sent over the bus.

---
## To-Do:

* Verify thermostat functions.

---
## Hardware

Uses an ESP32 with a CAN-Bus interface, either separate components, or more easily:

![ESP32 Module](docs/images/board_in_box.jpeg)

You can find this on the CopperHillTech Website:<br>
[ESP32 with WiFi, Bluetooth Classic, BLE, CAN Bus Module](https://copperhilltech.com/esp32-wifi-bluetooth-classic-ble-can-bus-module/)

This board has everything needed, including a regulator for powering the device off of the 12V provided by the RV-C connector.

---
## Wiring

Connector is a 3M 37104-A165-00E MB and can be sourced from [Digikey](https://www.digikey.com/en/products/detail/3m/37104-A165-00E%2520MB/1855697)

Insert 24AWG wires into Can-Bus connector and compress to make connections. Twist the data and power pairs together and screw into the terminal block on the CAN-Bus interface on the ESP32.

The CAN-Bus connector plugs into one of the available sockets on the system wiring panel.

|  |  |
| :---: | --- |
| <br>![Cable Wiring](docs/images/cable.jpeg) |![Can-Bus Connector Wiring](docs/images/CAN-connector-wiring.jpg) |
| ![G7 Panel](/docs/images/G7_panel.jpeg)<br>Available CAN-Bus sockets on G7 panel | ![spacer](/docs/images/spacer.png) |

---
## Firmware Setup

- Project is set up for compilation with PlatformIO
- config.h
    * Rename config-sample.h to config.h
    * Enter Wifi SSID and password for the RV network.
    * Include a definition file for the RV devices (see Miramar32_2.h for an example)
        * Each light will have an output number and a name, and a flag specifying if it can be dimmed.
        * Each fan has three output numbers, one for the fan power, and one each for the up and down output.
        * Each thermostat has a number, typically 0 based, an output numbers for the A/C compressor, low fan, high fan, and furnace outputs.
        * See "Finding Output Numbers" below for output numbers.
- Flashing
    * If using an ESP32 with a USB-C connector and flashing from a Mac, you may need to connect it via a USB hub due to some timing weirdness around resetting the ESP32 into boot mode. I use a USB-C to 4 port USB-A hub with a USB-A to USB-C cable.
- Startup
    * Connect to the ESP32 via Serial Monitor.
    * You should see all of the startup logging.
    * Then a message about being connected to Wifi.
- Pairing
    * In the Home app choose "Add Accessory".
    * Point the camera at this image:
    <br><br>
    ![Pairing Code](/docs/images/defaultSetupCode.png)
    <br><br>
    * Accept that this is an "unsupported" device.
    * Add the bridge and all of your accessories, choosing appropriate rooms for them.
    * Done!

---
## Finding Output Numbers

The whole multiplex system connects back to a panel with outputs for all of the lights and fans. Each of these outputs has a unique number which may be printed on the panel's cover, and should also be found on a Network Diagnostic screen on the main LCD control screen.

![G7 Outputs](/docs/images/G7_Outputs.jpeg)

*** ***USE CAUTION WHEN ENTERING OUTPUT NUMBERS. THERE ARE OUTPUTS FOR THE RV SLIDES AND THINGS LIKE MOVEABLE BUNKS. YOU DO NOT WANT TO MISTAKENLY PICK ONE OF THOSE OUTPUTS!*** ***

---
## 3D Printing

- A case will keep the microcontroller isolated from any exposed contacts in the wiring panel.
- STL Files are in the docs folder:
    * ***RV-Bridge_Box_Bottom.stl***
    * ***RV-Bridge_Box_Top.stl***
- Slicer
    * Prusa Slicer 2.5.0
- Filament
    * PETG (handles heat better than PLA)
- Settings
    * Layer height 0.3mm
    * Set extrusion width to 0.55 (eliminates tiny infill strips in walls)
    * Perimeter transitioning threshold angle to 20 (keeps the lettering connected)

---
## Notes

- If the bridge seems to become unresponsive at some point, verify that the controlling device is on the RV's Wifi and not some other weak Wifi.
- If the bridge doesn't seem available for pairing, it may already think it's paired. Try using the H command via the cli in the serial monitor, then reflash the ESP32 and try again.
- If pairing fails, sometimes HomeKit gets fussy about a device changing it's properties too much an refuses to pair. You can change the MAC address of the wifi interface by defining 'OVERRIDE_MAC_ADDRESS' in config.h and re-flashing.

---
## Links:

- [RV-C Organization](http://www.rv-c.com)
- [RV-C Spec 2022-12-01](http://www.rv-c.com/sites/rv-c.com/files/RV-C%20Protocol%20FullLayer-12-01-22.pdf)
- [SK Pang Electronics](https://www.skpang.co.uk)

---
## The docs folder contains:
- HAP-R2.pdf - HomeKit non-commercial protocol spec R2
- RV-C Protocol Fulllayer - RV-C protocol spec
- ESP32_CAN_revB - schematic for the CopperHillTech board.
