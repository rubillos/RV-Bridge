# RV-Bridge: HomeKit to RV-C Adapter

![RV-Bridge](/images/box_wire_scale.jpeg)

---
## Features

* Connects to the RV-C network in many recent model RVs.
    * RV-C is a subset of CAN-Bus running at 250kbps.
* Uses an ESP32 with a CAN-Bus interface.
* Connects lights, fans, and thermostats to HomeKit.
* Fits inside the RV wiring panel.
* Plugs into an unused CAN-Bus socket for power and data.
* STL files for a 3D printed case are included.

---
## Background

At the start of the pandemic we realized that international travel was going to be off the table for a significant duration. We decided it was time to see more of the US West so we bought a class-A motorhome which we call The Penguin Express. We've put [almost 30k miles on it so far](http://rickandrandy.com/?rvlife).

![Penguin Express](/images/Penguin_Express.jpg)

In addition to a bunch of 3D printed upgrades (we travel with a Prusa MK3S on board), I've done some arduino powered electronics work: a water valve for a reverse osmosis water filter with an LCD control panel, a GPS based clock for the bedroom that knows the exact timezone boundaries so it never needs to be set, and a gps based altimeter and tire pressure monitor with a 7" color display for the dash.

The RV's lights, fans, and climate are all controlled through a [Firefly Integrations](https://fireflyint.com) [Vegatouch Spectrum](https://www.vegatouch.com) multiplex system. A 10" LCD panel is used to control everything, in addition to wireless keypads around the RV. There is also a bluetooth module that connects to an iOS app for controlling via an iPhone.

|   |   |
| --- | --- |
| ![Firefly Main](/images/Firefly_main.jpeg) | ![Firefly Lights](/images/Firefly_lights.jpeg) |
| ![Firefly Climate](/images/Firefly_climate.jpeg) | ![Firefly Fans](/images/Firefly_fans.jpeg) |

It's a great system and works really well for control around the RV, but the iOS app is a bit slow to load/connect and can only be used in proximity to the RV. I've always wondered if there was a way to control it all via HomeKit and the Home app.

Recently I came across some documentation for the bus protocol that's used by the Spectrum system, RV-C, a subset of CAN-Bus, as well as the open source project [CoachProxyOS](https://github.com/linuxkidd/coachproxy-os) that documents getting a Raspberry Pi set up to host a web page for controlling an RV's network using RV-C.

I also recently started playing with [HomeSpan](https://github.com/HomeSpan/HomeSpan), a library for implementing HomeKit accessories on an ESP-32 microcontroller, for a HomeKit doorbell project.

This project is the result of putting these pieces together.

---
## Current Project State (v0.2.0)

* Homespan pairing works, devices show up in the Home app.
* CAN-Bus message receiving works.
* CAN-Bus messages route correctly to HomeKit devices.
* The correct RV-C messages are being sent over the bus based on changes made in the Home app for lights, switches, and fans.
* The RV devices respond correctly.
* Lights, switches, and fans are complete.
* The temperature readings from thermostats are reflected in the Home app.

---
## To-Do:

* Thermostat setting functions.

---
## Hardware

Uses an ESP32 with a CAN-Bus interface, either as separate components, or more easily, this board from _skpang.co.uk_:

![ESP32 Module](/images/board_in_box.jpeg)

You can find this on the CopperHillTech Website:<br>
[ESP32 with WiFi, Bluetooth Classic, BLE, CAN Bus Module](https://copperhilltech.com/esp32-wifi-bluetooth-classic-ble-can-bus-module/)

This board has everything needed, including a regulator for powering the device off of the 12V provided by the RV-C connector.

---
## Wiring

Connector is a 3M 37104-A165-00E MB and can be sourced from [Digikey](https://www.digikey.com/en/products/detail/3m/37104-A165-00E%2520MB/1855697)

Insert 24AWG wires into the Can-Bus connector (I used silicone covered wire as they are much more flexible) and compress to make the connections. Twist the data and power pairs together and screw them into the terminal block on the CAN-Bus interface on the ESP32.

The CAN-Bus connector plugs into one of the available sockets inside the system wiring panel.

|  |  |
| :---: | --- |
| <br>![Cable Wiring](/images/cable.jpeg) |![Can-Bus Connector Wiring](/images/CAN-connector-wiring.jpg) |
| ![G7 Panel](/images/G7_panel.jpeg) | ![Home App](/images/Home_App.PNG) |

---
## Firmware Setup

- The project is set up for compilation with PlatformIO.
- You can also use the Arduino IDE. You'll need to install the following libraries:
    * elapsedMillis
    * miwagner/ESP32CAN
    * homespan/HomeSpan
- `config.h`
    * Rename `config-sample.h` to `config.h`.
    * Enter Wifi SSID and password for your RV network.
    * Include a definition file for the RV devices (see `Miramar_2020_3202.h` for an example).
        * Each light will have an output number and a name, and a flag specifying if it can be dimmed.
        * Each fan has three output numbers, one for the fan power, and one each for the up and down outputs, which are optional.
        * Each thermostat has a number, typically 0 based, and output numbers for the A/C compressor, low fan, high fan, and a furnace output, which is optional.
        * See "Finding Output Numbers" below for details on output numbers.
- Flashing
    * If you are using an ESP32 with a USB-C connector and flashing from a Mac, you may need to connect it via a USB hub due to some timing weirdness around resetting the ESP32 into boot mode. I use a USB-C to 4 port USB-A hub with a USB-A to USB-C cable.
- Startup
    * Connect to the ESP32 via the Serial Monitor.
    * You should see a bunch of the startup logging.
    * Then a message about being connected to Wifi and not being paired.
- Pairing
    * Be on the same wifi network and close to the ESP32.
    * In the Home app choose "Add Accessory".
    * Point the camera at this image:
    <br><br>
    ![Pairing Code](/images/defaultSetupCode.png)
    <br><br>
    * Tap on `RV-Bridge`.
    * Accept that this is an "unsupported" device.
    * Add the bridge and all of your accessories, choosing appropriate rooms and names for them.
    * Done!

---
## Finding Output Numbers

The whole multiplex system connects back to a panel with outputs for all of the lights and fans. Each of these outputs has a unique number which may be printed on the panel's cover, and should also be found on a Network Diagnostic screen on the main LCD control screen.

![G7 Outputs](/images/G7_Outputs.jpeg)

*** ***USE CAUTION WHEN ENTERING OUTPUT NUMBERS. THERE ARE OUTPUTS FOR THE RV SLIDES AND THINGS LIKE MOVEABLE BUNKS. YOU DO NOT WANT TO MISTAKENLY PICK ONE OF THOSE OUTPUTS!*** ***

---
## Supported RV's

Currently the project includes definition files for these RVs in the `RV` folder:

`Miramar_2020_3202.h` - 2020 Thor Miramar 32.2<br>
`Aria_2019_3901.h` - 2019 Thor Aria 39.1

Additional definition files are welcome.

---
## 3D Printing

- A case will keep the microcontroller isolated from any exposed contacts in the wiring panel.
- STL Files are in the `3D` folder:
    * `RV-Bridge_Box_Bottom.stl`
    * `RV-Bridge_Box_Top.stl`
- Slicer
    * Prusa Slicer 2.5.0
- Filament
    * PETG (handles heat better than PLA).
- Settings
    * Layer height 0.3mm
    * Set extrusion width to 0.55 (eliminates tiny infill strips in walls).
    * Perimeter transitioning threshold angle to 20 (keeps the lettering connected).

---
## Notes

- If the bridge seems to become unresponsive at some point, verify that the controlling device is on the RV's Wifi and not some other weak Wifi.
- If the bridge doesn't seem available for pairing, it may already think it's paired. Try using the H command via the cli in the serial monitor, then reflash the ESP32 and try again.
- If pairing fails, it seems that sometimes HomeKit gets fussy about a device changing it's properties too much and refuses to pair. You can change the MAC address of the wifi interface by defining '`OVERRIDE_MAC_ADDRESS`' in `config.h` and re-flashing. Anecdotal evidence suggests that this can help.

---
## Links:

- [Apple's HomeKit Accessory Protocol Specification Release R2 (HAP-R2)](https://developer.apple.com/homekit/specification/)
    * For some reason this link appears to be broken at the moment... with a _small bit_ of hunting on the internet you can find it 😉
- [RV-C Organization](http://www.rv-c.com)
- [RV-C Spec 2022-12-01](http://www.rv-c.com/sites/rv-c.com/files/RV-C%20Protocol%20FullLayer-12-01-22.pdf)
- [SK Pang Electronics](https://www.skpang.co.uk)
- [Schematic for the ESP32 CAN-Bus Board above](https://cdn.shopify.com/s/files/1/0563/2029/5107/files/ESP32_CAN_rev_B.pdf?v=1620032162)
