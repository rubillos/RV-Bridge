# RV-Bridge: HomeKit to RV-C Adapter

![RV-Bridge](/images/box_wire_scale.jpeg)

---

1. [Features](#features)
2. [Background](#background)
3. [Current Project State](#state)
4. [To-Do](#todo)
5. [Hardware](#hardware)
6. [Wiring](#wiring)
7. [Firmware Setup](#firmware)
8. [Finding Output Numbers](#outputs)
9. [Supported RVs](#rvs)
10. [3D Printing](#3dprint)
11. [Notes and Tips](#notes)
12. [Links](#links)

---
## <a name="features"></a>Features

* Connects to the RV-C network in many modern RVs.
    * RV-C is a subset of CAN-Bus running at 250kbps.
* Uses an ESP32 with a CAN-Bus interface.
* Connects lights, fans, switches, and thermostats to HomeKit.
* Fits inside the RV wiring panel.
* Plugs into an unused CAN-Bus socket for power and data.
* STL files for a 3D printed case are included.

---
## <a name="background"></a>Background

At the start of the pandemic we realized that international travel was going to be off the table for a significant duration. We decided it was time [to see more of the Western US](http://rickandrandy.com/map/index.html?RV) so we bought a class-A motorhome which we call ***The Penguin Express***. We've put [almost 30k miles on it so far](http://rickandrandy.com/?rvlife).

![Penguin Express](/images/Penguin_Express.jpg)

In addition to a bunch of 3D printed upgrades (we travel with a Prusa MK3S on board), I've done some arduino powered electronics work: a water valve for a reverse osmosis water filter with an LCD control panel, a GPS based clock for the bedroom that knows the exact timezone boundaries so it never needs to be set, and a GPS based altimeter and tire pressure monitor with a 7" color touchscreen for the dash.

The RV's lights, fans, and climate are all controlled through a [Firefly Integrations](https://fireflyint.com) [Vegatouch Spectrum](https://www.vegatouch.com) multiplex system. A 7" LCD panel is used to control everything, in addition to wireless keypads around the RV. There is also a bluetooth module that connects to an iOS app for controlling via an iPhone.

|   |   |
| --- | --- |
| ![Firefly Main](/images/Firefly_main.jpeg) | ![Firefly Lights](/images/Firefly_lights.jpeg) |
| ![Firefly Climate](/images/Firefly_climate.jpeg) | ![Firefly Fans](/images/Firefly_fans.jpeg) |

It's a great system and works really well for control of the RV devices, but the iOS app is a bit slow to load/connect and can only be used in proximity to the RV. Since we already have an Apple TV onboard driving our TVs which could double as a hub, I've always wondered if there was a way to control it all via HomeKit and the Home app.

Recently I came across some documentation for the bus protocol that's used by the Spectrum system, RV-C, a subset of CAN-Bus, as well as the open source project [CoachProxyOS](https://github.com/linuxkidd/coachproxy-os) that documents getting a Raspberry Pi set up to host a web page for controlling an RV's network using RV-C.

I also recently started playing with [HomeSpan](https://github.com/HomeSpan/HomeSpan), a library for implementing HomeKit accessories on an ESP32 microcontroller, for a HomeKit doorbell project.

RV-Bridge is the result of putting these pieces together.

---
## <a name="state"></a>Current Project State (v1.0.0)

* Homespan pairing works, devices show up in the Home app.
* CAN-Bus packet receiving works.
* RV-C messages are routed correctly to the HomeKit tiles.
* The correct RV-C packets are being sent over the bus based on changes made in the Home app for lights, switches, and fans.
* The RV devices respond correctly.
* Lights, switches, and fans are complete.
* Temperature readings from thermostats are reflected in the Home app.
* Thermostat control is complete.

---
## <a name="todo"></a>To-Do

* Some RVs have AC units that can work as a heat pump; only the AC portion of these units is currently handled. (Ours only has a furnace so I don't currently have a way to implement this.)

---
## <a name="hardware"></a>Hardware

Uses an ESP32 with a CAN-Bus interface, either as separate components, or more easily, this board I found from [skpang.co.uk](https://www.skpang.co.uk):

![ESP32 Module](/images/board_in_box.jpeg)

In the U.S. it's available on the CopperHillTech Website:<br>
[ESP32 with WiFi, Bluetooth Classic, BLE, CAN Bus Module](https://copperhilltech.com/esp32-wifi-bluetooth-classic-ble-can-bus-module/)

This board has everything needed, including a regulator for powering the device off of the 12V provided by the RV-C connector.

---
## <a name="wiring"></a>Wiring

The connector used by the Firefly system is a ***3M 37104-A165-00E MB*** which can be sourced from [Digikey](https://www.digikey.com/en/products/detail/3m/37104-A165-00E%2520MB/1855697)

Insert four 24AWG wires into the Can-Bus connector (I used silicone covered wire as they are much more flexible) and compress to make the connections. Twist the data and power pairs together and screw them into the terminal block on the CAN-Bus interface on the ESP32.

The CAN-Bus connector plugs into one of the available sockets inside the system wiring panel.

|  |  |
| :---: | :---: |
| <br>![Cable Wiring](/images/cable.jpeg) |![Can-Bus Connector Wiring](/images/CAN-connector-wiring.jpg) |
| ![G7 Panel](/images/G7_panel.jpeg) | ![Home App](/images/Home_App.PNG) |

---
## <a name="firmware"></a>Firmware Setup

- The project is set up for compilation with PlatformIO.
    * I use it via Microsoft's Visual Studio Code.
- Pretty sure you could also use the Arduino IDE. You'd need to install the following libraries:
    * elapsedMillis
    * miwagner/ESP32CAN
    * homespan/HomeSpan
- `config.h`
    * Duplicate `config-sample.h` to `config.h`. (Do not check this file into git, it will have your wifi passowrd in it)
    * Enter Wifi SSID and password for your RV network.
    * Include a definition file for your RV devices (see `Miramar_2020_3202.h` for an example).
        * Each switch has:
            * Output number.
            * Type: Lamp, DimmableLamp, or Switch.
            * Name.
        * Each fan has:
            * Output number for fan power.
            * Output number for up - optional, -1 if not used.
            * Output number for down - optional, -1 if not used.
            * Name.
        * Each thermostat has:
            * ID number, typically 0 based.
            * Output number for the A/C compressor.
            * Output number for low fan.
            * Output number for high fan.
            * Output number for furnace - optional, -1 if not present.
            * Name.
        * See [Finding Output Numbers](#outputs) below for details on output numbers.
- Flashing
    * If you are using an ESP32 with a USB-C connector and flashing from a Mac, you may need to connect it via a USB hub due to some timing weirdness around resetting the ESP32 into boot mode. I use a USB-C to 4 port USB-A hub with a USB-A to USB-C cable.
- Startup
    * Connect to the ESP32 via the Serial Monitor.
    * You should see a bunch of startup logging.
    * Then a message about being connected to Wifi and not being paired.
    * HomeSpan provides a command line interface that you can access through the Serial Monitor. Type '?' to see the available comands.
- Pairing
    * Be on the same wifi network and in close proximity to the ESP32.
    * In the Home app choose "Add Accessory".
    * Point the camera at this image:
    <br><br>
    ![Pairing Code](/images/defaultSetupCode.png)
    <br><br>
    * Tap on `RV-Bridge`.
    * Accept that this is an "unsupported" device.
    * Add the bridge and all of your accessories, choosing appropriate rooms and names for them.
    * Done!
- The status LED will flash based on what the bridge is doing:
    * 🟢 Green every 2 seconds as a heartbeat indicator.
    * 🔴 Red when CAN-Bus packets are sent.
    * 🔵 Blue when HomeKit messages are received.

---
## <a name="outputs"></a>Finding Output Numbers

The whole multiplex system connects back to a panel with outputs for all of the lights and fans. Each of these outputs has a unique number which may be printed on the panel's cover, and should also be found on a Network Diagnostic screen on the main LCD control screen.

![G7 Outputs](/images/G7_Outputs.jpeg)

*** ***USE EXTREME CAUTION WHEN ENTERING OUTPUT NUMBERS. THERE ARE OUTPUTS FOR THE RV SLIDES AND THINGS LIKE MOVEABLE BUNKS. YOU DO NOT WANT TO MISTAKENLY PICK ONE OF THOSE OUTPUTS FOR A LIGHT OR FAN!*** ***

---
## <a name="rvs"></a>Supported RV's

Currently the project includes definition files for these RVs in the `RV` folder:

[Miramar_2020_3202.h](/src/RV/Miramar_2020_3202.h) - 2020 Thor Miramar 32.2<br>
[Aria_2019_3901.h](/src/RV/Aria_2019_3901.h) - 2019 Thor Aria 39.1<br>
[Tiffin_2019_34PA.h](/src/RV/Tiffin_2019_34PA.h) - 2019 Tiffin Open Road 34PA<br>
[Jayco_2023_Terrain.h](/src/RV/Jayco_2023_Terrain.h) - 2023 Jayco Terrain 19Y

(Additional definition files are welcome!)

---
## <a name="3dprint"></a>3D Printing

- A case will keep the microcontroller isolated from any exposed contacts inside the wiring panel.
- STL Files are in the `3D` folder:
    * [RV-Bridge_Box_Bottom.stl](/3D/RV-Bridge_Box_Bottom.stl)
    * [RV-Bridge_Box_Top.stl](/3D/RV-Bridge_Box_Top.stl)
- Slicer
    * I used Prusa Slicer 2.5.0
- Build Plate
    * A textured build plate give a nice surface finish for the top and bottom of the box.
- Filament
    * PETG - handles heat better than PLA and sticks to a textured build plate much better than PLA.
- Settings to adjust:
    * `Layer Height`: 0.3mm (faster printing)
    * `Extrusion Width`: 0.55mm (eliminates tiny infill strips in the walls).
    * `Perimeter Transitioning Threshold Angle`: 20 (keeps the lettering connected).
    * `Bridging Angle`: 180º (bridging in the layer above the lettering should be parallel to the baseline of the text.)

---
## <a name="notes"></a>Notes and Tips

- If the bridge seems to become unresponsive at some point, verify that the controlling device is on the RV's Wifi and not some other weak Wifi.
- If the bridge doesn't seem available for pairing, it may already think it's paired. Try using the H command via the cli in the serial monitor, then reflash the ESP32 and try again.
- If pairing fails, it seems that sometimes HomeKit gets fussy about a device changing it's properties too much and refuses to pair. You can change the MAC address of the wifi interface on the ESP32 by defining `OVERRIDE_MAC_ADDRESS` in `config.h` and re-flashing. Anecdotal evidence suggests that this can help.

---
## <a name="links"></a>Links:

- [Apple's HomeKit Accessory Protocol Specification Release R2 (HAP-R2)](https://developer.apple.com/homekit/specification/)
    * For some reason this link appears to be broken on the Apple side at the moment... with _just a tiny bit_ of hunting on the internet you can find it 😉
- [RV-C Organization](http://www.rv-c.com)
- [RV-C Spec 2022-12-01](http://www.rv-c.com/sites/rv-c.com/files/RV-C%20Protocol%20FullLayer-12-01-22.pdf)
- [SK Pang Electronics](https://www.skpang.co.uk)
- [Schematic for the ESP32 CAN-Bus Board above](https://cdn.shopify.com/s/files/1/0563/2029/5107/files/ESP32_CAN_rev_B.pdf?v=1620032162)

---

Copyright © 2023 [Randy Ubillos](http://rickandrandy.com)
