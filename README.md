# RV-Bridge: HomeKit to RV-C Bridge

## Features

* Uses ESP32 with a CAN-Bus interface
* Connects to RV-C network in many recent model RVs
* Connects lights, fans, and thermostats to HomeKit

## Current Project State

* Homespan pairing works, devices show up in Home app.
* Can-Bus receiving works.

To-Do:

* Routing Can-Bus messages to HomeKit devices
* Verify sending of correct RV-C messages

## Hardware

Uses an ESP32 with a CAN-Bus interface, either separate components, or more easily:

![ESP32 Module](https://cdn10.bigcommerce.com/s-7f2gq5h/products/272/images/967/ESP32_CAN-Bus_board_1__70903.1585405015.500.750.jpg?c=2)

[ESP32 WiFi, Bluetooth Classic, BLE, CAN Bus Module](https://copperhilltech.com/esp32-wifi-bluetooth-classic-ble-can-bus-module/)

## Wiring

![Can-Bus Connector Wiring](docs/CAN-connector-wiring.jpg)

## Usage

To-Do

## 3D Printing

- STL Files in the docs folder:
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

## References

To-Do
