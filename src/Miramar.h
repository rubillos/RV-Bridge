#include "devices.h"

// Device definitions for a 2020 Thor Miramar 32.2

// index, type, name
const SwitchDeviceRec switchList[] = {
    1, DimmableLamp, "Living Room",
    2, DimmableLamp, "Hall",
    4, DimmableLamp, "Bedroom",
    6, Lamp, "Kitchen Counter",
    17, Lamp, "Sofa",
    18, Lamp, "Bathroom",
    24, Lamp, "Cargo",
    39, Lamp, "Steps",
    57, Lamp, "Awning",
    59, Lamp, "Vanity",

    23, Switch, "Water Pump"
};

// index, upIndex, downIndex, name
const FanDeviceRec fanList[] = {
    20, 51, 52, "Kitchen",
    21, 47, 48, "Bathroom"
};

// index, compressorIndex, fanHIndex, fanLIndex, furnaceIndex, name
const ThermostatDeviceRec thermostatList[] = {
    1, 25, 26, 27, 33, "Front",
    2, 29, 30, 31, -1, "Back"
};
