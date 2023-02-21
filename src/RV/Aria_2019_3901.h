#include "devices.h"

// Device definitions for a 2020 Thor Aria 3901

// index, type, name
const SwitchDeviceRec switchList[] = {
    1, DimmableLamp, "Living Room",
    2, DimmableLamp, "Kitchen",
    3, Lamp, "Cab Ceiling",
    4, Lamp, "Rear Bathroom Ceiling",
    5, Lamp, "Aisle",
    6, DimmableLamp, "Bedroom Ceiling",
    7, Lamp, "Cargo",
    9, Lamp, "Under Cabinet",
    10, Lamp, "Theater Seats",
    12, Lamp, "Mid Bathroom Ceiling",
    13, Lamp, "Rear Bathroom Vanity?",
    14, Lamp, "Sofa",

    17, Switch, "TV Up",
    18, Switch, "TV Down",

    60, Lamp, "Awning",
};

// index, upIndex, downIndex, name
const FanDeviceRec fanList[] = {
    21, 25, 26, "Kitchen",
    22, 27, 28, "Mid Bathroom",
    23, 29, 30, "Rear Bathroom"
};

// index, compressorIndex, fanHIndex, fanLIndex, furnaceIndex, name
const ThermostatDeviceRec thermostatList[] = {
    0, 37, 38, 39, 33, "Front",
    1, 45, 46, 47, -1, "Back"
};
