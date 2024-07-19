/*********************************************************************************
 *  MIT License
 *  
 *  Copyright (c) 2023 Randy Ubillos
 *  
 *  https://github.com/rubillos/RV-Bridge
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *  
 ********************************************************************************/
 
// Device definitions for a 2020 Thor Miramar 32.2

#include "devices.h"

// index, type, name
const SwitchDeviceRec switchList[] = {
    1, DimmableLamp, "Living Room",
    2, DimmableLamp, "Hall",
    4, DimmableLamp, "Bedroom",
    16, Lamp, "Kitchen Counter",
    17, Lamp, "Sofa",
    18, Lamp, "Bathroom",
    24, Lamp, "Cargo",
    39, Lamp, "Steps",
    57, Lamp, "Awning Light",
    59, Lamp, "Vanity",

    23, Switch, "Water Pump"
};

// index, upIndex[or -1], downIndex[or -1], name
const FanDeviceRec fanList[] = {
    20, 51, 52, "Kitchen",
    21, -1, -1, "Bathroom"
};

// coolingInstance, compressorIndex, fanHIndex, fanLIndex, furnaceInstance[or -1], combustionIndex[or -1], name
const ThermostatDeviceRec thermostatList[] = {
    0, 25, 26, 27, 2, 33, "Front",
    1, 29, 30, 31, -1, -1, "Back"
};

// extendIndex, retractIndex, extendTimeMS, rollExtendTimeMS, retractTimeMS, rollRetractTimeMS, name
#define HAVE_AWNINGS
const AwningDeviceRec awningList[] = {
    5, 6, 23*1000, 7*1000, 28*1000, 7*1000, "Awning"
};
