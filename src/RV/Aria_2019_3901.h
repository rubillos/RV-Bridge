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
 
// Device definitions for a 2020 Thor Aria 3901

#include "devices.h"

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

// index, upIndex[or -1], downIndex[or -1], name
const FanDeviceRec fanList[] = {
    21, 25, 26, "Kitchen",
    22, 27, 28, "Mid Bathroom",
    23, 29, 30, "Rear Bathroom"
};

// coolingInstance, compressorIndex, fanHIndex, fanLIndex, furnaceInstance[or -1], combustionIndex[or -1], name
const ThermostatDeviceRec thermostatList[] = {
    0, 37, 38, 39, 2, 33, "Front",
    1, 45, 46, 47, -1, -1, "Back"
};
