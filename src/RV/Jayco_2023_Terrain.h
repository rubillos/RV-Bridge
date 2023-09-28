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
 
// Device definitions for a 2023 Jayco Terrain 19Y

#include "devices.h"

// index, type, name
const SwitchDeviceRec switchList[] = {
    // light switches
    1,  Lamp, "Spotlight",
    32, DimmableLamp, "Main Ceiling Lights",
    34, DimmableLamp, "Bunk Accent Lights",
    24, DimmableLamp, "Bed Ceiling Lights",
    22, DimmableLamp, "Kitchen Counter Lights",
    35, DimmableLamp, "Bench Lights",
    25, DimmableLamp, "Cargo Lights",
    21, Lamp, "Awning Lights",
    23, Lamp, "Step Lights",

    // accessory switches
    3,  Switch, "Awning Extend",
    4,  Switch, "Awning Retract",
    8,  Switch, "Grey Water Tank Heater",
    11, Switch, "Fan High",
    12, Switch, "Fan Low",
    43, Switch, "A/C Cool",
    44, Switch, "Water Pump"
    
};

// index, upIndex[or -1], downIndex[or -1], name
const FanDeviceRec fanList[] = {
    20, 51, 52, "Kitchen" //,
    // 21, -1, -1, "Bathroom"
};

// coolingInstance, compressorIndex, fanHIndex, fanLIndex, furnaceInstance[or -1], combustionIndex[or -1], name
const ThermostatDeviceRec thermostatList[] = {
    0, 25, 26, 27, 2, 33, "Front"//,
    //1, 29, 30, 31, -1, -1, "Back"
};
