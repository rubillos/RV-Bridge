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
 
// Device definitions for a 2019 Tiffin Open Road 34PA

#include "devices.h"

// index, type, name
const SwitchDeviceRec switchList[] = {
    1, Lamp, "Ceiling Light",
    2, Lamp, "Entry Light",
    3, Lamp, "Task Light",
    4, Lamp, "Hall Light",
    5, Lamp, "Bedroom Light",
    6, Lamp, "Bathroom Light",
    8, Lamp, "Floor Light",
    9, Lamp, "Dining Room Light",
    10, Lamp, "Living Room Sconce",
    11, Lamp, "TV Accent Light",
    12, Lamp, "Awning Light",
    94, Lamp, "Porch Light", 

    33, Switch, "Ceiling Fan",  
    93, Switch, "Water Pump",
    95, Switch, "Electric Water Heater",
    96, Switch, "Gas Water Heater"
};

// index, upIndex[or -1], downIndex[or -1], name
const FanDeviceRec fanList[] = {
    23, 21, 22, "Kitchen Fan",
    19, 17, 18, "Bathroom Fan"
};

// coolingInstance, compressorIndex, fanHIndex, fanLIndex, furnaceInstance[or -1], combustionIndex[or -1], name
const ThermostatDeviceRec thermostatList[] = {
    0, 25, 26, 27, 3, 35, "Front AC",
    2, 29, 30, 31, 4, 36, "Bedroom AC"
};
