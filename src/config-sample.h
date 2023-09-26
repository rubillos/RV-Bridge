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
 
#include <stdint.h>

// #define OVERRIDE_MAC_ADDRESS {0x53, 0xC3, 0xD3, 0xBD, 0x20, 0x51}
// define the setup Access Point settings here. To use the HomeSpan defaults comment out the next 2 lines
#define rb_accesspoint_ssid "RVBridge-Setup"
#define rb_accesspoint_pwd "444442220"
#define rb_accesspoint_autostart false

//define a RV Bridge Hostname prefix (8 characters max.)
// #define rb_hostname_override "Jayco-"

// To preconfigure a fixed SSID and Password for your Wi-Fi network, uncomment the 2 lines below
// #define rb_wireless_ssid ""
// #define rb_wireless_password ""

#define sourceAddress 145

// set loglevel
// sets the logging level for diagnostic messages, where:
// 0 = top-level HomeSpan status messages, and any LOG0() messages specified in the sketch by the user (default)
// 1 = all HomeSpan status messages, and any LOG1() messages specified in the sketch by the user
// 2 = all HomeSpan status messages plus all HAP communication packets to and from the HomeSpan device, as well as all LOG1() and LOG2() messages specified in the sketch by the user
// -1 = supresses ALL HomeSpan status messages, including all LOG0(), LOG1(), and LOG2() messages specified in the sketch by the user, freeing up the Serial port for other uses
#define rb_logLevel_override 2

// specify the RV model template you want to use
#include "RV\Jayco_2023_Terrain.h"
