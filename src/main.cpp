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
 
////////////////////////////////////////////////////////////////
//                                                            //
//    RV-Bridge: A HomeKit to RV-C interface for the ESP32    //
//                                                            //
////////////////////////////////////////////////////////////////

#include "Arduino.h"

#include "elapsedMillis.h"
#include "HomeSpan.h" 
#include "ESP32CAN.h"
#include "CAN_config.h"

#include "config.h"

//////////////////////////////////////////////

constexpr const char* versionString = "v1.0.0";

constexpr uint8_t indicatorPinR = 2;
constexpr uint8_t indicatorPinG = 15;
constexpr uint8_t indicatorPinB = 4;

constexpr gpio_num_t canTxPin = GPIO_NUM_25;
constexpr gpio_num_t canRxPin = GPIO_NUM_26;

constexpr uint16_t receiveQueueSize = 10;
constexpr uint16_t sendQueueSize = 8;

constexpr uint32_t sendPacketIntervalmS = 50;
constexpr uint32_t minSendPacketIntervalmS = 5;

constexpr uint32_t packetBlinkTime = 25;
constexpr uint32_t heatbeatRate = 3000;
constexpr uint32_t heartbeatBlinkTime = 10;

elapsedMillis lastPacketSendTime = 1000;
elapsedMillis lastPacketRecvTime = 1000;

CAN_device_t CAN_cfg;               	// CAN Config

//////////////////////////////////////////////

enum {
	packetPrintNo = 0,
	packetPrintYes,
	packetPrintIfUnknown,
	packetPrintIfKnown
};

void displayPacket(CAN_frame_t *packet, uint8_t printPacket);
uint8_t packetPrintMode = packetPrintNo;

//////////////////////////////////////////////
// RV-C dgn numbers

constexpr uint32_t DATE_TIME_STATUS = 0x1FFFF;
constexpr uint32_t DC_DIMMER_COMMAND_2 = 0x1FEDB;
constexpr uint32_t DC_DIMMER_STATUS_3 = 0x1FEDA;
constexpr uint32_t THERMOSTAT_AMBIENT_STATUS = 0x1FF9C;
constexpr uint32_t THERMOSTAT_STATUS_1 = 0x1FFE2;
constexpr uint32_t THERMOSTAT_COMMAND_1 = 0x1FEF9;

constexpr uint32_t AIR_CONDITIONER_STATUS = 0x1FFE0; // 1FFE1
constexpr uint32_t GENERIC_INDICATOR_COMMAND = 0x1FED9;
constexpr uint32_t GENERIC_CONFIGURATION_STATUS = 0x1FED8;
constexpr uint32_t DC_SOURCE_STATUS_1 = 0x1FFFD;
constexpr uint32_t TANK_STATUS = 0x1FFB7;
constexpr uint32_t GENERATOR_STATUS_1 = 0x1FFDC;

constexpr uint32_t FURNACE_STATUS = 0x1FFE4;
constexpr uint32_t FURNACE_COMMAND = 0x1FFE3;

constexpr uint32_t INVERTER_TEMPERATURE_STATUS = 0x1FEBD;	
constexpr uint32_t DC_LOAD_STATUS_2 = 0x1FED;
constexpr uint32_t ATS_AC_STATUS_4 = 0x1FF85;
constexpr uint32_t ATS_STATUS = 0x1FFAA;	
constexpr uint32_t ATS_AC_STATUS_3 = 0x1FFAB;
constexpr uint32_t ATS_AC_STATUS_1 = 0x1FFAD;
constexpr uint32_t DC_LOAD_COMMAND = 0x1FFBC;
constexpr uint32_t DC_LOAD_STATUS = 0x1FFBD;
constexpr uint32_t AC_LOAD_STATUS = 0x1FFBF;
constexpr uint32_t CHARGER_AC_STATUS_1 = 0x1FFCA;
constexpr uint32_t INVERTER_STATUS = 0x1FFD4;
constexpr uint32_t INVERTER_AC_STATUS_1 = 0x1FFD7;

constexpr uint32_t AWNING_STATUS = 0x1FEF3;
constexpr uint32_t AWNING_COMMAND = 0x1FEF2;
constexpr uint32_t AWNING_STATUS_2 = 0x1FDCD;

constexpr uint32_t BATTERY_STATUS = 0x1AAFD;

//////////////////////////////////////////////

typedef enum {
	DCDimmerCmdSetBrightness = 0,
	DCDimmerCmdOnDuration,
	DCDimmerCmdOnDelay,
	DCDimmerCmdOff,
	DCDimmerCmdStop,
	DCDimmerCmdToggle,
	DCDimmerCmdMemoryOff,
	DCDimmerCmdRampBrightness,
	DCDimmerCmdRampToggle,
	DCDimmerCmdRampUp,
	DCDimmerCmdRampDown,
	DCDimmerCmdRampUpDown,
	DCDimmerCmdLock,
	DCDimmerCmdUnlock,
	DCDimmerCmdFlash,
	DCDimmerCmdFlashMomentarily,

	DCDimmerCmdNA = 255
} DCDimmerCmd;

typedef enum {
	ACModeAuto = 0,
	ACModeManual,

	ACModeNA = 3
} ACMode;

typedef enum {
	ThermostatModeOff = 0,
	ThermostatModeCool,
	ThermostatModeHeat,
	ThermostatModeAuto,
	ThermostatModeFanOnly,
	ThermostatModeAuxHeat,
	ThermostatModeDehumidify,

	ThermostatModeNA = 7
} ThermostatMode;

typedef enum {
	FanModeAuto = 0,
	FanModeOn,

	FanModeNA = 3
} FanMode;

constexpr double TempC_NA = 1775.0;

constexpr uint8_t RVCPercentMax = 250;
constexpr uint8_t RVCBrightMax = 200;
constexpr uint8_t RVCFanMax = 200;
constexpr uint8_t HomeKitPercentMax = 100;

//////////////////////////////////////////////

constexpr float homeKitShadeOpenValue = 100.0;
constexpr float homeKitShadeClosedValue = 0.0;

constexpr uint8_t homeKitPositionStateClosing = 0;
constexpr uint8_t homeKitPositionStateOpening = 1;
constexpr uint8_t homeKitPositionStateStopped = 2;

constexpr uint8_t homeKitTemperatureDisplayCelsius = 0;
constexpr uint8_t homeKitTemperatureDisplayFahrenheit = 1;

// constexpr float closePerMS = HomeKitPercentMax / closeTimeMS;
// constexpr float openPerMS = HomeKitPercentMax / openTimeMS;

constexpr uint32_t updateTimeMS = 500;
constexpr uint32_t quickUpdateTimeMS = 100;

typedef uint16_t ShadeState;
constexpr uint16_t shadeStateOpen = 0;
constexpr uint16_t shadeStateStopped = 1;
constexpr uint16_t shadeStateClosed = 2;

constexpr uint16_t shadeStateClosing = 1 << 3;
constexpr uint16_t shadeStateOpening = 1 << 4;

constexpr uint16_t shadeStateUserAction = 1 < 5;
constexpr uint16_t shadeStateLocalAction = 1 << 6;
constexpr uint16_t shadeStateHomeKitAction = 1 << 7;

//////////////////////////////////////////////

uint64_t millis64() {
	volatile static uint32_t low32 = 0, high32 = 0;
	uint32_t new_low32 = millis();

	if (new_low32 < low32)
		high32++;

	low32 = new_low32;

	return (uint64_t) high32 << 32 | low32;
}

//////////////////////////////////////////////

uint32_t getMsgBits(uint32_t msg, int8_t startBit, int8_t numBits) {
	int8_t shift = startBit - numBits + 1;
	uint32_t mask = 0xFFFFFFFF >> (32 - numBits);

	return (msg >> shift) & mask;
}

uint32_t makeMsg(uint32_t dgn, uint8_t sourceID=0, uint8_t priority=6) {
	if (sourceID == 0) {
		sourceID = sourceAddress;
	}

	return (priority<<26) | (dgn << 8) | sourceID;	
}

//////////////////////////////////////////////

constexpr double tempCOffset = -273.0;
constexpr double tempCScale = 0.03125;
constexpr double tempCScaleInv = 1.0 / tempCScale;
// constexpr double tempCRoundingOffset = 0.25;
constexpr double tempCRoundingOffset = -0.25;

double convToTempC(uint16_t value) {
	return tempCOffset + value * tempCScale + tempCRoundingOffset;
}

uint16_t convFromTempC(double tempC) {
	return (tempC - tempCOffset + 0.5) / tempCScale;
}

//////////////////////////////////////////////

CAN_frame_t packetQueue[sendQueueSize];
bool packetShortGap[sendQueueSize];
uint16_t packetQueueHead = 0;
uint16_t packetQueueTail = 0;
bool doCANWrite = true;

void processPacketQueue() {
	if (packetQueueHead != packetQueueTail) {
		uint16_t nextIndex = (packetQueueTail + 1) % sendQueueSize;
		uint32_t interval = (packetShortGap[nextIndex]) ? minSendPacketIntervalmS : sendPacketIntervalmS;

		if (lastPacketSendTime >= interval) {
			if (doCANWrite) {
				printf("%u: CAN-Bus Send Packet\n", (uint32_t)millis());
				ESP32Can.CANWriteFrame(&packetQueue[nextIndex]);
			}
			else {
				printf("%u: ***SIMULATE*** CAN-Bus Send Packet\n", (uint32_t)millis());
			}
			packetQueueTail = nextIndex;
			lastPacketSendTime = 0;
		}
	}
}

void queuePacket(CAN_frame_t *packet, bool shortGap=false) {
	uint16_t nextIndex = (packetQueueHead + 1) % sendQueueSize;

	while (nextIndex == packetQueueTail) {	// wait if queue is full
		processPacketQueue();
	}

	packetQueueHead = nextIndex;
	packetQueue[packetQueueHead] = *packet;
	packetShortGap[packetQueueHead] = shortGap;
}

void initPacket(CAN_frame_t* packet, uint8_t index, uint32_t msgID) {
	packet->FIR.B.DLC = 8;
	packet->FIR.B.RTR = CAN_no_RTR;
	packet->FIR.B.FF = CAN_frame_ext;
	packet->MsgID = makeMsg(msgID);
	packet->data.u8[0] = index;
	for (auto i=1; i<8; i++) {
		packet->data.u8[i] = 0xFF;
	}
}

//////////////////////////////////////////////

void sendDCDimmerCmd(uint8_t index, uint8_t brightness, uint8_t cmd, uint8_t duration=0xFF) {
	CAN_frame_t packet;
	initPacket(&packet, index, DC_DIMMER_COMMAND_2);
	uint8_t* d = packet.data.u8;

	d[2] = min(brightness, RVCBrightMax);
	d[3] = cmd;
	d[4] = duration;
	d[5] = 0;				// no interlock

	queuePacket(&packet);

	// printf("%u: Queueing Dimmer packet: #%d, bright=%d, cmd=%d, dur=%d\n", (uint32_t)millis(), index, brightness, cmd, duration);
	// displayPacket(&packet, packetPrintYes);
	// printf("%u: ** Packet queued.\n", (uint32_t)millis());
}

void sendOnOff(uint8_t index, bool on, uint8_t brightness=RVCBrightMax) {
	printf("%u: sendOnOff: #%d to %d\n", (uint32_t)millis(), index, on);
	sendDCDimmerCmd(index, brightness, (on) ? DCDimmerCmdOnDuration : DCDimmerCmdOff);
	// sendDCDimmerCmd(index, brightness, DCDimmerCmdToggle);
}

void sendLampLevel(uint8_t index, uint8_t brightness) {
	if (brightness == 0) {
		sendOnOff(index, false);
	}
	else {
		printf("%u: sendLampLevel: #%d to %d\n", (uint32_t)millis(), index, brightness);
		sendDCDimmerCmd(index, brightness, DCDimmerCmdSetBrightness, 0);
	}
}

void sendThermostatCommand(uint8_t index, ThermostatMode mode, FanMode fanMode, uint8_t fanSpeed, double tempC) {
	CAN_frame_t packet;
	initPacket(&packet, index, THERMOSTAT_COMMAND_1);
	uint8_t* d = packet.data.u8;

	d[1] = mode | (fanMode << 4);
	d[2] = fanSpeed;

	uint16_t tempVal = convFromTempC(tempC);
	d[4] = d[6] = tempVal >> 8;
	d[3] = d[5] = tempVal & 0xFF;

	queuePacket(&packet);

	// printf("%u: Queueing Thermostat packet: #%d, mode=%d, fanMode=%d, temp=%0.1fºC\n", (uint32_t)millis(), index, mode, fanMode, tempC);
	// displayPacket(&packet, packetPrintYes);
	// printf("%u: ** Packet queued.\n", (uint32_t)millis());
}

//////////////////////////////////////////////

const char* switchTypes[3] = { "43", "43", "49" };
const char* switchHapNames[3] = { "LightBulb", "LightBulb", "Switch" };

struct RVSwitch : SpanService {
	SpanCharacteristic *_on;
	SpanCharacteristic *_brightness = NULL;
	int16_t _index;

	RVSwitch(SwitchDeviceRec *device) : SpanService( switchTypes[device->type], switchHapNames[device->type] ) {
		_index = device->index;

		REQ(On);
		OPT(Name);

		if (device->type == Lamp || device->type == DimmableLamp) {
			OPT(Brightness);
			OPT(Hue);
			OPT(Saturation);
			OPT(ColorTemperature);
		}

		_on = new Characteristic::On();

		if (device->type == DimmableLamp) {
			_brightness = new Characteristic::Brightness(HomeKitPercentMax);
			_brightness->setRange(0, HomeKitPercentMax, 5);
		}
	}
	
	boolean update() {
		if (_on->updated() || (_brightness && _brightness->updated())) {
			if (_brightness) {
				uint16_t newLevel = _on->getNewVal() * _brightness->getNewVal() * RVCBrightMax / HomeKitPercentMax;
				sendLampLevel(_index, newLevel);
			}
			else {
				sendOnOff(_index, _on->getNewVal());
			}
		}
		return(true);  
	}

	void setLevel(uint8_t index, uint8_t dcDimmerLevel) {
		bool on = dcDimmerLevel > 0;
		uint16_t level = dcDimmerLevel * HomeKitPercentMax / RVCBrightMax;

		if (index == _index && on != _on->getVal()) {
			printf("%u: Switch #%d: on = %d\n", (uint32_t)millis(), _index, on);
			_on->setVal(on);
			lastPacketRecvTime = 0;
		}
		if (_brightness && index == _index && on && level != _brightness->getVal()) {
			printf("%u: Switch #%d: level = %d\n", (uint32_t)millis(), _index, level);
			_brightness->setVal(level);
			lastPacketRecvTime = 0;
		}
	}
};

struct RVRoofFan : Service::Fan {
	SpanCharacteristic *_active;
	int16_t _index;
	int16_t _upIndex;
	int16_t _downIndex;

	bool _fanPower = false;
	bool _lidUp = false;

	RVRoofFan(FanDeviceRec *device) : Service::Fan() {
		_index = device->index;
		_upIndex = device->upIndex;
		_downIndex = device->downIndex;
		_active = new Characteristic::Active();
	}
	
	boolean update() {
		if (_active->updated()) {
			bool newValue = _active->getNewVal();
			sendOnOff(_index, newValue);
			if (_upIndex != -1 && _downIndex!=-1) {
				if (newValue) {
					sendOnOff(_downIndex, false);
					sendOnOff(_upIndex, true);
				}
				else {
					sendOnOff(_upIndex, false);
					sendOnOff(_downIndex, true);
				}
				_lidUp = newValue;
			}
			_fanPower = newValue;
		}

		return(true);  
	}
	
	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;
		bool changed = false;

		if (index == _index) {
			_fanPower = on;
			changed = true;
			lastPacketRecvTime = 0;
		}
		else if (index == _upIndex && on) {
			_lidUp = true;
			changed = true;
			lastPacketRecvTime = 0;
		}
		else if (index == _downIndex && on) {
			_lidUp = false;
			changed = true;
			lastPacketRecvTime = 0;
		}

		if (changed) {
			bool newState = _fanPower && (_upIndex==-1 || _lidUp);

			if (newState != _active->getVal()) {
				printf("%u: Fan #%d: active = %d\n", (uint32_t)millis(), _index, newState);
				_active->setVal(newState);
			}
		}
	}
};

// HomeKit thermostat values
enum {
	currentFanStateInactive = 0,
	currentFanStateIdle,
	currentFanStateBlowing,

	targetFanStateManual = 0,
	targetFanStateAuto,

	heatingCoolingStateOff = 0,
	heatingCoolingStateHeat,
	heatingCoolingStateCool
};

struct RVHVACFan : Service::Fan {
	SpanCharacteristic *_active;
	SpanCharacteristic *_currentState;
	SpanCharacteristic *_targetState;
	SpanCharacteristic *_speed;
	
	std::function<bool()> _updateFunction;

	int16_t _index;
	int16_t _fanHIndex;
	int16_t _fanLIndex;
	bool _fanHRunning = false;
	bool _fanLRunning = false;

	RVHVACFan(ThermostatDeviceRec *device, std::function<bool()> updateFunction) : Service::Fan() {
		_active = new Characteristic::Active(false);
		_speed = new Characteristic::RotationSpeed(0);
		_speed->setRange(0, HomeKitPercentMax, HomeKitPercentMax/2);
		_currentState = new Characteristic::CurrentFanState(currentFanStateIdle);
		_targetState = new Characteristic::TargetFanState(targetFanStateAuto);
	
		_index = device->coolingInstance;
		_fanHIndex = device->fanHIndex;
		_fanLIndex = device->fanLIndex;
		_updateFunction = updateFunction;
	}
	
	boolean update() {
		bool changed = false;

		if (_active->updated()) {
			printf("%u: RVHVACFan #%d - Active: %d\n", (uint32_t)millis(), _index, _active->getNewVal());
			changed = true;
		}
		if (_speed->updated()) {
			printf("%u: RVHVACFan #%d - Speed: %d\n", (uint32_t)millis(), _index, _speed->getNewVal());
			changed = true;
		}
		if (_targetState->updated()) {
			printf("%u: RVHVACFan #%d - Target State: %d\n", (uint32_t)millis(), _index, _targetState->getNewVal());
		}
		if (changed) {
			_updateFunction();
		}
		return(true);  
	}

	void setModeSpeed(FanMode fanMode, uint8_t speed) {
		bool newActive = fanMode == FanModeOn;
		speed = speed * HomeKitPercentMax / RVCPercentMax;

		if (newActive != _active->getVal()) {
			printf("%u: RVHVACFan #%d - setActive: %d\n", (uint32_t)millis(), _index, newActive);
			_active->setVal(newActive);
		}
		if (speed != _speed->getVal()) {
			printf("%u: RVHVACFan #%d - setSpeed: %d\n", (uint32_t)millis(), _index, speed);
			_speed->setVal(speed);
		}
	}

	void getModeSpeed(FanMode* fanMode, uint8_t *speed) {
		if (_active->getNewVal()) {
			*fanMode = FanModeOn;
			*speed = _speed->getNewVal() * RVCFanMax / HomeKitPercentMax;
		}
		else {
			*fanMode = FanModeAuto;
			*speed = 0;
		}
	}

	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;
		bool changed = false;

		if (index == _fanHIndex && on != _fanHRunning) {
			_fanHRunning = on;
			changed = true;
			lastPacketRecvTime = 0;
		}
		else if (index == _fanLIndex && on != _fanLRunning) {
			_fanLRunning = on;
			changed = true;
			lastPacketRecvTime = 0;
		}

		if (changed) {
			uint8_t newState = (_fanHRunning || _fanLRunning) ? currentFanStateBlowing : currentFanStateIdle;

			if (newState != _currentState->getVal()) {
				printf("%u: HVACFan #%d: currentState = %d\n", (uint32_t)millis(), _index, newState);
				_currentState->setVal(newState);
			}
		}
	}
};

struct RVThermostat : Service::Thermostat {
	SpanCharacteristic *_ambientTemp;
	SpanCharacteristic *_targetTemp;
	SpanCharacteristic *_targetState;
	SpanCharacteristic *_currentState;

	RVHVACFan *_fan;

	int16_t _coolingInstance;
	int16_t _compressorIndex;
	ThermostatMode _coolingMode = ThermostatModeOff;

	int16_t _furnaceInstance;
	int16_t _combustionIndex;
	ThermostatMode _furnaceMode = ThermostatModeOff;

	bool _compressorRunning = false;
	bool _furnaceRunning = false;

	RVThermostat(ThermostatDeviceRec *device) : Service::Thermostat() {
		_ambientTemp = new Characteristic::CurrentTemperature(20);
		_targetTemp = new Characteristic::TargetTemperature();
		_targetTemp->setRange(10, 32, (1.0 / 1.8))->setVal(20);
		_currentState = new Characteristic::CurrentHeatingCoolingState(heatingCoolingStateOff);
		_targetState = new Characteristic::TargetHeatingCoolingState(heatingCoolingStateOff);
		new Characteristic::TemperatureDisplayUnits(homeKitTemperatureDisplayFahrenheit);

		if (device->furnaceInstance != -1) {
			_targetState->setValidValues(3, heatingCoolingStateOff, heatingCoolingStateHeat, heatingCoolingStateCool);
		}
		else {
			_targetState->setValidValues(2, heatingCoolingStateOff, heatingCoolingStateCool);
		}

		_coolingInstance = device->coolingInstance;
		_compressorIndex = device->compressorIndex;

		_furnaceInstance = device->furnaceInstance;
		_combustionIndex = device->combustionIndex;

		_fan = new RVHVACFan(device, [this]()->bool { updateThermostat(); return true; });
	}

	void updateThermostat() {
		printf("%u: Thermostat #%d: Send thermostat info\n", (uint32_t)millis(), _coolingInstance);
		ThermostatMode modeLookup[] { ThermostatModeOff, ThermostatModeHeat, ThermostatModeCool };
		ThermostatMode opMode = modeLookup[_targetState->getNewVal()];
		FanMode fanMode;
		uint8_t speed;

		_fan->getModeSpeed(&fanMode, &speed);

		if (opMode == ThermostatModeOff && fanMode == FanModeOn) {
			opMode = ThermostatModeFanOnly;
		}

		if (_furnaceInstance!=-1 && opMode==ThermostatModeHeat) {
			if (_coolingMode != ThermostatModeOff) {
				sendThermostatCommand(_coolingInstance, ThermostatModeOff, FanModeAuto, 0xFF, _targetTemp->getNewVal<double>());
			}
			sendThermostatCommand(_furnaceInstance, opMode, FanModeAuto, 0xFF, _targetTemp->getNewVal<double>());
		}
		else {
			if (_furnaceMode != ThermostatModeOff) {
				sendThermostatCommand(_furnaceInstance, ThermostatModeOff, FanModeNA, 0xFF, _targetTemp->getNewVal<double>());
			}
			sendThermostatCommand(_coolingInstance, opMode, fanMode, speed, _targetTemp->getNewVal<double>());
		}
	}

	boolean update() {
		bool sendInfo = false;

		if (_targetState->updated()) {
			printf("%u: RVThermostat #%d, Target State: %d\n", (uint32_t)millis(), _coolingInstance, _targetState->getNewVal());
			sendInfo = true;
		}
		if (_targetTemp->updated()) {
			printf("%u: RVThermostat #%d - Target Temp: %f\n", (uint32_t)millis(), _coolingInstance, _targetTemp->getNewVal<double>());
			sendInfo = true;
		}
		if (sendInfo) {
			updateThermostat();
		}

		return(true);  
	}

	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;
		bool changed = false;

		if (index == _compressorIndex && on != _compressorRunning) {
			printf("%u: Thermostat #%d: compressor = %d\n", (uint32_t)millis(), index, on);
			_compressorRunning = on;
			changed = true;
		}
		if (index == _combustionIndex && on != _furnaceRunning) {
			printf("%u: Thermostat #%d: furnace = %d\n", (uint32_t)millis(), index, on);
			_furnaceRunning = on;
			changed = true;
		}
		if (changed) {
			uint8_t newState;

			if (_furnaceRunning) {
				newState = heatingCoolingStateHeat;
			}
			else if (_compressorRunning) {
				newState = heatingCoolingStateCool;
			}
			else {
				newState = heatingCoolingStateOff;
			}
			if (newState != _currentState->getVal()) {
				_currentState->setVal(newState);
				lastPacketRecvTime = 0;
			}
		}
		_fan->setLevel(index, level);
	}

	void setAmbientTemp(uint8_t index, double tempC) {
		if (index == _coolingInstance && fabs(tempC - _ambientTemp->getVal<double>()) > 0.2) {
			printf("%u: Set ambient temp #%d: %0.1fºC\n", (uint32_t)millis(), _coolingInstance, tempC);
			_ambientTemp->setVal(tempC);
			lastPacketRecvTime = 0;
		}
	}

	void setInfo(uint8_t index, ThermostatMode opMode, FanMode fanMode, uint8_t fanSpeed, double heatTemp, double coolTemp) {
		bool updateMode = false;

		if (index == _coolingInstance) {
			if (opMode != _coolingMode) {
				_coolingMode = opMode;
				updateMode = true;
			}
			if (fabs(coolTemp - _targetTemp->getVal<double>()) > 0.2) {
				printf("%u: Thermostat #%d: targetTemp = %f\n", (uint32_t)millis(), index, coolTemp);
				_targetTemp->setVal(coolTemp);
				lastPacketRecvTime = 0;
			}
			_fan->setModeSpeed(fanMode, fanSpeed);
		}
		else if (index == _furnaceInstance) {
			if (opMode != _furnaceMode) {
				_furnaceMode = opMode;
				updateMode = true;
			}
		}

		if (updateMode) {
			uint8_t mode;

			if (_furnaceMode == ThermostatModeHeat) {
				mode = heatingCoolingStateHeat;
			}
			else {
				uint8_t modeConvert[] { heatingCoolingStateOff, heatingCoolingStateCool, heatingCoolingStateHeat, heatingCoolingStateOff,
										heatingCoolingStateOff, heatingCoolingStateOff, heatingCoolingStateOff, heatingCoolingStateOff };
				
				mode = modeConvert[_coolingMode];
			}

			if (mode != _targetState->getVal()) {
				printf("%u: Thermostat #%d: targetState = %d\n", (uint32_t)millis(), index, mode);
				_targetState->setVal(mode);
				lastPacketRecvTime = 0;
			}
		}
	}
};

//////////////////////////////////////////////

inline float degCfromDegF(float degF) {
	return degF / 1.8;
}

inline float tempCfromTempF(float tempF) {
	return degCfromDegF(tempF - 32.0);
}

struct RVBattery : Service::Thermostat {
	SpanCharacteristic *_ambientTemp;
	SpanCharacteristic *_targetTemp;
	SpanCharacteristic *_targetState;
	SpanCharacteristic *_currentState;

	int16_t _instance;

	RVBattery(int16_t instance) : Service::Thermostat() {
		_instance = instance;
		_ambientTemp = new Characteristic::CurrentTemperature(tempCfromTempF(100));
		_ambientTemp->setRange(tempCfromTempF(100), tempCfromTempF(160), degCfromDegF(1.0));
		_targetTemp = new Characteristic::TargetTemperature(tempCfromTempF(100));
		_targetTemp->setRange(tempCfromTempF(100), tempCfromTempF(160), degCfromDegF(1.0));

		_currentState = new Characteristic::CurrentHeatingCoolingState(heatingCoolingStateOff);
		_targetState = new Characteristic::TargetHeatingCoolingState(heatingCoolingStateOff);

		new Characteristic::TemperatureDisplayUnits(homeKitTemperatureDisplayFahrenheit);
		_targetState->setValidValues(2, heatingCoolingStateOff, heatingCoolingStateCool);

	}

	void loop() {
		if (_targetState->getVal() != heatingCoolingStateOff) {
			_targetState->setVal((int)heatingCoolingStateOff);
		}
	}

	void setVoltage(uint8_t index, float voltage) {
		float voltageAsTemp = tempCfromTempF(voltage * 10.0);
		if (index == _instance && voltageAsTemp != _ambientTemp->getVal<float>()) {
			printf("%u: Set voltage #%d: %0.1fV\n", (uint32_t)millis(), _instance, voltage);
			_ambientTemp->setVal(voltageAsTemp);
			lastPacketRecvTime = 0;
		}
	}
};

constexpr auto maxBatteries = 2;
uint16_t batteryCount = 0;

RVBattery* batteries[maxBatteries];

void setBatteryVoltage(uint8_t index, float voltage) {
	for (auto i=0; i<batteryCount; i++) {
		batteries[i]->setVoltage(index, voltage);
	}
}

//////////////////////////////////////////////

constexpr uint32_t endStopExtraTimeMS = 500;

struct RVAwning : Service::WindowCovering {
	SpanCharacteristic* _currentPosition = NULL;
	SpanCharacteristic* _targetPosition = NULL;

	ShadeState _shadeState = 0;
	float _currentShadePosition = homeKitShadeOpenValue;
	uint64_t _nextPositionUpdateTime = 0;

	uint64_t _operationEndTime;

	uint8_t _retractIndex;
	uint8_t _extendIndex;
	uint32_t _openTimeMS;
	uint32_t _closeTimeMS;

	RVAwning(AwningDeviceRec* device) : Service::WindowCovering() {
		_currentPosition = new Characteristic::CurrentPosition(homeKitShadeOpenValue);
		_targetPosition = new Characteristic::TargetPosition(homeKitShadeOpenValue);
		_retractIndex = device->retractIndex;
		_extendIndex = device->extendIndex;
		_openTimeMS = device->retractTime;
		_closeTimeMS = device->extendTime;
	}

	boolean update() {
		if (_targetPosition->updated()) {
			uint64_t curTime = millis64();

			float targetValue = _targetPosition->getNewVal<float>();
			float moveAmount = targetValue - _currentShadePosition;
			uint32_t moveTimeMS = abs(moveAmount / HomeKitPercentMax) * ((moveAmount > 0) ? _openTimeMS : _closeTimeMS);

			_operationEndTime = curTime + moveTimeMS;

			if (targetValue == homeKitShadeClosedValue || targetValue == homeKitShadeOpenValue) {
				_operationEndTime += endStopExtraTimeMS;
			}

			_shadeState = shadeStateHomeKitAction | ((moveAmount > 0) ? shadeStateOpening : shadeStateClosing);

			printf("%u: Changing position from %0.1f%% to %0.1f%% over %dmS.\n", (uint32_t)millis(), _currentShadePosition, targetValue, (int)(_operationEndTime-curTime));
			printf("%u: State changed to 0x%02X\n", (uint32_t)millis(), _shadeState);
		}
		return true;
	}

	void loop() {
		static uint64_t lastTime = 0;
		uint64_t curTime = millis64();

		if (lastTime == 0) {
			lastTime = curTime;
		}

		if (curTime > lastTime) {
			bool needPositionUpdate = false;
			bool needTargetUpdate = false;

			// if (_userOpenButton->pressed()) {
			// 	_shadeState = shadeStateUserAction | shadeStateOpening;
			// }
			// else if (_userCloseButton->pressed()) {
			// 	_shadeState = shadeStateUserAction | shadeStateClosing;
			// }
			// else if (_shadeState & shadeStateUserAction) {
			// 	_shadeState = 0;
			// 	needTargetUpdate = true;
			// }

			bool moving = (_shadeState & (shadeStateOpening | shadeStateClosing)) != 0;
			bool movingOpen = (_shadeState & shadeStateOpening) != 0;

			if (moving) {
				_currentShadePosition += (curTime - lastTime) * ((movingOpen) ? (HomeKitPercentMax / _openTimeMS) : (HomeKitPercentMax / _closeTimeMS));
				_currentShadePosition = max(homeKitShadeClosedValue, min(homeKitShadeOpenValue, _currentShadePosition));

				if (_shadeState & shadeStateHomeKitAction && curTime > _operationEndTime) {
					_currentShadePosition = _targetPosition->getVal<float>();
					_shadeState = 0;
					printf("%u: Homekit operation complete.\n", (uint32_t)millis());
				}
			}

			// OutputState newOutputState;

			// if (_shadeState & shadeStateHomeKitAction) {
			// 	newOutputState = (_shadeState & shadeStateOpening) ? outputStateOpen : outputStateClose;
			// }
			// else {
			// 	newOutputState = outputStateIdle;
			// }

			// if (newOutputState != _outputState) {
			// 	SerPrintf("Output state changed to %d\n", newOutputState);

			// 	_controlPin->clearDelayedState();
			// 	_directionPin->clearDelayedState();

			// 	bool newDirectionOutputState;
			// 	bool newControlOutputState;

			// 	if (newOutputState == outputStateOpen) {
			// 		newDirectionOutputState = LOW;
			// 		newControlOutputState = HIGH;
			// 	}
			// 	else if (newOutputState == outputStateClose) {
			// 		newDirectionOutputState = HIGH;
			// 		newControlOutputState = HIGH;
			// 	}
			// 	else {
			// 		newDirectionOutputState = LOW;
			// 		newControlOutputState = LOW;
			// 	}

			// 	if (newDirectionOutputState != _directionPin->state() && newControlOutputState != _controlPin->state()) {
			// 		if (newControlOutputState == HIGH) {		// direction  needs time to settle before setting control
			// 			_controlPin->setState(newControlOutputState, relaySettlingTimeMS);
			// 			_directionPin->setState(newDirectionOutputState, 0);
			// 		}
			// 		else {									// direction needs to hold while control settles
			// 			_controlPin->setState(newControlOutputState, 0);
			// 			_directionPin->setState(newDirectionOutputState, relaySettlingTimeMS);
			// 		}
			// 	}
			// 	else {
			// 		_controlPin->setState(newControlOutputState, 0);
			// 		_directionPin->setState(newDirectionOutputState, 0);
			// 	}

			// 	_outputState = newOutputState;
			// 	needPositionUpdate = true;
			// }

			// if (needPositionUpdate || needTargetUpdate || (_nextPositionUpdateTime>0 && curTime > _nextPositionUpdateTime)) {
			// 	if (needTargetUpdate || _shadeState & shadeStateUserAction) {
			// 		float nextPosition = _currentShadePosition;
					
			// 		if (_shadeState & shadeStateOpening) {
			// 			 nextPosition = min(homeKitShadeOpenValue, _currentShadePosition + openPerMS * updateTimeMS);
			// 		}
			// 		else if (_shadeState & shadeStateClosing) {
			// 			 nextPosition = max(homeKitShadeClosedValue, _currentShadePosition - closePerMS * updateTimeMS);
			// 		}

			// 		if (_targetPosition->getVal<float>() != nextPosition) {
			// 			_targetPosition->setVal(nextPosition);
			// 			SerPrintf("Target changed to %0.1f%%\n", nextPosition);
			// 		}
			// 	}
			// 	if (_currentPosition->getVal<float>() != _currentShadePosition) {
			// 		_currentPosition->setVal(_currentShadePosition);
			// 		SerPrintf("Position changed to %0.1f%% - output=%d\n", _currentShadePosition, _outputState);
			// 	}
			// 	_nextPositionUpdateTime = 0;
			// }

			// if (moving && _nextPositionUpdateTime == 0) {
			// 	_nextPositionUpdateTime = curTime + updateTimeMS;
			// }

			// _controlPin->update(curTime);
			// _directionPin->update(curTime);

			// updateIndicator(_shadeState, _currentShadePosition);

			lastTime = curTime;
		}
	}

	void setLevel(uint8_t index, uint8_t dcDimmerLevel) {
		bool on = dcDimmerLevel > 0;
		uint16_t level = dcDimmerLevel * HomeKitPercentMax / RVCBrightMax;

		if (index == _retractIndex) {
			printf("%u: Awning #%d: retract = %d\n", (uint32_t)millis(), index, on);
			lastPacketRecvTime = 0;
		}
		else if (index == _extendIndex) {
			printf("%u: Awning #%d: extend = %d\n", (uint32_t)millis(), index, on);
			lastPacketRecvTime = 0;
		}
	}
};

//////////////////////////////////////////////
#ifndef HAVE_AWNINGS
const AwningDeviceRec awningList[] = {
};
#endif

constexpr uint16_t maxSwitches = sizeof(switchList) / sizeof(SwitchDeviceRec);
constexpr uint16_t maxFans = sizeof(fanList) / sizeof(FanDeviceRec);
constexpr uint16_t maxThermostats = sizeof(thermostatList) / sizeof(ThermostatDeviceRec);
constexpr uint16_t maxAwnings = sizeof(awningList) / sizeof(AwningDeviceRec);

uint16_t switchCount = 0;
uint16_t fanCount = 0;
uint16_t thermostatCount = 0;
uint16_t awningCount = 0;

RVSwitch* switches[maxSwitches];
RVRoofFan* fans[maxFans];
RVThermostat* thermostats[maxThermostats];
RVAwning* awnings[maxAwnings];

void setSwitchLevel(uint8_t index, uint8_t level) {
	for (auto i=0; i<switchCount; i++) {
		switches[i]->setLevel(index, level);
	}
	for (auto i=0; i<fanCount; i++) {
		fans[i]->setLevel(index, level);
	}
	for (auto i=0; i<thermostatCount; i++) {
		thermostats[i]->setLevel(index, level);
	}
	for (auto i=0; i<awningCount; i++) {
		awnings[i]->setLevel(index, level);
	}
}

void setAmbientTemp(uint8_t index, double tempC) {
	for (auto i=0; i<thermostatCount; i++) {
		thermostats[i]->setAmbientTemp(index, tempC);
	}
}

void setThermostatInfo(uint8_t index, ThermostatMode opMode, FanMode fanMode, uint8_t fanSpeed, double heatTemp, double coolTemp) {
	for (auto i=0; i<thermostatCount; i++) {
		thermostats[i]->setInfo(index, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
	}
}

//////////////////////////////////////////////

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

		const char* typeNames[] = { "Lamp", "Dimmable", "Switch" };
		for (SwitchDeviceRec device : switchList) {
			printf("%u: Creating %s #%d: \"%s\"\n", (uint32_t)millis(), typeNames[device.type], device.index, device.name);
			SPAN_ACCESSORY(device.name);
				switches[switchCount++] = new RVSwitch(&device);
		}

		for (FanDeviceRec fan : fanList) {
			printf("%u: Creating Fan #%d: \"%s\"\n", (uint32_t)millis(), fan.index, fan.name);
			SPAN_ACCESSORY(fan.name);
				fans[fanCount++] = new RVRoofFan(&fan);
		}

		for (ThermostatDeviceRec thermostat : thermostatList) {
			printf("%u: Creating Thermostat #%d: \"%s\"\n", (uint32_t)millis(), thermostat.coolingInstance, thermostat.name);
			SPAN_ACCESSORY(thermostat.name);
				thermostats[thermostatCount++] = new RVThermostat(&thermostat);
		}

		for (AwningDeviceRec awning : awningList) {
			printf("%u: Creating Awning \"%s\"\n", (uint32_t)millis(), awning.name);
			SPAN_ACCESSORY(awning.name);
				awnings[awningCount++] = new RVAwning(&awning);
		}

		#ifdef CREATE_BATTERIES
		SPAN_ACCESSORY("House - V*10");
			batteries[batteryCount++] = new RVBattery(1);
		SPAN_ACCESSORY("Chassis - V*10");
			batteries[batteryCount++] = new RVBattery(2);
		#endif
}

//////////////////////////////////////////////

const char* getValuePair(const char* buff, int16_t* val1, int16_t* val2) {
	*val1 = -1;
	*val2 = -2;

	const char* equals = strchr(buff, '=');

	if (equals) {
		equals += 1;
		*val1 = atoi(buff);
		*val2 = atoi(equals);

		buff = strchr(equals, ',');
		if (buff) {
			buff += 1;
		}
	}
	else {
		buff = NULL;
	}

	return buff;
}

const char* getNextValue(const char* buff, int16_t *val) {
	*val = -1;
	if (buff && buff[0]) {
		*val = atoi(buff);
		buff = strchr(buff, ',');
		if (buff) {
			buff += 1;
		}
	}
	else {
		buff = NULL;
	}
	return buff;
}

void cmdSet(const char *buff, uint8_t multiplier, const char* label) {
	int16_t index;
	int16_t val;

	buff += 1;
	
	while (buff) {
		buff = getValuePair(buff, &index, &val);

		if (index!=-1 && val!=-1) {
			printf("%u: %s: index=%d, val=%d\n", (uint32_t)millis(), label, index, val);
			setSwitchLevel(index, val*multiplier);
		}
		else {
			printf("%u: %s: parameter error!\n", (uint32_t)millis(), label);
		}
	}
}

void cmdSetLevel(const char *buff) {
	cmdSet(buff, 1, "cmdSetLevel");
}

void cmdSetState(const char *buff) {
	cmdSet(buff, RVCBrightMax, "cmdSetState");
}

void cmdSetAmbient(const char *buff) {
	int16_t index;
	int16_t tempF;

	buff += 1;
	
	while (buff) {
		buff = getValuePair(buff, &index, &tempF);

		if (index!=-1 && tempF!=-1) {
			double tempC = (tempF - 32.0) / 1.8;
			printf("%u: cmdSetAmbient: index=%d, tempF=%dºF (%0.1fºC)\n", (uint32_t)millis(), index, tempF, tempC);
			setAmbientTemp(index, tempC);
		}
		else {
			printf("%u: cmdSetAmbient: parameter error!\n", (uint32_t)millis());
		}
	}
}

void cmsSetThermostat(const char *buff) {
	int16_t index;
	int16_t opMode;
	int16_t tempF;
	int16_t fanMode;
	int16_t fanSpeed;

	buff += 1;
	
	buff = getValuePair(buff, &index, &opMode);
	buff = getNextValue(buff, &tempF);
	buff = getNextValue(buff, &fanMode);
	buff = getNextValue(buff, &fanSpeed);

	if (fanMode == -1) {
		fanMode = FanModeAuto;
	}
	if (fanSpeed == -1) {
		fanSpeed = 0;
	}

	if (index!=-1 && opMode!=-1 && fanMode!=-1 && fanSpeed!=-1 && tempF!=-1) {
		double tempC = (tempF - 32.0) / 1.8;
		printf("%u: cmsSetThermostat: index=%d, mode=%d, temp=%dºF (%0.1fºC), fanMode=%d, fanSpeed=%d\n", (uint32_t)millis(), index, opMode, tempF, tempC, fanMode, fanSpeed);
		setThermostatInfo(index, (ThermostatMode)opMode, (FanMode)fanMode, fanSpeed, tempC, tempC);
	}
	else {
		printf("%u: cmsSetThermostat: parameter error!\n", (uint32_t)millis());
	}
}

void cmdSendOnOff(const char *buff) {
	int16_t index;
	int16_t val;

	buff += 1;
	
	while (buff) {
		buff = getValuePair(buff, &index, &val);

		if (index!=-1 && val!=-1) {
			printf("%u: cmdSendOnOff: index=%d, val=%d\n", (uint32_t)millis(), index, val);
			sendOnOff(index, val);
		}
		else {
			printf("%u: cmdSendOnOff: parameter error!\n", (uint32_t)millis());
		}
	}
}

void cmdSetCANWrite(const char *buff) {
	buff++;
	while (buff[0]==' ') buff++;

	switch (buff[0]) {
		case '0':
			doCANWrite = false;
			printf("%u: CAN-Bus packet writes DISABLED.\n", (uint32_t)millis());
			break;
		case '1':
			doCANWrite = true;
			printf("%u: CAN-Bus packet writes ENABLED.\n", (uint32_t)millis());
			break;
		default:
			printf("%u: cmdSetCANWrite: parameter error!\n", (uint32_t)millis());
	}
}

void cmdSetPacketLog(const char *buff) {
	buff++;
	while (buff[0]==' ') buff++;

	switch (buff[0]) {
		case '0':
			packetPrintMode = packetPrintNo;
			printf("%u: Packet logging: Off.\n", (uint32_t)millis());
			break;
		case '1':
			packetPrintMode = packetPrintYes;
			printf("%u: Packet logging: On.\n", (uint32_t)millis());
			break;
		case '2':
			packetPrintMode = packetPrintIfUnknown;
			printf("%u: Packet logging: If Unknown.\n", (uint32_t)millis());
			break;
		case '3':
			packetPrintMode = packetPrintIfKnown;
			printf("%u: Packet logging: If Known.\n", (uint32_t)millis());
			break;
		default:
			printf("%u: cmdSetPacketLog: parameter error!\n", (uint32_t)millis());
	}
}

void addCommands() {
	new SpanUserCommand('l',"<index>=<level:0-250>,... - set level of <index>", cmdSetLevel);
	new SpanUserCommand('s',"<index>=<state:0-1>,... - set state of <index>", cmdSetState);
	new SpanUserCommand('o',"<index>=<state:0-1>,... - send onOff to <index>", cmdSendOnOff);
	new SpanUserCommand('a',"<index>=<tempºF>,... - set ambient temp of <index>", cmdSetAmbient);
	new SpanUserCommand('w',"<0-1>,... - set CAN-Bus write enable", cmdSetCANWrite);
	new SpanUserCommand('p',"<0-2>,... - set packet logging: 0=No, 1=Yes, 2=If unknown, 3=If known", cmdSetPacketLog);
	new SpanUserCommand('t',"<index>=<mode:0-2>,<tempºF>,optional(<fanmode:0-1>,<fanspeed:0-250>) - set info for thermostat <index>", cmsSetThermostat);
}

//////////////////////////////////////////////
typedef struct {
	uint8_t pinNumber;
	uint8_t mode;
	uint8_t state;
} PinSetup;

PinSetup pinList[] = {
	{ indicatorPinR, OUTPUT, HIGH },
	{ indicatorPinG, OUTPUT, HIGH },
	{ indicatorPinB, OUTPUT, HIGH },
};

void flashPin(uint8_t pin, uint16_t count, uint16_t period) {
	for (auto i=0; i<count; i++) {
		digitalWrite(pin, LOW);
		delay(period/2);
		digitalWrite(pin, HIGH);
		delay(period/2);
	}
}

void initPins() {
	for (PinSetup pin : pinList) {
		pinMode(pin.pinNumber, pin.mode);
		digitalWrite(pin.pinNumber, pin.state);
	}
}

//////////////////////////////////////////////

bool wifiConnected = false;
bool hadWifiConnection = false;

void wifiStatusChanged(HS_STATUS status) {
	if (status == HS_WIFI_CONNECTING) {
		wifiConnected = false;
		if (hadWifiConnection) {
			printf("%u: WIFI: Lost Connection..\n", (uint32_t)millis());
		}
	}
}

void wifiReady() {
	wifiConnected = true;
	hadWifiConnection = true;
	printf("%u: WIFI: Ready..\n", (uint32_t)millis());
}

//////////////////////////////////////////////

void setup() {
	initPins();
	flashPin(indicatorPinB, 20, 200);

	Serial.begin(115200);
	printf("%u: RV Bridge - Startup\n", (uint32_t)millis());

	#ifdef OVERRIDE_MAC_ADDRESS
 		uint8_t newMACAddress[] = OVERRIDE_MAC_ADDRESS;
 		esp_base_mac_addr_set(&newMACAddress[0]);
 		printf("%u: MAC address updated to: %s\n", (uint32_t)millis(), WiFi.macAddress().c_str());
 	#endif

	printf("%u: Init CAN module\n", (uint32_t)millis());
	CAN_cfg.speed = CAN_SPEED_250KBPS;
	CAN_cfg.tx_pin_id = canTxPin;
	CAN_cfg.rx_pin_id = canRxPin;
	CAN_cfg.rx_queue = xQueueCreate(receiveQueueSize, sizeof(CAN_frame_t));
	ESP32Can.CANInit();

	printf("%u: Init HomeSpan\n", (uint32_t)millis());
	#ifndef SKIP_WIFI_CREDENTIALS
		homeSpan.setWifiCredentials(ssid, sspwd);
	#endif
	homeSpan.setSketchVersion(versionString);
	homeSpan.setWifiCallback(wifiReady);
	homeSpan.setStatusCallback(wifiStatusChanged);
	homeSpan.begin(Category::Bridges, "RV-Bridge", DEFAULT_HOST_NAME, "RV-Bridge-ESP32");

	createDevices();
	addCommands();

	// Reduce processor frequency to lower current consumption
	setCpuFrequencyMhz(160);

	printf("%u: Init complete.\n", (uint32_t)millis());
}

//////////////////////////////////////////////

void processPacket(CAN_frame_t *packet) {
	if (packet->FIR.B.RTR != CAN_RTR) {
		uint32_t dgn = getMsgBits(packet->MsgID, 24, 17);
		// uint8_t sourceAddr = getMsgBits(packet->MsgID, 7, 8);
		// uint8_t priority = getMsgBits(packet->MsgID, 28, 3);

		uint8_t* d = packet->data.u8;

		if (dgn == DC_DIMMER_STATUS_3) {
			uint8_t instance = d[0];
			// uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			// uint8_t enable = (d[3] >> 6) & 3;
			// uint8_t delayDuration = d[4];
			// DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			// uint8_t status = (d[6] >> 2) & 3;

			// if (instance == 4) {
			// 	displayPacket(packet, packetPrintYes);
			// }
			setSwitchLevel(instance, brightness);
		}
		else if (dgn == THERMOSTAT_AMBIENT_STATUS) {
			uint8_t instance = d[0];
			double tempC = convToTempC(d[2]<<8 | d[1]);
			
			setAmbientTemp(instance, tempC);
		}
		else if (dgn == THERMOSTAT_STATUS_1) {
			uint8_t instance = d[0];
			ThermostatMode opMode = (ThermostatMode)(d[1] & 0x0F);
			FanMode fanMode = (FanMode)((d[1]>>4) & 0x03);
			// uint8_t scheduleEnabled = ((d[1]>>6) & 0x03);
			uint8_t fanSpeed = d[2];
			double heatTemp = convToTempC(d[4]<<8 | d[3]);
			double coolTemp = convToTempC(d[6]<<8 | d[5]);
			
			setThermostatInfo(instance, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
		}
		else if (dgn == BATTERY_STATUS) {
			uint8_t instance = d[0];
			uint16_t v_int = d[3]<<8 | d[2];
			float voltage = v_int * 0.050;
			setBatteryVoltage(instance, voltage);		
		}
	}
}

void displayPacket(CAN_frame_t *packet, uint8_t printPacket) {
	bool knownPacket = true;

	if (packet->FIR.B.RTR == CAN_RTR) {
		printf("%u: RTR from 0x%08X, DLC %d\r\n", (uint32_t)millis(), packet->MsgID,  packet->FIR.B.DLC);
	}
	else {
		uint32_t dgn = getMsgBits(packet->MsgID, 24, 17);
		uint8_t sourceAddr = getMsgBits(packet->MsgID, 7, 8);
		uint8_t priority = getMsgBits(packet->MsgID, 28, 3);

		uint8_t* d = packet->data.u8;

		if (dgn == DC_DIMMER_STATUS_3) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t enable = (d[3] >> 6) & 3;
			uint8_t delayDuration = d[4];
			DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			uint8_t status = (d[6] >> 2) & 3;

			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: DC_DIMMER_STATUS_3: inst=%d, grp=0X%02X, bright=%d, enable=%d, dur=%d, last cmd=%d, status=0X%02X\n", (uint32_t)millis(), instance, group, brightness, enable, delayDuration, lastCmd, status);				
			}
		}
		else if (dgn == DC_DIMMER_COMMAND_2) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = d[2];
			DCDimmerCmd cmd = (DCDimmerCmd)d[3];
			uint8_t delayDuration = d[4];

			// if (instance!=32 && instance!=43 && instance!=44 && instance!=53 && instance!=54 && instance!=55 && instance!=56) {
			// 	printPacket = packetPrintYes;
			// }

			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: DC_DIMMER_COMMAND_2: inst=%d, grp=0X%02X, bright=%d, cmd=0X%02X, dur=%d\n", (uint32_t)millis(), instance, group, brightness, cmd, delayDuration);	
				printPacket = packetPrintYes;			
			}
		}
		else if (dgn == THERMOSTAT_AMBIENT_STATUS) {
			uint8_t instance = d[0];
			double tempC = convToTempC(d[2]<<8 | d[1]);
			
			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: THERMOSTAT_AMBIENT_STATUS: #%d, temp=%0.1fºC\n", (uint32_t)millis(), instance, tempC);
			}
		}
		else if (dgn == THERMOSTAT_STATUS_1 || dgn == THERMOSTAT_COMMAND_1) {
			uint8_t instance = d[0];
			ThermostatMode opMode = (ThermostatMode)(d[1] & 0x0F);
			FanMode fanMode = (FanMode)((d[1]>>4) & 0x03);
			// uint8_t scheduleEnabled = ((d[1]>>6) & 0x03);
			uint8_t fanSpeed = d[2];
			double heatTemp = convToTempC(d[4]<<8 | d[3]);
			double coolTemp = convToTempC(d[6]<<8 | d[5]);

			const char* name = (dgn == THERMOSTAT_COMMAND_1) ? "THERMOSTAT_COMMAND_1" : "THERMOSTAT_STATUS_1";
			
			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: %s: inst=%d, opMode=%d, fanMode=%d, fanSpeed=%d, heatTemp=%0.1fºC, coolTemp=%0.1fºC\n", (uint32_t)millis(), name, instance, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
			}
		}
		else if (dgn == FURNACE_STATUS || dgn == FURNACE_COMMAND) {
			uint8_t instance = d[0];
			ACMode opMode = (ACMode)(d[1] & 0x03);
			uint8_t fanSpeed = d[2];
			uint8_t heatOutput = d[3];

			const char* name = (dgn == FURNACE_COMMAND) ? "FURNACE_COMMAND" : "FURNACE_STATUS";

			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: %s: inst=%d, opMode=%d, fanSpeed=%d, heatOutput=%d\n", (uint32_t)millis(), name, instance, opMode, fanSpeed, heatOutput);
			}
		}
		else if (dgn == DC_LOAD_COMMAND) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t direction = (d[3] >> 4) & 0xF;
			uint8_t command = d[4];

			printf("%u: DC_LOAD_COMMAND: inst=%d, grp=0X%02X, bright=%d, dir=%d, cmd=%d\n", (uint32_t)millis(), instance, group, brightness, direction, command);				
		}
		else if (dgn == AWNING_COMMAND) {
			uint8_t instance = d[0];
			uint8_t direction = d[2];
			uint8_t position = d[3];
			uint8_t retract = d[5];

			printf("%u: AWNING_COMMAND: inst=%d, dir=%d, pos=%d, retract=%d\n", (uint32_t)millis(), instance, direction, position, retract);			
		}
		else if (dgn == AIR_CONDITIONER_STATUS) { }
		else if (dgn == AWNING_STATUS) { }
		else if (dgn == AWNING_STATUS_2) { }
		else if (dgn == DATE_TIME_STATUS) { }
		else if (dgn == GENERIC_INDICATOR_COMMAND) { }
		else if (dgn == GENERIC_CONFIGURATION_STATUS) { }
		else if (dgn == DC_SOURCE_STATUS_1) {
			uint8_t instance = d[0];
			uint8_t priority = d[1];
			uint16_t v_int = d[3]<<8 | d[2];
			uint32_t i_int = d[7]<<24 | d[6]<<16 | d[5]<<8 | d[4];
			float voltage = v_int * 0.050;
			float current = -2000000.0 + i_int * 0.001;

			printf("%u: DC_SOURCE_STATUS_1: inst=%d, pri=%d, voltage=%0.1f, current=%0.1f\n", (uint32_t)millis(), instance, priority, voltage, current);		
		}
		else if (dgn == BATTERY_STATUS) { }
		else if (dgn == TANK_STATUS) { }
		else if (dgn == GENERATOR_STATUS_1) { }
		else if (dgn == 0x0FECA) { /* DM_1 - 1FECA - DM_RV */ }
		else if (dgn == 0x0E8FF) { }
		else if (dgn == 0x0EAFF) { }
		else if (dgn == 0x15FCE) { }
		else if (dgn == 0x1BBFD) { }
		else if (dgn == 0x1FACE) { }
		else if (dgn == 0x1FACF) { }
		else if (dgn == 0x1FBDA) { }
		else {
			knownPacket = false;
		}

		if (printPacket==packetPrintYes || (!knownPacket && printPacket==packetPrintIfUnknown) || (knownPacket && printPacket==packetPrintIfKnown)) {
			if (packet->FIR.B.FF == CAN_frame_std) {
				printf("%u: Std - ", (uint32_t)millis());
			}
			else {
				printf("%u: Ext - ", (uint32_t)millis());
			}

			printf("dgn=%05X, src=%02X, pri=%d, Data: ", dgn, sourceAddr, priority);

			for (auto i = 0; i < packet->FIR.B.DLC; i++) {
				printf("%02X ", packet->data.u8[i]);
			}
			printf("\n");
		}
	}
}

void debugPacket(CAN_frame_t *packet) {
	if (packet->FIR.B.RTR != CAN_RTR) {
		uint32_t dgn = getMsgBits(packet->MsgID, 24, 17);
		uint8_t sourceAddr = getMsgBits(packet->MsgID, 7, 8);
		uint8_t priority = getMsgBits(packet->MsgID, 28, 3);

		uint8_t* d = packet->data.u8;

		if (dgn == DC_LOAD_COMMAND) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t direction = (d[3] >> 4) & 0xF;
			uint8_t command = d[4];

			printf("%u: DC_LOAD_COMMAND: inst=%d, grp=0X%02X, bright=%d, dir=%d, cmd=%d\n", (uint32_t)millis(), instance, group, brightness, direction, command);				
		}
		else if (dgn == AWNING_COMMAND) {
			uint8_t instance = d[0];
			uint8_t direction = d[2];
			uint8_t position = d[3];
			uint8_t retract = d[5];

			printf("%u: AWNING_COMMAND: inst=%d, dir=%d, pos=%d, retract=%d\n", (uint32_t)millis(), instance, direction, position, retract);			
		}
		else if (dgn == AWNING_STATUS) {
			printf("%u: AWNING_STATUS\n", (uint32_t)millis());
		}
		else if (dgn == AWNING_STATUS_2) {
			printf("%u: AWNING_STATUS_2\n", (uint32_t)millis());			
		}
		else if (dgn == DC_DIMMER_STATUS_3) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t enable = (d[3] >> 6) & 3;
			uint8_t delayDuration = d[4];
			DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			uint8_t status = (d[6] >> 2) & 3;

			if (instance == 5 || instance == 6) {
				printf("%u: DC_DIMMER_STATUS - AWNING: inst=%d, grp=0X%02X, bright=%d, enable=%d, dur=%d, last cmd=%d, status=0X%02X\n", (uint32_t)millis(), instance, group, brightness, enable, delayDuration, lastCmd, status);				
			}
		}
		else if (dgn == DC_DIMMER_COMMAND_2) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = d[2];
			DCDimmerCmd cmd = (DCDimmerCmd)d[3];
			uint8_t delayDuration = d[4];

			if (instance == 5 || instance == 6) {
				printf("%u: DC_DIMMER_COMMAND - AWNING: inst=%d, grp=0X%02X, bright=%d, cmd=0X%02X, dur=%d\n", (uint32_t)millis(), instance, group, brightness, cmd, delayDuration);	
			}
		}
	}
}

//////////////////////////////////////////////

void loop() {
	static elapsedMillis heartbeatTime;

	bool sendIndicator = lastPacketSendTime < packetBlinkTime;
	bool recvIndicator = lastPacketRecvTime < packetBlinkTime;
    
	if (sendIndicator || recvIndicator) {
		heartbeatTime = heartbeatBlinkTime;		// adjust next heartbeat time to avoid overlap with send/recv indicators
	}

	bool hearbeatIndicator = (heartbeatTime % heatbeatRate) < heartbeatBlinkTime;

	if (!wifiConnected) {
		hearbeatIndicator = (heartbeatTime % (heatbeatRate/4)) > heartbeatBlinkTime*5;
	}

	digitalWrite(indicatorPinG, !hearbeatIndicator);
	digitalWrite(indicatorPinR, !sendIndicator);
	digitalWrite(indicatorPinB, !recvIndicator);

	homeSpan.poll();

	// HomeSpan does not call the wifi callback after the first connection
	// We do that manually here
	if (hadWifiConnection && !wifiConnected) {
		static uint64_t nextWifiCheck = 0;
		uint64_t time = millis64();

		if (time >= nextWifiCheck) {
			if (WiFi.status()==WL_CONNECTED) {
				wifiReady();
			}

			nextWifiCheck = time + 500;
		}
	}

	CAN_frame_t packet;

	if (xQueueReceive(CAN_cfg.rx_queue, &packet, 0) == pdTRUE) {
		if (packetPrintMode != packetPrintNo) {
			displayPacket(&packet, packetPrintMode);
		}
		debugPacket(&packet);
		processPacket(&packet);
	}

	processPacketQueue();
}
