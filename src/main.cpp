#include "Arduino.h"

#include "elapsedMillis.h"
#include "HomeSpan.h" 
#include "ESP32CAN.h"
#include "CAN_config.h"
#include <esp_task_wdt.h>

#include "config.h"

//////////////////////////////////////////////

constexpr const char* versionString = "v0.1";

constexpr uint32_t watchDogTimerSeconds = 3;

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
constexpr uint32_t heatbeatRate = 2000;
constexpr uint32_t heartbeatBlinkTime = 20;

elapsedMillis lastPacketSendTime = 1000;
elapsedMillis lastPacketRecvTime = 1000;

CAN_device_t CAN_cfg;               	// CAN Config

//////////////////////////////////////////////

enum {
	packetPrintNo = 0,
	packetPrintYes,
	packetPrintIfUnknown
};

void processPacket(CAN_frame_t *packet, uint8_t printPacket);
uint8_t packetPrintMode = packetPrintNo;

//////////////////////////////////////////////
// RV-C dgn numbers

constexpr uint32_t DATE_TIME_STATUS = 0x1FFFF;
constexpr uint32_t DC_DIMMER_COMMAND_2 = 0x1FEDB;
constexpr uint32_t DC_DIMMER_STATUS_3 = 0x1FEDA;
constexpr uint32_t THERMOSTAT_AMBIENT_STATUS = 0x1FF9C;
constexpr uint32_t THERMOSTAT_STATUS_1 = 0x1FFE2;
constexpr uint32_t THERMOSTAT_COMMAND_1 = 0x1FFE9;

constexpr uint32_t AIR_CONDITIONER_STATUS = 0x1FFE0; // 1FFE1
constexpr uint32_t GENERIC_INDICATOR_COMMAND = 0x1FED9;
constexpr uint32_t GENERIC_CONFIGURATION_STATUS = 0x1FED8;
constexpr uint32_t DC_SOURCE_STATUS_1 = 0x1FFFD;
constexpr uint32_t TANK_STATUS = 0x1FFB7;
constexpr uint32_t GENERATOR_STATUS_1 = 0x1FFDC;

constexpr uint32_t FURNACE_STATUS = 0x1FFE4;
constexpr uint32_t FURNACE_COMMAND = 0x1FFE3;

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
	DCDimmerCmdFlashMomentarily
} DCDimmerCmd;

typedef enum {
	ACModeAuto = 0,
	ACModeManual
} ACMode;

typedef enum {
	ThermostatModeOff = 0,
	ThermostatModeCool,
	ThermostatModeHeat,
	ThermostatModeFanOnly
} ThermostatMode;

typedef enum {
	FanModeAuto = 0,
	FanModeOn
} FanMode;

constexpr uint8_t RVCPercentMax = 250;
constexpr uint8_t RVCBrightMax = 200;
constexpr uint8_t HomeKitPercentMax = 100;

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

constexpr double tempCOffset = -273.0;
constexpr double tempCScale = 0.03125;
constexpr double tempCScaleInv = 1.0 / tempCScale;

double convToTempC(uint16_t value) {
	return tempCOffset + value * tempCScale;
}

uint16_t convFromTempC(double tempC) {
	return (tempC - tempCOffset) * tempCScaleInv;
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
				printf("%d: CAN-Bus Send Packet\n", millis());
				ESP32Can.CANWriteFrame(&packetQueue[nextIndex]);
			}
			else {
				printf("%d: ***SIMULATE*** CAN-Bus Send Packet\n", millis());
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

	// printf("Queueing Dimmer packet: #%d, bright=%d, cmd=%d, dur=%d\n", index, brightness, cmd, duration);
	// processPacket(&packet, packetPrintYes);
	// printf("** Packet queued.\n");
}

void sendOnOff(uint8_t index, bool on, uint8_t brightness=RVCBrightMax) {
	printf("sendOnOff: #%d to %d\n", index, on);
	sendDCDimmerCmd(index, brightness, (on) ? DCDimmerCmdOnDuration : DCDimmerCmdOff);
	// sendDCDimmerCmd(index, brightness, DCDimmerCmdToggle);
}

void sendLampLevel(uint8_t index, uint8_t brightness) {
	if (brightness == 0) {
		sendOnOff(index, false);
	}
	else {
		printf("sendLampLevel: #%d to %d\n", index, brightness);
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

	// printf("Queueing Thermostat packet: #%d, mode=%d, fanMode=%d, temp=%fºC\n", index, mode, fanMode, tempC);
	// processPacket(&packet, packetPrintIfUnknown);
	// printf("** Packet queued.\n");
}

//////////////////////////////////////////////

const char* switchTypes[3] = { "43", "43", "49" };
const char* switchHapNames[3] = { "LightBulb", "LightBulb", "Switch" };

struct RVSwitch : SpanService {
	SpanCharacteristic *_on;
	SpanCharacteristic *_brightness = NULL;
	uint8_t _index;
	SwitchType _type;

	RVSwitch(SwitchDeviceRec *device) : SpanService( switchTypes[device->type], switchHapNames[device->type] ) {
		_index = device->index;
		_type = device->type;

		REQ(On);
		OPT(Name);

		if (_type == Lamp || _type == DimmableLamp) {
			OPT(Brightness);
			OPT(Hue);
			OPT(Saturation);
			OPT(ColorTemperature);
		}

		_on = new Characteristic::On();

		if (_type == DimmableLamp) {
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
			printf("Switch #%d: on = %d\n", _index, on);
			_on->setVal(on);
			lastPacketRecvTime = 0;
		}
		if (_brightness && index == _index && on && level != _brightness->getVal()) {
			printf("Switch #%d: level = %d\n", _index, level);
			_brightness->setVal(level);
			lastPacketRecvTime = 0;
		}
	}
};

struct RVRoofFan : Service::Fan {
	SpanCharacteristic *_active;
	uint8_t _index;
	uint8_t _upIndex;
	uint8_t _downIndex;

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
			}
		}

		return(true);  
	}
	
	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;

		if (index == _index) {
			_fanPower = on;
			lastPacketRecvTime = 0;
		}
		else if (index == _upIndex && on) {
			_lidUp = true;
			lastPacketRecvTime = 0;
		}
		else if (index == _downIndex && on) {
			_lidUp = false;
			lastPacketRecvTime = 0;
		}

		bool newState = _fanPower && (_upIndex==-1 || _lidUp);

		if (newState != _active->getVal()) {
			printf("Fan #%d: active = %d\n", index, newState);
			_active->setVal(newState);
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

	uint8_t _index;
	uint8_t _fanHIndex;
	uint8_t _fanLIndex;
	bool _fanHRunning = false;
	bool _fanLRunning = false;

	RVHVACFan(ThermostatDeviceRec *device, std::function<bool()> updateFunction) : Service::Fan() {
		_active = new Characteristic::Active(false);
		_speed = new Characteristic::RotationSpeed(0);
		_speed->setRange(0, HomeKitPercentMax, HomeKitPercentMax/2);
		_currentState = new Characteristic::CurrentFanState(currentFanStateIdle);
		_targetState = new Characteristic::TargetFanState(targetFanStateAuto);
	
		_index = device->index;
		_fanHIndex = device->fanHIndex;
		_fanLIndex = device->fanLIndex;
		_updateFunction = updateFunction;
	}
	
	boolean update() {
		bool changed = false;

		if (_active->updated()) {
			printf("RVHVACFan #%d - Active: %d\n", _index, _active->getNewVal());
			changed = true;
		}
		if (_speed->updated()) {
			printf("RVHVACFan #%d - Speed: %d\n", _index, _speed->getNewVal());
			changed = true;
		}
		if (_targetState->updated()) {
			printf("RVHVACFan #%d - Target State: %d\n", _index, _targetState->getNewVal());
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
			printf("RVHVACFan #%d - setActive: %d\n", _index, newActive);
			_active->setVal(newActive);
		}
		if (speed != _speed->getVal()) {
			printf("RVHVACFan #%d - setSpeed: %d\n", _index, speed);
			_speed->setVal(speed);
		}
	}

	void getModeSpeed(FanMode* fanMode, uint8_t *speed) {
		if (_active->getNewVal()) {
			*fanMode = FanModeOn;
			*speed = _speed->getNewVal() * RVCPercentMax / HomeKitPercentMax;
		}
		else {
			*fanMode = FanModeAuto;
			*speed = 0;
		}
	}

	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;

		if (index == _fanHIndex && on != _fanHRunning) {
			_fanHRunning = on;
			lastPacketRecvTime = 0;
		}
		else if (index == _fanLIndex && on != _fanLRunning) {
			_fanLRunning = on;
			lastPacketRecvTime = 0;
		}

		uint8_t newState = (_fanHRunning || _fanLRunning) ? currentFanStateBlowing : currentFanStateIdle;

		if (newState != _currentState->getVal()) {
			printf("HVACFan #%d: currentState = %d\n", _index, newState);
			_currentState->setVal(newState);
		}
	}
};

struct RVThermostat : Service::Thermostat {
	SpanCharacteristic *_ambientTemp;
	SpanCharacteristic *_targetTemp;
	SpanCharacteristic *_targetState;
	SpanCharacteristic *_currentState;

	RVHVACFan *_fan;

	uint8_t _index;
	uint8_t _compressorIndex;
	uint8_t _furnaceIndex;

	bool _compressorRunning = false;
	bool _furnaceRunning = false;

	RVThermostat(ThermostatDeviceRec *device) : Service::Thermostat() {
		_ambientTemp = new Characteristic::CurrentTemperature(20);
		_targetTemp = new Characteristic::TargetTemperature();
		_targetTemp->setRange(10, 32, 0.5)->setVal(20);
		_currentState = new Characteristic::CurrentHeatingCoolingState(heatingCoolingStateOff);
		_targetState = new Characteristic::TargetHeatingCoolingState(heatingCoolingStateOff);
		new Characteristic::TemperatureDisplayUnits(1);

		if (device->furnaceIndex != -1) {
			_targetState->setValidValues(3, heatingCoolingStateOff, heatingCoolingStateHeat, heatingCoolingStateCool);
		}
		else {
			_targetState->setValidValues(2, heatingCoolingStateOff, heatingCoolingStateCool);
		}

		_index = device->index;
		_compressorIndex = device->compressorIndex;
		_furnaceIndex = device->furnaceIndex;

		_fan = new RVHVACFan(device, [this]()->bool { updateThermostat(); return true; });
	}

	void updateThermostat() {
		printf("Thermostat #%d: Send thermostat info\n", _index);
		ThermostatMode modeLookup[] { ThermostatModeOff, ThermostatModeHeat, ThermostatModeCool };
		ThermostatMode opMode = modeLookup[_targetState->getNewVal()];
		FanMode fanMode;
		uint8_t speed;

		_fan->getModeSpeed(&fanMode, &speed);

		sendThermostatCommand(_index, opMode, fanMode, speed, _targetTemp->getNewVal<double>());
	}

	boolean update() {
		bool sendInfo = false;

		if (_targetState->updated()) {
			printf("RVThermostat #%d, Target State: %d\n", _index, _targetState->getNewVal());
			sendInfo = true;
		}
		if (_targetTemp->updated()) {
			printf("RVThermostat #%d - Target Temp: %f\n", _index, _targetTemp->getNewVal<double>());
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
			printf("Thermostat #%d: compressor = %d\n", index, on);
			_compressorRunning = on;
			changed = true;
		}
		if (index == _furnaceIndex && on != _furnaceRunning) {
			printf("Thermostat #%d: furnace = %d\n", index, on);
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
		if (index == _index && fabs(tempC - _ambientTemp->getVal<double>()) > 0.2) {
			printf("Set ambient temp #%d: %fºC\n", _index, tempC);
			_ambientTemp->setVal(tempC);
			lastPacketRecvTime = 0;
		}
	}

	void setInfo(uint8_t index, ThermostatMode opMode, FanMode fanMode, uint8_t fanSpeed, double heatTemp, double coolTemp) {
		if (index == _index) {
			uint8_t modeConvert[] { heatingCoolingStateOff, heatingCoolingStateCool, heatingCoolingStateHeat, heatingCoolingStateOff };
			uint8_t mode = modeConvert[opMode];
			uint8_t newFan = fanMode == FanModeOn;

			if (mode != _targetState->getVal()) {
				printf("Thermostat #%d: targetState = %d\n", index, mode);
				_targetState->setVal(mode);
				lastPacketRecvTime = 0;
			}
			if (fabs(coolTemp - _targetTemp->getVal<double>()) > 0.2) {
				printf("Thermostat #%d: targetTemp = %f\n", index, coolTemp);
				_targetTemp->setVal(coolTemp);
				lastPacketRecvTime = 0;
			}
			_fan->setModeSpeed(fanMode, fanSpeed);
		}
	}
};

//////////////////////////////////////////////

constexpr uint16_t maxSwitches = sizeof(switchList) / sizeof(SwitchDeviceRec);
constexpr uint16_t maxFans = sizeof(fanList) / sizeof(FanDeviceRec);
constexpr uint16_t maxThermostats = sizeof(thermostatList) / sizeof(ThermostatDeviceRec);

uint16_t switchCount = 0;
uint16_t fanCount = 0;
uint16_t thermostatCount = 0;

RVSwitch* switches[maxSwitches];
RVRoofFan* fans[maxFans];
RVThermostat* thermostats[maxThermostats];

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
			printf("Creating %s #%d: \"%s\"\n", typeNames[device.type], device.index, device.name);
			SPAN_ACCESSORY(device.name);
				switches[switchCount++] = new RVSwitch(&device);
		}

		for (FanDeviceRec fan : fanList) {
			printf("Creating Fan #%d: \"%s\"\n", fan.index, fan.name);
			SPAN_ACCESSORY(fan.name);
				fans[fanCount++] = new RVRoofFan(&fan);
		}

		for (ThermostatDeviceRec thermostat : thermostatList) {
			printf("Creating Thermostat #%d: \"%s\"\n", thermostat.index, thermostat.name);
			SPAN_ACCESSORY(thermostat.name);
				thermostats[thermostatCount++] = new RVThermostat(&thermostat);
		}
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
			printf("%s: index=%d, val=%d\n", label, index, val);
			setSwitchLevel(index, val*multiplier);
		}
		else {
			printf("%s: parameter error!\n", label);
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
			printf("cmdSetAmbient: index=%d, tempF=%dºF (%fºC)\n", index, tempF, tempC);
			setAmbientTemp(index, tempC);
		}
		else {
			printf("cmdSetAmbient: parameter error!\n");
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
		printf("cmsSetThermostat: index=%d, mode=%d, temp=%dºF (%fºC), fanMode=%d, fanSpeed=%d\n", index, opMode, tempF, tempC, fanMode, fanSpeed);
		setThermostatInfo(index, (ThermostatMode)opMode, (FanMode)fanMode, fanSpeed, tempC, tempC);
	}
	else {
		printf("cmsSetThermostat: parameter error!\n");
	}
}

void cmdSendOnOff(const char *buff) {
	int16_t index;
	int16_t val;

	buff += 1;
	
	while (buff) {
		buff = getValuePair(buff, &index, &val);

		if (index!=-1 && val!=-1) {
			printf("cmdSendOnOff: index=%d, val=%d\n", index, val);
			sendOnOff(index, val);
		}
		else {
			printf("cmdSendOnOff: parameter error!\n");
		}
	}
}

void cmdSetCANWrite(const char *buff) {
	buff++;
	while (buff[0]==' ') buff++;

	switch (buff[0]) {
		case '0':
			doCANWrite = false;
			printf("CAN-Bus packet writes DISABLED.\n");
			break;
		case '1':
			doCANWrite = true;
			printf("CAN-Bus packet writes ENABLED.\n");
			break;
		default:
			printf("cmdSetCANWrite: parameter error!\n");
	}
}

void cmdSetPacketLog(const char *buff) {
	buff++;
	while (buff[0]==' ') buff++;

	switch (buff[0]) {
		case '0':
			packetPrintMode = packetPrintNo;
			printf("Packet logging: Off.\n");
			break;
		case '1':
			packetPrintMode = packetPrintYes;
			printf("Packet logging: On.\n");
			break;
		case '2':
			packetPrintMode = packetPrintIfUnknown;
			printf("Packet logging: If Unknown.\n");
			break;
		default:
			printf("cmdSetPacketLog: parameter error!\n");
	}
}

void addCommands() {
	new SpanUserCommand('l',"<index>=<level:0-250>,... - set level of <index>", cmdSetLevel);
	new SpanUserCommand('s',"<index>=<state:0-1>,... - set state of <index>", cmdSetState);
	new SpanUserCommand('o',"<index>=<state:0-1>,... - send onOff to <index>", cmdSendOnOff);
	new SpanUserCommand('a',"<index>=<tempºF>,... - set ambient temp of <index>", cmdSetAmbient);
	new SpanUserCommand('w',"<0-1>,... - set CAN-Bus write enable", cmdSetCANWrite);
	new SpanUserCommand('p',"<0-2>,... - set packet logging: 0=No, 1=Yes, 2=If unknown", cmdSetPacketLog);
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

void setup() {
	initPins();
	flashPin(indicatorPinR, 20, 200);

	Serial.begin(115200);
	printf("RV Bridge - Startup\n");

	#ifdef OVERRIDE_MAC_ADDRESS
 		uint8_t newMACAddress[] = OVERRIDE_MAC_ADDRESS;
 		esp_base_mac_addr_set(&newMACAddress[0]);
 		printf("MAC address updated to: %s\n", WiFi.macAddress().c_str());
 	#endif

	printf("Init CAN module\n");
	CAN_cfg.speed = CAN_SPEED_250KBPS;
	CAN_cfg.tx_pin_id = canTxPin;
	CAN_cfg.rx_pin_id = canRxPin;
	CAN_cfg.rx_queue = xQueueCreate(receiveQueueSize, sizeof(CAN_frame_t));
	ESP32Can.CANInit();

	printf("Init HomeSpan\n");
	homeSpan.setWifiCredentials(ssid, sspwd);
	homeSpan.setSketchVersion(versionString);
	homeSpan.begin(Category::Bridges, "RV-Bridge", DEFAULT_HOST_NAME, "RV-Bridge-ESP32");

	createDevices();
	addCommands();

	// printf("Setup Watchdog Timer\n");
	// esp_task_wdt_init(watchDogTimerSeconds, true); //enable panic so ESP32 restarts
	// esp_task_wdt_add(NULL); //add current thread to WDT watch

	printf("Init complete.\n");
}

//////////////////////////////////////////////

void processPacket(CAN_frame_t *packet, uint8_t printPacket) {
	bool knownPacket = true;

	if (packet->FIR.B.RTR == CAN_RTR) {
		printf("RTR from 0x%08X, DLC %d\r\n", packet->MsgID,  packet->FIR.B.DLC);
	}
	else {
		uint32_t dgn = getMsgBits(packet->MsgID, 24, 17);
		uint8_t sourceAddr = getMsgBits(packet->MsgID, 7, 8);
		uint8_t priority = getMsgBits(packet->MsgID, 28, 3);

		uint8_t* d = packet->data.u8;

		if (dgn == DATE_TIME_STATUS) {
			// uint8_t year = d[0];
			// uint8_t month = d[1];
			// uint8_t day = d[2];
			// uint8_t dayOfWeek = d[3];
			// uint8_t hour = d[4];
			// uint8_t minute = d[5];
			// uint8_t second = d[6];
			// uint8_t timeZone = d[7];

		}
		else if (dgn == DC_DIMMER_STATUS_3) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t enable = (d[3] >> 6) & 3;
			uint8_t delayDuration = d[4];
			DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			uint8_t status = (d[6] >> 2) & 3;

			// if (instance == 4) {
			// 	printf("DC_DIMMER_STATUS_3: inst=%d, grp=0X%02X, bright=%d, enable=%d, dur=%d, last cmd=%d, status=0X%02X\n", instance, group, brightness, enable, delayDuration, lastCmd, status);				
			// }
			setSwitchLevel(instance, brightness);
		}
		else if (dgn == DC_DIMMER_COMMAND_2) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = d[2];
			DCDimmerCmd cmd = (DCDimmerCmd)d[3];
			uint8_t delayDuration = d[4];

			// if (instance!=32 && instance!=43 && instance!=44 && instance!=53 && instance!=54 && instance!=55 && instance!=56) {
			// 	printf("DC_DIMMER_COMMAND_2: inst=%d, grp=0X%02X, bright=%d, cmd=0X%02X, dur=%d\n", instance, group, brightness, cmd, delayDuration);	
			// 	printPacket = packetPrintYes;			
			// }
		}
		else if (dgn == THERMOSTAT_AMBIENT_STATUS) {
			uint8_t instance = d[0];
			double tempC = convToTempC(d[2]<<8 | d[1]);
			
			// printf("THERMOSTAT_AMBIENT_STATUS: #%d, temp=%fºC\n", instance, tempC);
			setAmbientTemp(instance, tempC);
		}
		else if (dgn == THERMOSTAT_STATUS_1 || dgn == THERMOSTAT_COMMAND_1) {
			uint8_t instance = d[0];
			ThermostatMode opMode = (ThermostatMode)(d[1] & 0x0F);
			FanMode fanMode = (FanMode)((d[1]>>4) & 0x03);
			uint8_t scheduleEnabled = ((d[1]>>6) & 0x03);
			uint8_t fanSpeed = d[2];
			double heatTemp = convToTempC(d[4]<<8 | d[3]);
			double coolTemp = convToTempC(d[6]<<8 | d[5]);
			
			// const char* dgnName = (dgn == THERMOSTAT_STATUS_1) ? "THERMOSTAT_STATUS_1" : "THERMOSTAT_COMMAND_1";
			// printf("%s: inst=%d, opMode=%d, fanMode=%d, fanSpeed=%d, heatTemp=%fºC, coolTemp=%fºC\n", dgnName, instance, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
			setThermostatInfo(instance, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
		}
		else if (dgn == AIR_CONDITIONER_STATUS) {
			uint8_t instance = d[0];
			ACMode opMode = (ACMode)d[1];
			uint8_t maxFanSpeed = d[2];
			uint8_t maxOutputLevel = d[3];
			uint8_t fanSpeed = d[4];
			uint8_t outputLevel = d[5];
			uint8_t deadBand = d[6];
			uint8_t deadBand2 = d[7];
			
		}
		else if (dgn == FURNACE_STATUS) {
			uint8_t instance = d[0];
			ACMode opMode = (ACMode)(d[1] & 0x03);
			uint8_t fanSpeed = d[4];

		}
		else if (dgn == GENERIC_INDICATOR_COMMAND) {
			// printf("GENERIC_INDICATOR_COMMAND: inst=%d, grp=0X%02X, bright=%d, bank=%d, dur=%d, func=%d\n", d[0], d[1], d[2], d[3], d[4], d[6]);
		}
		else if (dgn == GENERIC_CONFIGURATION_STATUS) {
			
		}
		else if (dgn == DC_SOURCE_STATUS_1) {
			
		}
		else if (dgn == TANK_STATUS) {
			
		}
		else if (dgn == GENERATOR_STATUS_1) {
			
		}
		else if (dgn == 0x0FECA) { // DM_1 - 1FECA - DM_RV

		}
		else if (dgn == 0x0E8FF) {
			
		}
		else if (dgn == 0x0EAFF) {
			
		}
		else if (dgn == 0x15FCE) {
			
		}
		else if (dgn == 0x1AAFD) {
			
		}
		else if (dgn == 0x1BBFD) {
			
		}
		else if (dgn == 0x1FACE) {
			
		}
		else if (dgn == 0x1FACF) {
			
		}
		else if (dgn == 0x1FBDA) {
			
		}
		else {
			knownPacket = false;
		}

		if (printPacket==packetPrintYes || (!knownPacket && printPacket==packetPrintIfUnknown)) {
			if (packet->FIR.B.FF == CAN_frame_std) {
				printf("Std - ");
			}
			else {
				printf("Ext - ");
			}

			printf("dgn=%05X, src=%02X, pri=%d, Data: ", dgn, sourceAddr, priority);

			for (auto i = 0; i < packet->FIR.B.DLC; i++) {
				printf("%02X ", packet->data.u8[i]);
			}
			printf("\n");
		}
	}
}

//////////////////////////////////////////////

void loop() {
	static elapsedMillis heartbeatTime;

	// esp_task_wdt_reset();

	bool sendIndicator = lastPacketSendTime < packetBlinkTime;
	bool recvIndicator = lastPacketRecvTime < packetBlinkTime;

	if (sendIndicator || recvIndicator) {
		heartbeatTime = heartbeatBlinkTime;		// adjust next heartbeat time to avoid overlap with send/recv indicators
	}

	bool hearbeatIndicator = (heartbeatTime % heatbeatRate) < heartbeatBlinkTime;

	digitalWrite(indicatorPinR, !hearbeatIndicator);
	digitalWrite(indicatorPinG, !sendIndicator);
	digitalWrite(indicatorPinB, !recvIndicator);

	homeSpan.poll();

	CAN_frame_t packet;

	if (xQueueReceive(CAN_cfg.rx_queue, &packet, 0) == pdTRUE) {
		processPacket(&packet, packetPrintMode);
	}

	processPacketQueue();
}
