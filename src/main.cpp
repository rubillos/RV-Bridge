#include "Arduino.h"

#include "elapsedMillis.h"
#include "HomeSpan.h" 
#include "ESP32CAN.h"
#include "CAN_config.h"

#include "config.h"

//////////////////////////////////////////////

constexpr const char* versionString = "v0.1";

constexpr uint8_t indicatorPinR = 2;
constexpr uint8_t indicatorPinG = 15;
constexpr uint8_t indicatorPinB = 4;

CAN_device_t CAN_cfg;               	// CAN Config
constexpr int rx_queue_size = 10;       // Receive Queue size

elapsedMillis packetSendTimer = 1000;
elapsedMillis packetRecvTimer = 1000;

//////////////////////////////////////////////

void processPacket(CAN_frame_t *packet, bool printPacket=true);

//////////////////////////////////////////////

constexpr uint32_t DATE_TIME_STATUS = 0x1FFFF;
constexpr uint32_t DC_DIMMER_COMMAND_2 = 0x1FEDB;
constexpr uint32_t DC_DIMMER_STATUS_3 = 0x1FEDA;
constexpr uint32_t THERMOSTAT_AMBIENT_STATUS = 0x1FF9C;
constexpr uint32_t THERMOSTAT_STATUS_1 = 0x1FFE2;
constexpr uint32_t AIR_CONDITIONER_STATUS = 0x1FFE0; // 1FFE1
constexpr uint32_t GENERIC_INDICATOR_COMMAND = 0x1FED9;
constexpr uint32_t GENERIC_CONFIGURATION_STATUS = 0x1FED8;
constexpr uint32_t DC_SOURCE_STATUS_1 = 0x1FFFD;
constexpr uint32_t TANK_STATUS = 0x1FFB7;
constexpr uint32_t GENERATOR_STATUS_1 = 0x1FFDC;

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

void initPacket(CAN_frame_t* packet) {
	packet->FIR.B.DLC = 8;
	packet->FIR.B.RTR = CAN_no_RTR;
	packet->FIR.B.FF = CAN_frame_ext;
	for (auto i=0; i<8; i++) {
		packet->data.u8[i] = 0xFF;
	}
}

void sendDCDimmerCmd(uint8_t index, uint8_t brightness, uint8_t cmd, uint8_t duration=0xFF) {
	CAN_frame_t packet;
	initPacket(&packet);
	uint8_t* d = packet.data.u8;
	
	packet.MsgID = makeMsg(DC_DIMMER_COMMAND_2);

	d[0] = index;
	d[2] = brightness;
	d[3] = cmd;
	d[4] = duration;

	packetSendTimer = 0;
	// ESP32Can.CANWriteFrame(&packet);

	printf("Sending packet:\n");
	processPacket(&packet, true);
	printf("** Packet sent.\n");
}

void sendOnOff(uint8_t index, bool on, uint8_t brightness=200) {
	printf("sendOnOff: #%d to %d\n", index, on);
	sendDCDimmerCmd(index, brightness, (on) ? DCDimmerCmdOnDuration : DCDimmerCmdOff);
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
			_brightness = new Characteristic::Brightness(100);
			_brightness->setRange(0, 100, 1);
		}
	}
	
	boolean update() {
		if (_on->updated() || (_brightness && _brightness->updated())) {
			if (_brightness) {
				sendLampLevel(_index, _on->getNewVal() * _brightness->getNewVal() * 2);
			}
			else {
				sendOnOff(_index, _on->getNewVal());
			}
		}
		return(true);  
	}
	
	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;
		level /= 2;

		if (index == _index && on != _on->getVal()) {
			printf("Switch #%d: on = %d\n", index, on);
			_on->setVal(on);
			packetRecvTimer = 0;
		}
		if (index == _index && _brightness && on && level != _brightness->getVal()) {
			printf("Switch #%d: level = %d\n", index, level);
			_brightness->setVal(level);
			packetRecvTimer = 0;
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
			if (newValue) {
				sendOnOff(_downIndex, false);
				sendOnOff(_upIndex, true);
			}
			else {
				sendOnOff(_upIndex, false);
				sendOnOff(_downIndex, true);
			}
		}

		return(true);  
	}
	
	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;

		if (index == _index) {
			_fanPower = on;
			packetRecvTimer = 0;
		}
		else if (index == _upIndex && on) {
			_lidUp = true;
			packetRecvTimer = 0;
		}
		else if (index == _downIndex && on) {
			_lidUp = false;
			packetRecvTimer = 0;
		}

		bool newState = _fanPower && _lidUp;

		if (newState != _active->getVal()) {
			printf("Fan #%d: active = %d\n", index, newState);
			_active->setVal(newState);
		}
	}
};

struct RVHVACFan : Service::Fan {
	SpanCharacteristic *_active;
	SpanCharacteristic *_speed;

	uint8_t _index;

	RVHVACFan(uint8_t index) : Service::Fan() {
		_active = new Characteristic::Active();
		_speed = new Characteristic::RotationSpeed();
		_speed->setRange(0, 100, 50);

		_index = index;
	}
	
	boolean update() {
		if (_active->updated()) {
			printf("RVHVACFan #%d - Active: %d\n", _index, _active->getNewVal());
		}
		if (_speed->updated()) {
			printf("RVHVACFan #%d - Speed: %d\n", _index, _speed->getNewVal());
		}
		return(true);  
	}

	void setModeSpeed(FanMode mode, uint8_t speed) {

	}
};

struct RVThermostat : Service::Thermostat {
	SpanCharacteristic *_ambientTemp;
	SpanCharacteristic *_targetTemp;
	SpanCharacteristic *_targetState;
	SpanCharacteristic *_curState;

	RVHVACFan *_fan;

	uint8_t _index;
	uint8_t _compressorIndex;
	uint8_t _furnaceIndex;

	bool _compressorRunning = false;
	bool _furnaceRunning = false;

	enum {
		stateOff = 0,
		stateHeat,
		stateCool
	};

	RVThermostat(ThermostatDeviceRec *device) : Service::Thermostat() {
		_ambientTemp = new Characteristic::CurrentTemperature(22);
		_targetTemp = new Characteristic::TargetTemperature(22);
		_targetTemp->setRange(10, 32, 0.5)->setVal(20);
		_curState = new Characteristic::CurrentHeatingCoolingState(0);
		_targetState = new Characteristic::TargetHeatingCoolingState(0);
		new Characteristic::TemperatureDisplayUnits(1);

		if (device->furnaceIndex != -1) {
			_targetState->setValidValues(3, 0, 1, 2);
		}
		else {
			_targetState->setValidValues(2, 0, 2);
		}

		_index = device->index;
		_compressorIndex = device->compressorIndex;
		_furnaceIndex = device->furnaceIndex;

		_fan = new RVHVACFan(_index);
	}

	boolean update() {
		if (_targetState->updated()) {
			printf("RVThermostat #%d, Target State: %d\n", _index, _targetState->getNewVal());
		}
		if (_targetTemp->updated()) {
			printf("RVThermostat #%d - Target Temp: %f\n", _index, _targetTemp->getNewVal<double>());
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
			packetRecvTimer = 0;
		}
		if (index == _furnaceIndex && on != _furnaceRunning) {
			printf("Thermostat #%d: furnace = %d\n", index, on);
			_furnaceRunning = on;
			changed = true;
			packetRecvTimer = 0;
		}
		if (changed) {
			uint8_t newState;

			if (_furnaceRunning) {
				newState = stateHeat;
			}
			else if (_compressorRunning) {
				newState = stateCool;
			}
			else {
				newState = stateOff;
			}
			if (newState != _curState->getVal()) {
				_curState->setVal(newState);
			}
		}
	}

	void setAmbientTemp(uint8_t index, double tempC) {
		if (index == _index && fabs(tempC - _ambientTemp->getVal<double>()) > 0.2) {
			printf("Set ambient temp #%d: %fºC\n", _index, tempC);
			_ambientTemp->setVal(tempC);
			packetRecvTimer = 0;
		}
	}

	void setInfo(uint8_t index, ThermostatMode opMode, FanMode fanMode, uint8_t fanSpeed, double heatTemp, double coolTemp) {
		if (index == _index) {
			uint8_t modeConvert[] { stateOff, stateCool, stateHeat, stateOff };
			uint8_t mode = modeConvert[opMode];

			if (mode != _curState->getVal()) {
				_curState->setVal(mode);
				packetRecvTimer = 0;
			}
			if (fabs(coolTemp - _targetTemp->getVal<double>()) > 0.2) {
				_targetTemp->setVal(coolTemp);
				packetRecvTimer = 0;
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
	}
}

void cmdSetLevel(const char *buff) {
	cmdSet(buff, 1, "cmdSetLevel");
}

void cmdSetState(const char *buff) {
	cmdSet(buff, 200, "cmdSetState");
}

void cmsSetTemp(const char *buff) {
	int16_t index;
	int16_t val;

	buff += 1;
	
	while (buff) {
		buff = getValuePair(buff, &index, &val);

		if (index!=-1 && val!=-1) {
			double tempC = (val - 32.0) / 1.8;
			printf("cmsSetTemp: index=%d, val=%dºF (%fºC)\n", index, val, tempC);
			setAmbientTemp(index, tempC);
		}
	}
}

void addCommands() {
	new SpanUserCommand('t',"<index>=<temperature>,... - set temperature of <index> to <val:ºF>", cmsSetTemp);
	new SpanUserCommand('l',"<index>=<level>,... - set level of <index> to <val:0-200>", cmdSetLevel);
	new SpanUserCommand('s',"<index>=<level>,... - set state of <index> to <val:0-1>", cmdSetState);
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

void setup() {
	for (PinSetup pin : pinList) {
		pinMode(pin.pinNumber, pin.mode);
		digitalWrite(pin.pinNumber, pin.state);
	}

	for (auto i=0; i<20; i++) {
		digitalWrite(indicatorPinR, LOW);
		delay(100);
		digitalWrite(indicatorPinR, HIGH);
		delay(100);
	}

	Serial.begin(115200);
	Serial.println("RV Bridge - Startup");

	Serial.println("Init CAN module");
	CAN_cfg.speed = CAN_SPEED_250KBPS;
	CAN_cfg.tx_pin_id = GPIO_NUM_25;
	CAN_cfg.rx_pin_id = GPIO_NUM_26;
	CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
	ESP32Can.CANInit();

	Serial.println("Init HomeSpan");
	homeSpan.setWifiCredentials(ssid, sspwd);
	homeSpan.setSketchVersion(versionString);
	homeSpan.begin(Category::Bridges, "RV-Bridge", DEFAULT_HOST_NAME, "RV-Bridge-ESP32");

	createDevices();
	addCommands();

	Serial.println("Init complete.");
}

//////////////////////////////////////////////

double convToTempC(uint16_t value) {
	return -273.0 + value * 0.03125;
}

void processPacket(CAN_frame_t *packet, bool printPacket) {
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
			uint8_t brightness = d[2];
			uint8_t enable = (d[3] >> 6) & 3;
			uint8_t delayDuration = d[4];
			DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			uint8_t status = (d[6] >> 2) & 3;

			printf("DC_DIMMER_STATUS_3: inst=%d, grp=0X%02X, bright=%d, enable=%d, dur=%d, last cmd=%d, status=0X%02X\n", instance, group, brightness, enable, delayDuration, lastCmd, status);				
			setSwitchLevel(instance, brightness);
		}
		else if (dgn == DC_DIMMER_COMMAND_2) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = d[2];
			DCDimmerCmd cmd = (DCDimmerCmd)d[3];
			uint8_t delayDuration = d[4];

			printf("DC_DIMMER_COMMAND_2: inst=%d, grp=0X%02X, bright=%d, cmd=0X%02X, dur=%d\n", instance, group, brightness, cmd, delayDuration);				
			// printPacket = true;
		}
		else if (dgn == THERMOSTAT_AMBIENT_STATUS) {
			uint8_t instance = d[0];
			double tempC = convToTempC(d[2]<<8 | d[1]);
			
			printf("THERMOSTAT_AMBIENT_STATUS: temp=%fºC\n", tempC);
			setAmbientTemp(instance, tempC);
		}
		else if (dgn == THERMOSTAT_STATUS_1) {
			uint8_t instance = d[0];
			ThermostatMode opMode = (ThermostatMode)(d[1] & 0x0F);
			FanMode fanMode = (FanMode)((d[1]>>4) & 0x03);
			uint8_t fanSpeed = d[2];
			double heatTemp = convToTempC(d[4]<<8 | d[3]);
			double coolTemp = convToTempC(d[6]<<8 | d[5]);
			
			printf("THERMOSTAT_STATUS_1: inst=%d, opMode=%d, fanMode=%d, fanSpeed=%d, heatTemp=%fºC, coolTemp=%fºC\n", instance, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
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
			printPacket = true;
		}

		if (printPacket) {
			if (packet->FIR.B.FF == CAN_frame_std) {
				printf("Std - ");
			}
			else {
				printf("Ext - ");
			}

			printf("dgn=%05X, src=%02X, pri=%d, Data: ", dgn, sourceAddr, priority);

			// printf(" from 0x%08X, DLC %d, Data ", packet->MsgID,  packet->FIR.B.DLC);

			for (auto i = 0; i < packet->FIR.B.DLC; i++) {
				printf("%02X ", packet->data.u8[i]);
			}
			printf("\n");
		}
	}
}

//////////////////////////////////////////////

void loop() {
	static elapsedMillis blinkTime;

	digitalWrite(indicatorPinR, (blinkTime % 2000) > 20);
	digitalWrite(indicatorPinG, packetSendTimer > 20);
	digitalWrite(indicatorPinB, packetRecvTimer > 20);

	homeSpan.poll();

	CAN_frame_t packet;

	if (xQueueReceive(CAN_cfg.rx_queue, &packet, 0) == pdTRUE) {
		processPacket(&packet);
	}

	// static elapsedMillis toggleTime;

	// if (false && toggleTime > 3000) {
	// 	toggleTime = 0;

		// pri=6, dgn=0x1FEDB, src=9F
		// DC_DIMMER_COMMAND_2 (1FEDB):
		//    inst=17, grp=0XFF, bright=200, cmd=0X05, dur=255, lock=0X00

		// DC_DIMMER_COMMAND_2: inst=17, grp=0XFF, bright=200, cmd=0X05, dur=255, lock=0X00
		// Extended packet pri=6, dgn=0x1FEDB, src=FC, Data 0x11 0xFF 0xC8 0x05 0xFF 0x00 0xFF 0xFF 

		// Sending toggle
		// DC_DIMMER_COMMAND_2: inst=17, grp=0XFF, bright=200, cmd=0X05, dur=255, lock=0X00
		// Standard frame pri=6, dgn=0x1FEDB, src=81, Data 0x11 0xFF 0xC8 0x05 0xFF 0x00 0x00 0x00 

		// CAN_frame_t tx_packet;
		// uint8_t* d = tx_packet.data.u8;

		// tx_packet.FIR.B.FF = CAN_frame_ext;
		// tx_packet.MsgID = makeMsg(0x1FEDB);

		// tx_packet.FIR.B.DLC = 8;
		// d[0] = 17;
		// d[1] = 0xFF;
		// d[2] = 200;
		// d[3] = 5; // toggle
		// d[4] = 255;
		// d[5] = 0;

		// d[6] = 0xFF;
		// d[7] = 0xFF;

		// printf("Sending toggle\n");
		// ESP32Can.CANWriteFrame(&tx_packet);

		// processPacket(&tx_packet, true);
	}
	// Send CAN Message
//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis;
//     CAN_frame_t tx_packet;
//     tx_packet.FIR.B.FF = CAN_frame_std;
//     tx_packet.MsgID = 0x001;
//     tx_packet.FIR.B.DLC = 8;
//     tx_packet.data.u8[0] = 0x00;
//     tx_packet.data.u8[1] = 0x01;
//     tx_packet.data.u8[2] = 0x02;
//     tx_packet.data.u8[3] = 0x03;
//     tx_packet.data.u8[4] = 0x04;
//     tx_packet.data.u8[5] = 0x05;
//     tx_packet.data.u8[6] = 0x06;
//     tx_packet.data.u8[7] = 0x07;
//     ESP32Can.CANWriteFrame(&tx_packet);
//   }
// }
