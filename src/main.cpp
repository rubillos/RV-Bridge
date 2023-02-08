#include "Arduino.h"

#include "elapsedMillis.h"
#include "HomeSpan.h" 
#include "ESP32CAN.h"
#include "CAN_config.h"

#include "config.h"

//////////////////////////////////////////////

#define INDICATOR_PIN_R 2
#define INDICATOR_PIN_G 15
#define INDICATOR_PIN_B 4

#define HOMESPAN_CONTROL_PIN 21
#define HOMESPAN_CONTROL_GND 19

constexpr uint16_t maxSwitches = 100;
constexpr uint16_t maxFans = 100;
constexpr uint16_t maxThermostats = 5;

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

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

void sendOnOff(uint8_t index, bool on) {
	printf("sendOnOff: #%d to %d\n", index, on);
}

void sendLampLevel(uint8_t index, uint8_t level) {
	printf("sendLampLevel: #%d to %d\n", index, level);
}

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
		if (_on->updated()) {
			sendOnOff(_index, _on->getNewVal());
		}
		if (_brightness && _brightness->updated()) {
			sendLampLevel(_index, _brightness->getNewVal());
		}
		return(true);  
	}
	
	void setOnOff(uint8_t index, bool on) {
		if (index == _index && on != _on->getVal()) {
			printf("Switch #%d: setOnOff = %d\n", index, on);
			_on->setVal(on);
		}
	}
	void setLevel(uint8_t index, uint8_t level) {
		if (index == _index && level != _brightness->getVal()) {
			printf("Switch #%d: setOnOff = %d\n", index, level);
			_brightness->setVal(level);
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
	
	void setOnOff(uint8_t index, bool on) {
		if (index == _index) {
			_fanPower = on;
		}
		else if (index == _upIndex && on) {
			_lidUp = true;
		}
		else if (index == _downIndex && on) {
			_lidUp = false;
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
		_speed->setRange(0, 2, 1);

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
		_targetTemp->setRange(10,32,0.5)->setVal(20);
		_curState = new Characteristic::CurrentHeatingCoolingState(0);
		_targetState = new Characteristic::TargetHeatingCoolingState(0);
		new Characteristic::TemperatureDisplayUnits(1);

		if (device->furnaceIndex) {
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

	void setOnOff(uint8_t index, bool on) {
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
			printf("Set ambient temp #%d: %dºC\n", _index, tempC);
			_ambientTemp->setVal(tempC);
		}
	}

	void setThermostatInfo(uint8_t index, ThermostatMode opMode, FanMode fanMode, uint8_t fanSpeed, double heatTemp, double coolTemp) {
		if (index == _index) {
			uint8_t modeConvert[] = { stateOff, stateCool, stateHeat, stateOff };
			uint8_t mode = modeConvert[opMode];

			if (mode != _curState->getVal()) {
				_curState->setVal(mode);
			}
			if (fabs(coolTemp - _targetTemp->getVal<double>()) > 0.2) {
				_targetTemp->setVal(coolTemp);
			}
			_fan->setModeSpeed(fanMode, fanSpeed);
		}
	}
};

//////////////////////////////////////////////

RVSwitch* switches[maxSwitches];
RVRoofFan* fans[maxFans];
RVThermostat* thermostats[maxThermostats];

uint16_t switchCount = 0;
uint16_t fanCount = 0;
uint16_t thermostatCount = 0;

void setSwitchState(uint8_t index, bool on) {
	for (uint16_t i=0; i<switchCount; i++) {
		switches[i]->setOnOff(index, on);
	}
	for (uint16_t i=0; i<fanCount; i++) {
		fans[i]->setOnOff(index, on);
	}
	for (uint16_t i=0; i<thermostatCount; i++) {
		thermostats[i]->setOnOff(index, on);
	}
}

void setLampBrightness(uint8_t index, int8_t level) {
	for (uint16_t i=0; i<switchCount; i++) {
		switches[i]->setLevel(index, level);
	}
}

void setAmbientTemp(uint8_t index, double tempC) {
	for (uint16_t i=0; i<thermostatCount; i++) {
		thermostats[i]->setAmbientTemp(index, tempC);
	}
}

void setThermostatInfo(uint8_t index, ThermostatMode opMode, FanMode fanMode, uint8_t fanSpeed, double heatTemp, double coolTemp) {
	for (uint16_t i=0; i<thermostatCount; i++) {
		thermostats[i]->setThermostatInfo(index, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
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

typedef struct {
	uint8_t pinNumber;
	uint8_t mode;
	uint8_t state;
} PinSetup;

PinSetup pinList[] = {
	{ INDICATOR_PIN_R, OUTPUT, HIGH },
	{ INDICATOR_PIN_G, OUTPUT, HIGH },
	{ INDICATOR_PIN_B, OUTPUT, HIGH },
	{ HOMESPAN_CONTROL_GND, OUTPUT, LOW },
};

void setup() {
	for (PinSetup pin : pinList) {
		pinMode(pin.pinNumber, pin.mode);
		digitalWrite(pin.pinNumber, pin.state);
	}

	for (int i=0; i<20; i++) {
		digitalWrite(INDICATOR_PIN_R, LOW);
		delay(100);
		digitalWrite(INDICATOR_PIN_R, HIGH);
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

	#ifdef OVERRIDE_MAC_ADDRESS
		uint8_t newMACAddress[] = OVERRIDE_MAC_ADDRESS;
		esp_base_mac_addr_set(&newMACAddress[0]);
		printf("MAC address updated to: %s\n", WiFi.macAddress().c_str());
	#endif

	Serial.println("Init HomeSpan");
	homeSpan.setStatusPin(INDICATOR_PIN_B);
	homeSpan.setControlPin(HOMESPAN_CONTROL_PIN);
	homeSpan.setWifiCredentials(ssid, sspwd);
	homeSpan.setSketchVersion(versionString);
	homeSpan.begin(Category::Bridges, "RV-Bridge", DEFAULT_HOST_NAME, "RV-Bridge-ESP32");
	createDevices();

	Serial.println("Init complete.");
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

double convToTempC(uint16_t value) {
	return -273.0 + value * 0.03125;
}

void processFrame(CAN_frame_t *frame, bool force=true) {
	if (frame->FIR.B.RTR == CAN_RTR) {
		printf(" RTR from 0x%08X, DLC %d\r\n", frame->MsgID,  frame->FIR.B.DLC);
	}
	else {
		uint8_t priority = getMsgBits(frame->MsgID, 28, 3);
		uint32_t dgn = getMsgBits(frame->MsgID, 24, 17);
		uint8_t sourceAddr = getMsgBits(frame->MsgID, 7, 8);

		uint8_t* d = frame->data.u8;

		if (dgn == 0x1FFFF) { // DATE_TIME_STATUS
			// uint8_t year = d[0];
			// uint8_t month = d[1];
			// uint8_t day = d[2];
			// uint8_t dayOfWeek = d[3];
			// uint8_t hour = d[4];
			// uint8_t minute = d[5];
			// uint8_t second = d[6];
			// uint8_t timeZone = d[7];

		}
		else if (dgn == 0x1FEDA) { // DC_DIMMER_STATUS_3
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = d[2];
			uint8_t enable = (d[3] >> 6) & 3;
			uint8_t delayDuration = d[4];
			DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			uint8_t status = (d[6] >> 2) & 3;

			printf("DC_DIMMER_STATUS_3: inst=%d, grp=0X%02X, bright=%d, enable=%d, dur=%d, last cmd=%d, status=0X%02X\n", instance, group, brightness, enable, delayDuration, lastCmd, status);				
		}
		else if (dgn == 0x1FEDB) { // DC_DIMMER_COMMAND_2
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = d[2];
			DCDimmerCmd cmd = (DCDimmerCmd)d[3];
			uint8_t delayDuration = d[4];

			printf("DC_DIMMER_COMMAND_2: inst=%d, grp=0X%02X, bright=%d, cmd=0X%02X, dur=%d\n", instance, group, brightness, cmd, delayDuration);				
			// force = true;
		}
		else if (dgn == 0x1FF9C) { // THERMOSTAT_AMBIENT_STATUS
			uint8_t instance = d[0];
			double tempC = convToTempC(d[2]<<8 | d[1]);
			
			setAmbientTemp(instance, tempC);
		}
		else if (dgn == 0x1FFE2) { // THERMOSTAT_STATUS_1
			uint8_t instance = d[0];
			ThermostatMode opMode = (ThermostatMode)(d[1] & 0x0F);
			FanMode fanMode = (FanMode)((d[1]>>4) & 0x03);
			uint8_t fanSpeed = d[2];
			double heatTemp = convToTempC(d[4]<<8 | d[3]);
			double coolTemp = convToTempC(d[6]<<8 | d[5]);
			
			setThermostatInfo(instance, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
		}
		else if (dgn == 0x1FFE0) { // AIR_CONDITIONER_STATUS - 1FFE1
			uint8_t instance = d[0];
			ACMode opMode = (ACMode)d[1];
			uint8_t maxFanSpeed = d[2];
			uint8_t maxOutputLevel = d[3];
			uint8_t fanSpeed = d[4];
			uint8_t outputLevel = d[5];
			uint8_t deadBand = d[6];
			uint8_t deadBand2 = d[7];
			
		}
		else if (dgn == 0x1FED9) { // GENERIC_INDICATOR_COMMAND
			// printf("GENERIC_INDICATOR_COMMAND: inst=%d, grp=0X%02X, bright=%d, bank=%d, dur=%d, func=%d\n", d[0], d[1], d[2], d[3], d[4], d[6]);
		}
		else if (dgn == 0x1FED8) { // GENERIC_CONFIGURATION_STATUS
			
		}
		else if (dgn == 0x1FFFD) { // DC_SOURCE_STATUS_1
			
		}
		else if (dgn == 0x1FFB7) { // TANK_STATUS
			
		}
		else if (dgn == 0x1FFDC) { // GENERATOR_STATUS_1
			
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
			force = true;
		}

		if (force) {
			if (frame->FIR.B.FF == CAN_frame_std) {
				printf("Standard frame");
			}
			else {
				printf("Extended frame");
			}

			printf(" pri=%d, dgn=0x%05X, src=%02X, Data ", priority, dgn, sourceAddr);

			// printf(" from 0x%08X, DLC %d, Data ", frame->MsgID,  frame->FIR.B.DLC);

			for (int i = 0; i < frame->FIR.B.DLC; i++) {
				printf("0x%02X ", frame->data.u8[i]);
			}
			printf("\n");
		}
	}
}

//////////////////////////////////////////////

void loop() {
	static elapsedMillis blinkTime;

	digitalWrite(INDICATOR_PIN_R, (blinkTime % 2000) > 50);

	homeSpan.poll();

	CAN_frame_t rx_frame;

	unsigned long currentMillis = millis();

	// Receive next CAN frame from queue
	if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0 /*3 * portTICK_PERIOD_MS */) == pdTRUE) {
		processFrame(&rx_frame);
	}

	static elapsedMillis toggleTime;

	if (false && toggleTime > 3000) {
		toggleTime = 0;

		// pri=6, dgn=0x1FEDB, src=9F
		// DC_DIMMER_COMMAND_2 (1FEDB):
		//    inst=17, grp=0XFF, bright=200, cmd=0X05, dur=255, lock=0X00

		// DC_DIMMER_COMMAND_2: inst=17, grp=0XFF, bright=200, cmd=0X05, dur=255, lock=0X00
		// Extended frame pri=6, dgn=0x1FEDB, src=FC, Data 0x11 0xFF 0xC8 0x05 0xFF 0x00 0xFF 0xFF 

		// Sending toggle
		// DC_DIMMER_COMMAND_2: inst=17, grp=0XFF, bright=200, cmd=0X05, dur=255, lock=0X00
		// Standard frame pri=6, dgn=0x1FEDB, src=81, Data 0x11 0xFF 0xC8 0x05 0xFF 0x00 0x00 0x00 

		CAN_frame_t tx_frame;
		uint8_t* d = tx_frame.data.u8;

		tx_frame.FIR.B.FF = CAN_frame_ext;
		tx_frame.MsgID = makeMsg(0x1FEDB);

		tx_frame.FIR.B.DLC = 8;
		d[0] = 17;
		d[1] = 0xFF;
		d[2] = 200;
		d[3] = 5; // toggle
		d[4] = 255;
		d[5] = 0;

		d[6] = 0xFF;
		d[7] = 0xFF;

		printf("Sending toggle\n");
		ESP32Can.CANWriteFrame(&tx_frame);

		processFrame(&tx_frame, true);
	}
	// Send CAN Message
//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis;
//     CAN_frame_t tx_frame;
//     tx_frame.FIR.B.FF = CAN_frame_std;
//     tx_frame.MsgID = 0x001;
//     tx_frame.FIR.B.DLC = 8;
//     tx_frame.data.u8[0] = 0x00;
//     tx_frame.data.u8[1] = 0x01;
//     tx_frame.data.u8[2] = 0x02;
//     tx_frame.data.u8[3] = 0x03;
//     tx_frame.data.u8[4] = 0x04;
//     tx_frame.data.u8[5] = 0x05;
//     tx_frame.data.u8[6] = 0x06;
//     tx_frame.data.u8[7] = 0x07;
//     ESP32Can.CANWriteFrame(&tx_frame);
//   }
}
