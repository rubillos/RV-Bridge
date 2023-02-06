#include <Arduino.h>

#include "elapsedMillis.h"
#include "HomeSpan.h" 
#include <ESP32CAN.h>
#include <CAN_config.h>

#include "config.h"

#define INDICATOR_PIN_R 2
#define INDICATOR_PIN_G 15
#define INDICATOR_PIN_B 4

#define HOMESPAN_CONTROL_PIN 21
#define HOMESPAN_CONTROL_GND 19

constexpr uint16_t maxSwitches = 20;
constexpr uint16_t maxFans = 2;
constexpr uint16_t maxThermostats = 2;

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

void sendOnOff(uint8_t index, bool on) {
	printf("sendOnOff: #%d to %d\n", index, on);
}

void sendLampLevel(uint8_t index, uint8_t level) {
	printf("sendLampLevel: #%d to %d\n", index, level);
}

const char* switchTypes[3] = { "43", "43", "49" };
const char* switchHapNames[3] = { "LightBulb", "LightBulb", "Switch" };

struct RVSwitch : SpanService {
	SpanCharacteristic *_on = NULL;
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
		if (index == _index) {
			printf("Switch #%d: setOnOff = %d\n", index, on);
			_on->setVal(on);
		}
	}
	void setLevel(uint8_t index, uint8_t level) {
		if (index == _index) {
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
			sendOnOff(_upIndex, (newValue) ? 1 : 0);
			sendOnOff(_downIndex, (newValue) ? 0 : 1);
		}

		return(true);  
	}
	
	void setActive(uint8_t index, bool on) {
		if (index == _index) {
			printf("Fan #%d: setActive = %d\n", index, on);
			_active->setVal(on);
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
			printf("RVHVACFan #%d - Rotation: %d\n", _index, _active->getNewVal());
		}

		return(true);  
	}
};

struct RVThermostat : Service::Thermostat {
	SpanCharacteristic *_ambientTemp;
	SpanCharacteristic *_targetTemp;
	SpanCharacteristic *_curState;
	SpanCharacteristic *_targetState;

	SpanService *_fan;

	uint8_t _index;

	bool _updateState = false;

	RVThermostat(ThermostatDeviceRec *device) : Service::Thermostat() {
		_ambientTemp = new Characteristic::CurrentTemperature(22);
		_targetTemp = new Characteristic::TargetTemperature(22);
		_targetTemp->setRange(10,30,0.5)->setVal(20);
		_curState = new Characteristic::CurrentHeatingCoolingState(0);
		_targetState = new Characteristic::TargetHeatingCoolingState(0);
		new Characteristic::TemperatureDisplayUnits(1);

		if (device->hasHeating) {
			_targetState->setValidValues(3, 0, 1, 2);
		}
		else {
			_targetState->setValidValues(2, 0, 2);
		}

		_index = device->index;

		_fan = new RVHVACFan(device->fanIndex);
	}

	boolean update() {
		if (_targetState->updated()) {
			printf("RVThermostat #%d, Target State: %d\n", _index, _targetState->getNewVal());
			_updateState = true;
		}
		if (_targetTemp->updated()) {
			printf("RVThermostat #%d - Target Temp: %f\n", _index, _targetTemp->getNewVal<double>());
		}

		return(true);  
	}

	void loop() {
		if (_updateState) {
			printf("Updating Current State\n");
			_curState->setVal(_targetState->getVal());
			_updateState = false;
		}
	}
};

RVSwitch* switches[maxSwitches];
uint16_t switchCount = 0;

RVRoofFan* fans[maxFans];
uint16_t fanCount = 0;

RVThermostat* thermostats[maxThermostats];
uint16_t thermostatCount = 0;

void setSwitchState(uint8_t index, bool on) {
	for (uint16_t i=0; i<switchCount; i++) {
		switches[i]->setOnOff(index, on);
	}
}

void setLampBrightness(uint8_t index, int8_t level) {
	for (uint16_t i=0; i<switchCount; i++) {
		switches[i]->setLevel(index, level);
	}
}

void setFanActive(uint8_t index, bool on) {
	for (uint16_t i=0; i<fanCount; i++) {
		fans[i]->setActive(index, on);
	}
}

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

void setup() {
	pinMode(INDICATOR_PIN_R, OUTPUT);
	pinMode(INDICATOR_PIN_G, OUTPUT);
	pinMode(INDICATOR_PIN_B, OUTPUT);

	digitalWrite(INDICATOR_PIN_R, HIGH);
	digitalWrite(INDICATOR_PIN_G, HIGH);
	digitalWrite(INDICATOR_PIN_B, HIGH);

	pinMode(HOMESPAN_CONTROL_GND, OUTPUT);
	digitalWrite(HOMESPAN_CONTROL_GND, LOW);

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

uint32_t getBits(uint32_t val, int8_t startBit, int8_t numBits) {
	int8_t shift = startBit - numBits + 1;
	uint32_t mask = 0xFFFFFFFF >> (32 - numBits);

	return (val >> shift) & mask;
}

uint32_t makeMsg(uint32_t dgn, uint8_t sourceID=0, uint8_t priority=6) {
	if (sourceID == 0) {
		sourceID = sourceAddress;
	}

	return (priority<<26) | (dgn << 8) | sourceID;	
}

void analyzeFrame(CAN_frame_t *frame, bool force=false) {
	if (frame->FIR.B.RTR == CAN_RTR) {
		printf(" RTR from 0x%08X, DLC %d\r\n", frame->MsgID,  frame->FIR.B.DLC);
	}
	else {
		uint8_t priority = getBits(frame->MsgID, 28, 3);
		uint32_t dgn = getBits(frame->MsgID, 24, 17);
		uint8_t sourceAddr = getBits(frame->MsgID, 7, 8);

		uint8_t* d = frame->data.u8;

		if (dgn == 0x1FFFF) { // DATE_TIME_STATUS

		}
		else if (dgn == 0x0FECA) { // DM_1 - 1FECA - DM_RV

		}
		else if (dgn == 0x1FBDA) {
			
		}
		else if (dgn == 0x1AAFD) {
			
		}
		else if (dgn == 0x1BBFD) {
			
		}
		else if (dgn == 0x0E8FF) {
			
		}
		else if (dgn == 0x1FEDA) { // DC_DIMMER_STATUS_3
			// printf("DC_DIMMER_STATUS_3: inst=%d, grp=0X%02X, bright=%d, status=0X%02X, dur=%d, last cmd=%d, status=0X%02X\n", d[0], d[1], d[2], d[3], d[4], d[5], d[6]);				
		}
		else if (dgn == 0x1FEDB) { // DC_DIMMER_COMMAND_2
			printf("DC_DIMMER_COMMAND_2: inst=%d, grp=0X%02X, bright=%d, cmd=0X%02X, dur=%d, lock=0X%02X\n", d[0], d[1], d[2], d[3], d[4], d[5]);				
			// force = true;
		}
		else if (dgn == 0x0E8FF) {
			
		}
		else if (dgn == 0x15FCE) {
			
		}
		else if (dgn == 0x1FFE0) { // AIR_CONDITIONER_STATUS - 1FFE1
			
		}
		else if (dgn == 0x1FFDC) { // GENERATOR_STATUS_1
			
		}
		else if (dgn == 0x1FF9C) { // THERMOSTAT_AMBIENT_STATUS
			
		}
		else if (dgn == 0x1FFE2) { // THERMOSTAT_STATUS_1
			
		}
		else if (dgn == 0x0EAFF) {
			
		}
		else if (dgn == 0x1FED8) { // GENERIC_CONFIGURATION_STATUS
			
		}
		else if (dgn == 0x1FFFD) { // DC_SOURCE_STATUS_1
			
		}
		else if (dgn == 0x1FFB7) { // TANK_STATUS
			
		}
		else if (dgn == 0x1FACE) {
			
		}
		else if (dgn == 0x1FACF) {
			
		}
		else if (dgn == 0x1FED9) { // GENERIC_INDICATOR_COMMAND
			printf("GENERIC_INDICATOR_COMMAND: inst=%d, grp=0X%02X, bright=%d, bank=%d, dur=%d, func=%d\n", d[0], d[1], d[2], d[3], d[4], d[6]);
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

void loop() {
	static elapsedMillis blinkTime;

	digitalWrite(INDICATOR_PIN_R, (blinkTime % 2000) > 50);

	homeSpan.poll();

	CAN_frame_t rx_frame;

	unsigned long currentMillis = millis();

	// Receive next CAN frame from queue
	if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
		analyzeFrame(&rx_frame);
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

		analyzeFrame(&tx_frame, true);
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
