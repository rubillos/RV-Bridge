#ifndef _DEVICES_H_
#define _DEVICES_H_

#include <stdint.h>

typedef enum {
    Lamp = 0,
    DimmableLamp,
    Switch
} SwitchType;

typedef struct {
    uint8_t index;
    SwitchType type;
    const char* name;
} SwitchDeviceRec;

typedef struct {
    uint8_t index;
    uint8_t upIndex;
    uint8_t downIndex;
    const char* name;
} FanDeviceRec;

typedef struct {
    uint8_t index;
    uint8_t compressorIndex;
    uint8_t furnaceIndex;
    const char* name;
} ThermostatDeviceRec;

#endif
