#ifndef GLOBALS_H
#define GLOBALS_H

#include <FastLED.h>
#include "defines.hpp"

enum class LightStateState
{
    Off,
    On
};

struct LightStateInfo
{
    LightStateState state;
    LightStateState stateLastTick;
    int16_t timer;
};

namespace Globals {
    extern bool           motionStates[MOTION_SENSOR_COUNT]; // 0 = Not triggered. 1 = Triggered.
    extern CRGB           leds[LED_STRIP_LEDS];
    extern unsigned long  deltaTimeMS; // Measured in ms
    extern LightStateInfo lightState;
};

#endif