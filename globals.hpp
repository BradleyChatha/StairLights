#ifndef GLOBALS_H
#define GLOBALS_H

#include <FastLED.h>
#include <NewPing.h>
#include "defines.hpp"
#include "types.hpp"

enum class LightStateState
{
    Off,

    Starting, // Letting a lighting func perform it's starting animation.
    On,
    Ending // Letting a lighting func perform it's ending animation.
};

struct LightStateInfo
{
    LightStateState state;
    LightStateState stateLastTick;
    int16_t timer;
};

struct Calibration
{
    unsigned long normalLower;
    unsigned long normalHigher;

    Calibration()
    {
        normalLower = 0;
        normalHigher = 0;
    }

    unsigned long getDelta()
    {
        return normalHigher - normalLower;
    }

    unsigned long getTriggerThreshold()
    {
        return normalLower;
    }
};

namespace Globals {
    extern bool                      motionStates[MOTION_SENSOR_COUNT]; // 0 = Not triggered. 1 = Triggered.
    extern Calibration               calibration[MOTION_SENSOR_COUNT];
    extern NewPing                   sensors[MOTION_SENSOR_COUNT];
    extern CRGB                      leds[LED_STRIP_LEDS];
    extern unsigned long             deltaTimeMS; // Measured in ms
    extern LightStateInfo            lightState;
    extern LightingFunc              lightingFuncs[LIGHTING_FUNCS_COUNT];
};

#endif