#ifndef GLOBALS_H
#define GLOBALS_H

#include <FastLED.h>
#include "defines.hpp"
#include "types.hpp"

enum class LightStateState
{
    Off,
    Resting, // Letting the timer tick down until lighting is to start again.

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

namespace Globals {
    extern CRGB                      leds[LED_STRIP_LEDS];
    extern unsigned long             deltaTimeMS; // Measured in ms
    extern LightStateInfo            lightState;
    extern LightingFunc              lightingFuncs[LIGHTING_FUNCS_COUNT];
    extern unsigned long             randomSeed; // Will only be used at *The end of the setup function*
};

#endif