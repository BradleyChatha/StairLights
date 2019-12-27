#include "globals.hpp"

CRGB Globals::leds[LED_STRIP_LEDS];
LightingFunc Globals::lightingFuncs[LIGHTING_FUNCS_COUNT];
unsigned long Globals::deltaTimeMS;
LightStateInfo Globals::lightState;
unsigned long Globals::randomSeed;