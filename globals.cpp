#include "globals.hpp"

bool Globals::motionStates[MOTION_SENSOR_COUNT];
CRGB Globals::leds[LED_STRIP_LEDS];
LightingFunc Globals::lightingFuncs[LIGHTING_FUNCS_COUNT];
Calibration Globals::calibration[MOTION_SENSOR_COUNT];
NewPing Globals::sensors[MOTION_SENSOR_COUNT] = { NewPing(0, 0) };
unsigned long Globals::deltaTimeMS;
LightStateInfo Globals::lightState;
unsigned long Globals::randomSeed;