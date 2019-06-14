#include "globals.hpp"

bool Globals::motionStates[MOTION_SENSOR_COUNT];
CRGB Globals::leds[LED_STRIP_LEDS];
unsigned long Globals::deltaTimeMS;
LightStateInfo Globals::lightState;