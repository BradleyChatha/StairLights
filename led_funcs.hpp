#ifndef LED_FUNCS_H
#define LED_FUNCS_H

#include <FastLED.h>

namespace LED 
{
    void setAll(CRGB colour);
    void doRainbow();
};

#endif