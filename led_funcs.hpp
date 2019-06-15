#ifndef LED_FUNCS_H
#define LED_FUNCS_H

#include <FastLED.h>
#include "globals.hpp"

namespace LED 
{
    void setAll(CRGB colour);
    void doRainbow();
    bool doFadeout();
    bool doStepdownAnimation(CRGB);
    bool doStepupAnimation(CRGB);
    bool doCountdown(CRGB colour, uint16_t delayPerStep, uint8_t steps, uint8_t& stepsDone);

    template <typename NumT>
    void showBinary(uint16_t startLED, CRGB colour, NumT num)
    {
        const int NumTBits = sizeof(NumT) * 8;
        for(int bit = 0; bit < NumTBits; bit++)
            Globals::leds[startLED + bit] = (num & ((1 << (NumTBits - 1)) >> bit)) > 0 ? colour : CRGB(0);
        Globals::leds[startLED + NumTBits] = CRGB(255, 0, 0);
        FastLED.show();
    }
};

#endif