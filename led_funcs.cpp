#include "led_funcs.hpp"
#include "globals.hpp"

namespace LED 
{
    void setAll(CRGB colour)
    {
        for(uint16_t i = 0; i < LED_STRIP_LEDS; i++)
            Globals::leds[i] = colour;
            
        FastLED.show();
    }

    void doRainbow()
    {
        static uint8_t start = 0;

        fill_rainbow(Globals::leds, LED_STRIP_LEDS, start, RAINBOW_DELTA_HUE);
        FastLED.show();

        start += RAINBOW_DELTA_HUE;
    }
};