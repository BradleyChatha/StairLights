#include "led_funcs.hpp"
#include "globals.hpp"
#include <Arduino.h>

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

    // Returns: True if the fadeout has ended. False if it's still going.
    bool doFadeout()
    {
        static int16_t brightness = 255;

        FastLED.setBrightness(brightness);
        FastLED.show();

        delay(FADEOUT_DELAY_MS);

        brightness -= FADEOUT_STEP;
        if(brightness <= 0)
        {
            FastLED.setBrightness(0);
            FastLED.show();

            brightness = 255;
            FastLED.setBrightness(brightness);
            return true;
        }

        return false;
    }

    bool doStepdownAnimation(CRGB colour)
    {
        static uint8_t index = 0;

        Globals::leds[index++] = colour;
        FastLED.show();

        #ifdef DEBUG
        Serial.println("ANIMATION: STEPDOWN");
        Serial.print("LED: ");
        Serial.print(index);
        Serial.print("/");
        Serial.println(LED_STRIP_LEDS);
        #endif

        if(index >= LED_STRIP_LEDS)
        {
            index = 0;
            return true;
        }

        return false;
    }
    
    // Eww, duplication. But who cares for this.
    bool doStepupAnimation(CRGB colour)
    {
        static int16_t index = LED_STRIP_LEDS;

        Globals::leds[--index] = colour;
        FastLED.show();

        if(index < 0)
        {
            index = LED_STRIP_LEDS;
            return true;
        }

        return false;
    }

    bool doCountdown(CRGB colour, uint16_t delayPerStep, uint8_t steps, uint8_t& stepsDone)
    {
        static uint8_t index = 255;
        const uint8_t LEDS_TO_USE = steps;

        // At the start of a countdown.
        if(index == 255)
        {
            for(index = 0; index < LEDS_TO_USE; index++)
            {
                if(index < LED_STRIP_LEDS)
                    Globals::leds[index] = colour;
            }

            index--;
            FastLED.show();
        }

        delay(delayPerStep);
        stepsDone++;

        if(index < LED_STRIP_LEDS)
        {
            Globals::leds[index] = CRGB(0);
            FastLED.show();
        }
        index--;

        if(stepsDone >= steps)
        {
            index = 255;
            return true;
        }

        return false;
    }
};