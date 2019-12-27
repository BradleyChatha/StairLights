#ifndef DEFINES_INO
#define DEFINES_INO

/**** START LED strip info ****/
#define LED_STRIP_PIN        A0 // The data pin.
#define LED_STRIP_TIME_ON_MS 1000 * 15 // How long the strips stay on when triggered.
#define LED_STRIP_REST_MS    1000 * 30

#ifndef DEBUG
    #define LED_STRIP_LEDS 148 // How many LEDS we're using.
#else
    #define LED_STRIP_LEDS 5
#endif
/**** END LED strip info ****/

/**** START Misc info ****/
#define LIGHTING_FUNCS_COUNT       2
#define SERIAL_COMMAND_BUFFER_SIZE 32

#ifndef DEBUG
    #define CYCLE_DELAY_MS 50
#else
    #define CYCLE_DELAY_MS 1000
#endif

#define RAINBOW_DELTA_HUE          5

#define FADEOUT_STEP               5
#define FADEOUT_DELAY_MS           100
/**** END Misc info ****/

#endif