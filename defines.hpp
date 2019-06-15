#ifndef DEFINES_INO
#define DEFINES_INO

/**** START Motion Sensor info ****/
#define MOTION_SENSOR_TOP_INDEX    0

#define MOTION_SENSOR_TOP_TRIGGER_PIN 12
#define MOTION_SENSOR_TOP_ECHO_PING   11

#define MOTION_SENSOR_COUNT        1
#define CALIBRATION_TIME_MS        1000 * 15
/**** END Motion Sensor info ****/

/**** START LED strip info ****/
#define LED_STRIP_PIN        7 // The data pin.
#define LED_STRIP_TIME_ON_MS 1000 * 15 // How long the strips stay on when triggered.
#define LED_STRIP_COLOUR     CRGB(0x88888800)

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