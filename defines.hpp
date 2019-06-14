#ifndef DEFINES_INO
#define DEFINES_INO

/**** START Motion Sensor info ****/
#define MOTION_SENSOR_TOP_INDEX    0
#define MOTION_SENSOR_BOTTOM_INDEX 1

#define MOTION_SENSOR_TOP_PIN      2
#define MOTION_SENSOR_BOTTOM_PIN   2

#define MOTION_SENSOR_COUNT        2
/**** END Motion Sensor info ****/

/**** START LED strip info ****/
#define LED_STRIP_PIN        7 // The data pin.
#define LED_STRIP_LEDS       148 // How many LEDS we're using.
#define LED_STRIP_TIME_ON_MS 1000 * 15 // How long the strips stay on when triggered.
#define LED_STRIP_COLOUR     CRGB(0x88888800)
/**** END LED strip info ****/

/**** START Misc info ****/
#define SERIAL_COMMAND_BUFFER_SIZE 32
#define CYCLE_DELAY_MS             50
#define RAINBOW_DELTA_HUE          5
/**** END Misc info ****/

#endif