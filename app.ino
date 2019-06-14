#include <FastLED.h>

#include "defines.hpp"
#include "globals.hpp"
#include "types.hpp"
#include "led_funcs.hpp"

// Uncomment for debugging functions.
//#define DEBUG

void setup()
{
    // Setup pins.
	pinMode(LED_STRIP_PIN,            OUTPUT);
    pinMode(MOTION_SENSOR_BOTTOM_PIN, INPUT);
    pinMode(MOTION_SENSOR_TOP_PIN,    INPUT);

    digitalWrite(LED_STRIP_PIN, 0);

    // Setup serial.
#ifdef DEBUG
    Serial.begin(9600);
    while (!Serial) {;}
#endif

    // Setup FastLED.
    FastLED.addLeds<WS2812B, LED_STRIP_PIN>(Globals::leds, LED_STRIP_LEDS);

    // Setup globals.
    Globals::deltaTimeMS              = 0;
    Globals::motionStates[0]          = false;
    Globals::motionStates[1]          = false;
    Globals::lightState.state         = LightStateState::Off;
    Globals::lightState.stateLastTick = LightStateState::Off;
    Globals::lightState.timer         = 0;
}

void handleDeltaTime()
{
    static unsigned int prevTime = 0;
    unsigned int currTime = millis();

    // Handle when it overflows, though it'll probably need a battery change before then.
    if(currTime < prevTime)
        prevTime = 0;

    Globals::deltaTimeMS = currTime - prevTime;
    prevTime = currTime;
}

void handleSensors()
{
    Globals::motionStates[MOTION_SENSOR_BOTTOM_INDEX] = (digitalRead(MOTION_SENSOR_BOTTOM_PIN) > 0);
    Globals::motionStates[MOTION_SENSOR_TOP_INDEX]    = (digitalRead(MOTION_SENSOR_TOP_PIN) > 0);
}

// Serial commands are used for debugging.
void handleSerialCommands()
{
#ifdef DEBUG
    char buffer[SERIAL_COMMAND_BUFFER_SIZE];
    
    // Read in the command.
    uint8_t i = 0;
    while(Serial.available()) 
    {
        buffer[i++] = Serial.read();

        // If the command is too long, read in the rest of the buffer, then don't do anything.
        if(i >= SERIAL_COMMAND_BUFFER_SIZE)
        {
            while(Serial.available()) Serial.read();
            Serial.println("Command exceeds length limit");
            return;
        }
    }
    buffer[i] = '\0';

    // Execute it
    if(strcmp(buffer, "settop") == 0)
        Globals::motionStates[MOTION_SENSOR_TOP_INDEX] = true;
    else if(strcmp(buffer, "setbot") == 0)
        Globals::motionStates[MOTION_SENSOR_BOTTOM_INDEX] = true;
    else if(i > 0) // See this as an 'else' statement.
    {
        Serial.print("Unknown command: ");
        Serial.println(buffer);
    }
#endif
}

void onMotionDetected()
{
    Globals::lightState.stateLastTick = Globals::lightState.state;
    Globals::lightState.state         = LightStateState::On;
    Globals::lightState.timer         = LED_STRIP_TIME_ON_MS;
}

void handleEvents()
{
    for(uint8_t i = 0; i < MOTION_SENSOR_COUNT; i++)
    {
        if(Globals::motionStates[i])
        {
            onMotionDetected();
            return;
        }
    }
}

void doStateMachine()
{
    if(Globals::lightState.state == LightStateState::Off)
    {
        digitalWrite(LED_BUILTIN, LOW);
        if(Globals::lightState.stateLastTick == LightStateState::On)
            LED::setAll(CRGB(0));

        return;
    }

    if(Globals::lightState.state == LightStateState::On)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        if(Globals::lightState.stateLastTick == LightStateState::Off)
            LED::setAll(CRGB(128, 128, 128));

        Globals::lightState.timer -= Globals::deltaTimeMS;

        if(Globals::lightState.timer <= 0)
        {
            Globals::lightState.stateLastTick = Globals::lightState.state;
            Globals::lightState.state         = LightStateState::Off;
        }
    }
}

void doDebugPrint()
{
#ifdef DEBUG
    Serial.println("LightState");
    Serial.println("{");
        Serial.print("\tState: ");
        Serial.println((int)Globals::lightState.state);

        Serial.print("\tLastState: ");
        Serial.println((int)Globals::lightState.stateLastTick);

        Serial.print("\tTimer: ");
        Serial.println(Globals::lightState.timer);
    Serial.println("}");

    Serial.print("LEDs: [");
        for(uint16_t i = 0; i < LED_STRIP_LEDS; i++)
        {
            Serial.print("(");
                Serial.print(Globals::leds[i].r);
                Serial.print(",");

                Serial.print(Globals::leds[i].g);
                Serial.print(",");

                Serial.print(Globals::leds[i].b);
            Serial.print("), ");
        }
    Serial.println("]");
#endif
}

void loop()
{
    //Serial.println(digitalRead(MOTION_SENSOR_TOP_PIN));

#ifdef DEBUG
    Serial.println("=====START=====");
    doDebugPrint();
#endif

    handleDeltaTime();
    handleSensors();
    handleSerialCommands();
    handleEvents();
    //doStateMachine();

    LED::doRainbow();

#ifdef DEBUG
    Serial.println("=====END=====");
    doDebugPrint();
    Serial.println("=============");
#endif

    delay(CYCLE_DELAY_MS);
}
