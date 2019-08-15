#define ARDUINO 101

#include <NewPing.h>
#include <FastLED.h>
#include <LowPower.h>

// Uncomment for debugging functions.
//#define DEBUG

#include "defines.hpp"
#include "globals.hpp"
#include "types.hpp"
#include "led_funcs.hpp"

void onWakeup()
{
}

void setup()
{
    // Setup pins.
    pinMode(LED_BUILTIN,                   OUTPUT);
    pinMode(LED_STRIP_PIN,                 OUTPUT);
    pinMode(MOTION_SENSOR_TOP_ECHO_PING,   INPUT);
    pinMode(MOTION_SENSOR_TOP_TRIGGER_PIN, OUTPUT);

    digitalWrite(LED_STRIP_PIN, 0);

    // Setup serial.
#ifdef DEBUG
    Serial.begin(9600);
    while (!Serial) {;}
#endif

    // Setup FastLED.
    FastLED.addLeds<WS2812B, LED_STRIP_PIN>(Globals::leds, LED_STRIP_LEDS);

    uint8_t stepsDone = 0;
    while(!LED::doCountdown(CRGB(0, 255, 0), 1000, 5, stepsDone)) 
    {
    }

    // Setup sensors.
    calibrateSensor(MOTION_SENSOR_TOP_TRIGGER_PIN, MOTION_SENSOR_TOP_ECHO_PING, MOTION_SENSOR_TOP_INDEX);

    // Setup globals.
    Globals::deltaTimeMS              = 0;
    Globals::motionStates[0]          = false;
    Globals::motionStates[1]          = false;
    Globals::lightState.state         = LightStateState::Off;
    Globals::lightState.stateLastTick = LightStateState::Off;
    Globals::lightState.timer         = 0;
    Globals::lightingFuncs[0]         = &lightingRainbow;
    Globals::lightingFuncs[1]         = &lightingStepping;

#ifdef DEBUG
    Serial.println("Finished Setup");
#endif

    // Make the random seed a bit more random, by reading from a floating analog input.
    pinMode(A0, INPUT);
    for(int i = 0; i < 1000 / 50; i++)
    {
        delay(50);
        Globals::randomSeed += analogRead(A0);
    }

    randomSeed(Globals::randomSeed);
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

void calibrateSensor(int triggerPin, int echoPin, int sensorIndex)
{
#ifdef DEBUG
    Serial.print("Calibrating ");
    Serial.print(triggerPin);
    Serial.print(", ");
    Serial.print(echoPin);
    Serial.print(", ");
    Serial.println(sensorIndex);
#endif

    Globals::sensors[sensorIndex] = NewPing(triggerPin, echoPin);

    const uint8_t TOTAL_STEPS = CALIBRATION_TIME_MS / 1000;
    uint8_t stepsDone = 0;
    Calibration calib;
    calib.normalLower = 0;
    calib.normalHigher = 0;

    while(!LED::doCountdown(CRGB(255, 0, 0), 600, TOTAL_STEPS, stepsDone))
    {
        // Turn LEDs off so more power can go to the sensor (as they're on the same power line).
        LED::setAll(CRGB(0));
        delay(200);

        unsigned long reading = Globals::sensors[sensorIndex].ping_cm();
        Globals::randomSeed += reading; // So we can be a bit more random about things.

        // Re-light the countdown LEDs, and show the debug LEDs.
        for(int i = 0; i < (TOTAL_STEPS - stepsDone); i++)
            Globals::leds[i] = CRGB(255, 0, 0);
        LED::showBinary(TOTAL_STEPS + 1, CRGB(0, 0, 255), (uint16_t)reading);
        delay(200);

        if(reading == NO_ECHO)
        {
            if(stepsDone > 0)
                stepsDone--;

            flushSensor(echoPin);
        }

        if(calib.normalLower == 0 || reading < calib.normalLower)
            calib.normalLower = reading;

        if(reading > calib.normalHigher)
            calib.normalHigher = reading;

#ifdef DEBUG
        Serial.print("READING #");
        Serial.print(stepsDone);
        Serial.print(": R=");
        Serial.print(reading);
        Serial.print(", L=");
        Serial.print(calib.normalLower);
        Serial.print(", H=");
        Serial.println(calib.normalHigher);
#endif
    }
    
    LED::setAll(CRGB(0));
    LED::showBinary(0, CRGB(255, 0, 255), (uint16_t)calib.normalLower);
    LED::showBinary(17, CRGB(0, 0, 255), (uint16_t)calib.normalHigher);
    delay(5000);
    LED::setAll(CRGB(0));

    Globals::calibration[sensorIndex] = calib;

#ifdef DEBUG
    Serial.println("Done Calibrating.");
#endif
}

void flushSensor(int echoPin)
{
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, HIGH);
    delay(20);
    digitalWrite(echoPin, LOW);
    delay(20);
    pinMode(echoPin, INPUT);
    delay(20);
}

void handleSensors()
{
    if(Globals::lightState.state != LightStateState::Off)
        return;

    for(int i = 0; i < MOTION_SENSOR_COUNT; i++)
    {
        unsigned long reading = Globals::sensors[i].ping_cm();

        if(reading == NO_ECHO)
        {
            flushSensor(MOTION_SENSOR_TOP_ECHO_PING);
            Globals::motionStates[i] = false;
            continue;
        }

        Globals::motionStates[i] = (reading < Globals::calibration[i].getTriggerThreshold());

#ifdef DEBUG
        Serial.print("Sensor #");
        Serial.print(i);
        Serial.print(": R=");
        Serial.print(reading);
        Serial.print(", T=");
        Serial.println(Globals::calibration[i].getTriggerThreshold());
#endif
    }
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
    else if(i > 0) // See this as an 'else' statement.
    {
        Serial.print("Unknown command: ");
        Serial.println(buffer);
    }
#endif
}

void onMotionDetected()
{
    if(Globals::lightState.state != LightStateState::Off)
        return;

    Globals::lightState.state = LightStateState::Starting;
    Globals::lightState.timer = LED_STRIP_TIME_ON_MS; // Timer doesn't start until we're in the "On"/"Step" state.
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
    static LightingFunc selectedFunc = nullptr;

    switch(Globals::lightState.state)
    {
        case LightStateState::Off:
            digitalWrite(LED_BUILTIN, LOW);
            LED::setAll(CRGB(0));

            // Sleep (to conserve power) before we check again.
#ifndef DEBUG
            LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
#endif
            break;

        case LightStateState::Starting:
            if(selectedFunc == nullptr)
                selectedFunc = Globals::lightingFuncs[random(0, LIGHTING_FUNCS_COUNT)];

            if(selectedFunc(LightingFuncState::Start)) // True = starting animation is done.
                Globals::lightState.state = LightStateState::On;
            break;

        case LightStateState::On:
            digitalWrite(LED_BUILTIN, HIGH);

            Globals::lightState.timer -= Globals::deltaTimeMS;
            selectedFunc(LightingFuncState::Step);

            if(Globals::lightState.timer <= 0)
                Globals::lightState.state = LightStateState::Ending;
            break;

        case LightStateState::Ending:
            if(selectedFunc(LightingFuncState::End)) // True = starting animation is done.
            {
                selectedFunc = nullptr;
                Globals::lightState.state = LightStateState::Off;
                LED::setAll(CRGB(0));
            }

            break;

        default:
            break;
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

    Serial.println("Sensors: [");
        Serial.print("MOTION_TOP: ");
        Serial.println(Globals::motionStates[MOTION_SENSOR_TOP_INDEX]);
    Serial.println("]");

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
#ifdef DEBUG
    Serial.println("=====START=====");
    doDebugPrint();
#endif

    handleDeltaTime();
    handleSerialCommands();
    handleEvents();
    doStateMachine();
    handleSensors(); // Need to keep this *after* doStateMachine, otherwise the deltatime can be a bit higher than we expect.

#ifdef DEBUG
    Serial.println("=====END=====");
    doDebugPrint();
    Serial.println("=============");
#endif

    delay(CYCLE_DELAY_MS);
}

/* LIGHTING FUNCTIONS */
bool lightingRainbow(LightingFuncState state)
{
    if(state == LightingFuncState::Start)
        return true;

    if(state == LightingFuncState::Step)
    {
        LED::doRainbow();
        return false; // Doesn't matter for this state.
    }

    if(state == LightingFuncState::End)
    {
        LED::doRainbow();
        return LED::doFadeout();
    }

    return false;
}

bool lightingStepping(LightingFuncState state)
{
    if(state == LightingFuncState::Start)
    {
        FastLED.setBrightness(128);
        return LED::doStepdownAnimation(CRGB(128, 128, 128));
    }

    if(state == LightingFuncState::Step)
        return false;

    if(state == LightingFuncState::End)
    {
        bool done = LED::doStepupAnimation(CRGB(0, 0, 0));
        if(done)
            FastLED.setBrightness(255);

        return done;
    }

    return false;
}