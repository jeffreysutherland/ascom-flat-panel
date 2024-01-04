#include <Arduino.h>
#include "../components/relay.h"
#include "../components/motorcontrol.h"

#define POWER_DPIN 2
#define MOTOR_OPEN_APIN 3
#define MOTOR_CLOSE_APIN 4
#define MOTOR_SPEED 127
#define MOTOR_OPENED_LIMIT_DPIN 5
#define MOTOR_CLOSED_LIMIT_DPIN 6
#define SWITCH_DEBOUNCE_TIME 50


Relay powerRelay(POWER_DPIN);
LimitedMotorControl motorControl(MOTOR_OPEN_APIN, MOTOR_CLOSE_APIN, MOTOR_SPEED, MOTOR_OPENED_LIMIT_DPIN, MOTOR_CLOSED_LIMIT_DPIN, SWITCH_DEBOUNCE_TIME);

void setup()
{
    // Initialize serial port I/O.
    Serial.begin(9600);

    while (!Serial)
    {
        ; // Wait for serial port to connect. Required for native USB!
    }
    Serial.flush();

    Serial.println("Serial connected...");

    // set up limit switches
    Serial.println("Setting up motor.");
    motorControl.Initialize();
    Serial.println("Motor set up.");
    
    // turn on power switches
    Serial.println("Turning on power relay...");
    powerRelay.TurnOn();
    Serial.println("Power relay on...");
}

void loop()
{
    motorControl.Update();
}
