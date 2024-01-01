#include <Arduino.h>

void setup()
{
    Serial.begin(9600);

    while (!Serial)
    {
        // Wait for serial port to connect.
    }

    Serial.println("Connected.");
}

void loop()
{
    Serial.println("Hello, world!");
    delay(1000);
}