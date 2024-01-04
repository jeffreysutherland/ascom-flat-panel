#ifndef RELAY_H
#define RELAY_H


#include <Arduino.h>


enum PowerState
{
    On,
    Off,
};

class Relay
{
public:
    Relay(uint8_t pin) :
        _pin(pin),
        _state(PowerState::Off),
        _initialized(false)
    {

    }

    void Initialize()
    {
        pinMode(_pin, OUTPUT);
        _initialized = true;
    }

    void Set(bool on)
    {
        PowerState state = on ? PowerState::On : PowerState::Off;
        Set(state);
    }

    void Set(PowerState state)
    {
        if (state != _state)
        {
            if (state == PowerState::On)
            {
                TurnOn();
            }
            else if (state == PowerState::Off)
            {
                TurnOff();
            }
        }
    }

    void TurnOn()
    {
        if (!_initialized)
        {
            Initialize();
        }

        _state = PowerState::On;
        digitalWrite(_pin, HIGH);
        Serial.println("Turning on relay...");
    }

    void TurnOff()
    {
        if (!_initialized)
        {
            Initialize();
        }

        _state = PowerState::Off;
        digitalWrite(_pin, LOW);
        Serial.println("Turning off relay...");
    }

    PowerState GetState()
    {
        return _state;
    }

    uint8_t GetPin()
    {
        return _pin;
    }

private:
    uint8_t _pin;
    PowerState _state;
    bool _initialized;
};

#endif // RELAY_H