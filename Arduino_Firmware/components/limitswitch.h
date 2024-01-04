#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

#include <Arduino.h>

class LimitSwitch
{
public:
    enum State
    {
        Open,
        Closed,
        Unknown,
    };

    LimitSwitch(uint8_t pin, unsigned long debounceDuration) :
        _pin(pin),
        _lastState(State::Unknown),
        _debounceDuration(debounceDuration),
        _lastBounceTime(0)
    {

    }

    void Initialize()
    {
        pinMode(_pin, INPUT_PULLUP);
        _lastBounceTime = millis();
        _lastState = GetRawState();
        _state = _lastState;
    }

    bool IsPressed()
    {
        return _state == State::Open;
    }

    void Update()
    {
        State currentState = GetRawState();
        if (currentState != _lastState)
        {
            // state switched, reset the debounce timer
            _lastState = currentState;
            _lastBounceTime = millis();
            return;
        }

        if (currentState != _state && (millis() - _lastBounceTime > _debounceDuration))
        {
            // state is stable
            _state = currentState;
            Serial.println(_state == State::Open ? "Switch Open" : "Switch Closed");
        }
    }

private:
    State GetRawState()
    {
       return digitalRead(_pin) ? State::Open : State::Closed;
    }

    int _pin;
    State _state;
    State _lastState;
    unsigned long _debounceDuration;
    unsigned long _lastBounceTime;
};

#endif