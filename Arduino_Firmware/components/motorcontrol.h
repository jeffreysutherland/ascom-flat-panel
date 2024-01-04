#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "limitswitch.h"


enum MotionState
{
    Opening,
    Closing,
    Stopped,
    Opened,
    Closed,
};

class MotorControl
{
public:
    MotorControl(uint8_t openPin, uint8_t closePin, uint8_t speed) :
        _openPin(openPin),
        _closePin(closePin),
        _speed(speed),
        _state(MotionState::Stopped)
    {

    }

    void Initialize()
    {
        pinMode(_openPin, OUTPUT);
        pinMode(_closePin, OUTPUT);
        digitalWrite(_openPin, LOW);
        digitalWrite(_closePin, LOW);
    }

    void Move(MotionState state)
    {
        if (state != _state)
        {
            if (state == MotionState::Opening)
            {
                Open();
            }
            else if (state == MotionState::Closing)
            {
                Close();
            }
            else if (state == MotionState::Stopped)
            {
                Stop();
            }
        }
    }

    void Open()
    {
        digitalWrite(_openPin, HIGH);
        digitalWrite(_closePin, LOW);
        _state = MotionState::Opening;
    }

    void Close()
    {
        digitalWrite(_openPin, LOW);
        digitalWrite(_closePin, HIGH);
        _state = MotionState::Closing;
    }

    void Stop()
    {
        digitalWrite(_openPin, LOW);
        digitalWrite(_closePin, LOW);
        _state = MotionState::Stopped;
    }

private:
    uint8_t _openPin;
    uint8_t _closePin;
    uint8_t _speed;
    MotionState _state;
};

class LimitedMotorControl : public MotorControl
{
public:
    LimitedMotorControl(uint8_t openAnalogPin, uint8_t closeAnalogPin, uint8_t speed, uint8_t openedSwitchPin, uint8_t closedSwitchPin, uint8_t debounceTime) :
        MotorControl(openAnalogPin, closeAnalogPin, speed),
        _openedSwitch(openedSwitchPin, debounceTime),
        _closedSwitch(closedSwitchPin, debounceTime)
    {

    }

    void Initialize()
    {
        MotorControl::Initialize();
        _openedSwitch.Initialize();
        _closedSwitch.Initialize();
    }

    void Update()
    {
        _openedSwitch.Update();
        _closedSwitch.Update();

        if (_openedSwitch.IsPressed())
        {
            Stop();
            _state = MotionState::Opened;

        }
        else if (_closedSwitch.IsPressed())
        {
            Stop();
            _state = MotionState::Closed;
        }
    }

    MotionState GetState()
    {
        return _state;
    }

    void Stop()
    {
        MotorControl::Stop();
        _state = MotionState::Stopped;
    }

    void Open()
    {
        if (_state != MotionState::Opened && _state != MotionState::Opening)
        {
            MotorControl::Open();
            _state = MotionState::Opening;
        }
    }

    void Close()
    {
        if (_state != MotionState::Closed && _state != MotionState::Closing)
        {
            MotorControl::Close();
            _state = MotionState::Closing;
        }
    }

private:
    LimitSwitch _openedSwitch;
    LimitSwitch _closedSwitch;
    MotionState _state;
};

#endif