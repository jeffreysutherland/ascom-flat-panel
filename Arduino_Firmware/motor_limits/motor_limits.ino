#include <Arduino.h>

#define POWER_PIN 2
#define MOTOR_OPEN_PIN 3
#define MOTOR_CLOSE_PIN 4
#define MOTOR_OPENED_LIMIT_PIN 5
#define MOTOR_CLOSED_LIMIT_PIN 6
#define SWITCH_DEBOUNCE_TIME 50

#define MOTOR_SPEED 127

enum MotionState
{
    Opening,
    Closing,
    Stopped,
};

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

Relay powerRelay(POWER_PIN);
LimitSwitch openedLimitSwitch(MOTOR_OPENED_LIMIT_PIN, SWITCH_DEBOUNCE_TIME);
LimitSwitch closedLimitSwitch(MOTOR_CLOSED_LIMIT_PIN, SWITCH_DEBOUNCE_TIME);

MotionState motionState = Stopped;

unsigned long motionWaitTime;
unsigned long powerWaitTime;

void TestCyclePower();
void TestCycleMotor();

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
    Serial.println("Setting up limit switches...");
    openedLimitSwitch.Initialize();
    closedLimitSwitch.Initialize();
    Serial.println("Limit switches set up...");
    
    // turn on power switches
    Serial.println("Turning on power relay...");
    powerRelay.TurnOn();
    Serial.println("Power relay on...");

    // set up motor pins
    Serial.println("Setting up motor pins...");
    pinMode(MOTOR_OPEN_PIN, OUTPUT);
    pinMode(MOTOR_CLOSE_PIN, OUTPUT);
    digitalWrite(MOTOR_OPEN_PIN, LOW);
    digitalWrite(MOTOR_CLOSE_PIN, LOW);
    Serial.println("Motor pins set up...");

    // set the wait time
    motionWaitTime = millis() + 1000;
    powerWaitTime = millis() + 1000;
}


void loop()
{
    // update the limit switches
    openedLimitSwitch.Update();
    closedLimitSwitch.Update();

    powerRelay.Set(openedLimitSwitch.IsPressed());

    //TestCyclePower();
    //TestCycleMotor();
}
/*
void TestCyclePower()
{
    PowerState targPowerState = powerState;

    if (millis() > powerWaitTime)
    {
        if (powerState == On)
        {
            targPowerState = Off;
        }
        else if (powerState == Off)
        {
            targPowerState = On;
        }

        powerWaitTime = millis() + 500;
    }

    // update the power state
    if (targPowerState != powerState)
    {
        if (targPowerState == On)
        {
            // turn on
            Serial.println("Turning on power relay...");
            digitalWrite(POWER_PIN, HIGH);
        }
        else if (targPowerState == Off)
        {
            // turn off
            Serial.println("Turning off power relay...");
            digitalWrite(POWER_PIN, LOW);
        }
        
        powerState = targPowerState;
    }
}

void TestCycleMotor()
{
        MotionState targState = motionState;

    // temp test direction changing code
    if (millis() > motionWaitTime)
    {
        if (motionState == Stopped)
        {
            if (openedLimitSwitch.isPressed())
            {
                targState = Closing;
            }
            else if (closedLimitSwitch.isPressed())
            {
                targState = Opening;
            }
            else
            {
                targState = Opening;
            }
        }
        else if (motionState == Opening)
        {
            targState = Closing;
        }
        else if (motionState == Closing)
        {
            targState = Stopped;
        }

        motionWaitTime = millis() + 1000;
    }

    // check the limit switches
    if (openedLimitSwitch.isPressed())
    {
        Serial.println("Open limit switch reached");
        targState = Stopped;
    }
    else if (closedLimitSwitch.isPressed())
    {
        Serial.println("Closed limit switch reached");
        targState = Stopped;
    }

    // update the motor state
    if (targState != motionState)
    {
        if (targState == Opening)
        {
            // start opening
            Serial.println("Opening...");
            digitalWrite(MOTOR_OPEN_PIN, HIGH);
            digitalWrite(MOTOR_CLOSE_PIN, LOW);
        }
        else if (targState == Closing)
        {
            // start closing
            Serial.println("Closing...");
            digitalWrite(MOTOR_OPEN_PIN, LOW);
            digitalWrite(MOTOR_CLOSE_PIN, HIGH);
        }
        else if (targState == Stopped)
        {
            // stop
            Serial.println("Stopping...");
            digitalWrite(MOTOR_OPEN_PIN, LOW);
            digitalWrite(MOTOR_CLOSE_PIN, LOW);
        }
        
        motionState = targState;
    }
}*/