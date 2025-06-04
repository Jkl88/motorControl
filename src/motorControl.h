#pragma once
#include <Arduino.h>

#define MIN_PWM -255
#define MAX_PWM 255
#define PULSES_PER_REVOLUTION 60
#define MAX_MOTORS 6

class Motor {
public:
    Motor(uint8_t hallApin, uint8_t hallBpin, uint8_t pwmPin, uint8_t pwmCannel, uint8_t minPwm,
        uint8_t dirPin, uint8_t brakePin, uint16_t rampTime = 200) {
        _hallApin = hallApin;
        _hallBpin = hallBpin;
        _pwmPin = pwmPin;
        _pwmCannel = pwmCannel;
        _minPwm = minPwm;
        _dirPin = dirPin;
        _brakePin = brakePin;
        _rampTime = rampTime;
        _targetPwm = 0;
        _currentPwm = 0;
        _targetBrake = false;
        _lastAppliedDirection = 0;
        _brakeActive = false;
        _directionChangePending = false;
        _rampIncrement = 5;
    }

    void begin() {
        pinMode(_hallApin, INPUT_PULLUP);
        pinMode(_hallBpin, INPUT_PULLUP);
        pinMode(_dirPin, OUTPUT);
        pinMode(_brakePin, OUTPUT);
        pinMode(_pwmPin, OUTPUT);

        digitalWrite(_brakePin, HIGH);
        digitalWrite(_dirPin, LOW);

        ledcSetup(_pwmCannel, 5000, 8);
        ledcAttachPin(_pwmPin, _pwmCannel);
        ledcWrite(_pwmCannel, 0);

        _lastHallState = (digitalRead(_hallApin) << 1) | digitalRead(_hallBpin);

        for (int i = 0; i < MAX_MOTORS; i++) {
            if (_instances[i] == nullptr) {
                _instances[i] = this;
                attachInterrupt(digitalPinToInterrupt(_hallApin), _isrRouters[i], CHANGE);
                attachInterrupt(digitalPinToInterrupt(_hallBpin), _isrRouters[i], CHANGE);
                break;
            }
        }

        _lastUpdateTime = millis();
        _lastRampTime = millis();
        _encoderCount = 0;
        _lastCount = 0;
        _currentRPM = 0.0;
        _currentDirection = 0;
    }

    void setPwm(int pwm, bool brake = false) {
        pwm = constrain(pwm, MIN_PWM, MAX_PWM);
        _targetPwm = pwm;
        _targetBrake = brake;

        if (_rampTime > 0) {
            _rampIncrement = max(1, abs(_targetPwm - _currentPwm) * 20 / _rampTime);
        }
        else {
            _rampIncrement = MAX_PWM + 1;
        }
    }

    void setRampTime(uint16_t rampTime) {
        _rampTime = rampTime;
    }

    void update() {
        unsigned long now = millis();
        unsigned long dt = now - _lastUpdateTime;

        if (dt >= 100) {
            long currentCount = _encoderCount;
            long deltaCount = currentCount - _lastCount;

            if (dt > 0) {
                _currentRPM = (deltaCount * 60000.0) / (PULSES_PER_REVOLUTION * dt);

                if (deltaCount > 0) {
                    _currentDirection = 1;
                }
                else if (deltaCount < 0) {
                    _currentDirection = -1;
                }
                else {
                    _currentDirection = 0;
                }
            }

            _lastCount = currentCount;
            _lastUpdateTime = now;
        }

        applyMotorControl();
    }

    float getRPM() { return _currentRPM; }
    int getDirection() { return _currentDirection; }

private:
    uint8_t _hallApin, _hallBpin, _pwmPin, _pwmCannel, _minPwm, _dirPin, _brakePin;
    int _targetPwm, _currentPwm;
    bool _targetBrake, _brakeActive, _directionChangePending;
    int _lastAppliedDirection, _currentDirection, _lastDirection;
    uint16_t _rampTime, _rampIncrement;
    unsigned long _lastRampTime;

    volatile long _encoderCount;
    long _lastCount;
    uint8_t _lastHallState;
    float _currentRPM;
    unsigned long _lastUpdateTime;

    static constexpr int8_t _encoderStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
    static Motor* _instances[MAX_MOTORS];

    typedef void (*isr_func_t)();
    static void IRAM_ATTR _isrRouter0() { if (_instances[0]) _instances[0]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter1() { if (_instances[1]) _instances[1]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter2() { if (_instances[2]) _instances[2]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter3() { if (_instances[3]) _instances[3]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter4() { if (_instances[4]) _instances[4]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter5() { if (_instances[5]) _instances[5]->handleInterrupt(); }

    static constexpr isr_func_t _isrRouters[MAX_MOTORS] = {
        _isrRouter0,
        _isrRouter1,
        _isrRouter2,
        _isrRouter3,
        _isrRouter4,
        _isrRouter5
    };

    void applyMotorControl() {
        if (_targetBrake) {
            digitalWrite(_brakePin, LOW);
            ledcWrite(_pwmCannel, 0);
            _currentPwm = 0;
            _brakeActive = true;
            _directionChangePending = false;
            return;
        }
        else {
            digitalWrite(_brakePin, HIGH);
            _brakeActive = false;
        }

        int targetDir = (_targetPwm == 0) ? 0 : (_targetPwm > 0 ? 1 : -1);

        if (targetDir != 0 && targetDir != _currentDirection) {
            if (abs(_currentRPM) > 5.0) {
                digitalWrite(_brakePin, LOW);
                _directionChangePending = true;
                ledcWrite(_pwmCannel, 0);
                _currentPwm = 0;
                return;
            }
            else {
                _directionChangePending = false;
            }
        }

        if (millis() - _lastRampTime > 20) {
            if (_currentPwm < _targetPwm) {
                _currentPwm = min(_targetPwm, _currentPwm + _rampIncrement);
            }
            else if (_currentPwm > _targetPwm) {
                _currentPwm = max(_targetPwm, _currentPwm - _rampIncrement);
            }
            _lastRampTime = millis();
        }

        applyPWM();
    }

    void applyPWM() {
        int pwmValue = map(abs(_currentPwm), 0, MAX_PWM, _minPwm, MAX_PWM);
        int targetDir = (_currentPwm == 0) ? 0 : (_currentPwm > 0 ? 1 : -1);

        if (targetDir == 1) {
            digitalWrite(_dirPin, HIGH);
            ledcWrite(_pwmCannel, pwmValue);
        }
        else if (targetDir == -1) {
            digitalWrite(_dirPin, LOW);
            ledcWrite(_pwmCannel, pwmValue);
        }
        else {
            ledcWrite(_pwmCannel, 0);
        }

        _lastAppliedDirection = targetDir;
    }

    void handleInterrupt() {
        uint8_t a = digitalRead(_hallApin);
        uint8_t b = digitalRead(_hallBpin);
        uint8_t newState = (a << 1) | b;

        uint8_t state = (_lastHallState << 2) | newState;
        int8_t change = _encoderStates[state & 0x0F];

        _encoderCount += change;
        if (change != 0) {
            _lastDirection = (change > 0) ? 1 : -1;
        }

        _lastHallState = newState;
    }
};

Motor* Motor::_instances[MAX_MOTORS] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
