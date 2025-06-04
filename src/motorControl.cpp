/*
* v1.0.3
*/

#include "motorControl.h"

const int8_t Motor::_encoderStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

Motor::Motor(uint8_t hallApin, uint8_t hallBpin, uint8_t pwmPin, uint8_t pwmCannel, uint8_t minPwm, uint8_t dirPin, uint8_t brakePin, uint16_t rampTime) {
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
    _instanceA = nullptr;
    _instanceB = nullptr;
}

void Motor::begin() {
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

    attachInterruptArg(digitalPinToInterrupt(_hallApin), _handleInterruptA, this);
    attachInterruptArg(digitalPinToInterrupt(_hallBpin), _handleInterruptB, this);

    _lastUpdateTime = millis();
    _lastRampTime = millis();
    _encoderCount = 0;
    _lastCount = 0;
    _currentRPM = 0.0;
    _currentDirection = 0;
}

void Motor::setPwm(int pwm, bool brake) {
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

void Motor::setRampTime(uint16_t rampTime) {
    _rampTime = rampTime;
}

void Motor::applyMotorControl() {
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

void Motor::applyPWM() {
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

void IRAM_ATTR Motor::_handleInterruptA(void* arg) {
    Motor* instance = static_cast<Motor*>(arg);
    static unsigned long lastInterruptTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastInterruptTime > 2) {
        instance->handleInterrupt();
    }
    lastInterruptTime = currentTime;
}

void IRAM_ATTR Motor::_handleInterruptB(void* arg) {
    Motor* instance = static_cast<Motor*>(arg);
    static unsigned long lastInterruptTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastInterruptTime > 2) {
        instance->handleInterrupt();
    }
    lastInterruptTime = currentTime;
}

void Motor::handleInterrupt() {
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

void Motor::update() {
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

float Motor::getRPM() {
    return _currentRPM;
}

int Motor::getDirection() {
    return _currentDirection;
}
