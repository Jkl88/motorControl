#pragma once
#include <Arduino.h>

#define MIN_PWM -255
#define MAX_PWM 255
#define PULSES_PER_REVOLUTION 60
#define MAX_MOTORS 6

// Абстрактный базовый класс для двигателя
class BaseMotor {
public:
    virtual void begin() = 0;
    virtual void setPwm(int pwm, bool brake = false) = 0;
    virtual void update() = 0;
    virtual float getRPM() = 0;
    virtual int getDirection() = 0;
    virtual ~BaseMotor() = default;
};

// Универсальный класс управления двигателем с одним или двумя датчиками Холла
class HallMotor : public BaseMotor {
public:
    HallMotor(uint8_t hallApin, uint8_t hallBpin, uint8_t pwmPin, uint8_t pwmCannel, uint8_t minPwm,
        uint8_t dirPin, uint8_t brakePin, uint16_t rampTime = 200, uint16_t dirChangeDelay = 200)
        : _hallApin(hallApin), _hallBpin(hallBpin), _pwmPin(pwmPin),
        _pwmCannel(pwmCannel), _minPwm(minPwm), _dirPin(dirPin), _brakePin(brakePin),
        _rampTime(rampTime), _dirChangeDelay(dirChangeDelay),
        _targetPwm(0), _currentPwm(0), _targetBrake(false),
        _lastAppliedDirection(0), _brakeActive(false), _directionChangePending(false),
        _rampIncrement(5), _lastCount(0), _currentRPM(0.0), _lastUpdateTime(0),
        _encoderCount(0), _lastHallState(0), _currentDirection(0), _lastDirection(1),
        _directionChangeTime(0), _waitingForStop(false) {

        _singleHall = (hallBpin == 255);  // Режим одного датчика
    }

    void begin() override {
        pinMode(_hallApin, INPUT_PULLUP);
        if (!_singleHall) pinMode(_hallBpin, INPUT_PULLUP);
        pinMode(_dirPin, OUTPUT);
        pinMode(_brakePin, OUTPUT);
        pinMode(_pwmPin, OUTPUT);

        digitalWrite(_brakePin, HIGH);
        digitalWrite(_dirPin, LOW);

        ledcSetup(_pwmCannel, 5000, 8);
        ledcAttachPin(_pwmPin, _pwmCannel);
        ledcWrite(_pwmCannel, 0);

        _lastHallState = (digitalRead(_hallApin) << 1) | (_singleHall ? 0 : digitalRead(_hallBpin));

        for (int i = 0; i < MAX_MOTORS; i++) {
            if (_instances[i] == nullptr) {
                _instances[i] = this;
                attachInterrupt(digitalPinToInterrupt(_hallApin), _isrRouters[i], CHANGE);
                if (!_singleHall) attachInterrupt(digitalPinToInterrupt(_hallBpin), _isrRouters[i], CHANGE);
                break;
            }
        }

        _lastUpdateTime = millis();
        _lastRampTime = millis();
        _encoderCount = 0;
    }

    void setPwm(int pwm, bool brake = false) override {
        pwm = constrain(pwm, MIN_PWM, MAX_PWM);
        _targetPwm = pwm;
        _targetBrake = brake;

        if (_rampTime > 0) {
            _rampIncrement = max(1, abs(_targetPwm - _currentPwm) * 20 / _rampTime);
        }
        else {
            _rampIncrement = MAX_PWM + 1;
        }

        log_d("[setPwm] Target PWM: %d, Brake: %d", _targetPwm, _targetBrake);
    }

    void setRampTime(uint16_t rampTime) { _rampTime = rampTime; }
    void setMinPwm(uint8_t minPwm) { _minPwm = minPwm; }
    void setDirChangeDelay(uint16_t delay) { _dirChangeDelay = delay; }

    void update() override {
        unsigned long now = millis();
        unsigned long dt = now - _lastUpdateTime;

        if (dt >= 100) {
            long currentCount = _encoderCount;
            long deltaCount = currentCount - _lastCount;

            if (dt > 0) {
                _currentRPM = (deltaCount * 60000.0) / (PULSES_PER_REVOLUTION * dt);
                _currentDirection = (deltaCount > 0) ? 1 : (deltaCount < 0 ? -1 : 0);
            }

            log_d("[update] RPM: %.2f, Direction: %d", _currentRPM, _currentDirection);
            _lastCount = currentCount;
            _lastUpdateTime = now;
        }

        applyMotorControl();
    }

    float getRPM() override { return _currentRPM; }
    int getDirection() override { return _currentDirection; }

private:
    bool _singleHall;
    bool _waitingForStop;
    uint8_t _hallApin, _hallBpin, _pwmPin, _pwmCannel, _minPwm, _dirPin, _brakePin;
    int _targetPwm, _currentPwm;
    bool _targetBrake, _brakeActive, _directionChangePending;
    int _lastAppliedDirection, _currentDirection, _lastDirection;
    uint16_t _rampTime, _rampIncrement, _dirChangeDelay;
    unsigned long _lastRampTime, _directionChangeTime;

    volatile long _encoderCount;
    long _lastCount;
    uint8_t _lastHallState;
    float _currentRPM;
    unsigned long _lastUpdateTime;

    static constexpr int8_t _encoderStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
    static HallMotor* _instances[MAX_MOTORS];

    typedef void (*isr_func_t)();
    static void IRAM_ATTR _isrRouter0() { if (_instances[0]) _instances[0]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter1() { if (_instances[1]) _instances[1]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter2() { if (_instances[2]) _instances[2]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter3() { if (_instances[3]) _instances[3]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter4() { if (_instances[4]) _instances[4]->handleInterrupt(); }
    static void IRAM_ATTR _isrRouter5() { if (_instances[5]) _instances[5]->handleInterrupt(); }

    static constexpr isr_func_t _isrRouters[MAX_MOTORS] = {
        _isrRouter0, _isrRouter1, _isrRouter2,
        _isrRouter3, _isrRouter4, _isrRouter5
    };

    void applyMotorControl() {
        if (_targetBrake) {
            digitalWrite(_brakePin, LOW);
            ledcWrite(_pwmCannel, 0);
            _currentPwm = 0;
            _brakeActive = true;
            _directionChangePending = false;
            _waitingForStop = false;
            log_d("[brake] Brake active");
            return;
        }
        else {
            digitalWrite(_brakePin, HIGH);
            _brakeActive = false;
        }

        int targetDir = (_targetPwm == 0) ? 0 : (_targetPwm > 0 ? 1 : -1);

        if (targetDir != 0 && targetDir != _lastDirection) {
            if (!_waitingForStop) {
                if (abs(_currentRPM) > 1.0) {
                    digitalWrite(_brakePin, LOW);
                    ledcWrite(_pwmCannel, 0);
                    _waitingForStop = true;
                    log_d("[dir change] Waiting for stop. RPM=%.2f", _currentRPM);
                    return;
                }
                else {
                    _waitingForStop = true;
                    _directionChangeTime = millis();
                    log_d("[dir change] Stopped. Begin delay");
                    return;
                }
            }
            else {
                if (millis() - _directionChangeTime < _dirChangeDelay) {
                    digitalWrite(_brakePin, LOW);
                    ledcWrite(_pwmCannel, 0);
                    log_d("[dir change] In delay pause (%lu ms)", millis() - _directionChangeTime);
                    return;
                }
                _waitingForStop = false;
                _lastDirection = targetDir;
                log_d("[dir change] Direction changed to %d", _lastDirection);
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
            log_d("[ramp] Current PWM: %d", _currentPwm);
        }

        applyPWM();
    }

    void applyPWM() {
        int pwmValue = map(abs(_currentPwm), 0, MAX_PWM, _minPwm, MAX_PWM);
        int targetDir = (_currentPwm == 0) ? 0 : (_currentPwm > 0 ? 1 : -1);

        digitalWrite(_dirPin, targetDir == 1 ? HIGH : LOW);
        ledcWrite(_pwmCannel, pwmValue);

        log_d("[PWM] Dir=%d, PWM=%d (mapped=%d)", targetDir, _currentPwm, pwmValue);
        _lastAppliedDirection = targetDir;
    }

    void handleInterrupt() {
        uint8_t a = digitalRead(_hallApin);
        uint8_t b = _singleHall ? 0 : digitalRead(_hallBpin);
        uint8_t newState = (a << 1) | b;

        uint8_t state = (_lastHallState << 2) | newState;
        int8_t change = _encoderStates[state & 0x0F];

        _encoderCount += (change == 0) ? (_lastDirection) : change;
        if (change != 0) {
            _lastDirection = (change > 0) ? 1 : -1;
        }

        _lastHallState = newState;
        log_d("[ISR] A=%d B=%d Count=%ld", a, b, _encoderCount);
    }
};

HallMotor* HallMotor::_instances[MAX_MOTORS] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
