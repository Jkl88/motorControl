
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

class HallMotor : public BaseMotor {
public:
    HallMotor(uint8_t hallApin, uint8_t hallBpin, uint8_t pwmPin, uint8_t pwmCannel, uint8_t minPwm,
        uint8_t dirPin, uint8_t brakePin, uint16_t rampTime = 200, uint16_t dirChangeDelay = 200);

    void begin() override;
    void setPwm(int pwm, bool brake = false) override;
    void update() override;
    float getRPM() override;
    int getDirection() override;

    void setRampTime(uint16_t rampTime);
    void setMinPwm(uint8_t minPwm);
    void setDirChangeDelay(uint16_t delay);

private:
    void applyMotorControl();
    void applyPWM();
    void handleInterrupt();

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

    static const int8_t _encoderStates[16];
    static HallMotor* _instances[MAX_MOTORS];

    typedef void (*isr_func_t)();
    static void IRAM_ATTR _isrRouter0();
    static void IRAM_ATTR _isrRouter1();
    static void IRAM_ATTR _isrRouter2();
    static void IRAM_ATTR _isrRouter3();
    static void IRAM_ATTR _isrRouter4();
    static void IRAM_ATTR _isrRouter5();

    static const isr_func_t _isrRouters[MAX_MOTORS];
};
