// motorControl.h
#pragma once
#include <Arduino.h>

#define MIN_PWM -255
#define MAX_PWM 255
#define MAX_MOTORS 6

class HallMotor {
public:
    HallMotor(uint8_t hallPin, uint8_t pwmPin, uint8_t pwmChannel,
        uint8_t minPwm, uint8_t dirPin, uint8_t brakePin,
        uint16_t rampTime = 200, uint16_t dirChangeDelay = 200);

    void begin();
    void setPwm(int pwm, bool brake = false);
    void update();
    float getRPM();
    int getDirection();
    void setPower(int power);

private:
    void applyMotorControl();
    void handleInterrupt();
    void applyPWM();
    void loger();

    uint8_t _hallPin, _pwmPin, _pwmChannel, _minPwm, _maxPwm, _dirPin, _brakePin;
    int _targetPwm, _currentPwm;
    int _currentDir; // -1, 0, 1

    uint16_t _rampTime, _dirChangeDelay;
    unsigned long _lastRampTime, _directionChangeTime, _lastUpdateTime;

    bool _waitingForStop, _directionChangePending;
    bool _brake = true;

    volatile long _encoderCount;
    long _lastCount;
    float _currentRPM;

    static HallMotor* _instances[MAX_MOTORS];
    static void IRAM_ATTR _isrRouter0();
    static void IRAM_ATTR _isrRouter1();
    static void IRAM_ATTR _isrRouter2();
    static void IRAM_ATTR _isrRouter3();
    static void IRAM_ATTR _isrRouter4();
    static void IRAM_ATTR _isrRouter5();
};
