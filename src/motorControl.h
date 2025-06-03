/*
* v1.0.2
*/

#ifndef _motorControl_h
#define _motorControl_h
#include <Arduino.h>
#include <driver/ledc.h>
#include <stdint.h>
#include <math.h>

class Motor {
public:
    Motor(uint8_t hallApin, uint8_t hallBpin, uint8_t pwmPin, uint8_t pwmCannel, uint8_t dirPin, uint8_t brakePin, uint16_t rampTime = 500);
    void begin();
    void setSpeed(int speed, bool brake);
    void update();
    float getRPM();
    int getDirection(); // 1 = forward, -1 = backward, 0 = stopped
    void setRampTime(uint16_t rampTime);
    void PLOTTER_SERIAL();

private:
    uint8_t _hallApin, _hallBpin;
    uint8_t _pwmPin, _dirPin, _brakePin, _pwmCannel;
    static const int PULSES_PER_REVOLUTION = 60;

    volatile long _encoderCount;
    volatile uint8_t _lastHallState;
    volatile int _lastDirection;
    long _lastCount;
    unsigned long _lastUpdateTime;
    float _currentRPM;
    int _currentDirection;

    int _targetSpeed;
    int _currentSpeed;
    bool _targetBrake;
    int _lastAppliedDirection;
    bool _brakeActive;
    bool _directionChangePending;

    uint16_t _rampTime;          // Время плавного разгона (мс)
    unsigned long _lastRampTime; // Время последнего изменения скорости
    int _rampIncrement;          // Шаг изменения скорости за цикл

    static const int8_t _encoderStates[16];
    static Motor* _instance;

    void handleInterrupt();
    void applyMotorControl();
    void applyPWM();
    static void _handleInterruptA();
    static void _handleInterruptB();

    const int MAX_SPEED = 255;
    const int MIN_SPEED = -255;
};

#endif

