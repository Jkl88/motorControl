/*
* v1.0.3
*/

#ifndef _motorControl_h
#define _motorControl_h
#include <Arduino.h>

// Пределы ШИМ
#define MIN_PWM -255
#define MAX_PWM 255

// Кол-во импульсов на один оборот
#define PULSES_PER_REVOLUTION 60

class Motor {
public:
    Motor(uint8_t hallApin, uint8_t hallBpin, uint8_t pwmPin, uint8_t pwmCannel,
        uint8_t minPwm, uint8_t dirPin, uint8_t brakePin, uint16_t rampTime = 200);

    void begin();
    void setPwm(int pwm, bool brake = false);
    void setRampTime(uint16_t rampTime);
    void update();

    float getRPM();
    int getDirection();

private:
    // Пины
    uint8_t _hallApin;
    uint8_t _hallBpin;
    uint8_t _pwmPin;
    uint8_t _pwmCannel;
    uint8_t _minPwm;
    uint8_t _dirPin;
    uint8_t _brakePin;

    // Состояние
    int _targetPwm;
    int _currentPwm;
    bool _targetBrake;
    bool _brakeActive;
    bool _directionChangePending;

    // Направление и ШИМ
    int _lastAppliedDirection;
    int _currentDirection;
    int _lastDirection;

    // Плавность
    uint16_t _rampTime;
    uint16_t _rampIncrement;
    unsigned long _lastRampTime;

    // Энкодер
    volatile long _encoderCount;
    long _lastCount;
    uint8_t _lastHallState;
    float _currentRPM;
    unsigned long _lastUpdateTime;

    // Вспомогательные функции
    void applyMotorControl();
    void applyPWM();
    void handleInterrupt();

    static const int8_t _encoderStates[16];

    // Прерывания
    static void IRAM_ATTR _handleInterruptA(void* arg);
    static void IRAM_ATTR _handleInterruptB(void* arg);
};

#endif

