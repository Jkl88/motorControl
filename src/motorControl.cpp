/*
* v1
*/

#include "motorControl.h"


Motor* Motor::_instance = nullptr;
const int8_t Motor::_encoderStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
const int Motor::PULSES_PER_REVOLUTION = 60;

Motor::Motor(uint8_t hallApin, uint8_t hallBpin, uint8_t pwmPin, uint8_t dirPin, uint8_t brakePin, uint16_t rampTime) {
    _hallApin = hallApin;
    _hallBpin = hallBpin;
    _pwmPin = pwmPin;
    _dirPin = dirPin;
    _brakePin = brakePin;
    _rampTime = rampTime;
    _targetSpeed = 0;
    _currentSpeed = 0;
    _targetBrake = false;
    _lastAppliedDirection = 0;
    _brakeActive = false;
    _directionChangePending = false;
    _rampIncrement = 5; // По умолчанию
}

void Motor::begin() {
    pinMode(_hallApin, INPUT_PULLUP);
    pinMode(_hallBpin, INPUT_PULLUP);
    pinMode(_dirPin, OUTPUT);
    pinMode(_brakePin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);

    // Инициализация выходов
    digitalWrite(_brakePin, HIGH); // Тормоз неактивен (HIGH)
    digitalWrite(_dirPin, LOW);

    // Настройка ШИМ для ESP32
    ledcSetup(0, 5000, 8);     // Канал 0, 5 кГц, 8 бит
    ledcAttachPin(_pwmPin, 0);  // Привязка пина PWM к каналу 0
    ledcWrite(0, 0);            // Начальная скорость 0

    // Инициализация энкодера
    _lastHallState = (digitalRead(_hallApin) << 1) | digitalRead(_hallBpin);
    _instance = this;

    // Настройка прерываний с защитой от дребезга
    attachInterrupt(digitalPinToInterrupt(_hallApin), _handleInterruptA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_hallBpin), _handleInterruptB, CHANGE);

    _lastUpdateTime = millis();
    _lastRampTime = millis();
    _encoderCount = 0;
    _lastCount = 0;
    _currentRPM = 0.0;
    _currentDirection = 0;
}

void Motor::setSpeed(int speed, bool brake) {
    // Ограничение скорости
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    _targetSpeed = speed;
    _targetBrake = brake;

    // Рассчет шага плавности
    if (_rampTime > 0) {
        _rampIncrement = max(1, abs(_targetSpeed - _currentSpeed) * 20 / _rampTime);
    }
    else {
        _rampIncrement = MAX_SPEED + 1; // Мгновенное изменение
    }
}

void Motor::setRampTime(uint16_t rampTime) {
    _rampTime = rampTime;
}

void Motor::applyMotorControl() {
    // Обработка торможения
    if (_targetBrake) {
        digitalWrite(_brakePin, LOW);  // Активировать тормоз (GND)
        ledcWrite(0, 0);               // Отключить ШИМ
        _currentSpeed = 0;
        _brakeActive = true;
        _directionChangePending = false;
        return;
    }
    else {
        digitalWrite(_brakePin, HIGH); // Отпустить тормоз
        _brakeActive = false;
    }

    // Определение целевого направления
    int targetDir = (_targetSpeed == 0) ? 0 : (_targetSpeed > 0 ? 1 : -1);

    // Проверка необходимости смены направления
    if (targetDir != _lastAppliedDirection && _lastAppliedDirection != 0) {
        if (abs(_currentRPM) > 5.0) { // Мотор еще вращается
            digitalWrite(_brakePin, LOW); // Активировать тормоз
            _directionChangePending = true;
            ledcWrite(0, 0); // Отключить ШИМ
            _currentSpeed = 0;
            return;
        }
        else {
            _directionChangePending = false;
        }
    }

    // Плавное изменение скорости
    if (millis() - _lastRampTime > 20) { // Обновление каждые 20 мс
        if (_currentSpeed < _targetSpeed) {
            _currentSpeed = min(_targetSpeed, _currentSpeed + _rampIncrement);
        }
        else if (_currentSpeed > _targetSpeed) {
            _currentSpeed = max(_targetSpeed, _currentSpeed - _rampIncrement);
        }
        _lastRampTime = millis();
    }

    // Применение ШИМ и направления
    applyPWM();
}

void Motor::applyPWM() {
    int pwmValue = abs(_currentSpeed);
    int targetDir = (_currentSpeed == 0) ? 0 : (_currentSpeed > 0 ? 1 : -1);

    if (targetDir == 1) {
        digitalWrite(_dirPin, HIGH); // Вперед
        ledcWrite(0, pwmValue);
    }
    else if (targetDir == -1) {
        digitalWrite(_dirPin, LOW);  // Назад (GND)
        ledcWrite(0, pwmValue);
    }
    else {
        ledcWrite(0, 0); // Остановка
    }

    _lastAppliedDirection = targetDir;
}

void Motor::_handleInterruptA() {
    if (_instance) {
        static unsigned long lastInterruptTime = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastInterruptTime > 2) {
            _instance->handleInterrupt();
        }
        lastInterruptTime = currentTime;
    }
}

void Motor::_handleInterruptB() {
    if (_instance) {
        static unsigned long lastInterruptTime = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastInterruptTime > 2) {
            _instance->handleInterrupt();
        }
        lastInterruptTime = currentTime;
    }
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
    // Обновление RPM
    unsigned long now = millis();
    unsigned long dt = now - _lastUpdateTime;

    if (dt >= 100) { // Обновление каждые 100 мс
        long currentCount = _encoderCount;
        long deltaCount = currentCount - _lastCount;

        if (dt > 0) {
            _currentRPM = (deltaCount * 60000.0) / (PULSES_PER_REVOLUTION * dt);

            // Определение текущего направления
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

    // Применение управления мотором
    applyMotorControl();
}

float Motor::getRPM() {
    return _currentRPM;
}

int Motor::getDirection() {
    return _currentDirection;
}