// motorControl.cpp
#include "motorControl.h"

// Инициализация статического массива экземпляров
HallMotor* HallMotor::_instances[MAX_MOTORS] = { nullptr };

// Конструктор класса HallMotor
HallMotor::HallMotor(uint8_t hallPin, uint8_t pwmPin, uint8_t pwmChannel,
    uint8_t minPwm, uint8_t dirPin, uint8_t brakePin,
    uint16_t rampTime, uint16_t dirChangeDelay)
    : _hallPin(hallPin), _pwmPin(pwmPin), _pwmChannel(pwmChannel),
    _minPwm(minPwm), _dirPin(dirPin), _brakePin(brakePin),
    _rampTime(rampTime), _dirChangeDelay(dirChangeDelay),
    _maxPwm(MAX_PWM), _targetPwm(0), _currentPwm(0), _currentDir(0),
    _waitingForStop(false), _directionChangePending(false),
    _lastRampTime(0), _directionChangeTime(0), _lastUpdateTime(0),
    _encoderCount(0), _lastCount(0), _currentRPM(0.0f)
{
}

// Инициализация пинов, таймеров и прерываний
void HallMotor::begin() {
    pinMode(_hallPin, INPUT_PULLUP);
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_brakePin, OUTPUT);

    digitalWrite(_brakePin, LOW);  // brake ON by default
    digitalWrite(_dirPin, HIGH);   // set to HIGH (не GND!)

    ledcSetup(_pwmChannel, 5000, 8); // частота 5 кГц, разрешение 8 бит
    ledcAttachPin(_pwmPin, _pwmChannel);
    ledcWrite(_pwmChannel, 0);

    // Привязка ISR к экземпляру
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (_instances[i] == nullptr) {
            _instances[i] = this;
            attachInterrupt(digitalPinToInterrupt(_hallPin),
                i == 0 ? _isrRouter0 :
                i == 1 ? _isrRouter1 :
                i == 2 ? _isrRouter2 :
                i == 3 ? _isrRouter3 :
                i == 4 ? _isrRouter4 : _isrRouter5,
                RISING);
            break;
        }
    }

    _lastRampTime = millis();
    _lastUpdateTime = millis();
    _lastCount = 0;
}

// Установка новой цели для ШИМ и опциональная активация тормоза
void HallMotor::setPwm(int pwm, bool brake) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    _targetPwm = brake ? 0 : pwm;    
    _brake = brake;
}

void HallMotor::setPower(int power) {
    _maxPwm = MAX_PWM * power;
}

// Основной метод обновления, вызывается в loop()
void HallMotor::update() {
    unsigned long now = millis();
    loger();
    // Расчет RPM раз в 100 мс
    if (now - _lastUpdateTime >= 100) {
        long deltaCount = _encoderCount - _lastCount;
        _currentRPM = (deltaCount * 600.0) / 6.0; // упрощенная формула RPM (10 имп./сек = 50 об/мин)
        _lastCount = _encoderCount;
        _lastUpdateTime = now;
    }

    applyMotorControl();
    applyPWM();
}

float HallMotor::getRPM() {
    return _currentRPM;
}

int HallMotor::getDirection() {
    return _currentDir;
}

// Логика плавного разгона/торможения и смены направления
void HallMotor::applyMotorControl() {
    int targetDir = (_targetPwm == 0) ? 0 : (_targetPwm > 0 ? 1 : -1);
    _targetPwm = abs(_targetPwm);

    //если тормоз или полная остановка перед сменой направления
    if ((_currentDir != 0 && targetDir != 0 && targetDir != _currentDir) || _brake) {
        _currentPwm = 0;
        if (!_waitingForStop) {
            digitalWrite(_brakePin, LOW); // активируем тормоз
            ledcWrite(_pwmChannel, 0);
            _waitingForStop = true;            
            return;
        }
        //ждем остановку
        if (_currentRPM != 0) { _directionChangeTime = millis(); return; }
        // Ждем задержку перед сменой направления
        if ((millis() - _directionChangeTime < _dirChangeDelay))  return;

        // остановились        
        _currentDir = 0;

        if (_brake) return;
    }
    
    else if (_waitingForStop){
        digitalWrite(_brakePin, HIGH); // отпускаем тормоз
        _waitingForStop = false;
    }
    
    // плавный пуск если не задано setPwm = 0
    if (targetDir != 0 && millis() - _lastRampTime > _rampTime) {
        digitalWrite(_dirPin, targetDir == -1 ? HIGH : LOW);
        digitalWrite(_brakePin, HIGH); // отпускаем тормоз 
        // если текущее направление "стоим" и есть вращение а вращение не по заданному значение(самопроизвольно) ??????
        if (_currentDir == 0 && _currentRPM != 0) _currentDir = targetDir;

        if (_currentPwm < _targetPwm) {
            _currentPwm = min(_targetPwm, _currentPwm + 1);
        }

        else _currentPwm = _targetPwm;

        _lastRampTime = millis();
    }

    if (targetDir == 0) { 
        _currentPwm = 0;
        if (_currentRPM == 0) {
            _currentDir = 0;
            _brake; // активируем тормоз
        }
    }    
}

void HallMotor::loger() {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
    static unsigned long lastTime;
    if (millis() - lastTime >= 500) {
        Serial.printf(
            "======================================================\n"
            "PWM min:%d, max:%d, target:%d, current:%d\n"
            "RPM current:%d, encoderCount:%d\n"
            "DIR current:%d\n"
            "Time ramp:%d, dirChange:%d\n",
            _minPwm, maxPWM, _targetPwm, _currentPwm,
            _currentRPM, _encoderCount,
            _currentDir,
            _rampTime, _dirChangeDelay
        );
        lastTime = millis();
    }
#endif    
}

// Применение текущего PWM к пину
void HallMotor::applyPWM() {
    int pwmValue = map(_currentPwm, 0, MAX_PWM, _minPwm, _maxPwm);
    ledcWrite(_pwmChannel, pwmValue);
}

// Прерывание от холла: увеличиваем счетчик
void HallMotor::handleInterrupt() {
    _encoderCount++;
}

// ISR роутеры
void IRAM_ATTR HallMotor::_isrRouter0() { if (_instances[0]) _instances[0]->handleInterrupt(); }
void IRAM_ATTR HallMotor::_isrRouter1() { if (_instances[1]) _instances[1]->handleInterrupt(); }
void IRAM_ATTR HallMotor::_isrRouter2() { if (_instances[2]) _instances[2]->handleInterrupt(); }
void IRAM_ATTR HallMotor::_isrRouter3() { if (_instances[3]) _instances[3]->handleInterrupt(); }
void IRAM_ATTR HallMotor::_isrRouter4() { if (_instances[4]) _instances[4]->handleInterrupt(); }
void IRAM_ATTR HallMotor::_isrRouter5() { if (_instances[5]) _instances[5]->handleInterrupt(); }
