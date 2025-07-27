#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

typedef void (*RotateFunction)(class Rotary &);
typedef void (*ClickFunction)(Button2 &btn);

class InputManager {
public:
  InputManager(
    TemperatureManager *manager) {
    _temperatureManager = manager;
  }

  void setupRotaryEncoder(
    ClickFunction clickHandler,
    RotateFunction clockwiseHandler,
    RotateFunction counterClockwiseHandler) {
    Serial.println("Setup rotary encoder");

    pinMode(ROTARY_POWER_PIN, OUTPUT);
    digitalWrite(ROTARY_POWER_PIN, HIGH);
    delay(100);  // give it some time to power up

    r.begin(ROTARY_PIN1, ROTARY_PIN2, CLICKS_PER_STEP);
    r.setLeftRotationHandler(clockwiseHandler);
    r.setRightRotationHandler(counterClockwiseHandler);

    b.begin(BUTTON_PIN);
    b.setTapHandler(clickHandler);

    Serial.println("Handlers set.");
  }

  void serviceRotaryEncoder() {
    r.loop();
    b.loop();
  }

private:
  Rotary r;
  Button2 b;

  static TemperatureManager *_temperatureManager;

  const int ROTARY_PIN1 = 5;
  const int ROTARY_PIN2 = 4;
  const int BUTTON_PIN = 6;
  const int ROTARY_POWER_PIN = 8;
  const int CLICKS_PER_STEP = 4;  // this number depends on your rotary encoder
};

TemperatureManager *InputManager::_temperatureManager = nullptr;

#endif  // INPUT_MANAGER_H