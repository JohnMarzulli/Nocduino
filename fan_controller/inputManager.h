#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

StopWatch showNewTarget(1000);

class InputManager {
public:
  InputManager(
    TemperatureManager *manager) {
    _temperatureManager = manager;
  }

  bool isShowingNewTarget() {
    return showNewTarget.isWaiting();
  }

  void setupRotaryEncoder() {
    Serial.println("Setup rotary encoder");

    pinMode(ROTARY_POWER_PIN, OUTPUT);
    digitalWrite(ROTARY_POWER_PIN, HIGH);
    delay(100);  // give it some time to power up

    r.begin(ROTARY_PIN1, ROTARY_PIN2, CLICKS_PER_STEP);
    r.setLeftRotationHandler(InputManager::increaseTargetTemp);
    r.setRightRotationHandler(InputManager::decreaseTargetTemp);

    b.begin(BUTTON_PIN);
    b.setTapHandler(InputManager::click);

    Serial.println("Handlers set.");
  }

  void serviceRotaryEncoder() {
    r.loop();
    b.loop();
  }

private:
  static void increaseTargetTemp(Rotary &r) {
    _temperatureManager->increaseTargetTemperature();
    Serial.print("targetTemp() => ");
    Serial.println(_temperatureManager->getTargetTemperature());
    showNewTarget.reset();
  }

  static void decreaseTargetTemp(Rotary &r) {
    _temperatureManager->decreaseTargetTemperature();
    Serial.print("targetTemp() => ");
    Serial.println(_temperatureManager->getTargetTemperature());
    showNewTarget.reset();
  }

  // single click
  static void click(Button2 &btn) {
    Serial.println("Click!");
  }

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