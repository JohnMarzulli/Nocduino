#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

TemperatureManager *_temperatureManager = NULL;
StopWatch showNewTarget(1000);

void increaseTargetTemp(Rotary &r) {
  _temperatureManager->increaseTargetTemperature();
  Serial.print("targetTemp() => ");
  Serial.println(_temperatureManager->getTargetTemperature());
  showNewTarget.reset();
}

void decreaseTargetTemp(Rotary &r) {
  _temperatureManager->decreaseTargetTemperature();
  Serial.print("targetTemp() => ");
  Serial.println(_temperatureManager->getTargetTemperature());
  showNewTarget.reset();
}

// single click
void click(Button2 &btn) {
  Serial.println("Click!");
}


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
    r.setLeftRotationHandler(increaseTargetTemp);
    r.setRightRotationHandler(decreaseTargetTemp);

    b.begin(BUTTON_PIN);
    b.setTapHandler(click);

    Serial.println("Handlers set.");
  }

  void serviceRotaryEncoder() {
    r.loop();
    b.loop();
  }

private:
  Rotary r;
  Button2 b;

  const int ROTARY_PIN1 = 5;
  const int ROTARY_PIN2 = 4;
  const int BUTTON_PIN = 6;
  const int ROTARY_POWER_PIN = 8;
  const int CLICKS_PER_STEP = 4;  // this number depends on your rotary encoder
};

#endif  // INPUT_MANAGER_H