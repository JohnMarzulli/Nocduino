#ifndef TEMPERATURE_MANAGER_H
#define TEMPERATURE_MANAGER_H

// NOTE: All temps are in F

class TemperatureManager {
public:
  TemperatureManager(
    int newTargetTemperature) {
    targetTemperature = newTargetTemperature;
    currentTemperature = newTargetTemperature;
  }

  bool isFanRunning() {
    return isFanOn;
  }

  void increaseTargetTemperature() {
    if (targetTemperature < 100) {  // Assuming 50 is the max temperature
      targetTemperature++;
    } else {
      targetTemperature = 100;  // Cap at 100
    }
  }

  void decreaseTargetTemperature() {
    if (targetTemperature > 50) {  // Assuming 50 is the min temperature
      targetTemperature--;
    } else {
      targetTemperature = 50;  // Cap at 50
    }
  }

  int getTargetTemperature() {
    return targetTemperature;
  }

  int getCurrentTemperature() {
    return currentTemperature;
  }

  float getCutOffProportion() {
    float fanStopTemperature = targetTemperature - targetDelta;
    float fanStartTemperature = targetTemperature + targetDelta;

    if (currentTemperature < fanStopTemperature) {
      return 0.0;
    }

    float delta = currentTemperature - fanStopTemperature;
    float proportion = delta / (targetDelta * 2.0);

    return proportion;
  }

  void updateTemperature(int newTemperature) {
    int fanStartTemperature = targetTemperature + targetDelta;
    int fanStopTemperature = targetTemperature - targetDelta;

    currentTemperature = newTemperature;

    if (isFanOn && currentTemperature <= fanStopTemperature) {
      isFanOn = false;
    } else if (!isFanOn && currentTemperature >= fanStartTemperature) {
      isFanOn = true;
    }
  }



private:
  int targetTemperature;
  int currentTemperature;

  bool isFanOn = false;
  const int targetDelta = 2;
};

#endif  // TEMPERATURE_MANAGER_H