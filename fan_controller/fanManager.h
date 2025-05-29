#ifndef FAN_CONTROLLER_RENDERER_H
#define FAN_CONTROLLER_RENDERER_H

// NOTE: All temps are in F

class FanManager {
public:
  FanManager(
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

  void updateTemperature(int newTemperature) {
    int fanStartTemperature = targetTemperature + 2;
    int fanStopTemperature = targetTemperature - 2;

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
};

#endif  // FAN_CONTROLLER_RENDERER_H