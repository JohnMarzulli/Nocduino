#ifndef STOPWATCH_H
#define STOPWATCH_H

class StopWatch {
public:
  StopWatch(
    int delayInMilliSeconds) {
    delay = delayInMilliSeconds;
    timerStart = millis();
  }

  bool shouldRun() {
    uint32_t currentTime = millis();
    uint32_t delta = currentTime - timerStart;

    if (delta >= delay) {
      timerStart = currentTime;

      return true;
    }

    return false;
  }

  bool reset() {
    timerStart = millis();
  }

  bool isWaiting() {
    uint32_t currentTime = millis();
    uint32_t delta = currentTime - timerStart;

    return delta <= delay;
  }

private:
  int delay;
  uint32_t timerStart;
};

#endif  // STOPWATCH_H