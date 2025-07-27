#ifndef POWER_BASED_FAN_DRIVER_H
#define POWER_BASED_FAN_DRIVER_H

class PowerBasedFanManager {
public:

  PowerBasedFanManager() {
    _rotations = 0;
  }

  void setupPwmFan() {
    pinMode(PWN_MOTOR_OUT, OUTPUT);
    analogWrite(PWN_MOTOR_OUT, MOTOR_PWM);

    pinMode(PWN_PULSE_IN, INPUT_PULLUP);
    int intr = digitalPinToInterrupt(PWN_PULSE_IN);
    attachInterrupt(intr, pulse_isr, FALLING);

    lastProportionSet = proportionChange * 3.0;
  }

  void increasePower() {
    lastProportionSet += proportionChange;

    clampProportion();
  }

  void decreasePower() {
    lastProportionSet -= proportionChange;

    clampProportion();
  }

  float serviceFan() {
    static uint32_t timer_start = millis();

    // this is for a period of 1 second (1000 ms). Adjust as needed:
    uint32_t period = 1000;
    const int maxPulses = 512;
    const int minPulses = 64;  //128;
    const int stoppedPulses = 0;

    // Get the current pulse counter and reset it to 0 if we have
    // finished a period.
    if (millis() - timer_start >= period) {
      timer_start = millis();

      noInterrupts();           // disable interrupts while we read
                                // the counter to avoid slicing
      int pulses = _rotations;  // get the num rotations in the last 1000 ms
      _rotations = 0;           // clear the counter for the next period
      interrupts();             // re-enable interrupts again


      clampProportion();

      bool isRunning = lastProportionSet > 0.0;

#ifdef SERIAL_DEBUG_OUTPUT
      Serial.print(F("serviceFan(): Running = "));
      Serial.println(isRunning);
#endif  // SERIAL_DEBUG_OUTPUT

      int targetPulses = lastProportionSet * maxPulses;

      targetPulses = isRunning ? targetPulses : stoppedPulses;

#ifdef SERIAL_DEBUG_OUTPUT
      Serial.print(F("serviceFan(): targetPulses = "));
      Serial.println(targetPulses);

      Serial.print(F("serviceFan(): proportion = "));
      Serial.println(proportion);

      Serial.print(F("serviceFan(): effectiveProportion = "));
      Serial.println(effectiveProportion);

      Serial.print(F("serviceFan(): MOTOR_PWM = "));
      Serial.println(MOTOR_PWM);
#endif  // SERIAL_DEBUG_OUTPUT

      analogWrite(PWN_MOTOR_OUT, targetPulses);  //motor_pwm);
    }

    return lastProportionSet;
  }

private:

  void clampProportion() {
    if (lastProportionSet > 1.0) {
      lastProportionSet = 1.0;
    }

    if (lastProportionSet < 0.0) {
      lastProportionSet = 0.0;
    }
  }

  static void pulse_isr() {
    _rotations++;
  }

  static int _rotations;

  float lastProportionSet = 0.4;

  const int PWN_PULSE_IN = 7;   // Green wire. Only 2 or 3 can be used as external intr's
  const int PWN_MOTOR_OUT = 9;  // Blue wire
  const int MOTOR_PWM = 128;

  const float proportionChange = 0.125;
};

int PowerBasedFanManager::_rotations = 0;

#endif  // POWER_BASED_FAN_DRIVER_H