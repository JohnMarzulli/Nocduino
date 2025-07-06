#ifndef FAN_DRIVER_H
#define FAN_DRIVER_H

class FanDriver {
public:

  FanDriver() {
    _rotations = 0;
  }

  void setupPwmFan() {
    pinMode(PWN_MOTOR_OUT, OUTPUT);
    analogWrite(PWN_MOTOR_OUT, MOTOR_PWM);

    pinMode(PWN_PULSE_IN, INPUT_PULLUP);
    int intr = digitalPinToInterrupt(PWN_PULSE_IN);
    attachInterrupt(intr, pulse_isr, FALLING);
  }

  float serviceFan(
    TemperatureManager& temperatureManager) {
    static uint32_t timer_start = millis();

    // this is for a period of 1 second (1000 ms). Adjust as needed:
    uint32_t period = 1000;
    const int maxPulses = 512;
    const int minPulses = 128;
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

      bool isRunning = temperatureManager.isFanRunning();

#ifdef SERIAL_DEBUG_OUTPUT
      Serial.print(F("serviceFan(): Running = "));
      Serial.println(isRunning);
#endif  // SERIAL_DEBUG_OUTPUT

      // This should be one of the only two variables you really
      // have to mess with in addition to the period:
      float proportion = temperatureManager.getCutOffProportion();
      //int targetPulses = (int)(((maxPulses - minPulses) * proportion) + minPulses);

      int targetPulses = proportion * maxPulses;

      targetPulses = isRunning ? targetPulses : stoppedPulses;

      float effectiveProportion = (float)targetPulses / (float)maxPulses;

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

      lastProportionSet = effectiveProportion;
    }

    return lastProportionSet;
  }

private:
  static void pulse_isr() {
    _rotations++;
  }

  static int _rotations;

  float lastProportionSet = 0.0;

  const int PWN_PULSE_IN = 7;   // Green wire. Only 2 or 3 can be used as external intr's
  const int PWN_MOTOR_OUT = 9;  // Blue wire
  const int MOTOR_PWM = 128;
};

int FanDriver::_rotations = 0;

#endif  // FAN_DRIVER_H