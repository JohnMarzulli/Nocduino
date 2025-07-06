// NOTE:
// Please make sure you install the "Arduino UNO R4 Boards" pack
// using the "BOARDS MANAGER"
//
// You will need to install the following packages:
// "MD_Parola" by majicDesigns - https://github.com/MajicDesigns/MD_Parola
//                               https://github.com/MajicDesigns/MD_MAX72XX/blob/main/examples/MD_MAX72xx_Test/MD_MAX72xx_Test.ino
//                               https://majicdesigns.github.io/MD_Parola/class_m_d___parola.html#a45d97a582ca1adabfe5cb40d66c4bbd2
// "DHT Sensor Library" by AdaFruit - https://github.com/adafruit/DHT-sensor-library
// "Button2" by Lennart Hunnigs - https://github.com/LennartHennigs/Button2
// "Rotary" by KAthiR - https://github.com/skathir38/Rotary

#include "Rotary.h"
#include "Button2.h"
#include "sensorManager.h"
#include "temperatureManager.h"
#include "stopWatch.h"
#include "display.h"

const int SERIAL_SPEED = 9600;

TemperatureManager temperatureManager(67);
MatrixDisplay display(MatrixDisplay::INTENSITY_DIM);
SensorManager sensorManager(SensorManager::DHT_PIN);

/*** ROTARY ENCODER ***/

// Product page: https://www.amazon.com/dp/B07T3672VK?ref=ppx_yo2ov_dt_b_fed_asin_title

// https://github.com/skathir38/Rotary/blob/main/examples/SimpleCounterWithButton/SimpleCounterWithButton.ino
// https://github.com/LennartHennigs/Button2

// CLK = [pin 5]
// DT = [pin 4]
// SW = [pin 6] (Button)
// + = [Pin 5V]
// GND = [pin GND]

const int ROTARY_PIN1 = 5;
const int ROTARY_PIN2 = 4;
const int BUTTON_PIN = 6;
const int ROTARY_POWER_PIN = 8;
const int CLICKS_PER_STEP = 4;  // this number depends on your rotary encoder

Rotary r;
Button2 b;
StopWatch showNewTarget(1000);

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

/////////////////////////////////////////////////////////////////

void increaseTargetTemp(Rotary &r) {
  temperatureManager.increaseTargetTemperature();
  Serial.print("targetTemp() => ");
  Serial.println(temperatureManager.getTargetTemperature());
  showNewTarget.reset();
}

void decreaseTargetTemp(Rotary &r) {
  temperatureManager.decreaseTargetTemperature();
  Serial.print("targetTemp() => ");
  Serial.println(temperatureManager.getTargetTemperature());
  showNewTarget.reset();
}

// single click
void click(Button2 &btn) {
  Serial.println("Click!");
}

/*** FAN CONTROLLER ***/

// https://www.reddit.com/r/arduino/comments/14nung1/expertise_needed_for_4pin_pwm_powering/

const int PWN_PULSE_IN = 7;   // Green wire. Only 2 or 3 can be used as external intr's
const int PWN_MOTOR_OUT = 9;  // Blue wire

int rotations = 0;
int motorPwm = 128;
float lastProportionSet = 0.0;

void pulse_isr() {
  rotations++;
}

void setupPwmFan() {
  pinMode(PWN_MOTOR_OUT, OUTPUT);
  analogWrite(PWN_MOTOR_OUT, motorPwm);

  pinMode(PWN_PULSE_IN, INPUT_PULLUP);
  int intr = digitalPinToInterrupt(PWN_PULSE_IN);
  attachInterrupt(intr, pulse_isr, FALLING);
}

float serviceFan() {
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

    noInterrupts();          // disable interrupts while we read
                             // the counter to avoid slicing
    int pulses = rotations;  // get the num rotations in the last 1000 ms
    rotations = 0;           // clear the counter for the next period
    interrupts();            // re-enable interrupts again

    bool isRunning = temperatureManager.isFanRunning();

#ifdef SERIAL_DEBUG_OUTPUT
    Serial.print(F("serviceFan(): Running = "));
    Serial.println(isRunning);
#endif  // SERIAL_DEBUG_OUTPUT

    // This should be one of the only two variables you really
    // have to mess with in addition to the period:
    float proportion = temperatureManager.getCutOffProportion();
    // int targetPulses = (int)(((maxPulses - minPulses) * proportion) + minPulses);

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

    Serial.print(F("serviceFan(): motorPwm = "));
    Serial.println(motorPwm);
#endif  // SERIAL_DEBUG_OUTPUT

    analogWrite(PWN_MOTOR_OUT, targetPulses);  // motor_pwm);

    lastProportionSet = effectiveProportion;
  }

  return lastProportionSet;
}

/*** MAIN ***/

void setup() {
  Serial.begin(SERIAL_SPEED);  // 115200);

  sensorManager.setupTempProbe();
  setupRotaryEncoder();
  setupPwmFan();
  display.setupMatrix();
}

void loop() {
  int currentTemp = sensorManager.serviceTempProbe();
  temperatureManager.updateTemperature(currentTemp);
  serviceRotaryEncoder();
  float powerProportion = serviceFan();

  int tempToShow = currentTemp;
  if (showNewTarget.isWaiting()) {
    tempToShow = temperatureManager.getTargetTemperature();
  }

  display.serviceLedMatrix(tempToShow, powerProportion);
}