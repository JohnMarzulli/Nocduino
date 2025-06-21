// NOTE:
// Please make sure you install the "Arduino UNO R4 Boards" pack
// using the "BOARDS MANAGER"
//
// You will need to install the following packages:
// "MD_Parola" by majicDesigns - https://github.com/MajicDesigns/MD_Parola
//                               https://github.com/MajicDesigns/MD_MAX72XX/blob/main/examples/MD_MAX72xx_Test/MD_MAX72xx_Test.ino
// "DHT Sensor Library" by AdaFruit - https://github.com/adafruit/DHT-sensor-library
// "Button2" by Lennart Hunnigs - https://github.com/LennartHennigs/Button2
// "Rotary" by KAthiR - https://github.com/skathir38/Rotary

#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Rotary.h";
#include "Button2.h";
#include "renderer.h";
#include "fanManager.h";
#include "stopWatch.h"

const int SERIAL_SPEED = 9600;

FanManager fanManager(67);

/*** LED MATRIX ***/
// https://lastminuteengineers.com/max7219-dot-matrix-arduino-tutorial/
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
const int MAX_DEVICES = 4;
const int CS_PIN = 3;

MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
int lastTempShown = 0;
float lastProportionSet = 0.0;

#ifdef SERIAL_DEBUG_OUTPUT
int serialSpamLimiter = 0;
#endif  // SERIAL_DEBUG_OUTPUT

void setupMatrix() {
  myDisplay.begin();

  myDisplay.setIntensity(0);
  myDisplay.displayClear();
}

void serviceLedMatrix(
  int tempToShow,
  float fanPowerProportion) {

  fanPowerProportion = fanPowerProportion >= 1.0 ? 1.0 : fanPowerProportion;
  fanPowerProportion = fanPowerProportion <= 0.0 ? 0.0 : fanPowerProportion;

  float ledsToShow = (fanPowerProportion * 8.0);
  bool isBlink = (millis() % 1000) <= 500;

  if (tempToShow != lastTempShown) {
    myDisplay.displayClear();
    myDisplay.setIntensity(0);
    myDisplay.setTextAlignment(PA_CENTER);
    // add the text
    String text = String(tempToShow);  // + String("F");
    myDisplay.print(text);
    lastTempShown = tempToShow;
  }

#ifdef SERIAL_DEBUG_OUTPUT
  ++serialSpamLimiter;

  if (serialSpamLimiter == 5000) {
    Serial.print(F("fanPowerProportion: "));
    Serial.println(fanPowerProportion);
    Serial.print(F("LEDS to show: "));
    Serial.println(ledsToShow);

    serialSpamLimiter = 0;
  }
#endif  // SERIAL_DEBUG_OUTPUT

  for (int ledIndex = 1; ledIndex <= 7; ++ledIndex) {
    myDisplay.getGraphicObject()->setPoint(7 - ledIndex, 0, ledIndex <= (ledsToShow - 1));
  }

  bool isLastLedLit = (ledsToShow > 0) || isBlink;

  myDisplay.getGraphicObject()->setPoint(7, 0, isLastLedLit);
}

/*** TEMP PROBE ***/

// https://www.amazon.com/Replacement-Temperature-Humidity-Electronic-Practice/dp/B0DTHP4FGC/ref=sr_1_3_sspa?crid=32O8IXMXV11JB&dib=eyJ2IjoiMSJ9.Xs7bm5IGFitULL4ku2MBsb8h3E78np-GC6ppp-xfNQpUlxZnPwWp6KdplGjEXLUPp25g8CN1pHuuvrm_bgbu3OXJfQgaigs7d0sT5UdM8W2UBQXWDFpD3zeyHe-H2Hsd6NHX9vuQohMJ1QR0uH2D1lU0d6qoYcBhVb2oJmbohVUme-2uinsvctS8Zpnx1iY_Q5CqANOfXinE0U4g1YnEkgNUygHX0RiUdMRREVctIN1wXCo2oc_iGjomSHvsvbNPM6W-eUFY3OwHPax_ZPWJQh1JpgyLzOVu3h0WsoIGYCE.tQwCPgx7FJj9ET6APlQ34G-ZAJNtxumpCPmfGVxLQ0c&dib_tag=se&keywords=am2302&qid=1748717584&sprefix=am2302%2Caps%2C153&sr=8-3-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1
// This unit already has the pull-up resistor on the board.
// Wiring:
// "+""  -> Arduino 3V
// "out" -> Arduino Digital Pin 2
// "-"   -> Arduino GND
// Docs: https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHT_Unified_Sensor/DHT_Unified_Sensor.ino

const int DHTPIN = 2;       // Digital pin connected to the DHT sensor
const int DHTTYPE = DHT22;  // DHT 22 (AM2302)
// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

StopWatch probeInterval(2000);

int getFarenheitFromCelsius(
  float tempInCelsius) {
  return (int)((tempInCelsius * 9 / 5) + 32);
}

void setupTempProbe() {
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
  probeInterval = StopWatch(sensor.min_delay / 1000);
}

int serviceTempProbe() {
  float currentTemp = -255;
  // Delay between measurements.
  if (!probeInterval.shouldRun()) {
    return fanManager.getCurrentTemperature();
  }

  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    currentTemp = event.temperature;
#ifdef SERIAL_DEBUG_OUTPUT
    Serial.print(F("Temperature: "));
    Serial.print(currentTemp);
    Serial.println(F("°C"));
#endif  // SERIAL_DEBUG_OUTPUT
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
#ifdef SERIAL_DEBUG_OUTPUT
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
#endif  // SERIAL_DEBUG_OUTPUT

  return getFarenheitFromCelsius(currentTemp);
}

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

void increaseTargetTemp(Rotary& r) {
  fanManager.increaseTargetTemperature();
  Serial.print("targetTemp() => ");
  Serial.println(fanManager.getTargetTemperature());
  showNewTarget.reset();
}

void decreaseTargetTemp(Rotary& r) {
  fanManager.decreaseTargetTemperature();
  Serial.print("targetTemp() => ");
  Serial.println(fanManager.getTargetTemperature());
  showNewTarget.reset();
}

// single click
void click(Button2& btn) {
  Serial.println("Click!");
}

/*** FAN CONTROLLER ***/

// https://www.reddit.com/r/arduino/comments/14nung1/expertise_needed_for_4pin_pwm_powering/

const int PWN_PULSE_IN = 7;   // Green wire. Only 2 or 3 can be used as external intr's
const int PWN_MOTOR_OUT = 9;  // Blue wire

int rotations = 0;
int motorPwm = 128;

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

    bool isRunning = fanManager.isFanRunning();

#ifdef SERIAL_DEBUG_OUTPUT
    Serial.print(F("serviceFan(): Running = "));
    Serial.println(isRunning);
#endif  // SERIAL_DEBUG_OUTPUT

    // This should be one of the only two variables you really
    // have to mess with in addition to the period:
    float proportion = fanManager.getCutOffProportion();
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

    Serial.print(F("serviceFan(): motorPwm = "));
    Serial.println(motorPwm);
#endif  // SERIAL_DEBUG_OUTPUT

    analogWrite(PWN_MOTOR_OUT, targetPulses);  //motor_pwm);

    lastProportionSet = effectiveProportion;
  }

  return lastProportionSet;
}


/*** MAIN ***/

void setup() {
  Serial.begin(SERIAL_SPEED);  //115200);

  setupTempProbe();
  setupRotaryEncoder();
  setupPwmFan();
  setupMatrix();
}

void loop() {
  int currentTemp = serviceTempProbe();
  fanManager.updateTemperature(currentTemp);
  serviceRotaryEncoder();
  float powerProportion = serviceFan();

  int tempToShow = currentTemp;
  if (showNewTarget.isWaiting()) {
    tempToShow = fanManager.getTargetTemperature();
  }

  serviceLedMatrix(tempToShow, powerProportion);
}