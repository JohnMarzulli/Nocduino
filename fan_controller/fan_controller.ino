// NOTE:
// Please make sure you install the "Arduino UNO R4 Boards" pack
// using the "BOARDS MANAGER"
//
// You will need to install the following packages:
// "ArduinoGraphics" by Arduino - https://github.com/arduino-libraries/ArduinoGraphics
// "DHT Sensor Library" by AdaFruit - https://github.com/adafruit/DHT-sensor-library
// "Button2" by Lennart Hunnigs - https://github.com/LennartHennigs/Button2
// "Rotary" by KAthiR - https://github.com/skathir38/Rotary

#include "ArduinoGraphics.h";
#include "Arduino_LED_Matrix.h";
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Rotary.h";
#include "Button2.h";
#include "renderer.h";
#include "fanManager.h";
#include "stopWatch.h"

const int SERIAL_SPEED = 9600;
const int TEMP_SENSOR_DIGITAL_PIN = 2;

FanManager fanManager(67);

/*** LED MATRIX ***/
ArduinoLEDMatrix matrix;
uint8_t frame[8][12];
uint32_t timeofLastScroll = 0;

void render(
  uint8_t frameToShow[8][12]) {
  matrix.renderBitmap(frameToShow, 8, 12);
}

void setupMatrix() {
  matrix.begin();

  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  // add some static text
  // will only show "UNO" (not enough space on the display)
  const char text[] = "FAN";
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(text);
  matrix.endText();

  matrix.endDraw();

  delay(2000);

  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  // add some static text
  // will only show "UNO" (not enough space on the display)
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println("      ");
  matrix.endText();

  matrix.endDraw();
}

void serviceLedMatrix(
  int tempToShow) {
  matrix.beginDraw();

  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(200);

  // add the text
  String text = String(tempToShow); // + String("F");
  matrix.textFont(Font_5x7);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(text);
  matrix.endText(NO_SCROLL); //  SCROLL_LEFT);

  matrix.endDraw();
}

/*** TEMP PROBE ***/

// https://www.amazon.com/Replacement-Temperature-Humidity-Electronic-Practice/dp/B0DTHP4FGC/ref=sr_1_3_sspa?crid=32O8IXMXV11JB&dib=eyJ2IjoiMSJ9.Xs7bm5IGFitULL4ku2MBsb8h3E78np-GC6ppp-xfNQpUlxZnPwWp6KdplGjEXLUPp25g8CN1pHuuvrm_bgbu3OXJfQgaigs7d0sT5UdM8W2UBQXWDFpD3zeyHe-H2Hsd6NHX9vuQohMJ1QR0uH2D1lU0d6qoYcBhVb2oJmbohVUme-2uinsvctS8Zpnx1iY_Q5CqANOfXinE0U4g1YnEkgNUygHX0RiUdMRREVctIN1wXCo2oc_iGjomSHvsvbNPM6W-eUFY3OwHPax_ZPWJQh1JpgyLzOVu3h0WsoIGYCE.tQwCPgx7FJj9ET6APlQ34G-ZAJNtxumpCPmfGVxLQ0c&dib_tag=se&keywords=am2302&qid=1748717584&sprefix=am2302%2Caps%2C153&sr=8-3-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1
// This unit already has the pull-up resistor on the board.
// Wiring:
// "+""  -> Arduino 3V
// "out" -> Arduino Digital Pin 2
// "-"   -> Arduino GND
// Docs: https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHT_Unified_Sensor/DHT_Unified_Sensor.ino


#define DHTPIN 2       // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22  // DHT 22 (AM2302)
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
    Serial.print(F("Temperature: "));
    currentTemp = event.temperature;
    Serial.print(currentTemp);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }

  return getFarenheitFromCelsius(currentTemp);
}

/*** ROTARY ENCODER ***/

// Product page: https://www.amazon.com/dp/B07T3672VK?ref=ppx_yo2ov_dt_b_fed_asin_title

// https://github.com/skathir38/Rotary/blob/main/examples/SimpleCounterWithButton/SimpleCounterWithButton.ino
// https://github.com/LennartHennigs/Button2

// CLK = [pin 3]
// DT = [pin 4]
// SW = [pin 5] (Button)
// + = [Pin 5V]
// GND = [pin GND]

#define ROTARY_PIN1 3
#define ROTARY_PIN2 4
#define BUTTON_PIN 5

#define CLICKS_PER_STEP 4  // this number depends on your rotary encoder

Rotary r;
Button2 b;
StopWatch showNewTarget(1000);

void setupRotaryEncoder() {
  Serial.println("Setup rotary encoder");

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

#define PWN_PULSE_IN 2  // only 2 or 3 can be used as external intr's
#define PWN_MOTOR_OUT 5

int rotations = 0;
int motor_pwm = 128;

void pulse_isr() {
  rotations++;
}

void setupPwmFan() {
  pinMode(PWN_MOTOR_OUT, OUTPUT);
  analogWrite(PWN_MOTOR_OUT, motor_pwm);

  pinMode(PWN_PULSE_IN, INPUT_PULLUP);
  int intr = digitalPinToInterrupt(PWN_PULSE_IN);
  attachInterrupt(intr, pulse_isr, FALLING);
}

void serviceFan() {
  static uint32_t timer_start = millis();

  // this is for a period of 1 second (1000 ms). Adjust as needed:
  uint32_t period = 1000;

  // Get the current pulse counter and reset it to 0 if we have
  // finished a period.
  if (millis() - timer_start >= period) {
    timer_start = millis();

    noInterrupts();          // disable interrupts while we read
                             // the counter to avoid slicing
    int pulses = rotations;  // get the num rotations in the last 1000 ms
    rotations = 0;           // clear the counter for the next period
    interrupts();            // re-enable interrupts again

    // This should be one of the only two variables you really
    // have to mess with in addition to the period:
    int max_pulses = fanManager.isFanRunning() ? 1000 : 0;

    // Now we have a measurement of our velocity or "speed" in terms
    // of the number of rotations. So now we scale the motor speed
    // up or down relative to the desired speed:
    motor_pwm = map(pulses,
                    0, max_pulses,  // the range of rotation pulses from 0 - ???
                    255, 0          // the inverted range of analogWrite(...)
                                    // output from 0 to 255 (255 to 0 in this case)
    );

    analogWrite(PWN_MOTOR_OUT, motor_pwm);
  }
}


/*** MAIN ***/

void setup() {
  Serial.begin(SERIAL_SPEED);  //115200);

  setupTempProbe();
  setupRotaryEncoder();
  //setupPwmFan();
  setupMatrix();
}

void loop() {
  int currentTemp = serviceTempProbe();
  fanManager.updateTemperature(currentTemp);
  serviceRotaryEncoder();
  //serviceFan();

  int tempToShow = currentTemp;
  if (showNewTarget.isWaiting()) {
    tempToShow = fanManager.getTargetTemperature();
  }

  serviceLedMatrix(tempToShow);
}