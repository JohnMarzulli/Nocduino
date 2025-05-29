// NOTE:
// Please make sure you install the "Arduino UNO R4 Boards" pack
// using the "BOARDS MANAGER"
//
// You will need to install the following packages:
// "AM2303-Sensor" by Frank Hafele - https://github.com/hasenradball/AM2302-Sensor
// "Button2" by Lennart Hunnigs - https://github.com/LennartHennigs/Button2
// "Rotary" by KAthiR - https://github.com/skathir38/Rotary

#include "Arduino_LED_Matrix.h";
#include <AM2302-Sensor.h>;
#include "Rotary.h";
#include "Button2.h";
#include "renderer.h";
#include "fanManager.h";;

FanManager fanManager(67);

/*** LED MATRIX ***/
ArduinoLEDMatrix matrix;
uint8_t frame[8][12];

void render(
  uint8_t frameToShow[8][12]) {
  matrix.renderBitmap(frameToShow, 8, 12);
}

/*** TEMP PROBE ***/

// Docs: https://github.com/hasenradball/AM2302-Sensor
// https://www.instructables.com/Arduino-and-DHT22-AM2302-Temperature-Measurement/

constexpr unsigned int SENSOR_PIN{ 7U };
AM2302::AM2302_Sensor am2302{ SENSOR_PIN };

void setupTempProbe() {
  while (!Serial) {
    yield();
  }
  Serial.print(F("\n >>> AM2302-Sensor_Example <<<\n\n"));

  // set pin and check for sensor
  if (am2302.begin()) {
    // this delay is needed to receive valid data,
    // when the loop directly read again
    delay(3000);
  } else {
    while (true) {
      Serial.println("Error: sensor check. => Please check sensor connection!");
      delay(10000);
    }
  }
}

void serviceTempProbe() {
  auto status = am2302.read();
  Serial.print("\n\nstatus of sensor read(): ");
  Serial.println(status);

  int currentTemp = am2302.get_Temperature();
  fanManager.updateTemperature(currentTemp);

  Serial.print("Humidity:    ");
  Serial.println(am2302.get_Humidity());
  delay(5000);
}

/*** ROTARY ENCODER ***/

// Product page: https://www.amazon.com/dp/B07T3672VK?ref=ppx_yo2ov_dt_b_fed_asin_title

// https://github.com/skathir38/Rotary/blob/main/examples/SimpleCounterWithButton/SimpleCounterWithButton.ino
// https://github.com/LennartHennigs/Button2

// CLK = [pin 3]
// DT = [pin 4]
// Button = [pin 5]
// + = [Pin 5V]
// GND = [pin GND]

#define ROTARY_PIN1 3
#define ROTARY_PIN2 4
#define BUTTON_PIN 5

#define CLICKS_PER_STEP 4  // this number depends on your rotary encoder

Rotary r;
Button2 b;

void setupRotaryEncoderAndButton() {
  r.begin(ROTARY_PIN1, ROTARY_PIN2, CLICKS_PER_STEP);
  r.setChangedHandler(rotate);
  r.setLeftRotationHandler(rotateCounterClockwise);
  r.setRightRotationHandler(rotateClockwise);

  b.begin(BUTTON_PIN);
  b.setTapHandler(click);
  b.setLongClickHandler(resetPosition);
}

void rotate(Rotary& r) {
  Serial.println(r.getPosition());
}

// on left or right rotation
void rotateCounterClockwise(Rotary& r) {
  fanManager.decreaseTargetTemperature();
}

void rotateClockwise(Rotary& r) {
  fanManager.increaseTargetTemperature();
}

// single click
void click(Button2& btn) {
  Serial.println("Click!");
}

// long click
void resetPosition(Button2& btn) {
  r.resetPosition();
  Serial.println("Reset!");
}

void serviceRotaryEncoder() {
  r.loop();
  b.loop();
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
  Serial.begin(115200);

  setupTempProbe();
  setupRotaryEncoderAndButton();
  setupPwmFan();
}

void loop() {
  serviceTempProbe();
  serviceFan();
}