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
#include "fanDriver.h"
#include "inputManager.h"

const int SERIAL_SPEED = 9600;

TemperatureManager temperatureManager(67);
MatrixDisplay display(MatrixDisplay::INTENSITY_DIM);
SensorManager sensorManager(SensorManager::DHT_PIN);
FanDriver fanDriver;
InputManager inputManager(&temperatureManager);

/*** MAIN ***/

void setup() {
  Serial.begin(SERIAL_SPEED);  // 115200);

  sensorManager.setupTempProbe();
  inputManager.setupRotaryEncoder();
  fanDriver.setupPwmFan();
  display.setupMatrix();
}

void loop() {
  int currentTemp = sensorManager.serviceTempProbe();
  temperatureManager.updateTemperature(currentTemp);
  inputManager.serviceRotaryEncoder();
  float powerProportion = fanDriver.serviceFan(temperatureManager);

  int tempToShow = currentTemp;
  if (showNewTarget.isWaiting()) {
    tempToShow = temperatureManager.getTargetTemperature();
  }

  display.serviceLedMatrix(tempToShow, powerProportion);
}