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
#include "display/display.h"
#include "display/r4MatrixDisplay.h"
#include "display/max72xxDisplay.h"
#include "temperatureBasedFanManager.h"
#include "powerBasedFanManager.h"
#include "inputManager.h"

const int SERIAL_SPEED = 9600;

enum ControlMode {
  Temperature = 0,
  FanSpeed
};

ControlMode controlMode(ControlMode::Temperature);
TemperatureManager temperatureManager(67);
Display *display = NULL;
SensorManager sensorManager(SensorManager::DHT_PIN);
TemperatureBasedFanManager temperatureBasedFanManager;
PowerBasedFanManager powerBasedFanManager;
InputManager inputManager(&temperatureManager);
StopWatch showNewTarget(1000);

void handleKnobClick(Button2 &btn) {
  if (controlMode == ControlMode::Temperature) {
    controlMode = ControlMode::FanSpeed;
  } else {
    controlMode = ControlMode::Temperature;
  }

  Serial.println(F("Click!"));

  showNewTarget.reset();
}

void handleKnobClockwise(Rotary &r) {
  if (controlMode == ControlMode::Temperature) {
    temperatureManager.increaseTargetTemperature();
  } else {
    powerBasedFanManager.increasePower();
  }

  Serial.println(F("CW!"));

  showNewTarget.reset();
}

void handleKnobCounterClockwise(Rotary &r) {
  if (controlMode == ControlMode::Temperature) {
    temperatureManager.decreaseTargetTemperature();
  } else {
    powerBasedFanManager.decreasePower();
  }

  Serial.println(F("CCW!"));

  showNewTarget.reset();
}

void setup() {
  Serial.begin(SERIAL_SPEED);  // 115200);

  sensorManager.setupTempProbe();
  inputManager.setupRotaryEncoder(handleKnobClick, handleKnobClockwise, handleKnobCounterClockwise);
  temperatureBasedFanManager.setupPwmFan();


//#ifdef ARDUINO_UNOR4_WIFI
  Serial.println(F("Selecting R4 Matrix"));
  display = new R4MatrixDisplay();
//#else
//  Serial.println(F("Selecting MAX72xx"));
//  display = new Max72xxDisplay();
//#endif

  display->setup();
}

void loop() {
  bool isTempTarget = controlMode == ControlMode::Temperature;
  sensorManager.serviceTempProbe();
  temperatureManager.updateTemperature(sensorManager);
  inputManager.serviceRotaryEncoder();
  float powerProportion = 1.0;

  if (isTempTarget) {
    powerProportion = temperatureBasedFanManager.serviceFan(temperatureManager);
  } else {
    powerProportion = powerBasedFanManager.serviceFan();
  }

  int targetToShow = sensorManager.GetCurrentTemperature();
  char targetSuffix = 'F';

  if (showNewTarget.isWaiting()) {
    targetToShow = isTempTarget
                     ? temperatureManager.getTargetTemperature()
                     : (int)(powerProportion * 100.0);
    targetSuffix = isTempTarget
                     ? 'F'
                     : '%';
  }

  display->service(targetToShow, powerProportion, targetSuffix);
}