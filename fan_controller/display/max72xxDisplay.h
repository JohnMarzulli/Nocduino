#ifndef MAX72XX_DISPLAY_H
#define MAX72XX_DISPLAY_H

#include "display.h"

#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

/*** LED MATRIX ***/
// https://lastminuteengineers.com/max7219-dot-matrix-arduino-tutorial/
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

class Max72xxDisplay : public Display
{
public:
  static const int INTENSITY_BRIGHT = 15;
  static const int INTENSITY_MID = 7;
  static const int INTENSITY_DIM = 0;

  Max72xxDisplay()
  {
    brightness = INTENSITY_DIM;
  }

  void setup() override
  {
    myDisplay.begin();

    myDisplay.setIntensity(brightness);
    myDisplay.setInvert(0);
    myDisplay.displayClear();
  }

  void service(
      int tempToShow,
      float fanPowerProportion,
      char targetSuffix) override
  {

    fanPowerProportion = fanPowerProportion >= 1.0 ? 1.0 : fanPowerProportion;
    fanPowerProportion = fanPowerProportion <= 0.0 ? 0.0 : fanPowerProportion;

    float ledsToShow = (fanPowerProportion * 8.0);
    bool isBlink = (millis() % 1000) <= 500;

    if (tempToShow != lastTempShown)
    {
      myDisplay.displayClear();
      myDisplay.setIntensity(brightness);
      myDisplay.setInvert(0);
      myDisplay.setTextAlignment(PA_CENTER);
      // add the text
      String text = String(tempToShow) + String(targetSuffix);
      myDisplay.print(text);
      lastTempShown = tempToShow;
    }

#ifdef SERIAL_DEBUG_OUTPUT
    ++serialSpamLimiter;

    if (serialSpamLimiter == 5000)
    {
      Serial.print(F("fanPowerProportion: "));
      Serial.println(fanPowerProportion);
      Serial.print(F("LEDS to show: "));
      Serial.println(ledsToShow);

      serialSpamLimiter = 0;
    }
#endif // SERIAL_DEBUG_OUTPUT

    for (int ledIndex = 1; ledIndex <= 7; ++ledIndex)
    {
      bool isShown = ledIndex <= (ledsToShow - 1);
      int vertIndex = 7 - ledIndex;
      // Right edge
      myDisplay.getGraphicObject()->setPoint(vertIndex, 0, isShown);
      // Left edge
      myDisplay.getGraphicObject()->setPoint(vertIndex, 31, isShown);
    }

    bool isLastLedLit = (ledsToShow > 0) || isBlink;

    myDisplay.getGraphicObject()->setPoint(7, 0, isLastLedLit);
    myDisplay.getGraphicObject()->setPoint(7, 31, isLastLedLit);
  }

private:
  const int MAX_DEVICES = 4;
  const int CS_PIN = 3;

  MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
  int lastTempShown = 0;
  int brightness = 0;

#ifdef SERIAL_DEBUG_OUTPUT
  int serialSpamLimiter = 0;
#endif // SERIAL_DEBUG_OUTPUT
};

#endif // MAX72XX_DISPLAY_H