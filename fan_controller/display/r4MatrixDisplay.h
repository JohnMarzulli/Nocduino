#ifndef R4_MATRIX_DISPLAY_H
#define R4_MATRIX_DISPLAY_H

#include "display.h"

#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"

class R4MatrixDisplay : public Display
{
public:
    void setup() override
    {
        matrix.begin();

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

    void service(
        int tempToShow,
        float fanPowerProportion,
        char targetSuffix) override
    {
        uint8_t frame[8][12] = {0};

        fanPowerProportion = fanPowerProportion >= 1.0 ? 1.0 : fanPowerProportion;
        fanPowerProportion = fanPowerProportion <= 0.0 ? 0.0 : fanPowerProportion;

        int ledsToShow = static_cast<int>(fanPowerProportion * 8.0);
        bool isBlink = (millis() % 1000) <= 500;

        matrix.beginDraw();
        bool isOneDrawn = false;

        // Draw the LEDs
        for (int i = 0; i < ledsToShow; ++i)
        {
            isOneDrawn = true;

            frame[0][i] = 1;
            frame[11][i] = 1;
        }

        if (!isOneDrawn)
        {
            frame[0][0] = isBlink;
            frame[11][0] = isBlink;
        }

        matrix.renderBitmap(frame, 8, 12);

        matrix.textFont(Font_4x6);
        matrix.beginText(0, 1, 0xFFFFFF);
        String text = String(tempToShow) + String(targetSuffix);
        matrix.println(text);
        matrix.endText(NO_SCROLL);

        lastTempShown = tempToShow;

        matrix.endDraw();
    }

private:
    ArduinoLEDMatrix matrix;

    int lastTempShown = -1;
};

#endif // R4_MATRIX_DISPLAY_H