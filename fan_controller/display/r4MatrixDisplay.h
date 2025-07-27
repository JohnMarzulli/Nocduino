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
        matrix.print("FAN");
        matrix.endText();

        matrix.endDraw();
    }

    void service(
        int tempToShow,
        float fanPowerProportion,
        char targetSuffix) override
    {
        /*
        matrix.beginDraw();

        matrix.stroke(0xFFFFFFFF);
        matrix.textScrollSpeed(200);

        // add the text
        String text = String(tempToShow);  // + String("F");
        matrix.textFont(Font_5x7);
        matrix.beginText(0, 1, 0xFFFFFF);
        matrix.println(text);
        matrix.endText(NO_SCROLL);  //  SCROLL_LEFT);

        matrix.endDraw();
        */
    }

private:
    ArduinoLEDMatrix matrix;

    int lastTempShown = -1;
};

#endif // R4_MATRIX_DISPLAY_H