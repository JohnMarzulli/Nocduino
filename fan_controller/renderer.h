#ifndef RENDERER_H
#define RENDERER_H

#include <Arduino.h>

void clearFrame(uint8_t frameToChange[8][12]);
bool isBlinkFrame();
void setStatusIndicator(uint8_t frameToChange[8][12]);

#endif