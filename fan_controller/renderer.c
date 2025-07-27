#include "renderer.h"

void clearFrame(
  uint8_t frameToChange[8][12]) {
  memset(frameToChange, 0, sizeof(uint8_t) * 8 * 12);
}

bool isBlinkFrame() {
  return (millis() % 1000) < 500;
}

void setStatusIndicator(
  uint8_t frameToChange[8][12]) {
  frameToChange[0][0] = (int)isBlinkFrame();
}