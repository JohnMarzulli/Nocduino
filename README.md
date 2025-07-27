/*** FAN CONTROLLER ***/

// https://www.reddit.com/r/arduino/comments/14nung1/expertise_needed_for_4pin_pwm_powering/


/*** LED MATRIX ***/
// https://lastminuteengineers.com/max7219-dot-matrix-arduino-tutorial/

/*** TEMP PROBE ***/

// https://www.amazon.com/Replacement-Temperature-Humidity-Electronic-Practice/dp/B0DTHP4FGC/ref=sr_1_3_sspa?crid=32O8IXMXV11JB&dib=eyJ2IjoiMSJ9.Xs7bm5IGFitULL4ku2MBsb8h3E78np-GC6ppp-xfNQpUlxZnPwWp6KdplGjEXLUPp25g8CN1pHuuvrm_bgbu3OXJfQgaigs7d0sT5UdM8W2UBQXWDFpD3zeyHe-H2Hsd6NHX9vuQohMJ1QR0uH2D1lU0d6qoYcBhVb2oJmbohVUme-2uinsvctS8Zpnx1iY_Q5CqANOfXinE0U4g1YnEkgNUygHX0RiUdMRREVctIN1wXCo2oc_iGjomSHvsvbNPM6W-eUFY3OwHPax_ZPWJQh1JpgyLzOVu3h0WsoIGYCE.tQwCPgx7FJj9ET6APlQ34G-ZAJNtxumpCPmfGVxLQ0c&dib_tag=se&keywords=am2302&qid=1748717584&sprefix=am2302%2Caps%2C153&sr=8-3-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1
// This unit already has the pull-up resistor on the board.
// Wiring:
// "+""  -> Arduino 3V
// "out" -> Arduino Digital Pin 2
// "-"   -> Arduino GND
// Docs: https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHT_Unified_Sensor/DHT_Unified_Sensor.ino

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview


/*** ROTARY ENCODER ***/

// Product page: https://www.amazon.com/dp/B07T3672VK?ref=ppx_yo2ov_dt_b_fed_asin_title

// https://github.com/skathir38/Rotary/blob/main/examples/SimpleCounterWithButton/SimpleCounterWithButton.ino
// https://github.com/LennartHennigs/Button2

// CLK = [pin 5]
// DT = [pin 4]
// SW = [pin 6] (Button)
// + = [Pin 8]
// GND = [pin GND]