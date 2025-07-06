#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "stopWatch.h"

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

/*** TEMP PROBE ***/

// https://www.amazon.com/Replacement-Temperature-Humidity-Electronic-Practice/dp/B0DTHP4FGC/ref=sr_1_3_sspa?crid=32O8IXMXV11JB&dib=eyJ2IjoiMSJ9.Xs7bm5IGFitULL4ku2MBsb8h3E78np-GC6ppp-xfNQpUlxZnPwWp6KdplGjEXLUPp25g8CN1pHuuvrm_bgbu3OXJfQgaigs7d0sT5UdM8W2UBQXWDFpD3zeyHe-H2Hsd6NHX9vuQohMJ1QR0uH2D1lU0d6qoYcBhVb2oJmbohVUme-2uinsvctS8Zpnx1iY_Q5CqANOfXinE0U4g1YnEkgNUygHX0RiUdMRREVctIN1wXCo2oc_iGjomSHvsvbNPM6W-eUFY3OwHPax_ZPWJQh1JpgyLzOVu3h0WsoIGYCE.tQwCPgx7FJj9ET6APlQ34G-ZAJNtxumpCPmfGVxLQ0c&dib_tag=se&keywords=am2302&qid=1748717584&sprefix=am2302%2Caps%2C153&sr=8-3-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1
// This unit already has the pull-up resistor on the board.
// Wiring:
// "+""  -> Arduino 3V
// "out" -> Arduino Digital Pin 2
// "-"   -> Arduino GND
// Docs: https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHT_Unified_Sensor/DHT_Unified_Sensor.ino

class SensorManager {
public:
  static const int DHT_PIN = 2;  // Digital pin connected to the DHT sensor

  SensorManager(
    int sensorPin) {
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
      return lastTempMeasured;
    }

    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    } else {
      currentTemp = event.temperature;
#ifdef SERIAL_DEBUG_OUTPUT
      Serial.print(F("Temperature: "));
      Serial.print(currentTemp);
      Serial.println(F("°C"));
#endif  // SERIAL_DEBUG_OUTPUT
    }

    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
#ifdef SERIAL_DEBUG_OUTPUT
    else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
    }
#endif  // SERIAL_DEBUG_OUTPUT

    lastTempMeasured = getFarenheitFromCelsius(currentTemp);

    return lastTempMeasured;
  }

private:
  int getFarenheitFromCelsius(
    float tempInCelsius) {
    return (int)((tempInCelsius * 9 / 5) + 32);
  }

  int lastTempMeasured = 0;

  StopWatch probeInterval = StopWatch(2000);
  // See guide for details on sensor wiring and usage:
  //   https://learn.adafruit.com/dht/overview

  DHT_Unified dht = DHT_Unified(DHT_PIN, DHTTYPE);

  const int DHTTYPE = DHT22;  // DHT 22 (AM2302)
};

#endif  // SENSOR_MANAGER_H