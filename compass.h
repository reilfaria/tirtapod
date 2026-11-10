#include <Wire.h>
#include <math.h>
#include "DFRobot_BMM350.h"

DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);

unsigned long gettimecompass = 0;

float headingDegrees = 0;
bool headingInitialized = false;

// Kalibrasi magnetometer (nilai awal)
float minX = -4.00, maxX = 77.0;
float minY = -58.0, maxY = 26.0;

// Lokasi Lomba TIRTAPOD
// Latitude  : -7.2774367482
// Longitude : 112.7973752806

// Deklinasi magnetik WMM2025
// NOAA, 12 November 2026
// D = 0.765 derajat East
float declinationDegrees = 0.765;

// Low-pass filter
const float filterAlpha = 0.3;

// Interval pembacaan sensor
const unsigned long compassInterval = 200;

namespace compass {

void setup() {
  Serial.begin(38400);

  if (bmm350.begin()) {
    Serial.println("Ooops, no BMM350 detected ... Check your wiring!");
    while (1);
  }

  bmm350.setOperationMode(eBmm350NormalMode);

  bmm350.setPresetMode(
    BMM350_PRESETMODE_HIGHACCURACY,
    BMM350_DATA_RATE_25HZ
  );

  bmm350.setMeasurementXYZ();
}

float heading() {

  if (millis() - gettimecompass >= compassInterval) {
    gettimecompass = millis();

    sBmm350MagData_t d = bmm350.getGeomagneticData();

    // Hard iron correction
    float offsetX = (maxX + minX) / 2.0;
    float offsetY = (maxY + minY) / 2.0;

    // Soft iron correction
    float rangeX = (maxX - minX) / 2.0;
    float rangeY = (maxY - minY) / 2.0;

    float avgRange = (rangeX + rangeY) / 2.0;

    // Magnetometer calibration
    float x = (d.x - offsetX) * (avgRange / rangeX);
    float y = (d.y - offsetY) * (avgRange / rangeY);

    // Magnetic heading
    float h = atan2(y, x) * 180.0 / PI;

    // Magnetic declination correction
    h += declinationDegrees;

    // Normalize heading (0 - 360 degrees)
    h = fmod(h, 360.0);

    if (h < 0)
      h += 360.0;

    // Initialize filter with first reading
    if (!headingInitialized) {
      headingDegrees = h;
      headingInitialized = true;
    }

    else {
      // Circular low-pass filter
      float diff = h - headingDegrees;

      if (diff > 180)
        diff -= 360;

      if (diff < -180)
        diff += 360;

      headingDegrees += filterAlpha * diff;

      // Normalize filtered heading
      if (headingDegrees < 0)
        headingDegrees += 360;

      if (headingDegrees >= 360)
        headingDegrees -= 360;
    }
  }

  return headingDegrees;
}

}