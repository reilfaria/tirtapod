#include <Wire.h>
#include "DFRobot_BMM350.h"

DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);

unsigned long gettimecompass = 0;
float headingDegrees = 0;

// ISI dengan hasil kalibrasi kamu
float minX = -4.00, maxX = 77.0;
float minY = -58.0, maxY = 26.0;

namespace compass {

void setup() {
  Serial.begin(38400);
  if (bmm350.begin()) {
    Serial.println("Ooops, no BMM350 detected ... Check your wiring!");
    while (1);
  }
  bmm350.setOperationMode(eBmm350NormalMode);
  bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY, BMM350_DATA_RATE_25HZ);
  bmm350.setMeasurementXYZ();
}

float heading() {
  if (millis() - gettimecompass >= 200) {
    gettimecompass = millis();

    sBmm350MagData_t d = bmm350.getGeomagneticData();

    // Hard iron: geser ke tengah
    float offsetX = (maxX + minX) / 2.0;
    float offsetY = (maxY + minY) / 2.0;

    // Soft iron sederhana: samakan skala
    float rangeX = (maxX - minX) / 2.0;
    float rangeY = (maxY - minY) / 2.0;
    float avgRange = (rangeX + rangeY) / 2.0;

    float x = (d.x - offsetX) * (avgRange / rangeX);
    float y = (d.y - offsetY) * (avgRange / rangeY);

    float h = atan2(y, x) * 180.0 / PI;
    if (h < 0) h += 360.0;

    // Low-pass filter (circular) biar tidak noisy
    float diff = h - headingDegrees;
    if (diff > 180)  diff -= 360;
    if (diff < -180) diff += 360;
    headingDegrees += 0.3 * diff;   // 0.1 = halus, 0.5 = responsif
    if (headingDegrees < 0)    headingDegrees += 360;
    if (headingDegrees >= 360) headingDegrees -= 360;
  }
  return headingDegrees;
}

}
