#include <Wire.h>
#include "DFRobot_BMM350.h"

DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);

unsigned long int gettimecompass = 0;
// unsigned long int gettimepitch = 0;
float headingDegrees;
// float pitch;

namespace compass {

void setup() {
  Serial.begin(38400);
  Serial.println("BMM350 Magnetometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (bmm350.begin()) {
    /* There was a problem detecting the BMM350 ... check your connections */
    Serial.println("Ooops, no BMM350 detected ... Check your wiring!");
    while (1)
      ;
  }

  bmm350.setOperationMode(eBmm350NormalMode);
  bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY, BMM350_DATA_RATE_25HZ);
  bmm350.setMeasurementXYZ();
}

float heading() {
  if (millis() - gettimecompass >= 200) {
    gettimecompass = millis();
    headingDegrees = bmm350.getCompassDegree();
  }
  return headingDegrees;
}

// float getpitch() {
//   if (millis() - gettimepitch >= 250) {
//     gettimepitch = millis();
//
//     sBmm350MagData_t magData = bmm350.getGeomagneticData();
//     pitch = magData.x;
//   }
//   return pitch;
// }
}
