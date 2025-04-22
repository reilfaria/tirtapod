#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

unsigned long int gettimecompass = 0;
// unsigned long int gettimepitch = 0;
float headingDegrees;
// float pitch;

namespace compass {

void setup() {
  Serial.begin(38400);
  Serial.println("HMC5883 Magnetometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
      ;
  }
}

float heading() {
  if (millis() - gettimecompass >= 200) {
    gettimecompass = millis();

    sensors_event_t event;
    mag.getEvent(&event);

    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = 0.00755;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
      heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
      heading -= 2 * PI;

    // Convert radians to degrees for readability.
    headingDegrees = heading * 180 / M_PI;
  }
  return headingDegrees;
}

// float getpitch() {
//   if (millis() - gettimepitch >= 250) {
//     gettimepitch = millis();

//     sensors_event_t event;
//     mag.getEvent(&event);

//     pitch = event.magnetic.x;
//   }
//   return pitch;
// }
}
