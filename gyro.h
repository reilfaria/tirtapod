#include "MPU9250.h"

MPU9250 mpu;
float pitch;

unsigned long int gettimepitch = 0;

namespace gyro {

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  mpu.setup(0x68);  // change to your own address
}

float getpitch() {
  // if (millis() - gettimepitch >= 250) {
  gettimepitch = millis();
  if (mpu.update()) {
    pitch = mpu.getPitch();
  }

  // }
  return pitch;
}

}


// #include "MPU9250.h"

// MPU9250 mpu;  // You can also use MPU9255 as is

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   delay(2000);

//   mpu.setup(0x68);  // change to your own address
// }

// void loop() {
//   if (mpu.update()) {
//     Serial.print(mpu.getYaw());
//     Serial.print(", ");
//     Serial.print(mpu.getPitch());
//     Serial.print(", ");
//     Serial.println(mpu.getRoll());
//  }
//}
