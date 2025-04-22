#include <Wire.h>
#include <Adafruit_VL53L0X.h>

unsigned long int gettimekanan = 0;
unsigned long int gettimeserongkanan = 0;
unsigned long int gettimedepan = 0;
unsigned long int gettimeserongkiri = 0;
unsigned long int gettimekiri = 0;
unsigned long int gettimecapit = 0;

namespace TOF {

// Membuat instance untuk VL53L0X
Adafruit_VL53L0X lox[7];  // Array untuk menampung 5 sensor VL53L0X

#define PCAADDR 0x70  // Alamat I2C default untuk PCA9548A

int kanan, serongkanan, depan, serongkiri, kiri, capit;

void pcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(38400);
  Wire.begin();

  // Inisialisasi masing-masing sensor VL53L0X pada channel 2 sampai 6
  for (uint8_t i = 0; i <= 5; i++) {
    pcaSelect(i);  // Pilih kanal PCA9548A
    if (!lox[i + 1].begin()) {
      Serial.print("Failed to boot VL53L0X on channel ");
      Serial.println(i);
      while (1)
        ;
    }
    Serial.print("VL53L0X found on channel ");
    Serial.println(i);
  }
  delay(200);
}

// void loop() {
//   // Membaca data dari masing-masing sensor VL53L0X pada channel 2 sampai 6
//   for (uint8_t i = 2; i <= 6; i++) {
//     pcaSelect(i);  // Pilih kanal PCA9548A
//     VL53L0X_RangingMeasurementData_t measure;
//     lox[i - 2].rangingTest(&measure, false);  // parameter kedua false berarti tidak menampilkan debug data
//     if (measure.RangeStatus != 4) {
//       Serial.print(String() + measure.RangeMilliMeter + " || ");
//     } else {
//       Serial.print("MAX || ");
//     }
//   }
//   Serial.println();
// }

int getkanan() {
  if (millis() - gettimekanan >= 200) {
     gettimekanan = millis();
     
    pcaSelect(0);  // Pilih kanal PCA9548A
    VL53L0X_RangingMeasurementData_t measure;
    lox[1].rangingTest(&measure, false);  // parameter kedua false berarti tidak menampilkan debug data
    if (measure.RangeStatus != 4) {
      kanan = measure.RangeMilliMeter;
    } else {
      kanan = 6000;
    }
  }
  return kanan;
}

int getserongkanan() {
  if (millis() - gettimeserongkanan >= 200) {
     gettimeserongkanan = millis();
    
    pcaSelect(1);  // Pilih kanal PCA9548A
    VL53L0X_RangingMeasurementData_t measure;
    lox[2].rangingTest(&measure, false);  // parameter kedua false berarti tidak menampilkan debug data
    if (measure.RangeStatus != 4) {
      serongkanan = measure.RangeMilliMeter;
    } else {
      serongkanan = 6000;
    }
  }
  return serongkanan;
}

int getdepan() {
  if (millis() - gettimedepan >= 200) {
     gettimedepan = millis();
  
    pcaSelect(2);  // Pilih kanal PCA9548A
    VL53L0X_RangingMeasurementData_t measure;
    lox[3].rangingTest(&measure, false);  // parameter kedua false berarti tidak menampilkan debug data
    if (measure.RangeStatus != 4) {
      depan = measure.RangeMilliMeter;
    } else {
      depan = 6000;
    }
  }
  return depan;
}

int getserongkiri() {
  if (millis() - gettimeserongkiri >= 200) {
     gettimeserongkiri = millis();
  
    pcaSelect(3);  // Pilih kanal PCA9548A
    VL53L0X_RangingMeasurementData_t measure;
    lox[4].rangingTest(&measure, false);  // parameter kedua false berarti tidak menampilkan debug data
    if (measure.RangeStatus != 4) {
      serongkiri = measure.RangeMilliMeter;
    } else {
      serongkiri = 6000;
    }
  }
  return serongkiri;
}

int getkiri() {
  if (millis() - gettimekiri >= 200) {
     gettimekiri = millis();
  
    pcaSelect(4);  // Pilih kanal PCA9548A
    VL53L0X_RangingMeasurementData_t measure;
    lox[5].rangingTest(&measure, false);  // parameter kedua false berarti tidak menampilkan debug data
    if (measure.RangeStatus != 4) {
      kiri = measure.RangeMilliMeter;
    } else {
      kiri = 6000;
    }
  }
  return kiri;
}

int getcapit() {
  if (millis() - gettimecapit >= 200) {
     gettimecapit = millis();

    pcaSelect(5);  // Pilih kanal PCA9548A
    VL53L0X_RangingMeasurementData_t measure;
    lox[6].rangingTest(&measure, false);  // parameter kedua false berarti tidak menampilkan debug data
    if (measure.RangeStatus != 4) {
      capit = measure.RangeMilliMeter;
    } else {
      capit = 6000;
    }
  }
  return capit;
}



}
