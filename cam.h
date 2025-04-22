#include <Wire.h>
#define SLAVE_ADDRESS 0x08

namespace cam{
int camx,camy;
int bataskanan = 350;
int bataskiri = 280;

struct Data {
  byte id;
  byte value;
};

Data receivedData [4]; //array untuk menyimpan 4 data yang diterima
void receiveData(int byteCount) {
  int dataregister = Wire1.read();
  if(dataregister==0){
  if (byteCount >= sizeof(Data)) {
    // Mengabaikan register atau byte pertama
    

    int numDataPairs = (byteCount - 1) / sizeof(Data); // Hitung jumlah pasangan data

    for (int i = 0; i < 4;i++) {
      receivedData[i].value = Wire1.read(); // Baca nilai data
    }
  int byte1x = receivedData[0].value;
  int byte2x = receivedData[1].value;
  int byte1y = receivedData[2].value;
  int byte2y = receivedData[3].value;
   camx = byte1x * 256 + byte2x;
   camy = byte1y * 256 + byte2y;
//   Serial.print(String()+"Data x :"+x);
//   Serial.print("\t");
//   Serial.print(String()+"Data y :"+y);
//   Serial.println("");
  } else {
    // Jika jumlah byte yang diterima tidak cukup untuk satu struct Data, beri pesan kesalahan

  }}
  else{}
}

void setup1(){
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.onReceive(receiveData);
  Wire1.begin(0x03);
}

}
