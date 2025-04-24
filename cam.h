#include <Wire.h>
#define SLAVE_ADDRESS 0x08

namespace cam {
int camx = -1, camy = -1;
int camx_dummy = -1, camy_dummy = -1;
int bataskanan = 350;
int bataskiri = 280;

struct Data {
  byte id;
  byte value;
};

Data receivedData[10]; // array untuk menyimpan 9 data yang diterima

void receiveData(int byteCount) {
  if (byteCount > 0) {
    int dataregister = Wire1.read();
    if (dataregister == 0) {
      if (byteCount >= sizeof(Data) * 2 + 2) { // Memastikan jumlah byte cukup untuk 8 data dan 1 id
        for (int i = 0; i < byteCount - 1; i++) {
          receivedData[i].value = Wire1.read(); // Baca nilai data
        }

        // Mengurai data berdasarkan ID
        int index = 0;
        while (index < byteCount - 1) {
          byte id = receivedData[index].value;
          if (id == 1) { // ID 1 untuk korban
            int byte1x_korban = receivedData[index + 1].value;
            int byte2x_korban = receivedData[index + 2].value;
            int byte1y_korban = receivedData[index + 3].value;
            int byte2y_korban = receivedData[index + 4].value;
            camx = byte1x_korban * 256 + byte2x_korban;
            camy = byte1y_korban * 256 + byte2y_korban;
            index += 5;
          } else if (id == 2) { // ID 2 untuk dummy
            int byte1x_dummy = receivedData[index + 1].value;
            int byte2x_dummy = receivedData[index + 2].value;
            int byte1y_dummy = receivedData[index + 3].value;
            int byte2y_dummy = receivedData[index + 4].value;
            camx_dummy = byte1x_dummy * 256 + byte2x_dummy;
            camy_dummy = byte1y_dummy * 256 + byte2y_dummy;
            index += 5;
          } else {
            // Jika ID tidak diketahui, hentikan pengolahan
            break;
          }
        }


      } else {
        // Jika jumlah byte yang diterima tidak cukup, beri pesan kesalahan
      }
    }
  }
}

void setup1() {
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.onReceive(receiveData);
  Wire1.begin(0x03);

}
}
