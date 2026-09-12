// #include "pidmpu.h"
#include <math.h>
int state_nextStep;
int state_nextStep6;
namespace legs {
//========Spesifikasi Kaki===========//
const float cx = 2.8;  //coxa
const float fm = 5.5;  //femur
const float tb = 7.4;  // tibia
float kakiNaik, kakiTurun, travel, travelhalf, X1, X2, Y1, Y2, width;
//========Spesifikasi Kaki===========//

//========Instrumen Inverse Kinematic=====//
float L, L1;
float alpha, alpha1, alpha2, alpha3, gama, beta, beta2;
unsigned long int state_lastMoveRecord = 0;
//========Instrumen Inverse Kinematic=====//

//============ENUM=============//
enum type {
  eforward,
  eforward6,
  ebackward,
  ebackward6,
  erotate_left,
  erotate6_left,
  erotate_right,
  erotate6_right,
  eshift_left,
  eshift6_left,
  eshift_right,
  eshift6_right,
  eforward6_setupmpuup,
  eforward6_mpuup,
  eturn6_left_mpuup,
  eturn6_right_mpuup,
  eforward6_stuckmpuup,
  epoint0,
  epoint_ready,
  epoint_backward,
  epoint_forward,
  epoint_rotate_right,
  epoint_rotate_left,
  ewave_hello, // baruuuu
  edance,
  esapu,
};

type move;
// void setup() {
//   Serial2.begin(115200);
//   Serial.begin(38400);
//   // getmpu::setup();
// }


void setup() {
  Serial2.setRX(9);
  Serial2.setTX(8);
  Serial2.begin(115200);
  Serial.begin(38400);
  // getmpu::setup();
}

//=================================START CODE FUNCTION OF CONVERT==========================================///
//Konversi Sudut
String konversisudut(int x) {
  String sudut = String(x);
  return sudut;
}
int myFunctionPWM(float sudut) {
  float result, result2;
  result = 1000 * sudut;
  result2 = result / 90;
  return result2;
}
//==================================END CODE FUNCTION OF CONVERT=========================================


//===================================Start Get Sudut=============================================
//===Inverse Kaki Kiri
float getgamaL(float x, float y, float z) {
  L1 = sqrt(sq(x) + sq(y));
  gama = atan(x / y) / PI * 180;
  L = sqrt(sq(L1 - cx) + sq(z));
  beta = acos((sq(tb) + sq(fm) - sq(L)) / (2 * tb * fm)) / PI * 180;
  alpha1 = acos(z / L) / PI * 180;
  alpha2 = acos((sq(fm) + sq(L) - sq(tb)) / (2 * fm * L)) / PI * 180;
  alpha = alpha1 + alpha2;
  return -gama;  // invers 90-a
}
float getalphaL(float x, float y, float z) {
  L1 = sqrt(sq(x) + sq(y));
  gama = atan(x / y) / PI * 180;
  L = sqrt(sq(L1 - cx) + sq(z));
  beta = acos((sq(tb) + sq(fm) - sq(L)) / (2 * tb * fm)) / PI * 180;
  alpha1 = acos(z / L) / PI * 180;
  alpha2 = acos((sq(fm) + sq(L) - sq(tb)) / (2 * fm * L)) / PI * 180;
  alpha = alpha1 + alpha2;
  beta2 = beta;
  //  alpha2=90-alpha;
  return alpha;  // invers 180-a
}
float getbetaL(float x, float y, float z) {
  L1 = sqrt(sq(x) + sq(y));
  gama = atan(x / y) / PI * 180;
  L = sqrt(sq(L1 - cx) + sq(z));
  beta = acos((sq(tb) + sq(fm) - sq(L)) / (2 * tb * fm)) / PI * 180;
  alpha1 = acos(z / L) / PI * 180;
  alpha2 = acos((sq(fm) + sq(L) - sq(tb)) / (2 * fm * L)) / PI * 180;
  alpha = alpha1 + alpha2;
  //  beta2=90-beta;
  return 180 - beta;
}
//======Inverse Kaki Kanan
float getgamaR(float x, float y, float z) {
  L1 = sqrt(sq(x) + sq(y));
  gama = atan(x / y) / PI * 180;
  L = sqrt(sq(L1 - cx) + sq(z));
  beta = acos((sq(tb) + sq(fm) - sq(L)) / (2 * tb * fm)) / PI * 180;
  alpha1 = acos(z / L) / PI * 180;
  alpha2 = acos((sq(fm) + sq(L) - sq(tb)) / (2 * fm * L)) / PI * 180;
  alpha = alpha1 + alpha2;
  return gama;  // invers 90-a
}
float getalphaR(float x, float y, float z) {
  L1 = sqrt(sq(x) + sq(y));
  gama = atan(x / y) / PI * 180;
  L = sqrt(sq(L1 - cx) + sq(z));
  beta = acos((sq(tb) + sq(fm) - sq(L)) / (2 * tb * fm)) / PI * 180;
  alpha1 = acos(z / L) / PI * 180;
  alpha2 = acos((sq(fm) + sq(L) - sq(tb)) / (2 * fm * L)) / PI * 180;
  alpha = alpha1 + alpha2;
  beta2 = beta;
  //  alpha2=90-alpha;
  return 180 - alpha;  // invers 180-a
}
float getbetaR(float x, float y, float z) {
  L1 = sqrt(sq(x) + sq(y));
  gama = atan(x / y) / PI * 180;
  L = sqrt(sq(L1 - cx) + sq(z));
  beta = acos((sq(tb) + sq(fm) - sq(L)) / (2 * tb * fm)) / PI * 180;
  alpha1 = acos(z / L) / PI * 180;
  alpha2 = acos((sq(fm) + sq(L) - sq(tb)) / (2 * fm * L)) / PI * 180;
  alpha = alpha1 + alpha2;
  //  beta2=90-beta;
  return beta;
}
//===================================End of Get Sudut=============================================

//========================================= StartPosisi Kaki Awal/TItik 0 TIap Kaki===============//
//====Titik 0 Sudut
//int L01[3]={1465,545,555};
//int L02[3]={1630,460,460};
//int L03[3]={1500,535,450};
//int R01[3]={1460,520,450};
//int R02[3]={1490,570,450};
//int R03[3]={1455,465,500};

int L01[3]={1460,590,610};
int L02[3]={1550,530,480};
int L03[3]={1530,550,570};
int R01[3]={1540,450,435};
int R02[3]={1585,450,450};
int R03[3]={1570,490,560};

//L0,gamma,alpha,beta||cx,fm,tb


//===Titik 0 Coordinate
float C0L[3][3]{
  { 0, cx + fm, tb },  //
  { 0, cx + fm, tb },
  { 0, cx + fm, tb },
};
float C0R[3][3]{
  { 0, cx + fm, tb },  //
  { 0, cx + fm, tb },
  { 0, cx + fm, tb },
};


//====================SPEED====================//
int walkspeed = 140;  // between 200-500 for better stability and 150 for better speed

//=========================================END Posisi Kaki Awal/TItik 0 TIap Kaki=========



//===================================Start of Matriks Langkah====================================


// Forward
String forward_ssc(int x) {
  float Travel1[2] = { 4 + travelhalf, 4 - travelhalf };    // Maju,Mundur
  float Travel2[2] = { 0 + travelhalf, 0 - travelhalf };   // Maju,Mundur
  float Travel3[2] = { -4 + travelhalf, -4 - travelhalf };  // Maju,Mundur
  float L1[4][3] = {
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
  };
  float L2[4][3] = {
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[4][3] = {
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },  // KakiNaik
  };
  float R1[4][3] = {
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[4][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },  // KakiNaik
  };
  float R3[4][3] = {
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  // String hasil = "#0 P" + String(gamaL1)+"T"+String(walkspeed)+" \r";

  return hasil;
}

String forward6_ssc(int x) {
  float Travel1[2] = { 5 + travelhalf, 5 - travelhalf };    // Maju,Mundur
  float Travel2[2] = { 1 + travelhalf, 1 - travelhalf };   // Maju,Mundur
  float Travel3[2] = { -5 + travelhalf, -5 - travelhalf };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },
  };
  float L2[6][3] = {
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },  // x,y,z
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },  // kakiNaik
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },
  };
  float R1[6][3] = {
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },  // x,y,z
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },  // kakiNaik
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
  };
  float R3[6][3] = {
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  // String hasil = "#0 P" + String(gamaL1)+"T"+String(walkspeed)+" \r";

  return hasil;
}

// Backward
String backward_ssc(int x) {
  float Travel1[2] = { 4 + travelhalf, 4 - travelhalf };    // Maju,Mundur
  float Travel2[2] = { 0 + travelhalf, 0 - travelhalf };   // Maju,Mundur
  float Travel3[2] = { -4 + travelhalf, -4 - travelhalf };  // Maju,Mundur
  float L1[4][3] = {
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
  };
  float L2[4][3] = {
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[4][3] = {
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },  // KakiNaik
  };
  float R1[4][3] = {
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[4][3] = {
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },  // KakiNaik
  };
  float R3[4][3] = {
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String backward6_ssc(int x) {
  float Travel1[2] = { 4 + travelhalf, 4 - travelhalf };    // Maju,Mundur
  float Travel2[2] = { 0 + travelhalf, 0 - travelhalf };   // Maju,Mundur
  float Travel3[2] = { -4 + travelhalf, -4 - travelhalf };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },  // kakiNaik
  };
  float L2[6][3] = {
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },  // KakiNaik
  };
  float R1[6][3] = {
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },  // KakiNaik
  };
  float R3[6][3] = {
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

// Rotate
//String rotate_left_ssc(int x) {
//  float kakiNaik = -4;  // bates tinggi -
//  float kakiTurun = 0;
//  float Travel1[2] = { 6.5, 3 };    // Maju,Mundur
//  float Travel2[2] = { 2.5, -1 };   // Maju,Mundur
//  float Travel3[2] = { -2.5, -6 };  // Maju,Mundur
//  float L1[4][3] = {
//    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z //clear
//    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
//    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
//    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
//  };
//  float L2[4][3] = {
//    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
//    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
//    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
//    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
//  };
//  float L3[4][3] = {
//    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },  // clear
//    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
//    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
//    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },  // KakiNaik
//  };
//  float R1[4][3] = {
//    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
//    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
//    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
//    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
//  };
//  float R2[4][3] = {
//    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
//    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
//    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
//    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },  // KakiNaik
//  };
//  float R3[4][3] = {
//    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
//    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
//    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
//    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
//  };
//  //================================================== L1 ==================================================
//  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
//  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
//  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
//  //================================================== L2 ==================================================
//  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
//  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
//  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
//  //================================================== L3 ==================================================
//  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
//  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
//  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
//  //================================================== R1 ==================================================
//  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
//  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
//  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
//  //================================================== R2 ==================================================
//  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
//  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
//  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
//  //================================================== R3 ==================================================
//  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
//  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
//  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));
//
//  //All Servo
//  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";
//
//
//  return hasil;
//}

String rotate_left_ssc(int x) {
  float YTravel[2] = { -Y2, -Y1 };  // Geser,Tengah
  float Travel1[2] = { X1, X2 };    // Maju,Mundur
  float Travel2[2] = { travelhalf, -travelhalf };   // Maju,Mundur
  float Travel3[2] = { -X2, -X1 };  // Maju,Mundur
//  float Travel1[2] = { 6.5, 3 };    // Maju,Mundur
//  float Travel2[2] = { 2.5, -1 };   // Maju,Mundur
//  float Travel3[2] = { -2.5, -6 };  // Maju,Mundur
  float L1[4][3] = {
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z //clear
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
  };
  float L2[4][3] = {
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[4][3] = {
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiNaik + C0L[2][2] },  // clear
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },  // KakiNaik
  };
  float R1[4][3] = {
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[4][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },  // KakiNaik
  };
  float R3[4][3] = {
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

//  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String rotate6_left_ssc(int x) {
  float YTravel[2] = { -Y2, -Y1 };  // Geser,Tengah
  float Travel1[2] = { X1, X2 };    // Maju,Mundur
  float Travel2[2] = { travelhalf, -travelhalf };   // Maju,Mundur
  float Travel3[2] = { -X2, -X1 };  // Maju,Mundur
//  float Travel1[2] = { 6.5, 3 };    // Maju,Mundur
//  float Travel2[2] = { 2.5, -1 };   // Maju,Mundur
//  float Travel3[2] = { -2.5, -6 };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z //clear
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiNaik + C0L[0][2] },
  };
  float L2[6][3] = {
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiNaik + C0L[2][2] },  // clear
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },  // KakiNaik
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiNaik + C0L[2][2] },  // KakiNaik
  };
  float R1[6][3] = {
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },  // KakiNaik
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
  };
  float R3[6][3] = {
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String rotate_right_ssc(int x) {
  float YTravel[2] = { -Y2, -Y1 };  // Geser,Tengah
  float Travel1[2] = { X1, X2 };    // Maju,Mundur
  float Travel2[2] = { travelhalf, -travelhalf };   // Maju,Mundur
  float Travel3[2] = { -X2, -X1 };  // Maju,Mundur
  float L1[4][3] = {
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z //clear
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
  };
  float L2[4][3] = {
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[4][3] = {
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiNaik + C0L[2][2] },  // clear
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },  // KakiNaik
  };
  float R1[4][3] = {
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[4][3] = {
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },  // KakiNaik
  };
  float R3[4][3] = {
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String rotate6_right_ssc(int x) {
  float YTravel[2] = { -Y2, -Y1 };  // Geser,Tengah
  float Travel1[2] = { X1, X2 };    // Maju,Mundur
  float Travel2[2] = { travelhalf, -travelhalf };   // Maju,Mundur
  float Travel3[2] = { -X2, -X1 };  // Maju,Mundur
//  float Travel1[2] = { 6.5, 3 };    // Maju,Mundur
//  float Travel2[2] = { 2.5, -1 };   // Maju,Mundur
//  float Travel3[2] = { -2.5, -6 };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z //clear
    { Travel1[0] + C0L[0][0], YTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YTravel[0] + C0L[0][1], kakiNaik + C0L[0][2] },  // kakiNaik
  };
  float L2[6][3] = {
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiNaik + C0L[2][2] },  // clear
    { Travel3[0] + C0L[2][0], YTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YTravel[1] + C0L[2][1], kakiNaik + C0L[2][2] },  // KakiNaik
  };
  float R1[6][3] = {
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },  // KakiNaik
  };
  float R3[6][3] = {
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

// Shift
String shift_left_ssc(int x) {
//  float YLTravel[2] = { 0 + travel, 0 };  // Geser,Tengah
//  float YRTravel[2] = { 0 - travel, 0 };
  float YLTravel[2] = { travelhalf, -travelhalf};
  float YRTravel[2] = { -travelhalf, travelhalf};
  float Travel1[2] = { 4, 4 };    // Maju,Mundur
  float Travel2[2] = { 0, 0 };    // Maju,Mundur
  float Travel3[2] = { -4, -4 };  // Maju,Mundur
  float L1[4][3] = {
    { Travel1[0] + C0L[0][0], YLTravel[0] + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], YLTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
  };
  float L2[4][3] = {
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], YLTravel[0] + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], YLTravel[0] + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[4][3] = {
    { Travel3[0] + C0L[2][0], YLTravel[0] + C0L[2][1], kakiNaik + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YLTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },  // KakiNaik
  };
  float R1[4][3] = {
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[4][3] = {
    { Travel2[0] + C0R[1][0], YRTravel[0] + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[0] + C0R[1][0], YRTravel[0] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },  // KakiNaik
  };
  float R3[4][3] = {
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String shift6_left_ssc(int x) {
//  float YLTravel[2] = { 0 + travel, 0 };  // Geser,Tengah
//  float YRTravel[2] = { 0 - travel, 0 };
  float YLTravel[2] = { travelhalf, -travelhalf};
  float YRTravel[2] = { -travelhalf, travelhalf};
  float Travel1[2] = { width , width };    // Maju,Mundur
  float Travel2[2] = { 0, 0 };    // Maju,Mundur
  float Travel3[2] = { -width , -width };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], YLTravel[0] + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], YLTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiNaik + C0L[0][2] },  // kakiNaik
  };
  float L2[6][3] = {
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiNaik + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], YLTravel[0] + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], YLTravel[0] + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], YLTravel[0] + C0L[2][1], kakiNaik + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YLTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiNaik + C0L[2][2] },  // KakiNaik
  };
  float R1[6][3] = {
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },  // kakiNaik
    { Travel1[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], YRTravel[0] + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[0] + C0R[1][0], YRTravel[0] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiNaik + C0R[1][2] },  // KakiNaik
  };
  float R3[6][3] = {
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },  // kakiTurun
    { Travel3[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String shift_right_ssc(int x) {
//  float YLTravel[2] = { 0 - travel, 0 };  // Geser,Tengah
//  float YRTravel[2] = { 0 + travel, 0 };
  float YLTravel[2] = { -travelhalf, travelhalf};
  float YRTravel[2] = { travelhalf, -travelhalf};
  float Travel1[2] = { 4, 4 };    // Maju,Mundur
  float Travel2[2] = { 0, 0 };    // Maju,Mundur
  float Travel3[2] = { -4, -4 };  // Maju,Mundur
  float L1[4][3] = {
    { Travel1[0] + C0L[0][0], YLTravel[0] + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], YLTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
  };
  float L2[4][3] = {
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], YLTravel[0] + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], YLTravel[0] + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[4][3] = {
    { Travel3[0] + C0L[2][0], YLTravel[0] + C0L[2][1], kakiNaik + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YLTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },  // KakiNaik
  };
  float R1[4][3] = {
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[4][3] = {
    { Travel2[0] + C0R[1][0], YRTravel[0] + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[0] + C0R[1][0], YRTravel[0] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },  // KakiNaik
  };
  float R3[4][3] = {
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String shift6_right_ssc(int x) {
//  float YLTravel[2] = { 0 - travel, 0 };  // Geser,Tengah
//  float YRTravel[2] = { 0 + travel, 0 };
  float YLTravel[2] = { -travelhalf, travelhalf};
  float YRTravel[2] = { travelhalf, -travelhalf};
//  float Travel1[2] = { 4, 4 };    // Maju,Mundur
//  float Travel2[2] = { 0, 0 };    // Maju,Mundur
//  float Travel3[2] = { -4, -4 };  // Maju,Mundur
  float Travel1[2] = { width , width };    // Maju,Mundur
  float Travel2[2] = { 0, 0 };    // Maju,Mundur
  float Travel3[2] = { -width , -width };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], YLTravel[0] + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], YLTravel[0] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], YLTravel[1] + C0L[0][1], kakiNaik + C0L[0][2] },  // kakiNaik
  };
  float L2[6][3] = {
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], YLTravel[0] + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], YLTravel[0] + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], YLTravel[1] + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], YLTravel[0] + C0L[2][1], kakiNaik + C0L[2][2] },
    { Travel3[0] + C0L[2][0], YLTravel[0] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], YLTravel[1] + C0L[2][1], kakiNaik + C0L[2][2] },  // KakiNaik
  };
  float R1[6][3] = {
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], YRTravel[0] + C0R[1][1], kakiNaik + C0R[1][2] },
    { Travel2[0] + C0R[1][0], YRTravel[0] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], YRTravel[1] + C0R[1][1], kakiNaik + C0R[1][2] },  // KakiNaik
  };
  float R3[6][3] = {
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiTurun
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], YRTravel[0] + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], YRTravel[1] + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}


// MPU
String forward6_setupmpuup_ssc(int x) {
  float kakiNaik = -4;              // bates tinggi - 2
  float kakiTurun = 0;              // 0
  float Travel1[2] = { 6.5, 3 };    // Maju,Mundur
  float Travel2[2] = { 2.5, -1 };   // Maju,Mundur
  float Travel3[2] = { -2.5, -6 };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },  // kakiNaik
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] },
  };
  float L2[6][3] = {
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },  // kakiNaik
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },  // x,y,z
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },  // kakiNaik
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] },
  };
  float R1[6][3] = {
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },  // x,y,z
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },  // kakiNaik
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] },
  };
  float R3[6][3] = {
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },  // kakiNaik
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  // String hasil = "#0 P" + String(gamaL1)+"T"+String(walkspeed)+" \r";

  return hasil;
}

String forward6_mpuup_ssc(int x) {
  walkspeed = 200;
  float kakiNaik = -1;  // bates tinggi - 2
  float kakiTurun = 2.5;  // 0
  float Cal[3] = { -3, -3.2, 0.5 };
  float Travel1[2] = { 7, 4 };    // Maju,Mundur
  float Travel2[2] = { 3, 0 };    // Maju,Mundur
  float Travel3[2] = { -3, -6 };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiNaik + Cal[0] + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },  // kakiNaik
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiNaik + Cal[0] + C0L[0][2] },
  };
  float L2[6][3] = {
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },  // kakiNaik
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiNaik + Cal[2] + C0L[2][2] },  // x,y,z
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },  // kakiNaik
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiNaik + Cal[2] + C0L[2][2] },
  };
  float R1[6][3] = {
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + Cal[1] + C0R[1][2] },  // x,y,z
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },  // kakiNaik
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + Cal[1] + C0R[1][2] },
  };
  float R3[6][3] = {
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },  // kakiNaik
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  // String hasil = "#0 P" + String(gamaL1)+"T"+String(walkspeed)+" \r";

  return hasil;
}

String forward6_stuckmpuup_ssc(int x) {
  walkspeed = 200;
  float kakiNaik = -1;  // bates tinggi - 2
  float kakiTurun = 2.5;  // 0
  float Cal[3] = { -2.6, -1, 0.5 };
  float Travel1[2] = { 7, 4 };    // Maju,Mundur
  float Travel2[2] = { 3, 0 };    // Maju,Mundur
  float Travel3[2] = { -3, -6 };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiNaik + Cal[0] + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },  // kakiNaik
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiNaik + Cal[0] + C0L[0][2] },
  };
  float L2[6][3] = {
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },  // kakiNaik
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiNaik + Cal[2] + C0L[2][2] },  // x,y,z
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },  // kakiNaik
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiNaik + Cal[2] + C0L[2][2] },
  };
  float R1[6][3] = {
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + Cal[1] + C0R[1][2] },  // x,y,z
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },  // kakiNaik
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + Cal[1] + C0R[1][2] },
  };
  float R3[6][3] = {
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },  // kakiNaik
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  // String hasil = "#0 P" + String(gamaL1)+"T"+String(walkspeed)+" \r";

  return hasil;
}

String turn6_right_mpuup_ssc(int x) {
  walkspeed = 200;
  float kakiNaik = -1;    // bates tinggi - 2
  float kakiTurun = 2.5;  // 0
  float Cal[3] = { -3, -3.2, 0.5 };
  float Travel1[2] = { 7, 4 };    // Maju,Mundur
  float Travel2[2] = { 3, 0 };    // Maju,Mundur
  float Travel3[2] = { -3, -6 };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiNaik + Cal[0] + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },  // kakiNaik
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[1] + C0L[0][0], 0 + C0L[0][1], kakiNaik + Cal[0] + C0L[0][2] },
  };
  float L2[6][3] = {
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },  // kakiNaik
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiNaik + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiNaik + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[1] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiNaik + Cal[2] + C0L[2][2] },  // x,y,z
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },  // kakiNaik
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[1] + C0L[2][0], 0 + C0L[2][1], kakiNaik + Cal[2] + C0L[2][2] },
  };
  float R1[6][3] = {
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },  // kakiNaik
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },  // x,y,z
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },  // kakiNaik
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
  };
  float R3[6][3] = {
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },  // kakiNaik
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  // String hasil = "#0 P" + String(gamaL1)+"T"+String(walkspeed)+" \r";

  return hasil;
}

String turn6_left_mpuup_ssc(int x) {
  walkspeed = 200;
  float kakiNaik = -1;    // bates tinggi - 2
  float kakiTurun = 2.5;  // 0
  float Cal[3] = { -3, -3.2, 0.5 };
  float Travel1[2] = { 7, 4 };    // Maju,Mundur
  float Travel2[2] = { 3, 0 };    // Maju,Mundur
  float Travel3[2] = { -3, -6 };  // Maju,Mundur
  float L1[6][3] = {
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },  // x,y,z
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },  // kakiNaik
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
    { Travel1[0] + C0L[0][0], 0 + C0L[0][1], kakiTurun + Cal[0] + C0L[0][2] },
  };
  float L2[6][3] = {
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },  // kakiNaik
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
    { Travel2[0] + C0L[1][0], 0 + C0L[1][1], kakiTurun + Cal[1] + C0L[1][2] },
  };
  float L3[6][3] = {
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },  // x,y,z
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },  // kakiNaik
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
    { Travel3[0] + C0L[2][0], 0 + C0L[2][1], kakiTurun + Cal[2] + C0L[2][2] },
  };
  float R1[6][3] = {
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },  // kakiNaik
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[0] + C0R[2][2] },
    { Travel1[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
    { Travel1[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[0] + C0R[2][2] },
  };
  float R2[6][3] = {
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiNaik + Cal[1] + C0R[1][2] },  // x,y,z
    { Travel2[0] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },  // kakiNaik
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiTurun + Cal[1] + C0R[1][2] },
    { Travel2[1] + C0R[1][0], 0 + C0R[1][1], kakiNaik + Cal[1] + C0R[1][2] },
  };
  float R3[6][3] = {
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },  // kakiNaik
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiNaik + Cal[2] + C0R[2][2] },
    { Travel3[0] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
    { Travel3[1] + C0R[2][0], 0 + C0R[2][1], kakiTurun + Cal[2] + C0R[2][2] },
  };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[x][0], L2[x][1], L2[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[x][0], L2[x][1], L2[x][2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[x][0], L2[x][1], L2[x][2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[x][0], L3[x][1], L3[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[x][0], L3[x][1], L3[x][2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[x][0], L3[x][1], L3[x][2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[x][0], R2[x][1], R2[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[x][0], R2[x][1], R2[x][2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[x][0], R2[x][1], R2[x][2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[x][0], R3[x][1], R3[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[x][0], R3[x][1], R3[x][2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[x][0], R3[x][1], R3[x][2])));

  // All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  // String hasil = "#0 P" + String(gamaL1)+"T"+String(walkspeed)+" \r";

  return hasil;
}


String maintenance() {
  //=====Maintenance Position======//
  float LM01[3] = { 1500, 540, 450 };  //>Mundur,>Naik,>Masuk
  float LM02[3] = { 1565, 550, 700 };
  float LM03[3] = { 2450, 530, 490 };
  float RM01[3] = { 1500, 430, 490 };  //>Maju,>Turun,>Keluar //1520
  float RM02[3] = { 1540, 450, 520 };
  float RM03[3] = { 600, 580, 550 };
  //L0,gamma,alpha,beta||cx,fm,tb
  float kakiNaik = 0;  // bates tinggi -
  float kakiTurun = 0;
  float L1[3] = { 0 + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] };  //x,y,z
  float L2[3] = { 0 + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] };
  float L3[3] = { 0 + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] };
  float R1[3] = { 0 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  float R2[3] = { 0 + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] };
  float R3[3] = { 0 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  //================================================== L1 ==================================================
  int gamaL1 = round(LM01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(LM01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1 = round(LM01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(LM02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(LM02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2 = round(LM02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(LM03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(LM03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3 = round(LM03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(RM01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(RM01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1 = round(RM01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(RM02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(RM02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2 = round(RM02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(RM03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(RM03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3 = round(RM03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String PIDState() {
  float kakiNaik = 0;  // bates tinggi -
  float kakiTurun = 0;
  float PID = 1;
  float L1[3] = { 0 + C0L[0][0], 0 + C0L[0][1], kakiNaik + PID + C0L[0][2] };  //x,y,z
  float L2[3] = { 0 + C0L[1][0], 0 + C0L[1][1], kakiNaik + PID + C0L[1][2] };
  float L3[3] = { 0 + C0L[2][0], 0 + C0L[2][1], kakiNaik + PID + C0L[2][2] };
  float R1[3] = { 0 + C0R[2][0], 0 + C0R[2][1], kakiNaik - PID + C0R[2][2] };
  float R2[3] = { 0 + C0R[1][0], 0 + C0R[1][1], kakiNaik - PID + C0R[1][2] };
  float R3[3] = { 0 + C0R[2][0], 0 + C0R[2][1], kakiNaik - PID + C0R[2][2] };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));

  String spasi = " ";

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}


String point0() {
  float kakiNaik = 0;  // bates tinggi -4.5
  float kakiTurun = 0;
  float L1[3] = { 0 + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] };  // x,y,z
  float L2[3] = { 0 + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] };
  float L3[3] = { 0 + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] };
  float R1[3] = { 0 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  float R2[3] = { 0 + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] };
  float R3[3] = { 0 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String point_ready_state() {
  float kakiNaik = 0;  // bates tinggi -4.5
  float kakiTurun = 0;
  float L1[3] = { 2.84  + C0L[0][0], -0.5 + C0L[0][1], kakiNaik + C0L[0][2] };  // x,y,z
  float L2[3] = { 0     + C0L[1][0], 0    + C0L[1][1], kakiNaik + C0L[1][2] };
  float L3[3] = { -2.84 + C0L[2][0], -0.5 + C0L[2][1], kakiNaik + C0L[2][2] };
  float R1[3] = { 2.84  + C0R[2][0], -0.5 + C0R[2][1], kakiNaik + C0R[2][2] };
  float R2[3] = { 0     + C0R[1][0], 0   + C0R[1][1], kakiNaik + C0R[1][2] };
  float R3[3] = { -2.84 + C0R[2][0], -0.5 + C0R[2][1], kakiNaik + C0R[2][2] };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String point_backward() {
  float kakiNaik = 0;  // bates tinggi -4.5
  float kakiTurun = 0;
  float L1[3] = { 6 + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] };  // x,y,z
  float L2[3] = { 3 + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] };
  float L3[3] = { 0 + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] };
  float R1[3] = { 6 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  float R2[3] = { 3 + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] };
  float R3[3] = { 0 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String point_forward() {
  float kakiNaik = 0;  // bates tinggi -4.5
  float kakiTurun = 0;
  float L1[3] = { -1 + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] };  // x,y,z
  float L2[3] = { -4 + C0L[1][0], 0 + C0L[1][1], kakiNaik + C0L[1][2] };
  float L3[3] = { -7 + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] };
  float R1[3] = { -1 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  float R2[3] = { -4 + C0R[1][0], 0 + C0R[1][1], kakiNaik + C0R[1][2] };
  float R3[3] = { -7 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String point_rotate_right_state() {
//  float kakiNaik = 0;  // bates tinggi -4.5
//  float kakiTurun = 0;
  float Ytravel[2] = {-Y2, -Y1};
  float L1[3] = { -1 + C0L[0][0], 0 + C0L[0][1], kakiNaik + C0L[0][2] };  // x,y,z
  float L2[3] = { -4 + C0L[1][0], Ytravel[0] + C0L[1][1], kakiNaik + C0L[1][2] };
  float L3[3] = { -7 + C0L[2][0], Ytravel[1] + C0L[2][1], kakiNaik + C0L[2][2] };
  float R1[3] = { 7 + C0R[2][0], Ytravel[1] + C0R[2][1], kakiNaik + C0R[2][2] };
  float R2[3] = { 4 + C0R[1][0], Ytravel[0] + C0R[1][1], kakiNaik + C0R[1][2] };
  float R3[3] = { 1 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}

String point_rotate_left_state() {
//  float kakiNaik = 0;  // bates tinggi -4.5
//  float kakiTurun = 0;
  float Ytravel[2] = {-Y2, -Y1};
  float L1[3] = { 7 + C0L[0][0], Ytravel[1] + C0L[0][1], kakiNaik + C0L[0][2] };  // x,y,z
  float L2[3] = { 4 + C0L[1][0], Ytravel[0] + C0L[1][1], kakiNaik + C0L[1][2] };
  float L3[3] = { 1 + C0L[2][0], 0 + C0L[2][1], kakiNaik + C0L[2][2] };
  float R1[3] = { -1 + C0R[2][0], 0 + C0R[2][1], kakiNaik + C0R[2][2] };
  float R2[3] = { -4 + C0R[1][0], Ytravel[0] + C0R[1][1], kakiNaik + C0R[1][2] };
  float R3[3] = { -7 + C0R[2][0], Ytravel[1] + C0R[2][1], kakiNaik + C0R[2][2] };
  //================================================== L1 ==================================================
  int gamaL1 = round(L01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1 = round(L01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
  //================================================== L2 ==================================================
  int gamaL2 = round(L02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2 = round(L02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
  //================================================== L3 ==================================================
  int gamaL3 = round(L03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3 = round(L03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
  //================================================== R1 ==================================================
  int gamaR1 = round(R01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1 = round(R01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
  //================================================== R2 ==================================================
  int gamaR2 = round(R02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2 = round(R02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
  //================================================== R3 ==================================================
  int gamaR3 = round(R03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3 = round(R03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));

  //All Servo
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1) + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2) + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3) + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1) + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2) + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3) + " T" + String(walkspeed) + " \r";

  return hasil;
}
//====================================End Of Matrix Langkah==============================//



void testkoordinat(int n) {
  Serial2.print(String(shift_right_ssc(n)));
  Serial.println(String(shift_right_ssc(n)));
}

///=========START OF STEP MOVE==========
void forward_step() {
  switch (state_nextStep) {
    case 0:
      Serial2.print(String(forward_ssc(0)));
      // Serial.print(String(forward_ssc(0)));
      state_nextStep = 1;
      break;
    case 1:
      Serial2.print(String(forward_ssc(1)));
      // Serial.print(String(forward_ssc(1)));
      state_nextStep = 2;
      break;
    case 2:
      Serial2.print(String(forward_ssc(2)));
      // Serial.print(String(forward_ssc(2)));
      state_nextStep = 3;
      break;
    case 3:
      Serial2.print(String(forward_ssc(3)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep = 0;
      break;
  }
}

void forward6_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(forward6_ssc(0)));
      // Serial.print(String(forward_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(forward6_ssc(1)));
      // Serial.print(String(forward_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(forward6_ssc(2)));
      // Serial.print(String(forward_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(forward6_ssc(3)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(forward6_ssc(4)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(forward6_ssc(5)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 0;
      break;
  }
}

void backward_step() {
  switch (state_nextStep) {
    case 0:
      Serial2.print(String(backward_ssc(0)));
      state_nextStep = 1;
      break;
    case 1:
      Serial2.print(String(backward_ssc(1)));
      state_nextStep = 2;
      break;
    case 2:
      Serial2.print(String(backward_ssc(2)));
      state_nextStep = 3;
      break;
    case 3:
      Serial2.print(String(backward_ssc(3)));
      state_nextStep = 0;
      break;
  }
}

void backward6_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(backward6_ssc(0)));
      // Serial.print(String(forward_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(backward6_ssc(1)));
      // Serial.print(String(forward_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(backward6_ssc(2)));
      // Serial.print(String(forward_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(backward6_ssc(3)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(backward6_ssc(4)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(backward6_ssc(5)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 0;
      break;
  }
}

void rotate_left_step() {
  switch (state_nextStep) {
    case 0:
      Serial2.print(String(rotate_left_ssc(0)));
      state_nextStep = 1;
      break;
    case 1:
      Serial2.print(String(rotate_left_ssc(1)));
      state_nextStep = 2;
      break;
    case 2:
      Serial2.print(String(rotate_left_ssc(2)));
      state_nextStep = 3;
      break;
    case 3:
      Serial2.print(String(rotate_left_ssc(3)));
      state_nextStep = 0;
      break;
  }
}

void rotate6_left_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(rotate6_left_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(rotate6_left_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(rotate6_left_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(rotate6_left_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(rotate6_left_ssc(4)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(rotate6_left_ssc(5)));
      state_nextStep6 = 0;
      break;
  }
}

void rotate_right_step() {
  switch (state_nextStep) {
    case 0:
      Serial2.print(String(rotate_right_ssc(0)));
      state_nextStep = 1;
      break;
    case 1:
      Serial2.print(String(rotate_right_ssc(1)));
      state_nextStep = 2;
      break;
    case 2:
      Serial2.print(String(rotate_right_ssc(2)));
      state_nextStep = 3;
      break;
    case 3:
      Serial2.print(String(rotate_right_ssc(3)));
      state_nextStep = 0;
      break;
  }
}

void rotate6_right_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(rotate6_right_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(rotate6_right_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(rotate6_right_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(rotate6_right_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(rotate6_right_ssc(4)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(rotate6_right_ssc(5)));
      state_nextStep6 = 0;
      break;
  }
}

void shift_left_step() {
  switch (state_nextStep) {
    case 0:
      Serial2.print(String(shift_left_ssc(0)));
      state_nextStep = 1;
      break;
    case 1:
      Serial2.print(String(shift_left_ssc(1)));
      state_nextStep = 2;
      break;
    case 2:
      Serial2.print(String(shift_left_ssc(2)));
      state_nextStep = 3;
      break;
    case 3:
      Serial2.print(String(shift_left_ssc(3)));
      state_nextStep = 0;
      break;
  }
}

void shift6_left_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(shift6_left_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(shift6_left_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(shift6_left_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(shift6_left_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(shift6_left_ssc(4)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(shift6_left_ssc(5)));
      state_nextStep6 = 0;
      break;
  }
}

void shift_right_step() {
  switch (state_nextStep) {
    case 0:
      Serial2.print(String(shift_right_ssc(0)));
      state_nextStep = 1;
      break;
    case 1:
      Serial2.print(String(shift_right_ssc(1)));
      state_nextStep = 2;
      break;
    case 2:
      Serial2.print(String(shift_right_ssc(2)));
      state_nextStep = 3;
      break;
    case 3:
      Serial2.print(String(shift_right_ssc(3)));
      state_nextStep = 0;
      break;
  }
}

void shift6_right_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(shift6_right_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(shift6_right_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(shift6_right_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(shift6_right_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(shift6_right_ssc(4)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(shift6_right_ssc(5)));
      state_nextStep6 = 0;
      break;
  }
}

void forward6_setupmpuup_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(forward6_setupmpuup_ssc(0)));
      // Serial.print(String(forward_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(forward6_setupmpuup_ssc(1)));
      // Serial.print(String(forward_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(forward6_setupmpuup_ssc(2)));
      // Serial.print(String(forward_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(forward6_setupmpuup_ssc(3)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(forward6_setupmpuup_ssc(4)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(forward6_setupmpuup_ssc(5)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 0;
      break;
  }
}
void forward6_mpuup_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(forward6_mpuup_ssc(0)));
      // Serial.print(String(forward_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(forward6_mpuup_ssc(1)));
      // Serial.print(String(forward_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(forward6_mpuup_ssc(2)));
      // Serial.print(String(forward_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(forward6_mpuup_ssc(3)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(forward6_mpuup_ssc(4)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(forward6_mpuup_ssc(5)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 0;
      break;
  }
}

void forward6_stuckmpuup_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(forward6_stuckmpuup_ssc(0)));
      // Serial.print(String(forward_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(forward6_stuckmpuup_ssc(1)));
      // Serial.print(String(forward_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(forward6_stuckmpuup_ssc(2)));
      // Serial.print(String(forward_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(forward6_stuckmpuup_ssc(3)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(forward6_stuckmpuup_ssc(4)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(forward6_stuckmpuup_ssc(5)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 0;
      break;
  }
}

void turn6_right_mpuup_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(turn6_right_mpuup_ssc(0)));
      // Serial.print(String(forward_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(turn6_right_mpuup_ssc(1)));
      // Serial.print(String(forward_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(turn6_right_mpuup_ssc(2)));
      // Serial.print(String(forward_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(turn6_right_mpuup_ssc(3)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(turn6_right_mpuup_ssc(4)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(turn6_right_mpuup_ssc(5)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 0;
      break;
  }
}

void turn6_left_mpuup_step() {
  switch (state_nextStep6) {
    case 0:
      Serial2.print(String(turn6_left_mpuup_ssc(0)));
      // Serial.print(String(forward_ssc(0)));
      state_nextStep6 = 1;
      break;
    case 1:
      Serial2.print(String(turn6_left_mpuup_ssc(1)));
      // Serial.print(String(forward_ssc(1)));
      state_nextStep6 = 2;
      break;
    case 2:
      Serial2.print(String(turn6_left_mpuup_ssc(2)));
      // Serial.print(String(forward_ssc(2)));
      state_nextStep6 = 3;
      break;
    case 3:
      Serial2.print(String(turn6_left_mpuup_ssc(3)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 4;
      break;
    case 4:
      Serial2.print(String(turn6_left_mpuup_ssc(4)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 5;
      break;
    case 5:
      Serial2.print(String(turn6_left_mpuup_ssc(5)));
      // Serial.print(String(forward_ssc(3)));
      state_nextStep6 = 0;
      break;
  }
}
String sapu_ssc(int x) {
  // 5 kaki lain diam di posisi READY (sama kayak point_ready_state)
  float L2s[3] = { 0     + C0L[1][0], 0    + C0L[1][1], 0 + C0L[1][2] };
  float L3s[3] = { -7    + C0L[2][0], 2    + C0L[2][1], 0 + C0L[2][2] };
  float R1s[3] = { 2.84  + C0R[2][0], -0.5 + C0R[2][1], 0 + C0R[2][2] };
  float R2s[3] = { 0     + C0R[1][0], 0    + C0R[1][1], 0 + C0R[1][2] };
  float R3s[3] = { -7    + C0R[2][0], 2    + C0R[2][1], 0 + C0R[2][2] };

  // Kaki L1 yang gerak nyapu — 4 frame sesuai urutan yang diminta
  float L1[4][3] = {
    { 11 + C0L[0][0], -1 + C0L[0][1], -4 + C0L[0][2] },  // 0: angkat, posisi awal
    { 11 + C0L[0][0], -1 + C0L[0][1],  0 + C0L[0][2] },  // 1: turun (nyentuh/nyapu)
    {  0 + C0L[0][0],  5 + C0L[0][1],  0 + C0L[0][2] },  // 2: tarik nyapu, masih nempel
    {  0 + C0L[0][0],  5 + C0L[0][1], -4 + C0L[0][2] },  // 3: angkat lagi, siap ulang
  };

  //================================================== L1 (nyapu) ==================================================
  int gamaL1  = round(L01[0] + myFunctionPWM(getgamaL(L1[x][0], L1[x][1], L1[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[x][0], L1[x][1], L1[x][2])));
  int betaL1  = round(L01[2] + myFunctionPWM(getbetaL(L1[x][0], L1[x][1], L1[x][2])));
  //================================================== L2 (ready) ==================================================
  int gamaL2  = round(L02[0] + myFunctionPWM(getgamaL(L2s[0], L2s[1], L2s[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2s[0], L2s[1], L2s[2])));
  int betaL2  = round(L02[2] + myFunctionPWM(getbetaL(L2s[0], L2s[1], L2s[2])));
  //================================================== L3 (ready) ==================================================
  int gamaL3  = round(L03[0] + myFunctionPWM(getgamaL(L3s[0], L3s[1], L3s[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3s[0], L3s[1], L3s[2])));
  int betaL3  = round(L03[2] + myFunctionPWM(getbetaL(L3s[0], L3s[1], L3s[2])));
  //================================================== R1 (ready) ==================================================
  int gamaR1  = round(R01[0] + myFunctionPWM(getgamaR(R1s[0], R1s[1], R1s[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1s[0], R1s[1], R1s[2])));
  int betaR1  = round(R01[2] + myFunctionPWM(getbetaR(R1s[0], R1s[1], R1s[2])));
  //================================================== R2 (ready) ==================================================
  int gamaR2  = round(R02[0] + myFunctionPWM(getgamaR(R2s[0], R2s[1], R2s[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2s[0], R2s[1], R2s[2])));
  int betaR2  = round(R02[2] + myFunctionPWM(getbetaR(R2s[0], R2s[1], R2s[2])));
  //================================================== R3 (ready) ==================================================
  int gamaR3  = round(R03[0] + myFunctionPWM(getgamaR(R3s[0], R3s[1], R3s[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3s[0], R3s[1], R3s[2])));
  int betaR3  = round(R03[2] + myFunctionPWM(getbetaR(R3s[0], R3s[1], R3s[2])));

  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1)
    + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2)
    + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3)
    + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1)
    + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2)
    + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3)
    + " T" + String(walkspeed) + " \r";

  return hasil;
}
 
void sapu_step() {
  switch (state_nextStep) {
    case 0: Serial2.print(String(sapu_ssc(0))); state_nextStep = 1; break;
    case 1: Serial2.print(String(sapu_ssc(1))); state_nextStep = 2; break;
    case 2: Serial2.print(String(sapu_ssc(2))); state_nextStep = 3; break;
    case 3: Serial2.print(String(sapu_ssc(3))); state_nextStep = 0; break;
  }
}
// ---------------- WAVE HELLO (dengan weight-shift biar gak jatuh) ----------
String wave_hello_ssc(int x) {
  float naik  = -2.5;  // tinggi angkat R1, diturunin dari -4 (lebih aman)
  float turun = 0;
  float swing = 2;     // lebar ayunan, dikurangin dari 3
 
  // --- INI YANG PALING PENTING BUAT DI-TUNE ---
  // shift = seberapa jauh 5 kaki lain geser buat mindahin titik berat
  // menjauhi R1 sebelum R1 diangkat.
  // KALAU ROBOT MASIH/MALAH JATUH KE ARAH SEBALIKNYA, GANTI TANDANYA
  // dari -1.5 jadi +1.5 (atau sebaliknya).
  float shift = -1.5;
 
  // shift aktif dari frame 0 sampai 6 (selama R1 di udara),
  // balik ke 0 di frame 7 (pas R1 udah nempel tanah lagi)
  auto s = [&](int i) -> float { return (i < 7) ? shift : 0; };
 
  float L1s[8][3], L2s[8][3], L3s[8][3], R2s[8][3], R3s[8][3];
  for (int i = 0; i < 8; i++) {
    L1s[i][0] = 0 + C0L[0][0]; L1s[i][1] = s(i) + C0L[0][1]; L1s[i][2] = 0 + C0L[0][2];
    L2s[i][0] = 0 + C0L[1][0]; L2s[i][1] = s(i) + C0L[1][1]; L2s[i][2] = 0 + C0L[1][2];
    L3s[i][0] = 0 + C0L[2][0]; L3s[i][1] = s(i) + C0L[2][1]; L3s[i][2] = 0 + C0L[2][2];
    R2s[i][0] = 0 + C0R[1][0]; R2s[i][1] = s(i) + C0R[1][1]; R2s[i][2] = 0 + C0R[1][2];
    R3s[i][0] = 0 + C0R[2][0]; R3s[i][1] = s(i) + C0R[2][1]; R3s[i][2] = 0 + C0R[2][2];
  }
 
  float R1[8][3] = {
    { 0 + C0R[2][0], 0      + C0R[2][1], turun + C0R[2][2] },  // 0: badan mulai geser, R1 msh nempel
    { 0 + C0R[2][0], 0      + C0R[2][1], naik  + C0R[2][2] },  // 1: berat udah pindah, R1 diangkat
    { 0 + C0R[2][0], swing  + C0R[2][1], naik  + C0R[2][2] },  // 2: ayun kiri
    { 0 + C0R[2][0], -swing + C0R[2][1], naik  + C0R[2][2] },  // 3: ayun kanan
    { 0 + C0R[2][0], swing  + C0R[2][1], naik  + C0R[2][2] },  // 4: ayun kiri
    { 0 + C0R[2][0], -swing + C0R[2][1], naik  + C0R[2][2] },  // 5: ayun kanan
    { 0 + C0R[2][0], 0      + C0R[2][1], naik  + C0R[2][2] },  // 6: balik tengah, msh di udara
    { 0 + C0R[2][0], 0      + C0R[2][1], turun + C0R[2][2] },  // 7: R1 turun, badan balik tengah
  };
 
  int gamaL1  = round(L01[0] + myFunctionPWM(getgamaL(L1s[x][0], L1s[x][1], L1s[x][2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1s[x][0], L1s[x][1], L1s[x][2])));
  int betaL1  = round(L01[2] + myFunctionPWM(getbetaL(L1s[x][0], L1s[x][1], L1s[x][2])));
 
  int gamaL2  = round(L02[0] + myFunctionPWM(getgamaL(L2s[x][0], L2s[x][1], L2s[x][2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2s[x][0], L2s[x][1], L2s[x][2])));
  int betaL2  = round(L02[2] + myFunctionPWM(getbetaL(L2s[x][0], L2s[x][1], L2s[x][2])));
 
  int gamaL3  = round(L03[0] + myFunctionPWM(getgamaL(L3s[x][0], L3s[x][1], L3s[x][2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3s[x][0], L3s[x][1], L3s[x][2])));
  int betaL3  = round(L03[2] + myFunctionPWM(getbetaL(L3s[x][0], L3s[x][1], L3s[x][2])));
 
  int gamaR1  = round(R01[0] + myFunctionPWM(getgamaR(R1[x][0], R1[x][1], R1[x][2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[x][0], R1[x][1], R1[x][2])));
  int betaR1  = round(R01[2] + myFunctionPWM(getbetaR(R1[x][0], R1[x][1], R1[x][2])));
 
  int gamaR2  = round(R02[0] + myFunctionPWM(getgamaR(R2s[x][0], R2s[x][1], R2s[x][2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2s[x][0], R2s[x][1], R2s[x][2])));
  int betaR2  = round(R02[2] + myFunctionPWM(getbetaR(R2s[x][0], R2s[x][1], R2s[x][2])));
 
  int gamaR3  = round(R03[0] + myFunctionPWM(getgamaR(R3s[x][0], R3s[x][1], R3s[x][2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3s[x][0], R3s[x][1], R3s[x][2])));
  int betaR3  = round(R03[2] + myFunctionPWM(getbetaR(R3s[x][0], R3s[x][1], R3s[x][2])));
 
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1)
    + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2)
    + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3)
    + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1)
    + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2)
    + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3)
    + " T" + String(walkspeed) + " \r";
 
  return hasil;
}
 
void wave_hello_step() {
  switch (state_nextStep6) {
    case 0: Serial2.print(String(wave_hello_ssc(0))); state_nextStep6 = 1; break;
    case 1: Serial2.print(String(wave_hello_ssc(1))); state_nextStep6 = 2; break;
    case 2: Serial2.print(String(wave_hello_ssc(2))); state_nextStep6 = 3; break;
    case 3: Serial2.print(String(wave_hello_ssc(3))); state_nextStep6 = 4; break;
    case 4: Serial2.print(String(wave_hello_ssc(4))); state_nextStep6 = 5; break;
    case 5: Serial2.print(String(wave_hello_ssc(5))); state_nextStep6 = 6; break;
    case 6: Serial2.print(String(wave_hello_ssc(6))); state_nextStep6 = 7; break;
    case 7: Serial2.print(String(wave_hello_ssc(7))); state_nextStep6 = 0; break;
  }
}
 
// ---------------- DANCE (8 frame gradual biar gak kaku) --------------------
String dance_ssc(int x) {
  float swingMax = 1.5;  // goyang badan kiri-kanan, dikurangin dari 2
  float ampZ     = 1.5;  // naik-turun tripod, dikurangin dari 2
 
  // pola halus (triangle wave) 8 frame, gantiin lompatan ekstrem 4-frame yg lama
  float pat[8] = { 0, 0.5, 1, 0.5, 0, -0.5, -1, -0.5 };
 
  float y  = pat[x] * swingMax;
  float zA = pat[x] * ampZ;    // tripod A: L1, L2, L3
  float zB = -pat[x] * ampZ;   // tripod B: R1, R2, R3 (kebalikan fase)
 
  float L1[3] = { 0 + C0L[0][0], y + C0L[0][1], zA + C0L[0][2] };
  float L2[3] = { 0 + C0L[1][0], y + C0L[1][1], zA + C0L[1][2] };
  float L3[3] = { 0 + C0L[2][0], y + C0L[2][1], zA + C0L[2][2] };
  float R1[3] = { 0 + C0R[2][0], y + C0R[2][1], zB + C0R[2][2] };
  float R2[3] = { 0 + C0R[1][0], y + C0R[1][1], zB + C0R[1][2] };
  float R3[3] = { 0 + C0R[2][0], y + C0R[2][1], zB + C0R[2][2] };
 
  int gamaL1  = round(L01[0] + myFunctionPWM(getgamaL(L1[0], L1[1], L1[2])));
  int alphaL1 = round(L01[1] + myFunctionPWM(getalphaL(L1[0], L1[1], L1[2])));
  int betaL1  = round(L01[2] + myFunctionPWM(getbetaL(L1[0], L1[1], L1[2])));
 
  int gamaL2  = round(L02[0] + myFunctionPWM(getgamaL(L2[0], L2[1], L2[2])));
  int alphaL2 = round(L02[1] + myFunctionPWM(getalphaL(L2[0], L2[1], L2[2])));
  int betaL2  = round(L02[2] + myFunctionPWM(getbetaL(L2[0], L2[1], L2[2])));
 
  int gamaL3  = round(L03[0] + myFunctionPWM(getgamaL(L3[0], L3[1], L3[2])));
  int alphaL3 = round(L03[1] + myFunctionPWM(getalphaL(L3[0], L3[1], L3[2])));
  int betaL3  = round(L03[2] + myFunctionPWM(getbetaL(L3[0], L3[1], L3[2])));
 
  int gamaR1  = round(R01[0] + myFunctionPWM(getgamaR(R1[0], R1[1], R1[2])));
  int alphaR1 = round(R01[1] + myFunctionPWM(getalphaR(R1[0], R1[1], R1[2])));
  int betaR1  = round(R01[2] + myFunctionPWM(getbetaR(R1[0], R1[1], R1[2])));
 
  int gamaR2  = round(R02[0] + myFunctionPWM(getgamaR(R2[0], R2[1], R2[2])));
  int alphaR2 = round(R02[1] + myFunctionPWM(getalphaR(R2[0], R2[1], R2[2])));
  int betaR2  = round(R02[2] + myFunctionPWM(getbetaR(R2[0], R2[1], R2[2])));
 
  int gamaR3  = round(R03[0] + myFunctionPWM(getgamaR(R3[0], R3[1], R3[2])));
  int alphaR3 = round(R03[1] + myFunctionPWM(getalphaR(R3[0], R3[1], R3[2])));
  int betaR3  = round(R03[2] + myFunctionPWM(getbetaR(R3[0], R3[1], R3[2])));
 
  String hasil = "#23 P" + String(gamaL1) + "#22 P" + String(alphaL1) + "#21 P" + String(betaL1)
    + "#19 P" + String(gamaL2) + " #18 P" + String(alphaL2) + "#17 P" + String(betaL2)
    + "#15 P" + String(gamaL3) + " #14 P" + String(alphaL3) + " #13 P" + String(betaL3)
    + " #0 P" + String(gamaR1) + " #1 P" + String(alphaR1) + " #2 P" + String(betaR1)
    + " #4 P" + String(gamaR2) + " #5 P" + String(alphaR2) + " #6 P" + String(betaR2)
    + " #8 P" + String(gamaR3) + " #9 P" + String(alphaR3) + " #10 P" + String(betaR3)
    + " T" + String(walkspeed) + " \r";
 
  return hasil;
}
 
void dance_step() {
  switch (state_nextStep) {
    case 0: Serial2.print(String(dance_ssc(0))); state_nextStep = 1; break;
    case 1: Serial2.print(String(dance_ssc(1))); state_nextStep = 2; break;
    case 2: Serial2.print(String(dance_ssc(2))); state_nextStep = 3; break;
    case 3: Serial2.print(String(dance_ssc(3))); state_nextStep = 4; break;
    case 4: Serial2.print(String(dance_ssc(4))); state_nextStep = 5; break;
    case 5: Serial2.print(String(dance_ssc(5))); state_nextStep = 6; break;
    case 6: Serial2.print(String(dance_ssc(6))); state_nextStep = 7; break;
    case 7: Serial2.print(String(dance_ssc(7))); state_nextStep = 0; break;
  }
}
//=========END OF STEP MOVE==============

//=========Start of Function Move=====
void movetype(type move) {
  if (millis() - state_lastMoveRecord >= walkspeed) {
    state_lastMoveRecord = millis();
    switch (move) {
      case eforward:
        forward_step();
        break;
      case eforward6:
        forward6_step();
        break;
      case ebackward:
        backward_step();
        break;
      case ebackward6:
        backward6_step();
        break;
      case erotate_left:
        rotate_left_step();
        break;
      case erotate6_left:
        rotate6_left_step();
        break;
      case erotate_right:
        rotate_right_step();
        break;
      case erotate6_right:
        rotate6_right_step();
        break;
      case eshift_left:
        shift_left_step();
        break;
      case eshift6_left:
        shift6_left_step();
        break;
      case eshift_right:
        shift_right_step();
        break;
      case eshift6_right:
        shift6_right_step();
        break;
      case eforward6_setupmpuup:
        forward6_setupmpuup_step();
        break;
      case eforward6_mpuup:
        forward6_mpuup_step();
        break;
      case eforward6_stuckmpuup:
        forward6_stuckmpuup_step();
        break;
      case eturn6_right_mpuup:
        turn6_right_mpuup_step();
        break;
      case eturn6_left_mpuup:
        turn6_left_mpuup_step();
        break;
      case epoint0:
        Serial2.print(String(point0()));
        break;
      case epoint_ready:
        Serial2.print(String(point_ready_state()));
        break;
      case epoint_backward:
        Serial2.print(String(point_backward()));
        break;
      case epoint_forward:
        Serial2.print(String(point_forward()));
        break;
      case epoint_rotate_right:
        Serial2.print(String(point_rotate_right_state()));
        break;
      case epoint_rotate_left:
        Serial2.print(String(point_rotate_left_state()));
        break;  
      case ewave_hello:
        wave_hello_step();
        break;
      case edance:
        dance_step();
        break;
      case esapu:
        sapu_step();
        break;
    }
  }
}
void CapitTurun() {
  int pwm = 800;  //< turun
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + " #7 P800 #16 P0 T400 \r");  //kanan //kiri //engsel //capit
  delay(400);
}
void CapitTurun2() {
  int pwm = 1250;  //< turun
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + " #7 P1200 #16 P0 T400 \r");  //kanan //kiri //engsel //capit
  delay(400);
}
void CapitTurun3() {
  int pwm = 1000;  //< turun
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + " #7 P800 #16 P0 T400 \r");  //kanan //kiri //engsel //capit
  delay(400);
}
void CapitJepit() {
  int pwm = 800;  //< turun
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + " #7 P800 #16 P1125 T400 \r");  //kanan //kiri //engsel //capit
  delay(400);
}
void CapitJepit2() {
  int pwm = 1250;  //< turun
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + " #7 P1200 #16 P1125 T400 \r");  //kanan //kiri //engsel //capit
  delay(400);
}
void CapitJepit3() {
  int pwm = 800;  //< turun
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + " #7 P1000 #16 P0 T150 \r");  //kanan //kiri //engsel //capit
  delay(150);
}
void CapitNaik() {
  int pwm = 2100;  //< turun
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + " #7 P800 #16 P1125 T300 \r");  //kanan //kiri //engsel //capit
  delay(150);
}


void capitnaik(int type) {
  int pwme;
  int pwm = 2500;  //< turun
  switch (type){
    case 1:
      pwme = 1000;
      break;
    case 2:
      pwme = 2500;
  }
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + "#7 P" + String(pwme) + "T150 \r");
  delay(150);
}
void capitturun(int type) {
  int pwme;
  int pwm = 1000;  //< turun
  switch (type){
    case 1:
      pwme = 800;
      break;
    case 2:
      pwme = 2000;
      break;
    case 3:
      pwme = 500;
      break;
    case 4:
      pwme = 1200;
  }
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + "#7 P" + String(pwme) + "T150 \r");
  delay(150);
}
void capitturun2(int type) {
  int pwme;
  int pwm = 800;  //< turun
  switch (type){
    case 1:
      pwme = 800;
      break;
    case 2:
      pwme = 2000;
      break;
    case 3:
      pwme = 500;
      break;
    case 4:
      pwme = 1000;
  }
  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + "#7 P" + String(pwme) + "T150 \r");
  delay(150);
}
//void capitturun(int pwm, int pwme) {
//  Serial2.print("#3 P" + String(pwm) + " #20 P" + String(3000 - pwm) + "#7 P" + String(pwme) + "T150 \r");
//  delay(150);
//}
void capitbuka() {
  Serial2.print("#16 P0 T150 \r");
  delay(150);
}
void capitjepit() {
  Serial2.print("#16 P1105 T150 \r");
  delay(150);
}
//void engselnaik() {
//  Serial2.print("#20 p
//}



// Generate Movetype
float rad(float deg) {
  float radian = deg * (PI / 180);
  return radian;
}

void rotate_coor() {
  X1 = sin(rad(45)) * travelhalf + 2.876;
  Y1 = sqrt(pow(travelhalf, 2) - pow(sin(rad(45)) * travelhalf, 2)) + 2.423;
  X2 = X1 - sin(rad(45)) * travel;
  Y2 = Y1 - sin(rad(45)) * travel;
}

void forward() {
  kakiNaik = -4;
  kakiTurun = 0;
  travel = 4;
  travelhalf = travel / 2;
  movetype(eforward);
}
void forward_1cm() {
  kakiNaik = -4;
  kakiTurun = 0;
  travel = 1;
  travelhalf = travel / 2;
  movetype(eforward);
}
void forward_low() {
  kakiNaik = -1;
  kakiTurun = 0;
  travel = 2;
  travelhalf = travel / 2;
  movetype(eforward);
}
void forward_low_1cm() {
  kakiNaik = -1;
  kakiTurun = 0;
  travel = 1;
  travelhalf = travel / 2;
  movetype(eforward);
}
void forward6() {
  kakiNaik = -4;
  kakiTurun = 0;
  travel = 3;
  travelhalf = travel / 2;
  movetype(eforward6);
}
void forward6_1cm() {
  kakiNaik = -4;
  kakiTurun = 0;
  travel = 1;
  travelhalf = travel / 2;
  movetype(eforward6);
}
void forward6_high() {
  kakiNaik = -3;
  kakiTurun = 1;
  travel = 3;
  travelhalf = travel / 2;
  movetype(eforward6);
}
void backward() {
  kakiNaik = -4;
  kakiTurun = 0;
  travel = 3;
  travelhalf = travel / 2;
  movetype(ebackward);
}
void backward_low() {
  kakiNaik = -1;
  kakiTurun = 0;
  travel = 3;
  travelhalf = travel / 2;
  movetype(ebackward);
}
void backward6() {
  kakiNaik = -4;
  kakiTurun = 0;
  travel = 3;
  travelhalf = travel / 2;
  movetype(ebackward6);
}
void rotate_left() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 3;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_left);
}
void rotate_left_1cm() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 1;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_left);
}
void rotate_left_fast() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_left);
}
void rotate_left_slow() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 0.5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_left);
}
void rotate_left_low() {
  kakiNaik = -1;
  kakiTurun  = 0;
  travel = 3;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_left);
}
void rotate_left_low_1cm() {
  kakiNaik = -1;
  kakiTurun  = 0;
  travel = 1;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_left);
}
void rotate_left_low_fast() {
  kakiNaik = -1;
  kakiTurun  = 0;
  travel = 5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_left);
}
void rotate6_left() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 3;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_left);
}
void rotate6_left_1cm() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 1 ;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_left);
}
void rotate6_left_high_1cm() {
  kakiNaik = -3;
  kakiTurun  = 1;
  travel = 1 ;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_left);
}
void rotate6_left_slow() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 0.5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_left);
}
void rotate6_left_fast() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_left);
}
void rotate_right() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 3;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_right);
}
void rotate_right_1cm() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 1;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_right);
}
void rotate_right_fast() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_right);
}
void rotate_right_slow() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 0.5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_right);
}
void rotate_right_low() {
  kakiNaik = -1;
  kakiTurun  = 0;
  travel = 3;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_right);
}
void rotate_right_low_1cm() {
  kakiNaik = -1;
  kakiTurun  = 0;
  travel = 1;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_right);
}
void rotate_right_low_fast() {
  kakiNaik = -1;
  kakiTurun  = 0;
  travel = 5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate_right);
}
void rotate6_right() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 3;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_right);
}
void rotate6_right_1cm() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 1;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_right);
}
void rotate6_right_high_1cm() {
  kakiNaik = -3;
  kakiTurun  = 1;
  travel = 1;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_right);
}
void rotate6_right_slow() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 0.5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_right);
}
void rotate6_right_fast() {
  kakiNaik = -4;
  kakiTurun  = 0;
  travel = 5;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(erotate6_right);
}
void shift_left() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 2 ;
  travelhalf = travel / 2;
  movetype(eshift_left);
}
void shift_left_1cm() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 1 ;
  travelhalf = travel / 2;
  movetype(eshift_left);
}
void shift_left_fast() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 4 ;
  travelhalf = travel / 2;
  movetype(eshift_left);
}
void shift_left_slow() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 0.5 ;
  travelhalf = travel / 2;
  movetype(eshift_left);
}
void shift_left_low() {
  kakiNaik = -1;
  kakiTurun = 0;
  travel = 5;
  travelhalf = travel / 2;
  movetype(eshift_left);
}
void shift_left_low_1cm() {
  kakiNaik = -1;
  kakiTurun = 0;
  travel = 1;
  travelhalf = travel / 2;
  movetype(eshift_left);
}
void shift_left_low_slow() {
  kakiNaik = -1;
  kakiTurun = 0;
  travel = 0.5;
  travelhalf = travel / 2;
  movetype(eshift_left);
}
void shift6_left() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 2 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_left);
}
void shift6_left_1cm() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 1 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_left);
}
void shift6_left_high() {
  kakiNaik = -3 ; 
  kakiTurun = 1 ; 
  travel = 2 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_left);
}
void shift6_left_slow() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 0.5 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_left);
}
void shift6_left_fast() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 4 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_left);
}
void shift6_left_high_fast() {
  kakiNaik = -3 ; 
  kakiTurun = 1 ; 
  travel = 4 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_left);
}
void shift6_left_bridge() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 6 ;
  travelhalf = travel / 2;
  width = 2;
  movetype(eshift6_left);
}
void shift_right() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 2 ;
  travelhalf = travel / 2;
  movetype(eshift_right);
}
void shift_right_1cm() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 1 ;
  travelhalf = travel / 2;
  movetype(eshift_right);
}
void shift_right_slow() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 0.5 ;
  travelhalf = travel / 2;
  movetype(eshift_right);
}
void shift_right_fast() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 4 ;
  travelhalf = travel / 2;
  movetype(eshift_right);
}
void shift_right_low() {
  kakiNaik = -1 ; 
  kakiTurun = 0 ; 
  travel = 5 ;
  travelhalf = travel / 2;
  movetype(eshift_right);
}
void shift_right_low_1cm() {
  kakiNaik = -1 ; 
  kakiTurun = 0 ; 
  travel = 1 ;
  travelhalf = travel / 2;
  movetype(eshift_right);
}
void shift_right_low_slow() {
  kakiNaik = -1 ; 
  kakiTurun = 0 ; 
  travel = 0.5 ;
  travelhalf = travel / 2;
  movetype(eshift_right);
}
void shift6_right() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 2 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_right);
}
void shift6_right_1cm() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 1 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_right);
}
void shift6_right_high() {
  kakiNaik = -3 ; 
  kakiTurun = 1 ; 
  travel = 2 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_right);
}
void shift6_right_fast() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 6 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_right);
}
void shift6_right_high_fast() {
  kakiNaik = -3 ; 
  kakiTurun = 1 ; 
  travel = 4 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_right);
}
void shift6_right_slow() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 0.5 ;
  travelhalf = travel / 2;
  width = 4;
  movetype(eshift6_right);
}
void shift6_right_bridge() {
  kakiNaik = -4 ; 
  kakiTurun = 0 ; 
  travel = 6 ;
  travelhalf = travel / 2;
  width = 2;
  movetype(eshift6_right);
}
void forward6_setupmpuup() {
  movetype(eforward6_setupmpuup);
}
void forward6_mpuup() {
  movetype(eforward6_mpuup);
}
void forward6_stuckmpuup() {
  movetype(eforward6_stuckmpuup);
}
void turn6_right_mpuup() {
  movetype(eturn6_right_mpuup);
}
void turn6_left_mpuup() {
  movetype(eturn6_left_mpuup);
}
void point0state() {
  movetype(epoint0);
}
void point_ready() {
  movetype(epoint_ready);
}
void point_backward_state() {
  movetype(epoint_backward);
}
void point_forward_state() {
  movetype(epoint_forward);
}
void point_rotate_right() {
  travel = 6;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(epoint_rotate_right);
}
void point_rotate_left() {
  travel = 6;
  travelhalf = travel / 2;
  rotate_coor();
  movetype(epoint_rotate_left);
}
void wave_hello() {
  walkspeed = 250;
  movetype(ewave_hello);
}
 
void dance() {
  walkspeed = 250;
  movetype(edance);
}
// ============================================================================
// SECTION B — taruh SETELAH movetype(), bareng wave_hello()/dance()
// ============================================================================
void sapu() {
  walkspeed = 300;
  movetype(esapu);
}

//========End of Function Move======
}
