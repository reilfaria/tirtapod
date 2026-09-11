#include "legs.h"
#include "compass.h"
//#include "gyro.h"
//#include <SoftwareSerial.h
#include "TOF.h"
#include "cam.h"
//#include "pidmpu.h"
#include <Wire.h>
#include <U8g2lib.h>

enum mode{
  L,
  R,
  F,
  };


U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/U8X8_PIN_NONE);


//#define SLAVE_ADDRESS 0x08

const unsigned char URC_blue[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x7f, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x01, 0x00,
  0x00, 0xe0, 0x00, 0xe0, 0x00, 0x00, 0x80, 0xff, 0xff, 0x01, 0x00, 0x00, 0xfe, 0xff, 0x07, 0x00,
  0x00, 0xf0, 0x00, 0xc0, 0x01, 0x00, 0xc0, 0xff, 0xff, 0x07, 0x00, 0x80, 0xff, 0xff, 0x1f, 0x00,
  0x00, 0xfc, 0x00, 0xe0, 0x07, 0x00, 0xf0, 0xff, 0xff, 0x0f, 0x00, 0xc0, 0xff, 0xff, 0x3f, 0x00,
  0x00, 0xfe, 0x00, 0xe0, 0x0f, 0x00, 0xf8, 0xff, 0xff, 0x3f, 0x00, 0xe0, 0xff, 0xff, 0x7f, 0x00,
  0x00, 0xff, 0x00, 0xe0, 0x1f, 0x00, 0xfc, 0xff, 0xff, 0x7f, 0x00, 0xf0, 0xff, 0xff, 0xff, 0x00,
  0x80, 0xff, 0x00, 0xe0, 0x3f, 0x00, 0xfe, 0x7f, 0xfe, 0x7f, 0x00, 0xf8, 0xff, 0xf0, 0xff, 0x01,
  0x80, 0xff, 0x00, 0xe0, 0x7f, 0x00, 0xff, 0x07, 0xc0, 0xff, 0x00, 0xfc, 0x1f, 0x80, 0xff, 0x03,
  0xc0, 0x7f, 0x00, 0xc0, 0x7f, 0x00, 0xff, 0x01, 0x80, 0xff, 0x01, 0xfe, 0x07, 0x00, 0xfe, 0x07,
  0xe0, 0x3f, 0x00, 0x80, 0xff, 0x80, 0xff, 0x00, 0x00, 0xfe, 0x03, 0xfe, 0x01, 0x00, 0xf8, 0x07,
  0xe0, 0x1f, 0x00, 0x00, 0xff, 0xc0, 0x7f, 0x00, 0x00, 0xfc, 0x03, 0xff, 0x00, 0x00, 0xf0, 0x0f,
  0xf0, 0x0f, 0x00, 0x00, 0xfe, 0xc1, 0x3f, 0x00, 0x00, 0xf8, 0x07, 0xff, 0x00, 0x00, 0xf0, 0x0f,
  0xf0, 0x07, 0x00, 0x00, 0xfc, 0xc1, 0x1f, 0x00, 0x00, 0xf8, 0x87, 0x7f, 0x00, 0x00, 0xe0, 0x1f,
  0xf8, 0x07, 0x00, 0x00, 0xfc, 0xe3, 0x1f, 0x00, 0x00, 0xf0, 0x87, 0x3f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x03, 0x00, 0x00, 0xf8, 0xe3, 0x0f, 0x00, 0x00, 0xf0, 0x8f, 0x3f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x03, 0x00, 0x00, 0xf8, 0xe3, 0x0f, 0x00, 0x00, 0xe0, 0xcf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x03, 0x00, 0x00, 0xf0, 0xf3, 0x07, 0x00, 0x00, 0xe0, 0xcf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x01, 0x00, 0x00, 0xf0, 0xf7, 0x07, 0x00, 0x00, 0xc0, 0xcf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xfc, 0x01, 0x00, 0x00, 0xf0, 0xf7, 0x07, 0x00, 0x00, 0xc0, 0xcf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xfc, 0x01, 0x00, 0x00, 0xf0, 0xf7, 0x07, 0x00, 0x00, 0xc0, 0xdf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xfc, 0x01, 0x00, 0x00, 0xf0, 0xf7, 0x07, 0x00, 0x00, 0xc0, 0xdf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xfc, 0x01, 0x00, 0x00, 0xf0, 0xf7, 0x07, 0x00, 0x00, 0xc0, 0xdf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x01, 0x00, 0x00, 0xf0, 0xf7, 0x07, 0x00, 0x00, 0xc0, 0xcf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x01, 0x00, 0x00, 0xf0, 0xf7, 0x07, 0x00, 0x00, 0xe0, 0xcf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x03, 0x00, 0x00, 0xf0, 0xf3, 0x07, 0x00, 0x00, 0xe0, 0xcf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x03, 0x00, 0x00, 0xf8, 0xf3, 0x0f, 0x00, 0x00, 0xe0, 0xcf, 0x1f, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x07, 0x00, 0x00, 0xf8, 0xf3, 0x0f, 0x00, 0x00, 0xf0, 0x8f, 0x3f, 0x00, 0x00, 0x00, 0x00,
  0xf0, 0x07, 0x00, 0x00, 0xfc, 0xf3, 0x1f, 0x00, 0x00, 0xf0, 0x87, 0x3f, 0x00, 0x00, 0x00, 0x0f,
  0xf0, 0x0f, 0x00, 0x00, 0xfe, 0xf1, 0x1f, 0x00, 0x00, 0xf8, 0x07, 0x7f, 0x00, 0x00, 0xe0, 0x0f,
  0xe0, 0x0f, 0x00, 0x00, 0xfe, 0xf1, 0x3f, 0x00, 0x00, 0xfc, 0x03, 0xff, 0x00, 0x00, 0xf0, 0x0f,
  0xe0, 0x1f, 0x00, 0x00, 0xff, 0xf0, 0x7f, 0x00, 0x00, 0xfe, 0x03, 0xff, 0x01, 0x00, 0xf8, 0x07,
  0xc0, 0x7f, 0x00, 0x80, 0xff, 0xf0, 0xff, 0x00, 0x00, 0xff, 0x01, 0xfe, 0x03, 0x00, 0xfc, 0x07,
  0xc0, 0xff, 0x00, 0xe0, 0x7f, 0xf0, 0xff, 0x03, 0x80, 0xff, 0x01, 0xfc, 0x0f, 0x00, 0xfe, 0x03,
  0x80, 0xff, 0x03, 0xf8, 0x3f, 0xf0, 0xff, 0x0f, 0xe0, 0xff, 0x03, 0xfc, 0x1f, 0x80, 0xff, 0x03,
  0x00, 0xff, 0xff, 0xff, 0x1f, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x07, 0xf8, 0xff, 0xff, 0xff, 0x01,
  0x00, 0xfe, 0xff, 0xff, 0x0f, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xf0, 0xff, 0xff, 0xff, 0x00,
  0x00, 0xfc, 0xff, 0xff, 0x07, 0xf0, 0xf7, 0xff, 0xff, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0x7f, 0x00,
  0x00, 0xf8, 0xff, 0xff, 0x03, 0xf0, 0xe7, 0xff, 0xff, 0xff, 0x07, 0x80, 0xff, 0xff, 0x1f, 0x00,
  0x00, 0xe0, 0xff, 0xff, 0x00, 0xf0, 0x87, 0xff, 0xff, 0xe3, 0x03, 0x00, 0xff, 0xff, 0x0f, 0x00,
  0x00, 0x80, 0xff, 0x3f, 0x00, 0xf0, 0x07, 0xff, 0xff, 0xc0, 0x03, 0x00, 0xfc, 0xff, 0x03, 0x00,
  0x00, 0x00, 0xfe, 0x0f, 0x00, 0xf0, 0x07, 0xf8, 0x3f, 0x80, 0x01, 0x00, 0xe0, 0x7f, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t degreeSymbol[] U8X8_PROGMEM = {
  0x06,  // 00000110
  0x09,  // 00001001
  0x09,  // 00001001
  0x06,  // 00000110
  0x00   // 00000000
};


#define button0 0
#define button1 1
#define toggle 10

bool tengah = false;

bool capitnaik = false;
bool Activation = false;
long int waktu = 0;
bool buttonstate = false;
bool point0state = true;
bool posisi[] = { false, false, false, false, false, false };
bool human[] = { false, false, false, false, false, false };
bool camerastate[] = { false, false, false, false, false, false };
volatile int TOFKanan, TOFSKanan, TOFDepan, TOFKiri, TOFSKiri, TOFCapit;
bool isRight;
char incomingByte;

//raspicam2 parameter
int x;
int xDummy;
int bataskanan = 450;
int bataskiri = 325;
int center = 280;
int camoffset = 8;
unsigned long int camtime = 0;

//// Compass Parameter
//// untuk arena kanan
//int cposhomeR = 28;
////////////////| HOME |////
//
//int cposR[] = { 0, 0, 54, 126, 51, 63, 314, 311, 289, 276 };
//////////////////////| R2 | R3 | R4 | R5 | R6 | R7 | R8 | R9 |///////Ruangan
//
//int cposhumanR[] = { 0 ,330, 295, 352, 349, 323 };
////////////////////////| H1 | H2 | H3 | H4 | H5 |///////Human
//
//int cpossafeR[] = { 0, 147, 95, 133, 80, 102 };
////////////////////////| S1 | S2 | S3 | S4 | S5 |///////SafeZone

//// untuk arena latihan kanan
//int cposhomeR = 194;
////////////////| HOME |////
//
//int cposR[] = { 0, 0, 215, 304, 216, 212, 140, 105, 113, 109 };
//////////////////////| R2 | R3 | R4 | R5 | R6 | R7 | R8 | R9 |///////Ruangan
//
//int cposhumanR[] = { 0 ,330, 133, 352, 349, 137 };
////////////////////////| H1 | H2 | H3 | H4 | H5 |///////Human
//
//int cpossafeR[] = { 0, 354, 248, 133, 80, 102 };
//////////////////////| S1 | S2 | S3 | S4 | S5 |///////SafeZone

// untuk arena latihan kanan
int cposhomeR = 57;
//////////////| HOME |////

int cposR[] = { 0, 0, 57, 108, 25, 24, 188, 102, 94, 88 };
////////////////////| R2 | R3 | R4 | R5 | R6 | R7 | R8 | R9 |///////Ruangan

int cposhumanR[] = { 0 ,1, 200, 352, 349, 115 };
//////////////////////| H1 | H2 | H3 | H4 | H5 |///////Human

int cpossafeR[] = { 0, 119, 222, 133, 78, 102 };
/////////////////////| S1 | S2 | S3 | S4 | S5 |///////SafeZone


//untuk arena kiri
int cposhomeL = 203;
//////////////| HOME |///
                               //204//223
int cposL[] = { 0, 0, 201, 133, 220, 223, 302, 302, 295, 305 }; 
////////////////////| R2 | R3 | R4 | R5 | R6 | R7 | R8 | R9|///////Ruangan 

int cposhumanL[] =  { 0, 12, 304, 352, 349, 259 };
//////////////////////| H1 | H2 | H3 | H4 | H5 |///////Human

int cpossafeL[] = { 0, 103, 188, 133, 322, 109 };
//////////////////////| S1 | S2 | S3 | S4 | S5 |///////SafeZone


////untuk arena latian kiri
//int cposhomeL = 23;
////////////////| HOME |///
//                                    
//int cposL[] = { 0, 0, 23, 243, 32, 48, 129, 117, 105, 106 }; 
/////////////////////| R2 | R3 | R4 | R5 | R6 | R7 | R8 | R9|///////Ruangan 
//
//int cposhumanL[] =  { 0, 12, 125, 352, 349, 82 };
////////////////////////| H1 | H2 | H3 | H4 | H5 |///////Human
//
//int cpossafeL[] = { 0, 190, 352, 133, 130, 109 };
////////////////////////| S1 | S2 | S3 | S4 | S5 |///////SafeZone


//int coffset = 15;

// Logic Home Activation
bool homestate = false;
bool overallhome = false;
unsigned long int gettimehome = 0;

// Logic For R1
bool overallR1 = false;
unsigned long int gettimeR1 = 0;
unsigned long int gettimeR1N = 0;
bool positioningR3 = false;
bool camerastate1 = false;

// Logic For R2
bool overallR2 = false;
bool tembok2kiri = false;
bool positioningR2 = false;
unsigned long int gettimeR2 = 0;

// Logic For R3
bool overallR3 = false;
bool front3 = false;
bool tembok3 = false;
bool kompas3 = false;
bool human1aman = false;
bool stateback = false;
bool tembok3kanan = false;
unsigned long int gettimeR3 = 0;

// Logic For R4
bool overallR4 = false;
bool tembok4 = false;
bool kompas4 = false;
bool tembok4kanan = false;
unsigned long int gettimeR4 = 0;
unsigned long int timetembok4 = 0;
bool human2aman = false;
bool positioningR4 = false;
bool front4 = false;
bool camerastate2 = false;

//Logic for R5
bool overallR5 = false;
bool positioningR5 = false;
bool right = false;
bool kompas5 = false;
bool tembok5 = false;
unsigned long int gettimeR5 = 0;

//Logic for R6
bool overallR6 = false;
bool positioningR6 = false;
bool wallfollowingR6 = false;
bool front6 = false;
bool tembok6kanan = false;
unsigned long int gettimeR6 = 0;

//Logic for R7
bool overallR7 = false;
bool kiri = false;
bool stuck = false;
bool tembok7 = false;
bool sebelumnanjak = false;
bool nanjak = false;
bool posisi7 = false;
unsigned long int gettimeR7 = 0;

//Logic for R8
bool overallR8 = false;
bool positioningR8 = false;
bool front8 = false;
bool human4aman = false;
bool kompas8 = false;
unsigned long int gettimeR8 = 0;

//Logic for R9
bool overallR9 = false;
bool geser9kiri = false;
bool human5aman = false;
bool lurus = false;
bool positioningR9 = false;
bool kompas9 = false;
bool left = false;


// ==== TEST BYPASS CONFIG ====
int TEST_START_ROOM = 4;   // ganti sesuai ruangan yang mau ditest. 0 = normal run dari home

void applyTestBypass(int room) {
  // Home
  if (room >= 1) {
    homestate = true;
    overallhome = true;
  }

  // Ruangan 1 selesai
  if (room >= 2) {
    overallR1 = true;
    human[1] = true;
    camerastate[1] = true;
    capitnaik = true;      // <-- pindahkan ke sini
  }

  // Ruangan 2 selesai / mau mulai R3
  if (room >= 3) {
    overallR2 = true;
    tembok2kiri = true;
    positioningR2 = true;
    // capitnaik dihapus dari sini
  }

  // Ruangan 3 selesai
  if (room >= 4) {
    overallR3 = true;
    front3 = true;
    tembok3 = true;
    kompas3 = true;
    human1aman = true;
    stateback = true;
    tembok3kanan = true;
    // capitnaik dihapus dari sini
  }

  // Ruangan 4 selesai  (cek ulang flag ini vs logic asli R4 kamu)
  if (room >= 5) {
    overallR4 = true;
    tembok4 = true;
    kompas4 = true;
    tembok4kanan = true;
    human2aman = true;
    positioningR4 = true;
    front4 = true;
    camerastate2 = true;
  }

  // Ruangan 5 selesai
  if (room >= 6) {
    overallR5 = true;
    positioningR5 = true;
    right = true;
    kompas5 = true;
    tembok5 = true;
  }

  // Ruangan 6 selesai
  if (room >= 7) {
    overallR6 = true;
    positioningR6 = true;
    wallfollowingR6 = true;
    front6 = true;
    tembok6kanan = true;
  }

  // Ruangan 7 selesai
  if (room >= 8) {
    overallR7 = true;
    kiri = true;
    stuck = true;
    tembok7 = true;
    sebelumnanjak = true;
    nanjak = true;
    posisi7 = true;
  }

  // Ruangan 8 selesai
  if (room >= 9) {
    overallR8 = true;
    positioningR8 = true;
    front8 = true;
    human4aman = true;
    kompas8 = true;
  }
}

void setup() {
  Serial.begin(38400);
  
  compass::setup();
  legs::setup();
  pinMode(button0, INPUT_PULLUP);
  pinMode(button1, INPUT_PULLUP);
  pinMode(toggle, INPUT_PULLUP);
  TOF::setup();
  cam::setup1();
  oled.begin();
  oled.clearBuffer();
  oled.drawXBMP(0, 0, 128, 64, URC_blue);
  oled.sendBuffer();
  //  getmpu::setup();
  Serial.println("Ready");
  legs::point_ready();
  legs::capitnaik(1);
  delay(200);
  legs::capitbuka();
  delay(200);
  isRight = digitalRead(toggle);
  applyTestBypass(TEST_START_ROOM);
//  Serial.println(String() + "Compass : " + compass::heading());
////    Serial.println(String() + "MPU : " + compass::pitch());
//  Serial.println(String() + TOF::getkanan() + "\t" + TOF::getserongkanan() + "\t" + TOF::getdepan() + "\t" + TOF::getserongkiri() + "\t" + TOF::getkiri() + "\t" + TOF::getcapit() + "\t");
////    Serial.println(String()+"Korban: "+cam::camx+","+cam::camy+"\t Dummy: "+cam::camx_dummy+","+cam::camy_dummy);
}


//=====Fungsi KOMPASSSSSSSSSSSSSSSSS


// Fungsi Kompas
int radius(int targetHeading, mode type = F, int coffset = 15) {
  int heading = compass::heading();
  int lowerBound, upperBound;
  switch (type){
  case R:
    lowerBound = targetHeading;
    upperBound = targetHeading + (2 * coffset);
    break;
  case L:
    lowerBound = targetHeading - (2 * coffset);
    upperBound = targetHeading;
    break;
  case F:
    lowerBound = targetHeading - coffset;
    upperBound = targetHeading + coffset;
    break;
  
  }

  if (lowerBound < 0) lowerBound += 360;
  if (upperBound >= 360) upperBound -= 360;

  if ((lowerBound < upperBound && (heading >= lowerBound && heading <= upperBound)) || (lowerBound > upperBound && (heading >= lowerBound || heading <= upperBound))) {
    return 0;
  } else {
    int clockwiseDistance = (targetHeading - heading + 360) % 360;
    int counterClockwiseDistance = (heading - targetHeading + 360) % 360;

    if (clockwiseDistance < counterClockwiseDistance) {
      return 1;
    } else {
      return -1;
    }
  }
}


//== END GUUNSI KOMPASSSSSSSSSSSSS

void setup1() {
  //  Serial.begin(38400);
  //  cam::setup1();
  //TOF::setup();
}

void loop() {
  cam::checkTimeout();
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'p') {
      point0state = !point0state;
      Activation = !Activation;
    }
  }

  int buttonValue1 = digitalRead(button1);
  if (buttonValue1 == LOW || incomingByte == 'r') {
    rp2040.reboot();
  }

  if (millis() - waktu <= 3000) {
    legs::point0state();
    Serial.println("Titik 0");
  }
  int buttonValue = digitalRead(button0);
  if (buttonValue == LOW && buttonstate == false) {
    buttonstate = true;
  }
  if (buttonValue == HIGH && buttonstate == true) {
    int loop = 0;
    if (Activation == false) {
      Activation = true;
      point0state = false;

      oled.clearBuffer();
      oledPrint("Home", 40);
      oled.sendBuffer();
      
      while (loop < 20) {
        legs::forward_low();
        Serial.println("forward while");
        loop = loop + 1;
      }
    } else if (Activation == true) {
      Activation = false;
      point0state = true;
    }
    buttonstate = false;
    delay(100);
  }
  if (point0state == true && Activation == false) {
    float compasshead = compass::heading();
    oled.clearBuffer();
    oledPrint("Heading", 20);
    oledPrintVal(compasshead, 40);
    if (isRight == LOW){
      oledPrint("Right", 60);
    } else {
      oledPrint("Left", 60);
    }
    oled.sendBuffer();
    legs::point_ready();
//    legs::forward6_mpuup();
    //    WallFollowingRight6();
    //    legs::shift_left_fast();
//    legs::capitturun(3);
    //    legs::capitnaik(1);
    //    legs::capitbuka();
    //    legs::capitjepit();
        
    Serial.println(String() + "Compass : " + compasshead);
//    Serial.println(String() + "MPU : " + compass::pitch());
    Serial.println(String() + TOF::getkanan() + "\t" + TOF::getserongkanan() + "\t" + TOF::getdepan() + "\t" + TOF::getserongkiri() + "\t" + TOF::getkiri() + "\t" + TOF::getcapit() + "\t");
//    Serial.println(String()+"Korban: "+cam::camx+","+cam::camy+"\t Dummy: "+cam::camx_dummy+","+cam::camy_dummy);
  }

  if (Activation == true && point0state == false) {

    //Bypass, comment to normal run
//     homestate = true;
//     overallhome = true;
//     overallR1 = true;
//     overallR2 = true;
//     capitnaik = true;
//        overallR3 = true;
//        tembok4 = true;
//        camerastate[2] = true;
//        overallR4 = true;
//        overallR5 = true;
//        wallfollowingR6 = true;
//        positioningR6 = true;
//        overallR6 = true;
//        overallR7 = true;
    //    overallR8 = true;

    //Kondisi Kanan =======
    if (isRight == LOW) {
      if (homestate == false && overallhome == false) {
        legs::walkspeed = 150;
        int comVal = radius(cposhomeR,F);
        if (comVal == 0) {
//          Serial.println("Forward Low");
//          oled.clearBuffer();
//          oled.setFont(u8g2_font_fub14_tr);
//          oled.drawStr(12, 27, "FORWARD");
//          oled.drawStr(35, 51, "LOW");
//          oled.sendBuffer();
//          gettimehome = millis();
//          while (millis() - gettimehome <= 2500) {
//            legs::walkspeed = 100;
//            legs::forward_low();
//          }
          homestate = true;
          overallhome = true;
          
          oled.clearBuffer();
          oledPrint("Ruangan", 30);
          oledPrint("1", 50);
          oled.sendBuffer();
          
        } else if (comVal == 1){
          legs::rotate_right_low_fast();
        } else {
          legs::rotate_left_low_fast();
        }
      } else {
        //Ruangan 1
        if (overallhome == true && overallR1 == false) {
          if (human[1] == false && camerastate[1] == false) {
            TOFKiri = TOF::getkiri();
            if (TOFKiri > 550) {
              camerastate[1] = true;
              while (millis() - gettimeR1N <= 2000){
              legs::walkspeed = 150;
              legs::backward_low();
              oled.clearBuffer();
              oledPrint("Get", 30);
              oledPrint("Human", 50);
              oled.sendBuffer();
      
             }
             delay(500);
              
            } else {
              legs::walkspeed = 150;
              legs::shift_right_low();
              gettimeR1N = millis();
            }
          }
          if (human[1] == false && camerastate[1] == true) {

            Serial.println("Get Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(38, 27, "GET");
//            oled.drawStr(32, 51, "HUMAN");
//            oled.sendBuffer();
            gethuman_low(1, 57);
            capitnaik = true;
            gettimeR1 = millis();
          }
          if (human[1] == true && camerastate[1] == true) {
            while (millis() - gettimeR1 <= 2000) {
              legs::backward_low();
            }
            overallR1 = true;
            oled.clearBuffer();
            oledPrint("Ruangan", 30);
            oledPrint("2", 50);
            oled.sendBuffer();
          }
        }

        //      if (overallhome == true && overallR1 == false) {
        //        if (human[1] == false && camerastate[1] == false) {
        //          Serial.println(String()+"Saat ini Rotate Left R1||"+compass::heading());
        //          oled.clearBuffer();
        //          oled.setFont(u8g2_font_fub14_tr);
        //          oled.drawStr(32,27,"ROTATE");
        //          oled.drawStr(24,51,"LEFT R1");
        //          oled.sendBuffer();
        //          if (compass::heading() <= coffright(cposhumanR[1])) {
        //            while (millis() - gettimeR1N <= 700){
        //              legs::walkspeed = 100;
        //              legs::backward_low();
        //            }
        //            camerastate[1] = true;
        //            delay(500);
        //          }
        //          else {
        //            legs::walkspeed = 110;
        //            legs::rotate_left_low();
        //            gettimeR1N = millis();
        //          }
        ////          while (millis() - gettimeR1N <= 1000) {
        ////            legs::walkspeed = 150;
        ////            legs::backward_low();
        ////          }
        //        }
        //        if (human[1] == false && camerastate[1] == true) {
        //          Serial.println("Get Human");
        //          oled.clearBuffer();
        //          oled.setFont(u8g2_font_fub14_tr);
        //          oled.drawStr(38,27,"GET");
        //          oled.drawStr(32,51,"HUMAN");
        //          oled.sendBuffer();
        //          gethuman_low(1);
        //          capitnaik = true;
        //          gettimeR1 = millis();
        //        }
        //        if (human[1] == true && camerastate[1] == true) {
        //          while (millis() - gettimeR1 <= 2000) {
        //            legs::backward_low();
        //          }
        //          overallR1 = true;
        ////          if (compass::heading() <= cposhomeR + coffset && compass::heading() >= cposhomeR - coffset) {
        ////            overallR1 = true;
        ////          } else {
        ////            legs::walkspeed = 110;
        ////            legs::rotate_right_low();
        ////          }
        //        }
        //      }

        // Ruangan 2
        if (overallR1 == true && overallR2 == false) {
          if (tembok2kiri == false && positioningR2 == false) {
            TOFDepan = TOF::getdepan();
            legs::walkspeed = 150;
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 27, "SHIFT");
//            oled.drawStr(34, 51, "RIGHT");
//            oled.sendBuffer();
            if (TOFDepan > 700) {
              tembok2kiri = true;
            } else {
              int comVal = radius(cposR[2],F);
              Serial.println(compass::heading());
              if (comVal == 0 ) {
                legs::shift6_right_high_fast();
              } else if (comVal == 1 ) {
                legs::rotate6_right_high_1cm();
              } else {
                legs::rotate6_left_high_1cm();
              }
            }
          }
          if (tembok2kiri == true && positioningR2 == false) {
            legs::walkspeed = 150;
            Serial.println("cek compas 2");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            int comVal = radius(cposR[3],R);
            if (comVal == 0) {
              positioningR2 = true;
              overallR2 = true;
              oled.clearBuffer();
              oledPrint("Ruangan", 30);
              oledPrint("3", 50);
              oled.sendBuffer();
            } else {
              legs::rotate6_right();            
            } 

            //          legs::walkspeed = 130;
            //          Serial.println("cek compas 2");
            //          oled.clearBuffer();
            //          oled.setFont(u8g2_font_fub14_tr);
            //          oled.drawStr(34,27,"CHECK");
            //          oled.drawStr(32,51,"COMPAS");
            //          oled.sendBuffer();
            //          if (compass::heading() >= cposR[3] + coffset) {
            //            legs::rotate6_left_1cm();
            //          } else if (compass::heading() <= cposR[3] - coffset) {
            //            legs::rotate6_right_1cm();
            //          } else {
            //            positioningR2 = true;
            //          }
          }
          if (tembok2kiri == true && positioningR2 == true) {
            TOFDepan = TOF::getdepan();
            Serial.println("ruangan 3");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(18, 39, "RUANGAN 3");
//            oled.sendBuffer();
            legs::forward6();
            if (TOFDepan < 190) {
              // Memulai penghitungan waktu jika belum dimulai
              if (gettimeR2 == 0) {
                gettimeR2 = millis();
              }
              // Memeriksa apakah telah 2 detik
              if (millis() - gettimeR2 >= 1500) {
                overallR2 = true;
                //              kompas3 = true;
              }
            } else {
              // Mengatur ulang waktu mulai jika jarak lebih besar dari 20 cm
              gettimeR2 = 0;
            }
          }
        }
        
        // Ruangan 3
        if (overallR2 == true && overallR3 == false) {
          if (capitnaik == true && human1aman == false && positioningR3 == false && kompas3 == false && tembok3 == false) {
            Serial.println("Cek Kompas 3");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            legs::walkspeed = 150;
            int comVal = radius(cposR[3],F);
            //checkpoin
            if (comVal == 0) {
              kompas3 = true;
            } else if (comVal == 1) {
              legs::rotate6_right_1cm();
            } else {
              legs::rotate6_left_1cm();
            }
          }
          if (capitnaik == true && human1aman == false && positioningR3 == false && kompas3 == true && tembok3 == false) {
            TOFDepan = TOF::getdepan();
            TOFKanan = TOF::getkanan();
            Serial.println("cek tembok 3 depan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 3");
//            oled.drawStr(34, 58, "DEPAN");
//            oled.sendBuffer();
            if (front3 == false) {
              if (TOFDepan < 270) {
                legs::backward6();
              } else if (TOFDepan > 290) {
                legs::forward6();
              } else {
                front3 = true;
              }
            }
            if (front3 == true) {
              if (TOFKanan < 130) {
                legs::shift6_left();
              } else if (TOFKanan > 150) {
                legs::shift6_right();
              } else {
                tembok3 = true;
              }
            }
//            if (front3 == true) {
//              if (TOFKanan < 210) {
//                legs::shift6_left();
//              } else if (TOFKanan > 230) {
//                legs::shift6_right();
//              } else {
//                tembok3 = true;
//              }
//            }
          }
          if (capitnaik == true && human1aman == false && positioningR3 == false && kompas3 == true && tembok3 == true) {
            Serial.println("Cek Positioning 3");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            legs::walkspeed = 180;
            gettimeR3 = millis();
            while (millis() - gettimeR3 <= 1200) {
              legs::point_rotate_right();
            }
            positioningR3 = true;
//            int comVal = radius(cpossafeR[1],R);
//            if (comVal == 0) {
//              positioningR3 = true;
//            } else {
//              legs::rotate6_right();
//            }
          }
          if (capitnaik == true && human1aman == false && positioningR3 == true && kompas3 == true && tembok3 == true) {
            legs::walkspeed = 200;
            Serial.println("Human aman");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "HUMAN");
//            oled.drawStr(36, 51, "SAFE");
//            oled.sendBuffer();
            legs::CapitJepit();
            delay(500);
            capitnaik = false;
            legs::CapitTurun();
            human1aman = true;
            gettimeR3 = millis();
          }
          if (human1aman == true && positioningR3 == true && kompas3 == true && tembok3 == true && stateback == false) {
            Serial.println("stateback");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "STATEBACK");
//            oled.sendBuffer();
            while (millis() - gettimeR3 <= 2000) {
              legs::walkspeed = 150;
              legs::backward6();
            }
            stateback = true;
          }
                  if (human1aman == true && positioningR3 == true && kompas3 == true && tembok3 == true && stateback == true) {
            TOFKanan = TOF::getkanan();
            legs::walkspeed = 150;
            Serial.println("Ruangan 4");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 4");
//            oled.sendBuffer();
            if (capitnaik == false && tembok3kanan == false) {
              legs::capitnaik(1);
              capitnaik = true;
              delay(200);
            }
            if (capitnaik == true && tembok3kanan == false) {
              legs::walkspeed = 150;
              int comVal = radius(cposR[4],L);
              if (comVal == 0) {
                unsigned long t = millis();
                while (millis() - t <= 1000) {
                  legs::forward6();
                }
                tembok3kanan = true;
              } else {
                legs::rotate6_left_fast();
              }
            }
            if (capitnaik == true && tembok3kanan == true) {
              TOFKanan = TOF::getkanan();
              if (TOFKanan > 130) {
                legs::shift6_right();
              } else {
                overallR3 = true;
                oled.clearBuffer();
                oledPrint("Ruangan", 30);
                oledPrint("4", 50);
                oled.sendBuffer();
              }
            }
          }  
        }

        // Ruangan 4
        if (overallR3 == true && overallR4 == false) {
          if (tembok4 == false && tembok4kanan == false && human[2] == false && camerastate[2] == false && kompas4 == false && positioningR4 == false && human2aman == false) {
            TOFDepan = TOF::getdepan();
            Serial.println("cek tembok 4 depan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 4");
//            oled.drawStr(34, 58, "DEPAN");
//            oled.sendBuffer();
            legs::walkspeed = 150;
            WallFollowingRight6();
            if (TOFDepan < 510) {
              // Memulai penghitungan waktu jika belum dimulai
              if (gettimeR4 == 0) {
                gettimeR4 = millis();
              }
              // Memeriksa apakah telah 2 detik
              if (millis() - gettimeR4 >= 1000) {
                tembok4 = true;
                tembok4kanan = true;
              }
            } else {
              // Mengatur ulang waktu mulai jika jarak lebih besar dari 20 cm
              gettimeR4 = 0;
            }
          }

          if (tembok4 == true && tembok4kanan == false && human[2] == false && camerastate[2] == false && kompas4 == false && positioningR4 == false && human2aman == false) {
            TOFKanan = TOF::getkanan();
            Serial.println("Cek Tembok 4 kanan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 4");
//            oled.drawStr(34, 58, "KANAN");
//            oled.sendBuffer();
            legs::walkspeed = 150;
            if (TOFKanan < 320) {
              legs::shift6_left();
            } else if (TOFKanan > 350) {
              legs::shift6_right();
            } else {
              tembok4kanan = true;
            }
          }

          if (tembok4 == true && tembok4kanan == true && human[2] == false && camerastate[2] == false && kompas4 == false && positioningR4 == false && human2aman == false) {
            Serial.println("prepare camera");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(30, 27, "PREPARE");
//            oled.drawStr(32, 51, "CAMERA");
//            oled.sendBuffer();
            legs::walkspeed = 150;
            int comVal = radius(cposhumanR[2],L);
            if (comVal == 0){
              camerastate[2] = true;
              oled.clearBuffer();
              oledPrint("Get", 30);
              oledPrint("Human", 50);
              oled.sendBuffer();
              delay(1000);
            } else {
              legs::rotate6_left();
            }
          }
          if (tembok4 == true && tembok4kanan == true && human[2] == false && camerastate[2] == true && kompas4 == false && positioningR4 == false && human2aman == false) {
            Serial.println("Get Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(38, 27, "GET");
//            oled.drawStr(32, 51, "HUMAN");
//            oled.sendBuffer();
            //legs::walkspeed = 200;
            gethuman2(2);
            capitnaik = true;
            gettimeR4 = millis();
          }
          if (tembok4 == true && tembok4kanan == true && human[2] == true && camerastate[2] == true && kompas4 == false && positioningR4 == false && human2aman == false) {
            Serial.println("Cek Kompas 4");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            while (millis() - gettimeR4 <= 1500) {
              legs::walkspeed = 150;
              legs::backward6();
            }
            int comVal = radius(cposR[4],R);
            if (comVal == 0) {
              kompas4 = true;
              //            legs::walkspeed = 150;
              //            legs::forward6();
              //            if(TOF::getdepan() < 350){
              //              kompas4 = true;
              //            }
            } else {
              legs::rotate6_right();
            }
          }
          if (tembok4 == true && tembok4kanan == true && human[2] == true && camerastate[2] == true && kompas4 == true && positioningR4 == false && human2aman == false) {
            TOFDepan = TOF::getdepan();
            TOFKanan = TOF::getkanan();
            legs::walkspeed = 120;
            Serial.println("Cek Positioning 4");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            if (front4 == false) {
              if (TOFDepan < 240) {
                legs::backward6();
              } else if (TOFDepan > 260) {
//                legs::forward6_high();
                WallFollowingRight6();
              } else {
                front4 = true;
              }
            }

            if (front4 == true) {
              if (TOFKanan < 140) { //230
                legs::shift6_left_high();
              } else if (TOFKanan > 170) { //260
                legs::shift6_right_high();
              } else {
                positioningR4 = true;
              }
            }
          }
          if (tembok4 == true && tembok4kanan && human[2] == true && camerastate[2] == true && kompas4 == true && positioningR4 == true && human2aman == false) {
            Serial.println("Naro Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "HUMAN");
//            oled.drawStr(36, 51, "SAFE");
//            oled.sendBuffer();
              legs::walkspeed = 150;
              gettimeR4 = millis();
              while (millis() - gettimeR4 <= 1200){
                legs::point_rotate_right();
              }
              legs::walkspeed = 150;
              legs::CapitJepit2();
              delay(500);
              legs::CapitTurun2();
              capitnaik = false;
              human2aman = true;
//            int comVal = radius(cpossafeR[2],R);
//            if (comVal == 0) {
//              legs::walkspeed = 150;
//              legs::CapitJepit2();
//              delay(500);
//              legs::CapitTurun2();
//              gettimeR4 = millis();
//              while (millis() - gettimeR4 <= 2000) {
//                legs::backward6();
//              }
//              capitnaik = false;
//              human2aman = true;
//            } else {
//              legs::rotate6_right_1cm();
//            }
          }

          if (tembok4 == true && tembok4kanan && human[2] == true && camerastate[2] == true && kompas4 == true && positioningR4 == true && human2aman == true) {
            Serial.println("ruangan 5");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 5");
//            oled.sendBuffer();
            if (capitnaik == false) {
              legs::CapitNaik();
              capitnaik = true;

              delay(250);
            }
            if (capitnaik == true) {
              int comVal = radius(cposR[5],L);
              //  int comVal = radius(60);
              if (comVal == 0) {
                overallR4 = true;
                oled.clearBuffer();
                oledPrint("Ruangan", 30);
                oledPrint("5", 50);
                oled.sendBuffer();
              } else {
                legs::walkspeed = 150;
                legs::rotate6_left_1cm();
                //              gettimeR4 = millis();
              }
            }
          }
        }

        // Ruangan 5
        if (overallR4 == true && overallR5 == false) {
          legs::walkspeed = 150;
          if (positioningR5 == false && kompas5 == false && tembok5 == false) {
            TOFKanan = TOF::getkanan();
            TOFDepan = TOF::getdepan();
            Serial.println("cek positioning 5");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            if (right == false) {
              if (TOFKanan < 295) {
                legs::walkspeed = 150;
                legs::shift6_left_fast();
              } else {
                right = true;
              }
            }
            if (right == true) {
              if (TOFDepan > 200) {
                legs::forward6();
              } else {
                positioningR5 = true;
              }
            }
          }
          if (positioningR5 == true && kompas5 == false && tembok5 == false) {
            Serial.println("cek kompas 5");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            //          while (millis() - gettimeR5 <= 1500) {
            //            legs::walkspeed = 150;
            //            legs::backward6();
            //          }
            //          if (compass::heading() <= cposR[6] + coffset && compass::heading() >= cposR[6] - coffset) {
            // int comVal = radius(40);
            int comVal = radius(cposR[5],F);
            if (comVal == 0) {
              legs::walkspeed = 150;
              kompas5 = true;
              //            if(TOF::getkanan() < 220){
              //              kompas5 = true;
              //            } else {
              //              legs::shift6
              //            }
            } else if (comVal == -1){
              legs::rotate6_left_1cm();
            } else {
              legs::rotate6_right_1cm();
            }
          }
          if (positioningR5 == true && kompas5 == true && tembok5 == false) {
            TOFDepan = TOF::getdepan();
            TOFKanan = TOF::getkanan();
            Serial.println("cek tembok 5 depan");
            tembok5 = true;
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 5");
//            oled.drawStr(34, 58, "DEPAN");
//            oled.sendBuffer();
//            if (TOFDepan < 160) {
//              tembok5 = true;
//            } else {
//              legs::walkspeed = 180;
//              if (compass::heading() >= cposR[6] + 10) {
//                legs::rotate_left_1cm();
//              } else if (compass::heading() <= cposR[6] - 10) {
//                legs::rotate_right_1cm();
//              } else {
//                if (TOFKanan > 190) {
//                  legs::shift_right_fast();
//                } else {
//                  legs::forward();
//                }
//              }
//            }
            tof geser//
            if (TOFKanan > 520) {
              tembok5 = true;
            } else {
              legs::walkspeed = 180;
              int comVal = radius(cposR[5],F);
              // int comVal = radius(25);
              if(comVal == -1){
                legs::rotate_left_1cm();
              } else if (comVal == 1){
                legs::rotate_right_1cm();
              } else if (TOFDepan > 190){
                legs::forward();
              } else {
                legs::shift_left_fast();
              }
            }
          }
          if (positioningR5 == true && kompas5 == true && tembok5 == true) {
            TOFKiri = TOF::getkiri();
            TOFDepan = TOF::getdepan();
            Serial.println("ruangan 6");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 6");
//            oled.sendBuffer();
            if (TOFDepan < 460){
              legs::backward();
              gettimeR5 = millis();
            } else {
              while (millis() - gettimeR5 <= 3000){
                legs::rotate_left_fast();
              }
              overallR5 = true;
              oled.clearBuffer();
              oledPrint("Ruangan", 30);
              oledPrint("6", 50);
              oled.sendBuffer();
            }
//            if (TOFKiri > 220) {
//              legs::shift_left_fast();
//            } else {
//              overallR5 = true;
//            }
          }
        }

        // Ruangan 6
        if (overallR5 == true && overallR6 == false) {
          if (wallfollowingR6 == false && positioningR6 == false && tembok6kanan == false) {
            TOFKiri = TOF::getkiri();
            TOFDepan = TOF::getdepan();
            Serial.println("cek tembok 6 depan");
//           led.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 6");
//            oled.drawStr(34, 58, "DEPAN");
//            oled.sendBuffer();
            legs::walkspeed = 150;
            int comVal = radius(cposR[6],F);
            if (comVal == -1){
              legs::rotate6_left_high_1cm();
            } else if (comVal == 1){
              legs::rotate6_right_high_1cm();
            } else if (TOFKiri > 190){
              legs::shift6_left_high();
            } else {
              legs::forward6_high();
            }
            
            if (TOFDepan < 140) {
              if (gettimeR6 == 0) {
                gettimeR6 = millis();
              }
              // Memeriksa apakah telah 2 detik
              if (millis() - gettimeR6 >= 2000) {
                wallfollowingR6 = true;
              }
            } else {
              gettimeR6 = 0;
            }
          }
          if (wallfollowingR6 == true && positioningR6 == false && tembok6kanan == false) {
            TOFDepan = TOF::getdepan();
            TOFKiri = TOF::getkiri();
            Serial.println("cek positioning 6");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            if (front6 == false) {
              if (TOFDepan < 145) {
                legs::backward6();
              } else if (TOFDepan > 175) {
                legs::forward6();
              } else {
                front6 = true;
              }
            }
            if (front6 == true) {
              if (TOFKiri < 210) {
                legs::shift6_right();
              } else if (TOFKiri > 240) {
                legs::shift6_left();
              } else {
                positioningR6 = true;
              }
            }
          }

          if (wallfollowingR6 == true && positioningR6 == true && tembok6kanan == false) {
            legs::walkspeed = 150;
            TOFKanan = TOF::getkanan();
            Serial.println(TOFKanan);
            Serial.println("cek tembok 6 kanan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 6");
//            oled.drawStr(34, 58, "KANAN");
//            oled.sendBuffer();
            legs::shift6_right_fast();
            if (TOFKanan <= 180) {
              // Memulai penghitungan waktu jika belum dimulai
              if (gettimeR6 == 0) {
                gettimeR6 = millis();
              }
              // Memeriksa apakah telah 2 detik
              if (millis() - gettimeR6 >= 2000) {
                tembok6kanan = true;
                legs::capitjepit();
                legs::capitnaik(2);
              }
            } else {
              // Mengatur ulang waktu mulai jika jarak lebih besar dari 20 cm
              gettimeR6 = 0;
            }
          }
          if (wallfollowingR6 == true && positioningR6 == true && tembok6kanan == true) {
            Serial.println("ruangan 7");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 7");
//            oled.sendBuffer();
            legs::walkspeed = 150;
            int comVal = radius(cposR[7],F);
            if (comVal == 0){
              overallR6 = true;
            } else if (comVal == 1) {
              legs::rotate6_right();
            } else {
              legs::rotate6_left();
            }
//            if (compass::heading() >= cposR[7] + coffset) {
//              legs::rotate6_left_1cm();
//            } else if (compass::heading() <= cposR[7] - coffset) {
//              legs::rotate6_right_1cm();
//            } else {
//              overallR6 = true;
//            }
          }
        }


        //Ruangan 7
        if (overallR6 == true && overallR7 == false) {
          if (kiri == false && stuck == false && tembok7 == false) {
            TOFKiri = TOF::getkiri();
            TOFDepan = TOF::getdepan();
            Serial.println("cek tembok 7 kiri");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 7");
//            oled.drawStr(36, 58, "KIRI");
//            oled.sendBuffer();
            if (TOFKiri < 300 && TOFDepan > 350) {
              kiri = true;
              gettimeR7 = millis();
            } else {
              legs::walkspeed = 150;
              legs::forward6_setupmpuup();
            }
          }
          if (kiri == true && stuck == false && tembok7 == false) {
            if (millis() - gettimeR7 >= 2000) {
              stuck = true;
            } else {
              legs::forward6_stuckmpuup();
            }
          }
          if (kiri == true && stuck == true && tembok7 == false) {
            TOFKiri = TOF::getkiri();
            TOFDepan = TOF::getdepan();
            Serial.println("stuckmpuup");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "STUCKMPUUP");
//            oled.sendBuffer();
            if (TOFKiri > 300 && TOFDepan < 500) {
              tembok7 = true;
            } else {
              legs::walkspeed = 200;
              int comVal = radius(cposR[7],F,10);
              if(comVal == -1){
                legs::turn6_left_mpuup();
              } else if (comVal == 1){
                legs::turn6_right_mpuup();
              } else {
                legs::forward6_mpuup();
              }
//              if (compass::heading() >= cposR[7] + 10) {
//                legs::turn6_left_mpuup();
//              } else if (compass::heading() <= cposR[7] - 10) {
//                legs::turn6_right_mpuup();
//              } else {
//                legs::forward6_mpuup();
//              }
            }
          }
          if (kiri == true && stuck == true && tembok7 == true) {
            TOFDepan = TOF::getdepan();
            Serial.println("ruangan 8");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 8");
//            oled.sendBuffer();
            if (TOFDepan < 380) {
              overallR7 = true;
            } else {
              legs::forward6();
            }
          }
        }

        //Ruangan 8
        if (overallR7 == true && overallR8 == false) {
          if (positioningR8 == false && human4aman == false && kompas8 == false && human[5] == false) {
            TOFDepan = TOF::getdepan();
            TOFKanan = TOF::getkanan();
            Serial.println("cek positioning 8");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            legs::walkspeed = 200;
            if (front8 == false) {
              if (TOFDepan < 320) {
                legs::backward6();
              } else if (TOFDepan > 340) {
                legs::forward6();
              } else {
                front8 = true;
              }
            }
            if (front8 == true) {
              if (TOFKanan < 180) {
                legs::shift6_left();
              } else if (TOFKanan > 210) {
                legs::shift6_right();
              } else {
                positioningR8 = true;
                human4aman = true;
              }
            }
          }

          if (positioningR8 == true && human4aman == false && kompas8 == false && human[5] == false) {
            Serial.println("Naro Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "HUMAN");
//            oled.drawStr(36, 51, "SAFE");
//            oled.sendBuffer();
            int comVal = radius(cpossafeR[4],L);
            if(comVal == 0){
              legs::CapitJepit2();
              delay(400);
              legs::CapitTurun2();
              legs::capitnaik(1);
              gettimeR8 = millis();
              human4aman = true;
            } else {
              legs::rotate6_left_1cm();
            }
//            if (compass::heading() < cpossafeR[4]) {
//              legs::CapitJepit2();
//              delay(400);
//              legs::CapitTurun2();
//              legs::capitnaik(1);
//              gettimeR8 = millis();
//              //            legs::walkspeed = 150;
//              //            while (millis() - gettimeR8 <= 1500) {
//              //              legs::backward6();
//              //            }
//              //            capitnaik = false;
//              human4aman = true;
//            } else {
//              legs::rotate6_left_1cm();
//            }
          }
          if (positioningR8 == true && human4aman == true && kompas8 == false && human[5] == false) {
            legs::walkspeed = 180;
            Serial.println("cek kompas 8");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            //          while (millis() - gettimeR8 <= 1500) {
            //              legs::backward6();
            //            }
            //          if (capitnaik == false) {
            //            legs::CapitNaik();
            //            capitnaik = true;

            //            delay(250);
            ////            overallR4 = true;
            //          }
            int comVal = radius(cposhumanR[5],R);
            if (comVal == 0){
              kompas8 = true;
              front8 = false;
              delay(500);
              x = cam::camx;
              xDummy = cam::camx_dummy;
              if ( x < xDummy || x == 0){
                human[5] = true;
              }
            } else {
              legs::rotate6_right_1cm();
            }
//            if (compass::heading() > cposhumanR[5]) {
//              kompas8 = true;
//              front8 = false;
//            } else {
//              legs::rotate6_right_1cm();
//            }
          }
          if (positioningR8 == true && human4aman == true && kompas8 == true && human[5] == false) {
            Serial.println("Get Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(38, 27, "GET");
//            oled.drawStr(32, 51, "HUMAN");
//            oled.sendBuffer();
            //legs::walkspeed = 200;
            gethuman4(5);
            capitnaik = true;
            //          gettimeR8 = millis();
          }
          if (positioningR8 == true && human4aman == true && kompas8 == true && human[5] == true) {
            TOFDepan = TOF::getdepan();
            TOFKanan = TOF::getkanan();
            Serial.println("ruangan 9");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 9");
//            oled.sendBuffer();
            int comVal = radius(cposR[8],L);
            if(comVal == 0){
              if (front8 == false) {
                if (TOFDepan < 365) {
                  legs::backward6();
                } else if (TOFDepan > 385) {
                  legs::forward6();
                } else {
                  front8 = true;
                }
              }
              if (front8 == true) {
                if (TOFKanan < 300) {
                  legs::shift6_left();
                } else {
                  overallR8 = true;
                }
              }
            } else {
              legs::rotate6_left_1cm();
            }
            //          while(millis() - gettimeR8 <= 1500){
            //            legs::backward6();
            //          }
          }
        }
        //Ruangan 9
        if (overallR8 == true && overallR9 == false) {
          legs::walkspeed = 150;
          if (geser9kiri == false && kompas9 == false && positioningR9 == false && human5aman == false) {
            TOFKiri = TOF::getkiri();
            TOFKanan = TOF::getkanan();
            Serial.println("cek tembok 9 kiri");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 9");
//            oled.drawStr(36, 58, "KIRI");
//            oled.sendBuffer();
            if (TOFKiri < 160 && TOFKanan > 580) {
              geser9kiri = true;
            } else {
              legs::walkspeed = 150;
              legs::shift6_left_bridge();
            }
          }
          if (geser9kiri == true && kompas9 == false && positioningR9 == false && human5aman == false) {
            Serial.println("cek kompas 9");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            int comVal = radius(cposR[9]);
            if(comVal == 0){
              kompas9 = true;
            } else if (comVal == 1) {
              legs::rotate_right_slow();
            } else {
              legs::rotate_left_slow();
            }
          }

          if (geser9kiri == true && kompas9 == true && positioningR9 == false && human5aman == false) {
            TOFKiri = TOF::getkiri();
            TOFDepan = TOF::getdepan();
            Serial.println("cek positioning 9");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            legs::walkspeed = 200;
            if (left == false) {
              if (TOFKiri < 162) {
                legs::shift_right_1cm();
              } else if (TOFKiri > 182) {
                legs::shift_left_1cm();
              } else {
                left = true;
              }
            }
            if (left == true) {
              if (TOFDepan < 230) {
                legs::backward();
              } else if (TOFDepan > 250) {
                legs::forward_1cm();
              } else {
                positioningR9 = true;
              }
            }
          }
          if (geser9kiri == true && kompas9 == true && positioningR9 == true && human5aman == false) {
            Serial.println("Naro Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "HUMAN");
//            oled.drawStr(36, 51, "SAFE");
//            oled.sendBuffer();
            legs::CapitJepit2();
            delay(500);
            legs::CapitTurun2();
            gettimeR4 = millis();
            legs::walkspeed = 180;
            while (millis() - gettimeR4 <= 1500) {
              legs::backward();
            }
            capitnaik = false;
            human5aman = true;
          }
          if (geser9kiri == true && kompas9 == true && positioningR9 == true && human5aman == true) {
            Serial.println("Finish");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(32, 39, "FINISH");
//            oled.sendBuffer();
            if (capitnaik == false) {
              legs::capitnaik(2);
              legs::point_ready();
            }
          }
        }
      }
    }
    //Kondisi Kiri =========
    else {
      if (homestate == false && overallhome == false) {
        legs::walkspeed = 115;
        int comVal = radius(cposhomeL,F);
        if (comVal == 0) {
//          Serial.println("Forward Low");
//          oled.clearBuffer();
//          oled.setFont(u8g2_font_fub14_tr);
//          oled.drawStr(12, 27, "FORWARD");
//          oled.drawStr(35, 51, "LOW");
//          oled.sendBuffer();
//          gettimehome = millis();
//          while (millis() - gettimehome <= 2500) {
//            legs::walkspeed = 100;
//            legs::forward_low();
//          }
          homestate = true;
          overallhome = true;
          
          oled.clearBuffer();
          oledPrint("Ruangan", 30);
          oledPrint("1", 50);
          oled.sendBuffer();
          
        } else if (comVal == 1){
          legs::rotate_right_low_fast();
        } else {
          legs::rotate_left_low_fast();
        }
      } else {
        //Ruangan 1

        if (overallhome == true && overallR1 == false) {
          if (human[1] == false && camerastate[1] == false) {
            TOFKanan = TOF::getkanan();
            if (TOFKanan > 550) {
              camerastate[1] = true;
              while (millis() - gettimeR1N <= 1200){
              legs::walkspeed = 100;
              legs::backward_low();
              
              oled.clearBuffer();
              oledPrint("Get", 30);
              oledPrint("Human", 50);
              oled.sendBuffer();
      
             }
             delay(500);
              
            } else {
              legs::walkspeed = 100;
              legs::shift_left_low();
              gettimeR1N = millis();
            }
          }
          if (human[1] == false && camerastate[1] == true) {

            Serial.println("Get Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(38, 27, "GET");
//            oled.drawStr(32, 51, "HUMAN");
//            oled.sendBuffer();
            // gethuman_low(1);
            capitnaik = true;
            gettimeR1 = millis();
          }
          if (human[1] == true && camerastate[1] == true) {
            while (millis() - gettimeR1 <= 2000) {
              legs::backward_low();
            }
            overallR1 = true;
            oled.clearBuffer();
            oledPrint("Ruangan", 30);
            oledPrint("2", 50);
            oled.sendBuffer();
          }
        }

        //      if (overallhome == true && overallR1 == false) {
        //        if (human[1] == false && camerastate[1] == false) {
        //          Serial.println(String()+"Saat ini Rotate Left R1||"+compass::heading());
        //          oled.clearBuffer();
        //          oled.setFont(u8g2_font_fub14_tr);
        //          oled.drawStr(32,27,"ROTATE");
        //          oled.drawStr(24,51,"LEFT R1");
        //          oled.sendBuffer();
        //          if (compass::heading() <= coffright(cposhumanL[1])) {
        //            while (millis() - gettimeR1N <= 700){
        //              legs::walkspeed = 100;
        //              legs::backward_low();
        //            }
        //            camerastate[1] = true;
        //            delay(500);
        //          }
        //          else {
        //            legs::walkspeed = 110;
        //            legs::rotate_left_low();
        //            gettimeR1N = millis();
        //          }
        ////          while (millis() - gettimeR1N <= 1000) {
        ////            legs::walkspeed = 150;
        ////            legs::backward_low();
        ////          }
        //        }
        //        if (human[1] == false && camerastate[1] == true) {
        //          Serial.println("Get Human");
        //          oled.clearBuffer();
        //          oled.setFont(u8g2_font_fub14_tr);
        //          oled.drawStr(38,27,"GET");
        //          oled.drawStr(32,51,"HUMAN");
        //          oled.sendBuffer();
        //          gethuman_low(1);
        //          capitnaik = true;
        //          gettimeR1 = millis();
        //        }
        //        if (human[1] == true && camerastate[1] == true) {
        //          while (millis() - gettimeR1 <= 2000) {
        //            legs::backward_low();
        //          }
        //          overallR1 = true;
        ////          if (compass::heading() <= cposhomeR + coffset && compass::heading() >= cposhomeR - coffset) {
        ////            overallR1 = true;
        ////          } else {
        ////            legs::walkspeed = 110;
        ////            legs::rotate_right_low();
        ////          }
        //        }
        //      }

        // Ruangan 2
        if (overallR1 == true && overallR2 == false) {
          if (tembok2kiri == false && positioningR2 == false) {
            TOFDepan = TOF::getdepan();
            legs::walkspeed = 100;
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 27, "SHIFT");
//            oled.drawStr(34, 51, "RIGHT");
//            oled.sendBuffer();
            if (TOFDepan > 700) {
              tembok2kiri = true;
            } else {
              legs::walkspeed = 80;
              int comVal = radius(cposL[2],F);
              Serial.println(compass::heading());
              if (comVal == 0 ) {
                //dibenerin
                legs::shift6_left_high_fast();
              } else if (comVal == 1 ) {
                legs::rotate6_right_high_1cm();
              } else {
                legs::rotate6_left_high_1cm();
              }
            }
          }
          if (tembok2kiri == true && positioningR2 == false) {
            legs::walkspeed = 110;
            Serial.println("cek compas 2");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            int comVal = radius(cposL[3],L);
            if (comVal == 0) {
              positioningR2 = true;
              overallR2 = true;
              oled.clearBuffer();
              oledPrint("Ruangan", 30);
              oledPrint("3", 50);
              oled.sendBuffer();
            } else {
              legs::rotate6_left();            
            } 

            //          legs::walkspeed = 130;
            //          Serial.println("cek compas 2");
            //          oled.clearBuffer();
            //          oled.setFont(u8g2_font_fub14_tr);
            //          oled.drawStr(34,27,"CHECK");
            //          oled.drawStr(32,51,"COMPAS");
            //          oled.sendBuffer();
            //          if (compass::heading() >= cposL[3] + coffset) {
            //            legs::rotate6_left_1cm();
            //          } else if (compass::heading() <= cposL[3] - coffset) {
            //            legs::rotate6_right_1cm();
            //          } else {
            //            positioningR2 = true;
            //          }
          }
          if (tembok2kiri == true && positioningR2 == true) {
            TOFDepan = TOF::getdepan();
            Serial.println("ruangan 3");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(18, 39, "RUANGAN 3");
//            oled.sendBuffer();
            legs::forward6();
            if (TOFDepan < 190) {
              // Memulai penghitungan waktu jika belum dimulai
              if (gettimeR2 == 0) {
                gettimeR2 = millis();
              }
              // Memeriksa apakah telah 2 detik
              if (millis() - gettimeR2 >= 1500) {
                overallR2 = true;
                //              kompas3 = true;
              }
            } else {
              // Mengatur ulang waktu mulai jika jarak lebih besar dari 20 cm
              gettimeR2 = 0;
            }
          }
        }


        // Ruangan 3
        if (overallR2 == true && overallR3 == false) {
          if (capitnaik == true && human1aman == false && positioningR3 == false && kompas3 == false && tembok3 == false) {
            Serial.println("Cek Kompas 3");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            legs::walkspeed = 120;
            int comVal = radius(cposL[3],F);
            if (comVal == 0) {
              kompas3 = true;
            } else if (comVal == 1) {
              legs::rotate6_right_1cm();
            } else {
              legs::rotate6_left_1cm();
            }
          }
          if (capitnaik == true && human1aman == false && positioningR3 == false && kompas3 == true && tembok3 == false) {
            TOFDepan = TOF::getdepan();
            TOFKiri = TOF::getkiri();
            Serial.println("cek tembok 3 depan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 3");
//            oled.drawStr(34, 58, "DEPAN");
//            oled.sendBuffer();
            if (front3 == false) {
              if (TOFDepan < 210) {
                legs::backward6();
              } else if (TOFDepan > 230) {
                legs::forward6();
              } else {
                front3 = true;
              }
            }

            if (front3 == true) {
              if (TOFKiri < 210) {
                legs::shift6_right();
              } else if (TOFKiri > 230) {
                legs::shift6_left();
              } else {
                tembok3 = true;
              }
            }
          }
          if (capitnaik == true && human1aman == false && positioningR3 == false && kompas3 == true && tembok3 == true) {
            Serial.println("Cek Positioning 3");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            legs::walkspeed = 180;
            //          gettimeR3 = millis();
            //          while (millis() - gettimeR3 <= 1850) {
            //            legs::rotate6_right();
            //          }
            int comVal = radius(cpossafeL[1],L);
            if (comVal == 0) {
              positioningR3 = true;
            } else {
              legs::rotate6_left();
            }
          }
          if (capitnaik == true && human1aman == false && positioningR3 == true && kompas3 == true && tembok3 == true) {
            legs::walkspeed = 200;
            Serial.println("Human aman");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "HUMAN");
//            oled.drawStr(36, 51, "SAFE");
//            oled.sendBuffer();
            legs::CapitJepit();
            delay(500);
            capitnaik = false;
            legs::CapitTurun();
            human1aman = true;
            gettimeR3 = millis();
          }
          if (human1aman == true && positioningR3 == true && kompas3 == true && tembok3 == true && stateback == false) {
            Serial.println("stateback");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "STATEBACK");
//            oled.sendBuffer();
            while (millis() - gettimeR3 <= 2000) {
              legs::walkspeed = 150;
              legs::backward6();
            }
            stateback = true;
          }
          if (human1aman == true && positioningR3 == true && kompas3 == true && tembok3 == true && stateback == true) {
            TOFKiri = TOF::getkiri();
            legs::walkspeed = 150;
            Serial.println("Ruangan 4");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 4");
//            oled.sendBuffer();
            if (capitnaik == false && tembok3kanan == false) {
              legs::capitnaik(1);
              capitnaik = true;
              delay(200);
            }
            if (capitnaik == true && tembok3kanan == false) {
              legs::walkspeed = 150;
              int comVal = radius(cposL[4],R);
              if (comVal == 0) {
                tembok3kanan = true;
              } else {
                legs::rotate6_right_fast();
              }
            }
            if (capitnaik == true && tembok3kanan == true) {
              if (TOFKiri > 160) {
                legs::shift6_left();
              } else {
                overallR3 = true;
                oled.clearBuffer();
                oledPrint("Ruangan", 30);
                oledPrint("4", 50);
                oled.sendBuffer();
              }
            }
          }
        }

        // Ruangan 4
        if (overallR3 == true && overallR4 == false) {
          if (tembok4 == false && tembok4kanan == false && human[2] == false && camerastate[2] == false && kompas4 == false && positioningR4 == false && human2aman == false) {
            TOFDepan = TOF::getdepan();
            Serial.println("cek tembok 4 depan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 4");
//            oled.drawStr(34, 58, "DEPAN");
//            oled.sendBuffer();
            legs::walkspeed = 120;
            WallFollowingLeft6();
            if (TOFDepan < 510) {
              // Memulai penghitungan waktu jika belum dimulai
              if (gettimeR4 == 0) {
                gettimeR4 = millis();
              }
              // Memeriksa apakah telah 2 detik
              if (millis() - gettimeR4 >= 1000) {
                tembok4 = true;
                tembok4kanan = true;
              }
            } else {
              // Mengatur ulang waktu mulai jika jarak lebih besar dari 20 cm
              gettimeR4 = 0;
            }
          }

          if (tembok4 == true && tembok4kanan == false && human[2] == false && camerastate[2] == false && kompas4 == false && positioningR4 == false && human2aman == false) {
            TOFKiri = TOF::getkiri();
            Serial.println("Cek Tembok 4 kanan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 4");
//            oled.drawStr(34, 58, "KANAN");
//            oled.sendBuffer();
            legs::walkspeed = 150;
            if (TOFKiri < 320) {
              legs::shift6_right();
            } else if (TOFKiri > 350) {
              legs::shift6_left();
            } else {
              tembok4kanan = true;
            }
          }

          if (tembok4 == true && tembok4kanan == true && human[2] == false && camerastate[2] == false && kompas4 == false && positioningR4 == false && human2aman == false) {
            Serial.println("prepare camera");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(30, 27, "PREPARE");
//            oled.drawStr(32, 51, "CAMERA");
//            oled.sendBuffer();
            legs::walkspeed = 150;
            int comVal = radius(cposhumanL[2],R);
            if (comVal == 0){
              camerastate[2] = true;
              oled.clearBuffer();
              oledPrint("Get", 30);
              oledPrint("Human", 50);
              oled.sendBuffer();
              delay(1000);
            } else {
              legs::rotate6_right();
            }
          }
          if (tembok4 == true && tembok4kanan == true && human[2] == false && camerastate[2] == true && kompas4 == false && positioningR4 == false && human2aman == false) {
            Serial.println("Get Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(38, 27, "GET");
//            oled.drawStr(32, 51, "HUMAN");
//            oled.sendBuffer();
            //legs::walkspeed = 200;
            gethuman2(2);
            capitnaik = true;
            gettimeR4 = millis();
          }
          if (tembok4 == true && tembok4kanan == true && human[2] == true && camerastate[2] == true && kompas4 == false && positioningR4 == false && human2aman == false) {
            Serial.println("Cek Kompas 4");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            while (millis() - gettimeR4 <= 1500) {
              legs::walkspeed = 150;
              legs::backward6();
            }
            int comVal = radius(cposL[4],L);
            if (comVal == 0) {
              kompas4 = true;
              //            legs::walkspeed = 150;
              //            legs::forward6();
              //            if(TOF::getdepan() < 350){
              //              kompas4 = true;
              //            }
            } else {
              legs::rotate6_left();
            }
          }
          if (tembok4 == true && tembok4kanan == true && human[2] == true && camerastate[2] == true && kompas4 == true && positioningR4 == false && human2aman == false) {
            TOFDepan = TOF::getdepan();
            TOFKiri = TOF::getkiri();
            Serial.println("Cek Positioning 4");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            if (front4 == false) {
              if (TOFDepan < 250) {
                legs::backward6();
              } else if (TOFDepan > 280) {
//                legs::forward6_high();
                // NOTE, bisa diganti dengan
                 WallFollowingLeft6();
                // supaya lebih lurus dan mengkoreksi sesuai tembok
              } else {
                front4 = true;
              }
            }

            if (front4 == true) {
              if (TOFKiri < 230) {
                legs::shift6_right_high();
              } else if (TOFKiri > 260) {
                legs::shift6_left_high();
              } else {
                positioningR4 = true;
              }
            }
          }
          if (tembok4 == true && tembok4kanan && human[2] == true && camerastate[2] == true && kompas4 == true && positioningR4 == true && human2aman == false) {
            Serial.println("Naro Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "HUMAN");
//            oled.drawStr(36, 51, "SAFE");
//            oled.sendBuffer();
            int comVal = radius(cpossafeL[2],L);
            if (comVal == 0) {
              legs::CapitJepit2();
              delay(500);
              legs::CapitTurun2();
              gettimeR4 = millis();
              legs::walkspeed = 150;
              while (millis() - gettimeR4 <= 2000) {
                legs::backward6();
              }
              capitnaik = false;
              human2aman = true;
            } else {
              legs::rotate6_left_1cm();
            }
          }

          if (tembok4 == true && tembok4kanan && human[2] == true && camerastate[2] == true && kompas4 == true && positioningR4 == true && human2aman == true) {
            Serial.println("ruangan 5");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 5");
//            oled.sendBuffer();
            if (capitnaik == false) {
              legs::CapitNaik();
              capitnaik = true;

              delay(250);
            }
            if (capitnaik == true) {
              int comVal = radius(cposL[5],L);
              if (comVal == 0) {
                overallR4 = true;
                oled.clearBuffer();
                oledPrint("Ruangan", 30);
                oledPrint("5", 50);
                oled.sendBuffer();
              } else {
                legs::walkspeed = 140;
                // diganti
                legs::rotate6_right_1cm();
                //              gettimeR4 = millis();
              }
            }
          }
        }

        // Ruangan 5
        if (overallR4 == true && overallR5 == false) {
          legs::walkspeed = 120;
          if (positioningR5 == false && kompas5 == false && tembok5 == false) {
            TOFKiri = TOF::getkiri();
            TOFDepan = TOF::getdepan();
            Serial.println("cek positioning 5");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            if (right == false) {
              legs::walkspeed = 100;
              if (TOFKiri < 395) {
                legs::shift6_right_fast();
              } else {
                right = true;
              }
            }
            if (right == true) {
              if (TOFDepan > 200) {
                legs::forward6();
              } else {
                positioningR5 = true;
              }
            }
          }
          if (positioningR5 == true && kompas5 == false && tembok5 == false) {
            Serial.println("cek kompas 5");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            //          while (millis() - gettimeR5 <= 1500) {
            //            legs::walkspeed = 150;
            //            legs::backward6();
            //          }
            //          if (compass::heading() <= cposL[6] + coffset && compass::heading() >= cposL[6] - coffset) {
            int comVal = radius(cposL[5],F);
            if (comVal == 0) {
              legs::walkspeed = 150;
              kompas5 = true;
              //            if(TOF::getkanan() < 220){
              //              kompas5 = true;
              //            } else {
              //              legs::shift6
              //            }
            } else if (comVal == -1){
              legs::rotate6_left_1cm();
            } else {
              legs::rotate6_right_1cm();
            }
          }
          if (positioningR5 == true && kompas5 == true && tembok5 == false) {
            TOFDepan = TOF::getdepan();
            TOFKiri = TOF::getkiri();
            Serial.println("cek tembok 5 depan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 5");
//            oled.drawStr(34, 58, "DEPAN");
//            oled.sendBuffer();
//            if (TOFDepan < 160) {
//              tembok5 = true;
//            } else {
//              legs::walkspeed = 180;
//              if (compass::heading() >= cposL[6] + 10) {
//                legs::rotate_left_1cm();
//              } else if (compass::heading() <= cposL[6] - 10) {
//                legs::rotate_right_1cm();
//              } else {
//                if (TOFKanan > 190) {
//                  legs::shift_right_fast();
//                } else {
//                  legs::forward();
//                }
//              }
//            }
            //tof geser//
            if (TOFKiri > 605) {
              tembok5 = true;
            } else {
              legs::walkspeed = 180;
              int comVal = radius(cposL[5],F);
              if(comVal == -1){
                legs::rotate_left_1cm();
              } else if (comVal == 1){
                legs::rotate_right_1cm();
              } else if (TOFDepan > 190){
                legs::forward();
              } else {
                legs::shift_right_fast();
              }
              
            }
          }
          if (positioningR5 == true && kompas5 == true && tembok5 == true) {
            TOFKanan = TOF::getkanan();
            TOFDepan = TOF::getdepan();
            Serial.println("ruangan 6");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 6");
//            oled.sendBuffer();
            if (TOFDepan < 460){
              legs::backward();
            } else {
              overallR5 = true;
              oled.clearBuffer();
              oledPrint("Ruangan", 30);
              oledPrint("6", 50);
              oled.sendBuffer();
            }
//            if (TOFKanan > 220) {
//              legs::shift_right_fast();
//            } else {
//              overallR5 = true;
//            }
          }
          //NOTE, bisa ditambahkan rotate yang cepat
        }

        // Ruangan 6
        if (overallR5 == true && overallR6 == false) {
          if (wallfollowingR6 == false && positioningR6 == false && tembok6kanan == false) {
            TOFKanan = TOF::getkanan();
            TOFDepan = TOF::getdepan();
            Serial.println("cek tembok 6 depan");
//           led.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 6");
//            oled.drawStr(34, 58, "DEPAN");
//            oled.sendBuffer();
            int comVal = radius(cposL[6],F);
            if (comVal == -1){
              legs::rotate6_left_high_1cm();
            } else if (comVal == 1){
              legs::rotate6_right_high_1cm();
            } else if (TOFKanan > 190){
              legs::shift6_right_high();
            } else {
              legs::forward6_high();
            }
            
            if (TOFDepan < 140) {
              if (gettimeR6 == 0) {
                gettimeR6 = millis();
              }
              // Memeriksa apakah telah 2 detik
              if (millis() - gettimeR6 >= 2000) {
                wallfollowingR6 = true;
              }
            } else {
              gettimeR6 = 0;
            }
          }
          if (wallfollowingR6 == true && positioningR6 == false && tembok6kanan == false) {
            TOFDepan = TOF::getdepan();
            TOFKanan = TOF::getkanan();
            Serial.println("cek positioning 6");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            if (front6 == false) {
              if (TOFDepan < 140) {
                legs::backward6();
              } else if (TOFDepan > 170) {
                legs::forward6();
              } else {
                front6 = true;
              }
            }
            if (front6 == true) {
              if (TOFKanan < 210) {
                legs::shift6_left();
              } else if (TOFKanan > 240) {
                legs::shift6_right();
              } else {
                positioningR6 = true;
              }
            }
          }

          if (wallfollowingR6 == true && positioningR6 == true && tembok6kanan == false) {
            legs::walkspeed = 100;
            TOFKiri = TOF::getkiri();
            Serial.println(TOFKiri);
            Serial.println("cek tembok 6 kanan");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 6");
//            oled.drawStr(34, 58, "KANAN");
//            oled.sendBuffer();
            legs::shift6_left_fast();
            if (TOFKiri <= 180) {
              // Memulai penghitungan waktu jika belum dimulai
              if (gettimeR6 == 0) {
                gettimeR6 = millis();
              }
              // Memeriksa apakah telah 2 detik
              if (millis() - gettimeR6 >= 2000) {
                tembok6kanan = true;
                legs::capitjepit();
                legs::capitnaik(2);
              }
            } else {
              // Mengatur ulang waktu mulai jika jarak lebih besar dari 20 cm
              gettimeR6 = 0;
            }
          }
          if (wallfollowingR6 == true && positioningR6 == true && tembok6kanan == true) {
            Serial.println("ruangan 7");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 7");
//            oled.sendBuffer();
            legs::walkspeed = 130;
            int comVal = radius(cposL[7],F);
            if (comVal == 0){
              overallR6 = true;
            } else if (comVal == 1) {
              legs::rotate6_right();
            } else {
              legs::rotate6_left();
            }
//            if (compass::heading() >= cposL[7] + coffset) {
//              legs::rotate6_left_1cm();
//            } else if (compass::heading() <= cposL[7] - coffset) {
//              legs::rotate6_right_1cm();
//            } else {
//              overallR6 = true;
//            }
          }
        }


        //Ruangan 7
        if (overallR6 == true && overallR7 == false) {
          if (kiri == false && stuck == false && tembok7 == false) {
            TOFKanan = TOF::getkanan();
            TOFDepan = TOF::getdepan();
            Serial.println("cek tembok 7 kiri");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 7");
//            oled.drawStr(36, 58, "KIRI");
//            oled.sendBuffer();
            if (TOFKanan < 300 && TOFDepan > 350) {
              kiri = true;
              gettimeR7 = millis();
            } else {
              legs::walkspeed = 150;
              legs::forward6_setupmpuup();
            }
          }
          if (kiri == true && stuck == false && tembok7 == false) {
            if (millis() - gettimeR7 >= 2000) {
              stuck = true;
            } else {
              legs::forward6_stuckmpuup();
            }
          }
          if (kiri == true && stuck == true && tembok7 == false) {
            TOFKanan = TOF::getkanan();
            TOFDepan = TOF::getdepan();
            Serial.println("stuckmpuup");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "STUCKMPUUP");
//            oled.sendBuffer();
            if (TOFKanan > 500 && TOFDepan < 500) {
              tembok7 = true;
            } else {
              legs::walkspeed = 170;
              int comVal = radius(cposL[7],F);
              if(comVal == -1){
                legs::turn6_left_mpuup();
              } else if (comVal == 1){
                legs::turn6_right_mpuup();
              } else {
                legs::forward6_mpuup();
              }
//              if (compass::heading() >= cposL[7] + 10) {
//                legs::turn6_left_mpuup();
//              } else if (compass::heading() <= cposL[7] - 10) {
//                legs::turn6_right_mpuup();
//              } else {
//                legs::forward6_mpuup();
//              }
            }
          }
          if (kiri == true && stuck == true && tembok7 == true) {
            TOFDepan = TOF::getdepan();
            Serial.println("ruangan 8");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 8");
//            oled.sendBuffer();
            if (TOFDepan < 380) {
              overallR7 = true;
            } else {
              legs::walkspeed = 100;
              legs::forward6();
            }
          }
        }

        //Ruangan 8
        if (overallR7 == true && overallR8 == false) {
          if (positioningR8 == false && human4aman == false && kompas8 == false && human[5] == false) {
            TOFDepan = TOF::getdepan();
            TOFKiri = TOF::getkiri();
            Serial.println("cek positioning 8");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            legs::walkspeed = 200;
            if (front8 == false) {
              if (TOFDepan < 320) {
                legs::backward6();
              } else if (TOFDepan > 340) {
                legs::forward6();
              } else {
                front8 = true;
              }
            }
            if (front8 == true) {
              if (TOFKiri < 180) {
                legs::shift6_right();
              } else if (TOFKiri > 210) {
                legs::shift6_left();
              } else {
                positioningR8 = true;
                human4aman = true;
              }
            }
          }

          if (positioningR8 == true && human4aman == false && kompas8 == false && human[5] == false) {
            Serial.println("Naro Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "HUMAN");
//            oled.drawStr(36, 51, "SAFE");
//            oled.sendBuffer();
            int comVal = radius(cpossafeL[4],R);
            if(comVal == 0){
              legs::CapitJepit2();
              delay(400);
              legs::CapitTurun2();
              legs::capitnaik(1);
              gettimeR8 = millis();
              human4aman = true;
            } else {
              legs::rotate6_right_1cm();
            }
//            if (compass::heading() < cpossafeL[4]) {
//              legs::CapitJepit2();
//              delay(400);
//              legs::CapitTurun2();
//              legs::capitnaik(1);
//              gettimeR8 = millis();
//              //            legs::walkspeed = 150;
//              //            while (millis() - gettimeR8 <= 1500) {
//              //              legs::backward6();
//              //            }
//              //            capitnaik = false;
//              human4aman = true;
//            } else {
//              legs::rotate6_left_1cm();
//            }
          }
          if (positioningR8 == true && human4aman == true && kompas8 == false && human[5] == false) {
            legs::walkspeed = 180;
            Serial.println("cek kompas 8");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            //          while (millis() - gettimeR8 <= 1500) {
            //              legs::backward6();
            //            }
            //          if (capitnaik == false) {
            //            legs::CapitNaik();
            //            capitnaik = true;

            //            delay(250);
            ////            overallR4 = true;
            //          }
            int comVal = radius(cposhumanL[5],L);
            if (comVal == 0){
              kompas8 = true;
              front8 = false;
              delay(500);
              x = cam::camx;
              xDummy = cam::camx_dummy;
              if ( x > xDummy || x == 0){
                human[5] = true;
              }
            } else {
              legs::rotate6_left_1cm();
            }
//            if (compass::heading() > cposhumanL[5]) {
//              kompas8 = true;
//              front8 = false;
//            } else {
//              legs::rotate6_right_1cm();
//            }
          }
          if (positioningR8 == true && human4aman == true && kompas8 == true && human[5] == false) {
            Serial.println("Get Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(38, 27, "GET");
//            oled.drawStr(32, 51, "HUMAN");
//            oled.sendBuffer();
            //legs::walkspeed = 200;
            gethuman4(5);
            capitnaik = true;
            //          gettimeR8 = millis();
          }
          if (positioningR8 == true && human4aman == true && kompas8 == true && human[5] == true) {
            TOFDepan = TOF::getdepan();
            TOFKiri = TOF::getkiri();
            Serial.println("ruangan 9");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(20, 39, "RUANGAN 9");
//            oled.sendBuffer();
            int comVal = radius(cposL[8],R);
            if(comVal == 0){
              if (front8 == false) {
                if (TOFDepan < 365) {
                  legs::backward6();
                } else if (TOFDepan > 385) {
                  legs::forward6();
                } else {
                  front8 = true;
                }
              }
              if (front8 == true) {
                if (TOFKiri < 300) {
                  legs::shift6_right();
                } else {
                  overallR8 = true;
                }
              }
            } else {
              //diganti
              legs::rotate6_right_1cm();
            }
 
            //          while(millis() - gettimeR8 <= 1500){
            //            legs::backward6();
            //          }
          }
        }
        //Ruangan 9
        if (overallR8 == true && overallR9 == false) {
          legs::walkspeed = 150;
          if (geser9kiri == false && kompas9 == false && positioningR9 == false && human5aman == false) {
            TOFKiri = TOF::getkiri();
            TOFKanan = TOF::getkanan();
            Serial.println("cek tembok 9 kiri");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub11_tr);
//            oled.drawStr(34, 18, "CHECK");
//            oled.drawStr(24, 38, "TEMBOK 9");
//            oled.drawStr(36, 58, "KIRI");
//            oled.sendBuffer();
            if (TOFKanan < 200 && TOFKiri > 380){
              geser9kiri = true;
            } else {
              legs::walkspeed = 150;
              legs::shift6_right_bridge();
            }
          }
          if (geser9kiri == true && kompas9 == false && positioningR9 == false && human5aman == false) {
            Serial.println("cek kompas 9");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(32, 51, "COMPAS");
//            oled.sendBuffer();
            int comVal = radius(cposL[9]);
            if(comVal == 0){
              kompas9 = true;
            } else if (comVal == 1) {
              legs::rotate_right_slow();
            } else {
              legs::rotate_left_slow();
            }
          }

          if (geser9kiri == true && kompas9 == true && positioningR9 == false && human5aman == false) {
            TOFKanan = TOF::getkanan();
            TOFDepan = TOF::getdepan();
            Serial.println("cek positioning 9");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "CHECK");
//            oled.drawStr(16, 51, "POSITIONING");
//            oled.sendBuffer();
            legs::walkspeed = 200;
            if (left == false) {
              if (TOFKanan < 162) {
                legs::shift_left_1cm();
              } else if (TOFKanan > 182) {
                legs::shift_right_1cm();
              } else {
                left = true;
              }
            }
            if (left == true) {
              if (TOFDepan < 230) {
                legs::backward();
              } else if (TOFDepan > 250) {
                legs::forward_1cm();
              } else {
                positioningR9 = true;
              }
            }
          }
          if (geser9kiri == true && kompas9 == true && positioningR9 == true && human5aman == false) {
            Serial.println("Naro Human");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(34, 27, "HUMAN");
//            oled.drawStr(36, 51, "SAFE");
//            oled.sendBuffer();
            legs::CapitJepit2();
            delay(500);
            legs::CapitTurun2();
            gettimeR4 = millis();
            legs::walkspeed = 180;
            while (millis() - gettimeR4 <= 1500) {
              legs::backward();
            }
            capitnaik = false;
            human5aman = true;
          }
          if (geser9kiri == true && kompas9 == true && positioningR9 == true && human5aman == true) {
            Serial.println("Finish");
//            oled.clearBuffer();
//            oled.setFont(u8g2_font_fub14_tr);
//            oled.drawStr(32, 39, "FINISH");
//            oled.sendBuffer();
            if (capitnaik == false) {
              legs::capitnaik(2);
              legs::point_ready();
            }
          }
        }
      }
    }
  }
}


void oledPrint(const char* text, int y) {
  oled.setFont(u8g2_font_fub14_tr);
  int textWidth = oled.getStrWidth(text);
  int xPosText = (128 - textWidth) / 2;
  oled.setCursor(xPosText, y);
  oled.print(text);
}

void oledPrintVal(float val, int y) {
  oled.setFont(u8g2_font_fub14_tr);
  String valStr = String(val);
  const char* valChar = valStr.c_str();
  int xPosVal = (128 - oled.getStrWidth(valChar)) / 2;
  oled.setCursor(xPosVal, y);
  oled.print(valChar);
}

void WallFollowingRight6() {
  int TOFKanan = TOF::getkanan();
  int TOFSKanan = TOF::getserongkanan();

  if (TOF::getkanan() <= 140 && TOF::getserongkanan() >= 180) {
    Serial.println("Shift Left");
    legs::shift6_left_high();
  } else if (TOF::getkanan() <= 180 && TOF::getserongkanan() <= 160) {
    Serial.println("Rotate left");
    legs::rotate6_left_high_1cm();
  } else if (TOF::getkanan() >= 200 && TOF::getserongkanan() < 280 || TOF::getkanan() > 290) {
    Serial.println("Shift Right");
    legs::shift6_right_high();
  } else if (TOF::getkanan() <= 180 && TOF::getserongkanan() >= 280 || TOF::getserongkanan() > 330) {
    Serial.println("Rotate right");
    legs::rotate6_right_high_1cm();
  } else {
    Serial.println("forward6");
    legs::forward6_high();
  }
}

// // Wall Following Right
void WallFollowingRight(bool tinggi) {
  int TOFKanan = TOF::getkanan();
  int TOFSKanan = TOF::getserongkanan();

  if (tinggi == true) {
    if (TOFKanan <= 160 && TOFSKanan >= 200) {
      Serial.println("Shift Left");
      legs::shift_left();
    } else if (TOFKanan <= 200 && TOFSKanan <= 180) {
      Serial.println("Rotate left");
      legs::rotate_left();
    } else if (TOFKanan >= 220 && TOFSKanan < 300 || TOFKanan > 310) {
      Serial.println("Shift Right");
      legs::shift_right();
    } else if (TOFKanan <= 200 && TOFSKanan >= 300 || TOFSKanan > 350) {
      Serial.println("Rotate right");
      legs::rotate_right();
    } else {
      Serial.println("forward6");
      legs::forward();
    }
  }
  if (tinggi == false) {
    if (TOFKanan <= 200 && TOFSKanan >= 200) {
      Serial.println("Shift Left");
      legs::shift_left_low();
    } else if (TOFKanan <= 200 && TOFSKanan <= 180) {
      Serial.println("Rotate left");
      legs::rotate_left_low();
    } else if (TOFKanan >= 240 && TOFSKanan < 390) {
      Serial.println("Shift Right");
      legs::shift_right_low();
    } else if (TOFKanan >= 250 && TOFSKanan >= 390) {
      Serial.println("Rotate right");
      legs::rotate_right_low();
    } else {
      Serial.println("forward");
      legs::forward_low();
    }
  }
}

//Wall Following Left
// void WallFollowingLeft() {
//   if (TOF::getdepan() >= 5 && TOF::getsampingkiri() <= 20 && TOF::getserongkiri() >= 20) {
//     Serial.println("shift Right");
//     legs::shift_right();
//   } else if (TOF::getdepan() >= 5 && TOF::getsampingkiri() <= 20 && TOF::getserongkiri() <= 18) {
//     Serial.println("rotate Right");
//     legs::rotate_right();
//   } else if (TOF::getdepan() >= 5 && TOF::getsampingkiri() >= 24 && TOF::getserongkiri() < 39) {
//     Serial.println("shift Left");
//     legs::shift_left();
//   } else if (TOF::getdepan() >= 5 && TOF::getsampingkiri() >= 25 && TOF::getserongkiri() >= 39) {
//     Serial.println("rotate Left");
//     legs::rotate_left();
//   } else {
//     Serial.println("forward");
//     legs::forward();
//   }
// }

void WallFollowingLeft6() {
  int TOFKiri = TOF::getkiri();
  int TOFSKiri = TOF::getserongkiri();

  if (TOFKiri <= 140 && TOFSKiri >= 180) {
    Serial.println("Shift Right");
    legs::shift6_right_high();
  } else if (TOFKiri <= 180 && TOFSKiri <= 160) {
    Serial.println("Rotate Right");
    legs::rotate6_right_high_1cm();
  } else if (TOFKiri >= 200 && TOFSKiri < 280 || TOFKiri > 290) {
    Serial.println("Shift Left");
    legs::shift6_left_high();
  } else if (TOFKiri <= 180 && TOFSKiri >= 280 || TOFSKiri > 330) {
    Serial.println("Rotate Left");
    legs::rotate6_left_high_1cm();
  } else {
    Serial.println("Forward6");
    legs::forward6_high();
  }
}
//int getx(){
//  if(camx<=bataskanan && camx>=bataskiri){
//    legs::forward_low();
//    Serial.println("forward");
//  } else {
//    if(camx<bataskiri){
//      legs::rotate_right_low();
//      Serial.println("rotate right");
//    }
//  } else {
//    if(camx>bataskanan){
//      legs::rotate_left_low();
//      Serial.print("rotate left");
//    }
//  }
//}

//void gethuman_low(int nhuman) {
//  x = cam::camx;
//  Serial.println(String()+"X  :"+x);
//  Serial.println(String()+"Capit : "+TOF::getcapit());
//  legs::walkspeed = 150;
////  legs::CapitTurun();
//  if (posisi[nhuman] == false) {
//    if (TOF::getcapit() <= 65){
//      delay(500);
//      legs::point_backward_state();
//      delay(500);
//      legs::point_forward_state();
//      posisi[nhuman] = true;
//    }
//    else{
//      if(x <= bataskanan && x >= bataskiri){
////        legs::forward_low_1cm();
////        Serial.println("forward");
//      } else if (x < bataskiri){
//        legs::shift_left_low();
//        Serial.println("shift left");
//      } else if (x > bataskanan) {
//        legs::shift_right_low();
//        Serial.println("shift right");
////      } else if (x > 366 ) {
////        legs::rotate_right_low();
////        Serial.println("rotate right");
////      } else if (x < 265 ) {
////        legs::rotate_left_low();
////        Serial.println("rotate left");
//      }
//    }
//  }
//  if (posisi[nhuman] == true){
//      delay(250);
//      legs::CapitJepit();
//      delay(250);
//      legs::CapitNaik();
//      human[nhuman] = true;
//  }
//}

// INI ASLIIIIII
// void gethuman_low(int nhuman) {
//   x = cam::camx;
//   int depan = TOF::getdepan();
//   //  Serial.println(String()+"X  :"+x);
//   //  Serial.println(String()+"Capit : "+TOF::getcapit());
//   //  legs::CapitTurun();
//   if (posisi[nhuman] == false) {
//     if (tengah == false) {
// //      tengah = true;
//       legs::walkspeed = 120;
//       if (x <= center + camoffset && x >= center - camoffset) {
//         tengah = true;
//       } else if (x < center - camoffset) {
//         legs::shift_left_low_1cm();
//         Serial.println("shift left");
//       } else if (x > center + camoffset) {
//         legs::shift_right_low_1cm();
//         Serial.println("shift right");
//       }
//     }
//     if (tengah == true) {
//       if (depan < 245) {
//         posisi[nhuman] = true;
//       } else {
//         legs::walkspeed = 200;
//         legs::forward_low();
//       }
//     }
//   }
//   if (posisi[nhuman] == true) {
//     delay(250);
//     legs::capitbuka();
//     legs::point_backward_state();
//     delay(500);
//     legs::capitturun2(1);
//     delay(200);
//     legs::point_forward_state();
//     delay(250);
//     legs::capitjepit();
//     legs::point_backward_state();
//     delay(200);
//     legs::capitnaik(2);
//     human[nhuman] = true;
//     tengah = false;
//   }
// }

void gethuman_low(int nhuman, int targetHeading) {
  x = cam::camx;
  int depan = TOF::getdepan();

  if (posisi[nhuman] == false) {
    if (tengah == false) {
      legs::walkspeed = 120;

      int headingCheck = radius(targetHeading);  // pakai default type=F, coffset=15

      if (headingCheck != 0) {
        // heading belum pas, rotate dulu sebelum cek kamera
        if (headingCheck == 1) {
          legs::rotate_right_low_1cm();
          Serial.println("rotate right (heading)");
        } else {
          legs::rotate_left_low_1cm();
          Serial.println("rotate left (heading)");
        }
      } else {
        // heading sudah pas, baru cek centering kamera
        if (x <= center + camoffset && x >= center - camoffset) {
          tengah = true;
        } else if (x < center - camoffset) {
          legs::shift_left_low_1cm();
          Serial.println("shift left");
        } else if (x > center + camoffset) {
          legs::shift_right_low_1cm();
          Serial.println("shift right");
        }
      }
    }
    if (tengah == true) {
      if (depan < 245) {
        posisi[nhuman] = true;
      } else {
        legs::walkspeed = 200;
        legs::forward_low();
      }
    }
  }
  if (posisi[nhuman] == true) {
    delay(250);
    legs::capitbuka();
    legs::point_backward_state();
    delay(500);
    legs::capitturun2(1);
    delay(200);
    legs::point_forward_state();
    delay(250);
    legs::capitjepit();
    legs::point_backward_state();
    delay(200);
    legs::capitnaik(2);
    human[nhuman] = true;
    tengah = false;
  }
}

//void gethuman_low(int nhuman) {
//  x = cam::camx;
////  Serial.println(String()+"X  :"+x);
////  Serial.println(String()+"Capit : "+TOF::getcapit());
//  legs::walkspeed = 200;
////  legs::CapitTurun();
//  if (posisi[nhuman] == false) {
//
//      if(x <= center + camoffset && x >= center - camoffset){
//        if (TOF::getdepan() < 220){
//          posisi[nhuman] = true;
//        } else {
//          legs::forward_low_1cm();
//        }
//      } else if (x < center - camoffset){
//        legs::shift_left_low();
//        Serial.println("shift left");
//      } else if (x > center + camoffset) {
//        legs::shift_right_low();
//        Serial.println("shift right");
//      }
//  }
//  if (posisi[nhuman] == true){
//      delay(250);
//      legs::point_backward_state();
//      delay(250);
//      legs::CapitTurun();
//      delay(250);
//      legs::point_forward_state();
//      delay(250);
//      legs::CapitJepit();
//      delay(250);
//      legs::CapitNaik();
//      human[nhuman] = true;
//      tengah = false;
//  }
//}

void gethuman3(int nhuman) {
  x = cam::camx;
  //  Serial.println(String()+"X  :"+x);
  //  Serial.println(String()+"Capit : "+TOF::getcapit());
  legs::walkspeed = 180;
  //  legs::CapitTurun();
  if (posisi[nhuman] == false) {
    if (x <= center + camoffset && x >= center - camoffset) {
      if (TOF::getdepan() < 220) {
        posisi[nhuman] = true;
      } else {
        legs::forward_1cm();
      }
    } else if (x < center - camoffset) {
      legs::shift_left_1cm();
      Serial.println("shift left");
    } else if (x > center + camoffset) {
      legs::shift_right_1cm();
      Serial.println("shift right");
    }
  }
  if (posisi[nhuman] == true) {
    delay(250);
    legs::point_backward_state();
    delay(250);
    legs::CapitTurun();
    delay(250);
    legs::point_forward_state();
    delay(250);
    legs::CapitJepit();
    delay(250);
    legs::CapitNaik();
    human[nhuman] = true;
    tengah = false;
  }
}

void gethuman5(int nhuman) {
  x = cam::camx;
  //  Serial.println(String()+"X  :"+x);
  //  Serial.println(String()+"Capit : "+TOF::getcapit());
  legs::walkspeed = 500;
  //  legs::CapitTurun();
  if (posisi[nhuman] == false) {
    if (tengah == false) {
      if (x <= center + camoffset && x >= center - camoffset) {
        tengah = true;
      } else if (x < center - camoffset) {
        legs::point_rotate_left();
        Serial.println("shift left");
      } else if (x > center + camoffset) {
        legs::point_rotate_right();
        Serial.println("shift right");
      }
    }
    if (tengah == true) {
      if (TOF::getdepan() <= 220) {
        // Memulai penghitungan waktu jika belum dimulai
        if (camtime == 0) {
          camtime = millis();
        }
        // Memeriksa apakah telah 2 detik
        if (millis() - camtime >= 2000) {
          posisi[nhuman] = true;
        }
      } else {
        // Mengatur ulang waktu mulai jika jarak lebih besar dari 20 cm
        camtime = 0;
      }
    }
  }
  if (posisi[nhuman] == true) {
    //      delay(250);
    //      legs::point_backward_state();
    delay(400);
    legs::CapitTurun3();
    //      delay(200);
    //      legs::point_forward_state();
    delay(400);
    legs::CapitJepit();
    //      delay(200);
    //      legs::point_backward_state();
    delay(400);
    legs::CapitNaik();
    human[nhuman] = true;
    tengah = false;
  }
}

void gethuman2(int nhuman) {
  x = cam::camx;
  int depan = TOF::getdepan();
  //  Serial.println(String()+"X  :"+x);
  //  Serial.println(String()+"Capit : "+TOF::getcapit());
  //  legs::CapitTurun();
  if (posisi[nhuman] == false) {
    if (tengah == false) {
      legs::walkspeed = 400;
      if (x <= center + camoffset && x >= center - camoffset) {
        tengah = true;
      } else if (x > 465) {
        legs::rotate_right_1cm();
        Serial.println("rotate left");
      } else if (x < 165) {
        legs::rotate_left_1cm();
        Serial.println("rotate right");
      } else if (x < 465 && x < center - camoffset) {
        legs::shift_left_1cm();
        Serial.println("shift left");
      } else if (x > 165 && x > center + camoffset) {
        legs::shift_right_1cm();
        Serial.println("shift right");
      }
    }
    if (tengah == true) {
      if (depan < 220) {
        posisi[nhuman] = true;
      } else {
        legs::walkspeed = 400;
        legs::forward();
      }
    }
  }
  if (posisi[nhuman] == true) {
    delay(250);
    legs::capitbuka();
    legs::point_backward_state();
    delay(250);
    legs::capitturun(3);
    delay(200);
    legs::capitturun(4);
    legs::point_forward_state();
    delay(250);
    legs::capitjepit();
    legs::point_backward_state();
    delay(250);
    legs::CapitNaik();
    human[nhuman] = true;
    tengah = false;
  }
}

void gethuman4(int nhuman) {
  x = cam::camx;
  int depan;
  //  Serial.println(String()+"X  :"+x);
  //  Serial.println(String()+"Capit : "+TOF::getcapit());
  legs::walkspeed = 200;
  //  legs::CapitTurun();
  if (posisi[nhuman] == false) {
    if (tengah == false) {
      if (x <= center + camoffset && x >= center - camoffset) {
        tengah = true;
      } else if (x < center - camoffset) {
        legs::rotate6_left_slow();
        Serial.println("shift left");
      } else if (x > center + camoffset) {
        legs::rotate6_right_slow();
        Serial.println("shift right");
      }
    }
    if (tengah == true) {
//      if (x < center - camoffset){
//       depan = 175;
//      } else {
//        depan = 200;
//      }
      if (TOF::getdepan() < 210) {
        posisi[nhuman] = true;
      } else {
        legs::forward6_1cm();
      }
    }
  }
  if (posisi[nhuman] == true) {
    delay(250);
    legs::point_backward_state();
    delay(200);
    legs::capitturun2(1);
    delay(200);
    legs::capitbuka();
    delay(300);
//    legs::walkspeed = 250;
    legs::point_forward_state();
//    legs::capitbuka();
    delay(250);
//    while (TOF::getcapit() > 50){
//      legs::forward6_1cm();
//    }
    legs::CapitJepit();
    delay(200);
    legs::point_backward_state();
    delay(200);
    legs::CapitNaik();
    human[nhuman] = true;
    tengah = false;
  }
}


// if(millis()-waktu <= 3000){
// legs::forward();
//   Serial.println("Titik 0");
//   // legs::CapitNaik();
// }

// int buttonValue = digitalRead(button);
// if (buttonValue == LOW && buttonstate == false) {
//   buttonstate = true;
// }
// if (buttonValue == HIGH && buttonstate == true) {
//   int loop = 0;
//   if (Activation == false) {
//     Activation = true;
//     point0state = false;
//     // lcd.clear();
//     // LCDPrint(0, 0, "Home");
//     while (loop < 20) {
//       legs::forward_low();
//       Serial.println("forward while");
//       loop = loop + 1;
//     }
//   } else if (Activation == true) {
//     Activation = false;
//     point0state = true;
//   }
//   buttonstate = false;
//   delay(100);
// }

// if(Activation == true && point0state == false){
//   if (homestate == false && overallhome == false){
//     if (compass::heading() <= cposhomeL + coffset && compass::heading() >= cposhomeL - coffset) {
//       gettimehome = millis();
//       while (millis() - gettimehome <= 2500) {
//         legs::walkspeed = 100;
//         legs::forward_low();
//       }
//       homestate = true;
//       overallhome = true;
//     } else {
//       Serial.println("Rotate Left || Initializing Home State");
//       legs::walkspeed = 100;
//       legs::rotate_left_low();
//     }
//   }
// }
// if(camx>bataskiri && camx<bataskanan) {
//
//    legs::forward();
//    Serial.println("Forward");
// } else if(camx < bataskiri){
//    legs::rotate_right();
//    Serial.println("Rotate Right");
// } else if (camx > bataskanan){
//    legs::rotate_left();
//    Serial.println("Rotate Left");
// }



//WallFollowingLeft();
//legs::forward();
//max kiri korban 145
//max kanan korban 188

//   pixy.ccc.getBlocks();
//   // grab blocks!
//   Serial.print("Value X : "+String(getx()));
//   Serial.println("|||Value Y : "+String(gety()));
// if(Capit==false){

// if(turun==false){
//   legs::CapitTurun();
//   delay(200);
// turun=true;

// }
// if(getx()<bataskiri && gety()<depan){

//   Serial.println("Rotate Kiri");
//   legs::movetype("rotate_left");
//   }

// if(getx()>bataskanan & gety()<depan){

//   Serial.println("Rotate Kanan");
//   legs::movetype("rotate_right");
//   // legs::CapitTurun();


//   }
// if(getx()>=bataskiri && getx()<=bataskanan && gety()>=depan){
// Serial.println("Capit");
//   legs::CapitNaik();
// Capit =true;
// delay(1000);
// }
// if(getx()>=bataskiri && getx()<=bataskanan && gety()<depan){
//   Serial.println("Forward");
// legs::movetype("forward");
//   // legs::CapitTurun();
// }
// }
// if(Capit==true){
//   Serial.println("Maju Keluar Loop");
// legs::movetype("point0");}

// delay(500);

// If there are detect blocks, print them!
// if (pixy.ccc.numBlocks)
// {
//   Serial.print("Detected ");
//   Serial.println(pixy.ccc.numBlocks);

//     Serial.print("  block ");
//     Serial.print(0);
//     Serial.print(": ");
//     pixy.ccc.blocks[0].print();

// }



// Serial.println("Sensor Depan : "+ String(TOF::getdepan()) + "||Sensor Samping : "+ String(TOF::getsamping())+ "||Sensor Serong : "+ String(TOF::getserong()));
// // // Serial.println("Sensor Depan : "+ String(TOF::getdepan()) + "||Sensor Samping : "+ String(TOF::getsamping()));

// if(millis()-waktu <=5000){legs::movetype("point0");Serial.println("Titik 0");legs::CapitNaik();}

// if(millis()-waktu > 5000 && millis()<=10000){legs::movetype("forward");}
// // if(millis()>10000){Home=true;}


// int buttonValue = digitalRead(button);
// Serial.println(buttonValue);
//  if (buttonValue == LOW && buttonstate==false){
//       buttonstate=true;
//    }
//   if(buttonValue==HIGH && buttonstate==true){
//     int loop=0;
//     if(Home==false){Home=true; point0state=false;while(loop<20){legs::movetype("forward");Serial.println("forward while");loop=loop+1;} }
//     else if(Home==true){Home=false; point0state=true;}
//     buttonstate=false;
//     delay(100);
//   }

// if(point0state==true && Home==false){
//   legs::movetype("point0");
//   // Serial.println("Saat ini Point 0");
// }
// if(Home==true && point0state==false){

// if(TOF::getdepan()>=5 && TOF::getsamping()<= 20 && TOF::getserong()>=20){
//   Serial.println("Shift Left");
//   legs::movetype("shift_left");
// }
// else if(TOF::getdepan()>=5 && TOF::getsamping()<=20 && TOF::getserong()<=18){
//   Serial.println("Rotate left");
//   legs::movetype("rotate_left");
// }
// else if(TOF::getdepan()>=5 && TOF::getsamping()>= 24 && TOF::getserong()<39){
//   Serial.println("Shift Right");
//   legs::movetype("shift_right");
// }
// else if(TOF::getdepan()>=5 && TOF::getsamping()>=25 && TOF::getserong()>=39){
//   Serial.println("Rotate right");
//   legs::movetype("rotate_right");
// }
// else{
//Serial.println("shift_left");
// legs::movetype("point0");
// Serial.println("forward");
// }

// }
// Wall Following Right



// bool val = digitalRead(Switch);

// legs::movetype("CapitTurun");
// legs::movetype("point0");
// if(val==true && state==false){
//   state=true;
// }
// if(val ==false && state ==true){
//   state=false;
//   move=move+1;
//   if(move==3){
//     move=0;
//   }
//   Serial.println("move : " + String(move));
// }

// if(move==0){
// Serial.println("point0");
// legs::movetype("point0");
// }

// if(move==1){
// Serial.println("forward");
// legs::movetype("forward");
// }

// if(move==2){
//   Serial.println("backward");
//  legs::movetype("backward");
// }



//  if(compass::getZ()>=-42 && compass::getZ() <=-37){
//   legs::movetype("forward");
//   Serial.println("forward");
// }
// if(compass::getZ()<-42){
//   legs::movetype("rotate_right");
//     Serial.println("rotate_right");
// }
// if(compass::getZ()>-37){
//   legs::movetype("rotate_left");
//   Serial.println("rotate_left");

// }
// legs::testkoordinat(2);
// legs::movetype("point0");


// legs::movetype("point0");
//legs::movetype("rotate_left");
// legs::movetype("rotate_right");
// legs::movetype("shift_right");
//legs::fungsimaju();


// // Wall Following Right
// void WallFollowingRight(bool tinggi) {
//  int TOFKanan = TOF::getsampingkanan();
//   int TOFSKanan = TOF::getserongkanan();

//   if (tinggi == true) {
//     if (TOFKanan <= 16 && TOFSKanan >= 20) {
//       Serial.println("Shift Left");
//       legs::shift_left_step();
//     } else if (TOFKanan <= 20 && TOFSKanan <= 18) {
//       Serial.println("Rotate left");
//       legs::rotate_left_step();
//     } else if (TOFKanan >= 22 && TOFSKanan < 30 || TOFKanan > 31) {
//       Serial.println("Shift Right");
//       legs::shift_right_step();
//     } else if (TOFKanan <= 20 && TOFSKanan >= 30 || TOFSKanan > 35) {
//       Serial.println("Rotate right");
//       legs::rotate_right_step();
//     } else {
//       Serial.println("forward6");
//       legs::forward_step();
//     }
//   }
//   if (tinggi == false) {
//     if (TOF::getdepan() >= 5 && TOF::getsampingkanan() <= 20 && TOF::getserongkanan() >= 20) {
//       Serial.println("Shift Left");
//       legs::shift_left_low_step();
//     } else if (TOF::getdepan() >= 5 && TOF::getsampingkanan() <= 20 && TOF::getserongkanan() <= 18) {
//       Serial.println("Rotate left");
//       legs::rotate_left_low_step();
//     } else if (TOF::getdepan() >= 5 && TOF::getsampingkanan() >= 24 && TOF::getserongkanan() < 39) {
//       Serial.println("Shift Right");
//       legs::shift_right_low_step();
//     } else if (TOF::getdepan() >= 5 && TOF::getsampingkanan() >= 25 && TOF::getserongkanan() >= 39) {
//       Serial.println("Rotate right");
//       legs::rotate_right_low_step();
//     } else {
//       Serial.println("forward");
//       legs::forward_low_step();
//     }
//   }
// }

// void WallFollowingRight6() {
//   float TOFKanan = TOF::getsampingkanan();
//   float TOFSKanan = TOF::getserongkanan();

//   if (TOFKanan <= 16 && TOFSKanan >= 20) {
//     Serial.println("Shift Left");
//     legs::shift6_left();
//   } else if (TOFKanan <= 20 && TOFSKanan <= 18) {
//     Serial.println("Rotate left");
//     legs::rotate6_left();
//   } else if (TOFKanan >= 22 && TOFSKanan < 30 || TOFKanan > 31) {
//     Serial.println("Shift Right");
//     legs::shift6_right();
//   } else if (TOFKanan <= 20 && TOFSKanan >= 30 || TOFSKanan > 35) {
//     Serial.println("Rotate right");
//     legs::rotate6_right();
//   } else {
//     Serial.println("forward6");
//     legs::forward6();
//   }
// }

// //Wall Following Left
// void WallFollowingLeft() {
//   if (TOF::getdepan() >= 5 && TOF::getsampingkiri() <= 20 && TOF::getserongkiri() >= 20) {
//     Serial.println("shift Right");
//     legs::shift_right();
//   } else if (TOF::getdepan() >= 5 && TOF::getsampingkiri() <= 20 && TOF::getserongkiri() <= 18) {
//     Serial.println("rotate Right");
//     legs::rotate_right();
//   } else if (TOF::getdepan() >= 5 && TOF::getsampingkiri() >= 24 && TOF::getserongkiri() < 39) {
//     Serial.println("shift Left");
//     legs::shift_left();
//   } else if (TOF::getdepan() >= 5 && TOF::getsampingkiri() >= 25 && TOF::getserongkiri() >= 39) {
//     Serial.println("rotate Left");
//     legs::rotate_left();
//   } else {
//     Serial.println("forward");
//     legs::forward();
//   }
// }

// void WallFollowingLeft6() {
//   int TOFKiri = TOF::getsampingkiri();
//   int TOFSKiri = TOF::getserongkiri();

//   if (TOFKiri <= 16 && TOFSKiri >= 20) {
//     Serial.println("Shift Right");
//     legs::shift6_right();
//   } else if (TOFKiri <= 20 && TOFSKiri <= 18) {
//     Serial.println("Rotate Right");
//     legs::rotate6_right();
//   } else if (TOFKiri >= 22 && TOFSKiri < 30 || TOFKiri > 31) {
//     Serial.println("Shift Left");
//     legs::shift6_left();
//   } else if (TOFKiri <= 20 && TOFSKiri >= 30 || TOFSKiri > 35) {
//     Serial.println("Rotate Left");
//     legs::rotate6_left();
//   } else {
//     Serial.println("Forward6");
//     legs::forward6();
//   }
