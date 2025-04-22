#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

namespace getmpu{

// PID constants for roll and pitch
float Kp = 5.0;
float Ki = 0.4;
float Kd = 0.1;
float Proll =0;
float Iroll=0;
float Droll=0;
float Ppitch =0;
float Ipitch=0;
float Dpitch=0;

// Setpoints for roll and pitch
float rollSetpoint = 0.0;
float pitchSetpoint = 0.0;
float lastErrorRoll=0;
float lastErrorPitch=0;

void setup() {
  Serial.begin(38400);
  Wire.begin();
  mpu.initialize();
}

float get_pitch_PID(){
 // Read MPU sensor values
  float accX = mpu.getAccelerationX() / 16384.0;
  float accY = mpu.getAccelerationY() / 16384.0;
  float accZ = mpu.getAccelerationZ() / 16384.0;
  


  // Calculate roll and pitch angles in degrees
  float roll = atan2(accY, accZ) * 180 / M_PI;
  float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / M_PI;

  // Calculate errors for roll and pitch
  float errorRoll = rollSetpoint - roll;
  float errorPitch = pitchSetpoint - pitch;

  // Calculate PID output for roll
  double PitchValue;
  errorRoll = rollSetpoint - roll;
  errorPitch = pitchSetpoint - pitch;

  Ppitch = errorPitch;
  Ipitch = Ipitch + errorPitch;
  Dpitch = errorPitch-lastErrorPitch;
  lastErrorPitch = errorPitch;

  PitchValue = (Ppitch*Kp + Ipitch*Ki + Dpitch*Kd)/180;

  

  if(PitchValue > 3){PitchValue=3;Ipitch=1000;}
  if(PitchValue < -3){PitchValue=-3;Ipitch=-1000;}
 
  Serial.println("P :"+String(Ppitch)+" I :"+String(Ipitch)+" D :"+String(Dpitch)+" PID Value : "+ String(PitchValue));

  return PitchValue;
}

float get_roll_PID(){
  // Read MPU sensor values
  float accX = mpu.getAccelerationX() / 16384.0;
  float accY = mpu.getAccelerationY() / 16384.0;
  float accZ = mpu.getAccelerationZ() / 16384.0;
  


  // Calculate roll and pitch angles in degrees
  float roll = atan2(accY, accZ) * 180 / M_PI;
  float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / M_PI;

  // Calculate errors for roll and pitch
  float errorRoll = rollSetpoint - roll;
  float errorPitch = pitchSetpoint - pitch;

  // Calculate PID output for roll
  double RollValue;
  errorRoll = rollSetpoint - roll;
  errorPitch = pitchSetpoint - pitch;

  Proll = errorRoll;
  Iroll = Iroll + errorRoll;
  Droll = errorRoll-lastErrorRoll;
  lastErrorRoll = errorRoll;

  RollValue = (Proll*Kp + Iroll*Ki + Droll*Kd)/180;

  

  if(RollValue > 3){RollValue=3;Iroll=1000;}
  if(RollValue < -3){RollValue=-3;Iroll=-1000;}
 

  return RollValue;
}



}
