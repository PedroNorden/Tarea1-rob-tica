#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <SimpleKalmanFilter.h>

MPU9250_asukiaaa mpu;
SimpleKalmanFilter kalmanPitch(2, 2, 0.01);


unsigned long lastTime;
float pitch = 0;

//Valores acc y gyro


const int ruedaD=0,ruedaL=0;



void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  delay(1000);

  lastTime=millis();
}



void loop() {
  mpu.gyroUpdate();
  mpu.accelUpdate();
  

  float accPitch= atan2(mpu.accelY(), sqrt(mpu.accelX()*mpu.accelX()+mpu.accelZ()*mpu.accelZ() )) *180/PI;

  float dt = (millis() - lastTime) / 1000.0;
  pitch += (mpu.gyroX() * 180 / PI) * dt;
  lastTime = millis();


  float pitchKalman = kalmanPitch.updateEstimate(accPitch);

  
  if (abs(pitchKalman) > 10)
{
  Serial.print(" AccPitch:");
  Serial.print(pitchKalman);
  Serial.println("   Esta volcadisimo");
} 
else 
{
  Serial.print(" AccPitch:");
  Serial.print(pitchKalman);
  Serial.println("  Esta estable");
}
  delay(50);
}