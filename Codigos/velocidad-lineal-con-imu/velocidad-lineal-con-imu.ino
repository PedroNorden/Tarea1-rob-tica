#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <SimpleKalmanFilter.h>

const float L = 13.4;
const float R = 6.9 / 2;

const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 5;
const int IN4 = 4;
const int ENA = 9;
const int ENB = 6;

float velAngular = 0;
float thetaIMU = 0;
float acceleration[3] = {0, 0, 0};
float rawAccel[3] = {0, 0, 0};
float position[3] = {0, 0, 0};
float velocity[3] = {0, 0, 0};

float positionEstimated[2] = {0,0};


float dt = 0;
float setpoint = 0.0;

const int PWMbase = 100;
const float w = 4.9*PI;

unsigned long lastTime;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000; // ms

const float alpha = 0.9; // Constante del filtro
const float Kp = 2.5;

MPU9250_asukiaaa mpuSensor;

SimpleKalmanFilter kalmanX(0.5, 1,0.01);
SimpleKalmanFilter kalmanY(0.5, 1, 0.01);
SimpleKalmanFilter kalmanZ(0.5,1.5, 0.01);
SimpleKalmanFilter kalmanAngular(1,2,0.01);


void setup() {
  Serial.begin(19200);
  Wire.begin();

  mpuSensor.setWire(&Wire);
  mpuSensor.beginAccel();
  mpuSensor.beginGyro();

  delay(2000);
  mpuSensor.accelUpdate();
  mpuSensor.gyroUpdate();
  lastTime = millis();
  lastPrintTime = millis();

  //Motores

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);


  moverSetup();
}

void deltatime() {
  unsigned long now = millis();
  dt = float((now - lastTime) / 1000.0);
  lastTime = now;
}

void calcMPU() {
  mpuSensor.accelUpdate();

  // Filtrar las aceleraciones con Kalman
  rawAccel[0] = kalmanZ.updateEstimate(mpuSensor.accelZ() * 9.81 );
  rawAccel[1] = kalmanY.updateEstimate(mpuSensor.accelY() * 9.81-.58 );
  rawAccel[2] = kalmanX.updateEstimate(mpuSensor.accelX() * 9.81 + 10.0);

  // Guardamos el resultado filtrado directamente como aceleración
  for (int i = 0; i < 3; i++) {
    if (abs(rawAccel[i]) < 0.5) {
      acceleration[i] = 0;
    } else {
      acceleration[i] = rawAccel[i];
    }
  }

  

  // Integrar aceleración → velocidad → posición
  for (int i = 0; i < 3; i++) {
    if (acceleration[i] == 0) {
      velocity[i] = 0;
    } else {
      velocity[i] += acceleration[i] * dt;
    }
    position[i] += velocity[i] * dt;
  }


  mpuSensor.gyroUpdate();

  float rawVelAngular = mpuSensor.gyroX() + 4.64;  // Si el sesgo es negativo, lo sumas
  if (abs(rawVelAngular) < 0.15) rawVelAngular = 0;  // Filtrar ruido pequeño
  velAngular = kalmanAngular.updateEstimate(rawVelAngular);

  thetaIMU += velAngular * dt;  // ° acumulados


}

void estimarPosicion()
{

  float v = dt* (R/2)*(w+w);
  positionEstimated[1] = v*cos(thetaIMU);
  positionEstimated[0] = v*sin(thetaIMU);
}

void moverSetup()
{

  //Rueda izquierda
  digitalWrite(IN2,LOW);
  digitalWrite(IN1,HIGH);
  digitalWrite(ENA,PWMbase);

  //Rueda derecha
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  digitalWrite(ENB,PWMbase);
  delay(1000);
}

void mover()
{
  float error = thetaIMU - setpoint;
  error = constrain(error, -90, 90);
  if (error < 5) return;
  int pwmA = constrain(PWMbase + (Kp * error), 0, 255);
  int pwmB = constrain(PWMbase - (Kp * error), 0, 255);
  
  //Rueda izquierda
  analogWrite(ENA,PWMbase+(thetaIMU*5));

  //Rueda derecha
  analogWrite(ENB,PWMbase-(thetaIMU*5));
  
}

void loop() {

  deltatime();
  calcMPU();
  estimarPosicion();


  mover();
  


  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {

    Serial.print("pos estimada: ");
    Serial.println(positionEstimated[1]);

    Serial.print("Theta IMU: ");
    Serial.println(thetaIMU);
    Serial.print("Vel angular");
    Serial.println(velAngular);
    Serial.print("Aceleracion: ");
    for (int i = 0; i < 3; i++) {
      Serial.print(acceleration[i]); Serial.print(" ");
    }
    Serial.println();

  Serial.print("rawAceleracion: ");
    for (int i = 0; i < 3; i++) {
      Serial.print(rawAccel[i]); Serial.print(" ");
    }
    Serial.println();
    Serial.println("-------------------");
    lastPrintTime = currentTime;
  }
}
/// ultimo XD
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <SimpleKalmanFilter.h>

const float L = 13.4;
const float R = 6.9 / 2;

const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 5;
const int IN4 = 4;
const int ENA = 9;
const int ENB = 6;

float velAngular = 0;
float thetaIMU = 0;
float acceleration[3] = {0, 0, 0};
float rawAccel[3] = {0, 0, 0};
float position[3] = {0, 0, 0};
float velocity[3] = {0, 0, 0};

float positionEstimated[2] = {0,0};


float dt = 0;
float setpoint = 0.0;

const int PWMbase = 100;
const float w = 4.9*PI;

unsigned long lastTime;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000; // ms

const float alpha = 0.9; // Constante del filtro
const float Kp = 2.5;

MPU9250_asukiaaa mpuSensor;

SimpleKalmanFilter kalmanX(0.5, 1,0.01);
SimpleKalmanFilter kalmanY(0.5, 1, 0.01);
SimpleKalmanFilter kalmanZ(0.5,1.5, 0.01);
SimpleKalmanFilter kalmanAngular(1,2,0.01);


void setup() {
  Serial.begin(19200);
  Wire.begin();

  mpuSensor.setWire(&Wire);
  mpuSensor.beginAccel();
  mpuSensor.beginGyro();

  delay(2000);
  mpuSensor.accelUpdate();
  mpuSensor.gyroUpdate();
  lastTime = millis();
  lastPrintTime = millis();

  //Motores

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);


  moverSetup();
}

void deltatime() {
  unsigned long now = millis();
  dt = float((now - lastTime) / 1000.0);
  lastTime = now;
}

void calcMPU() {
  mpuSensor.accelUpdate();

  // Filtrar las aceleraciones con Kalman
  rawAccel[0] = kalmanZ.updateEstimate(mpuSensor.accelZ() * 9.81 );
  rawAccel[1] = kalmanY.updateEstimate(mpuSensor.accelY() * 9.81-.58 );
  rawAccel[2] = kalmanX.updateEstimate(mpuSensor.accelX() * 9.81 + 10.0);

  // Guardamos el resultado filtrado directamente como aceleración
  for (int i = 0; i < 3; i++) {
    if (abs(rawAccel[i]) < 0.5) {
      acceleration[i] = 0;
    } else {
      acceleration[i] = rawAccel[i];
    }
  }

  

  // Integrar aceleración → velocidad → posición
  for (int i = 0; i < 3; i++) {
    if (acceleration[i] == 0) {
      velocity[i] = 0;
    } else {
      velocity[i] += acceleration[i] * dt;
    }
    position[i] += velocity[i] * dt;
  }


  mpuSensor.gyroUpdate();

  float rawVelAngular = mpuSensor.gyroX() + 4.64;  // Si el sesgo es negativo, lo sumas
  if (abs(rawVelAngular) < 0.15) rawVelAngular = 0;  // Filtrar ruido pequeño
  velAngular = kalmanAngular.updateEstimate(rawVelAngular);

  thetaIMU += velAngular * dt;  // ° acumulados


}

void estimarPosicion()
{

  float v = dt* (R/2)*(w+w);
  positionEstimated[1] = v*cos(thetaIMU);
  positionEstimated[0] = v*sin(thetaIMU);
}

void moverSetup()
{

  //Rueda izquierda
  digitalWrite(IN2,LOW);
  digitalWrite(IN1,HIGH);
  digitalWrite(ENA,PWMbase);

  //Rueda derecha
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  digitalWrite(ENB,PWMbase);
  delay(1000);
}

void mover()
{
  float error = thetaIMU - setpoint;
  error = constrain(error, -90, 90);
  if (error < 5) return;
  int pwmA = constrain(PWMbase + (Kp * error), 0, 255);
  int pwmB = constrain(PWMbase - (Kp * error), 0, 255);
  
  //Rueda izquierda
  analogWrite(ENA,PWMbase+(thetaIMU*5));

  //Rueda derecha
  analogWrite(ENB,PWMbase-(thetaIMU*5));
  
}

void loop() {

  deltatime();
  calcMPU();
  estimarPosicion();


  mover();
  


  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {

    Serial.print("pos estimada: ");
    Serial.println(positionEstimated[1]);

    Serial.print("Theta IMU: ");
    Serial.println(thetaIMU);
    Serial.print("Vel angular");
    Serial.println(velAngular);
    Serial.print("Aceleracion: ");
    for (int i = 0; i < 3; i++) {
      Serial.print(acceleration[i]); Serial.print(" ");
    }
    Serial.println();

  Serial.print("rawAceleracion: ");
    for (int i = 0; i < 3; i++) {
      Serial.print(rawAccel[i]); Serial.print(" ");
    }
    Serial.println();
    Serial.println("-------------------");
    lastPrintTime = currentTime;
  }
}
