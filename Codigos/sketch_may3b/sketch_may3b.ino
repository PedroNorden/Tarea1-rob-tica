#include <L298NX2.h>

const int IN1 =2,IN2=3,IN3=4,IN4=5,ENA=6,ENB=7; 


void setup() {
  pinMode(IN1, OUTPUT); // IN1
  pinMode(IN2, OUTPUT); // IN2
  pinMode(ENA, OUTPUT); // ENA

  pinMode(IN3, OUTPUT); // IN1
  pinMode(IN4, OUTPUT); // IN2
  pinMode(ENB, OUTPUT);

}

void forward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB,200);
  analogWrite(ENA,200);

}

void backward()
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB,200);
  analogWrite(ENA,200);

}

void turn()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB,200);
  analogWrite(ENA,200);
  delay(500);
  stop();
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB,200);
  analogWrite(ENA,200);
  delay(1000);
  stop();

}

void stop() {
  // Apagar motores
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Opcional: cortar PWM
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {

  forward();
  delay(200);
  stop();
  delay(500);


  backward();
  delay(200);
  stop();
  delay(1000);
  turn();
  delay(2000);
  stop();
  delay(2000);

}