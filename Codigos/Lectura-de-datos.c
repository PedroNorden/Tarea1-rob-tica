// Sensor infrasonido
const int trigPin = 9;
const int echoPin = 10;
const int ledPin = 13;

#include "Adafruit_TCS34725.h"

const int umbral = 10; // cm

// Sensor RGB
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  if (tcs.begin()) {
    Serial.println("Sensor encontrado");
  } else {
    Serial.println("No se encontro el sensor");
    while (1); // Esperar
  }
   
  #if defined(ARDUINO_ARCH_ESP32)
    ledcAttach(redpin, 12000, 8);
    ledcAttach(greenpin, 12000, 8);
    ledcAttach(bluepin, 12000, 8);
  #else
    pinMode(redpin, OUTPUT);
    pinMode(greenpin, OUTPUT);
    pinMode(bluepin, OUTPUT);
  #endif
  
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }  
}


void loop() {
  float red, green, blue;
  
  tcs.setInterrupt(false);  // encender LED
  delay(60);  // espera para lectura
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // apagar LED
  
  // Emitir pulso
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Medir duración del eco
  long duracion = pulseIn(echoPin, HIGH);
  float distancia = duracion * 0.034 / 2;

  // Imprimir valores formateados
  Serial.println(F("*******************************"));
  Serial.print(F("R: ")); Serial.print(int(red)); 
  Serial.print(F("\tG: ")); Serial.print(int(green)); 
  Serial.print(F("\tB: ")); Serial.println(int(blue));

  Serial.print(F("Distancia: ")); 
  Serial.print(distancia, 2); // dos decimales
  Serial.println(F(" cm"));

  if (distancia <= umbral) {
    Serial.println(F(">> ¡Obstáculo detectado! <<"));
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  Serial.println(F("*******************************\n"));

  delay(200);
}
