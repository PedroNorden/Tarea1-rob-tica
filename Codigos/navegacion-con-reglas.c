#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Definir led comun anodo
#define commonAnode true

// Pines sensor ultrasónico
const int trigPin = 9;
const int echoPin = 10;
const int ledPin = 13;

// Pines motor A (izquierdo)
const int IN1 = 2;
const int IN2 = 3;
const int ENA = 11;  // PWM

// Pines motor B (derecho)
const int IN3 = 5;
const int IN4 = 4;
const int ENB = 6;   // PWM

// Pines y declaración del servo motor 
Servo servoMotor;
const int pinServo = 9;

// Pines, objetos y declaraciones para sensor RGB
const int redPin = 3;
const int bluePin = 5;
const int greenPin = 6;
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);


// Variables de control
const int umbral = 10;  // cm
const int velA = 120; // velocidad motor A 
const int velB = 155; // velocidad motor B 

void setup() {
  Serial.begin(9600);

  // Configurar sensor ultrasónico
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // Configurar motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Configurar servo
  servoMotor.attach(pinServo);

  // Configurar RGB
  if (tcs.begin()) {
    Serial.println("Se encontro TCS34725");
  }

#if defined(ARDUINO_ARCH_ESP32)
  ledcAttach(redpin, 12000, 8);
  ledcAttach(greenpin, 12000, 8);
  ledcAttach(bluepin, 12000, 8);
#else
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
#endif

  // Generar tabla gamma (RGB)
  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
    gammatable[i] = commonAnode ? 255 - x : x;
  }

  // 2 segundos antes de partir el sistema
  delay(2000); // ELIMINAR DESPUES
}

void loop() {

  // Medicion de distancia frontal
  float distanciaAdelante = mirarAdelante();
  Serial.print("Distancia adelante: ");
  Serial.print(distanciaAdelante);
  Serial.println(" cm");
  

  if (distanciaAdelante <= umbral) {
    // Detectamos un obstaculo
    detener();
    digitalWrite(ledPin, HIGH);
    Serial.println("Obstáculo detectado. Analizando ruta...");
    
    // Leemos la informacion del sensor RGB
    float red, green, blue;
    obtenerColorPromediado(&red, &green, &blue);
    actualizarLED(red, green, blue);
    mostrarColorSerial(red, green, blue);
    detectarColorDominante(red, green, blue);

    // Dependiendo del color realizar accion
    accionPorColor(red, green, blue);
  }
  else {
    avanzar();
    digitalWrite(ledPin, LOW);
  }
  
  delay(200);
}

// Mide la distancia con sensor infra-sonido
float medirDistancia() {
  // Enviar pulso ultrasónico
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Leer duración del eco
  long duracion = pulseIn(echoPin, HIGH);

  // Convertir a distancia en cm
  float distancia = duracion * 0.034 / 2;

  return distancia;
}


// Gira las ruedas del robot hacia adelante
void avanzar() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velA+30);

  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(ENB, velB+20);
}

// Detiene al robot por 3s
void detener() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);

  delay(3000);
}

// Gira el robot hacia la izquierda de manera pivoteada
void doblarIzquierda() {
  detener(); 

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); // Motor A atrás
  analogWrite(ENA, velA+80);

  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);  // Motor B adelante
  analogWrite(ENB, velB+80);

  delay(500);
  detener();
}

// Gira el robot hacia la derecha de manera pivoteada
void doblarDerecha() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velA+80);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, velB+80);

  delay(500); 
  detener();
}

// Detenerse pero con mas tiempo
void detenerseDefinitivo() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 0);

  delay(10000); 
}

// Gira el servo hacia al frente
float mirarAdelante() {
  servoMotor.write(110);
  delay(300);
  return medirDistancia();
}


// Actualiza la salida en los pines de los LEDs
void actualizarLED(float red, float green, float blue) {
#if defined(ARDUINO_ARCH_ESP32)
  ledcWrite(1, gammatable[(int)red]);
  ledcWrite(2, gammatable[(int)green]);
  ledcWrite(3, gammatable[(int)blue]);
#else
  analogWrite(redPin, gammatable[(int)red]);
  analogWrite(greenPin, gammatable[(int)green]);
  analogWrite(bluePin, gammatable[(int)blue]);
#endif
}

// Muestra los valores RGB por Serial
void mostrarColorSerial(float red, float green, float blue) {
  Serial.print("R:\t"); Serial.print(int(red));
  Serial.print("\tG:\t"); Serial.print(int(green));
  Serial.print("\tB:\t"); Serial.println(int(blue));
}

// Determina qué color es predominante
void detectarColorDominante(float red, float green, float blue) {
  if (red > green && red > blue) {
    Serial.println("Color detectado: Rojo");
  } else if (green > red && green > blue) {
    Serial.println("Color detectado: Verde");
  } else if (blue > red && blue > green) {
    Serial.println("Color detectado: Azul");
  } else {
    Serial.println("Color indeterminado o mezcla similar");
  }
}

// Obtiene valores RGB normalizados por Clear
void obtenerColorPromediado(float* red, float* green, float* blue) {
  float sumR = 0, sumG = 0, sumB = 0;
  uint16_t r, g, b, c;

  for (int i = 0; i < 5; i++) {
    tcs.setInterrupt(false);
    delay(60);
    tcs.getRawData(&r, &g, &b, &c);
    tcs.setInterrupt(true);

    if (c == 0) c = 1;

    float rNorm = ((float)r / c) * 255.0;
    float gNorm = ((float)g / c) * 255.0;
    float bNorm = ((float)b / c) * 255.0;

    sumR += rNorm;
    sumG += gNorm;
    sumB += bNorm;
  }

  float redPromedio = sumR / 5.0;
  float greenPromedio = sumG / 5.0;
  float bluePromedio = sumB / 5.0;

  *red = redPromedio ;
  *green = greenPromedio;
  *blue = bluePromedio ;
}

// Realizar las distintas acciones en base al color
void accionPorColor(float red, float green, float blue) {
  if (red > green && red > blue) {
    Serial.println("Color Rojo detectado: Detenerse por 10s.");
    detenerseDefinitivo();
  } 
  else if (green > red && green > blue) {
    Serial.println("Color Verde detectado: Doblar izquierda.");
    doblarIzquierda();
  } 
  else if (blue > red && blue > green) {
    Serial.println("Color Azul detectado: Doblar derecha.");
    doblarDerecha();
  }
  else {
    Serial.println("Color indeterminado: Retroceder.");
  }
}


