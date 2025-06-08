#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Crear objeto del sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

void setup() {
  Serial.begin(9600);
  
  if (tcs.begin()) {
    Serial.println("Sensor TCS34725 detectado.");
  } else {
    Serial.println("No se detecta el TCS34725");
    while (1); // Detener si no encuentra sensor
  }
}

void loop() {
  // Iniciar medición de tiempo
  unsigned long tiempoInicio = millis();

  // Leer datos RGB y Clear del sensor
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Normalizar RGB a valores de 0.0 a 1.0
  float r_norm, g_norm, b_norm;
  if (c == 0) c = 1; // evitar división por cero

  r_norm = (float)r / c;
  g_norm = (float)g / c;
  b_norm = (float)b / c;

  // Mostrar valores normalizados
  Serial.print("R: "); Serial.print(r_norm, 3);
  Serial.print(" G: "); Serial.print(g_norm, 3);
  Serial.print(" B: "); Serial.print(b_norm, 3);
  Serial.print(" C: "); Serial.println(c);

  // Clasificar color
  if (r_norm > 0.4 && g_norm < 0.3 && b_norm < 0.3) {
    Serial.println("Rojo");

  } else if (r_norm < 0.3 && g_norm > 0.4 && b_norm < 0.3) {
    Serial.println("Verde");

  } else if (r_norm < 0.3 && g_norm < 0.3 && b_norm > 0.4) {
    Serial.println("Azul");

  } else if (c < 100) {
    Serial.println("Negro");

  } else if (r_norm > 0.3 && g_norm > 0.3 && b_norm > 0.25) {
    Serial.println("Blanco");
  }

  // Medir tiempo de respuesta
  unsigned long tiempoFin = millis();
  unsigned long tiempoRespuesta = tiempoFin - tiempoInicio;

  Serial.print("Tiempo de respuesta: ");
  Serial.print(tiempoRespuesta);
  Serial.println(" ms");

  delay(1000);
}

