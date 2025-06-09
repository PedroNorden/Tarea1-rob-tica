# Tarea2-robótica

Vicente Rosales<br>
Manuel Aguilera<br>
Alex Alfaro<br>
Eliseo Guarda<br>
Pedro Nordenflycht<br>

## Videos demostración

Parte 1: https://youtu.be/qeGlbYjwYgY <br>
Parte 2: https://youtube.com/shorts/9f7LtR_2HHI?feature=share

## Preguntas parte 1:

**¿Qué es la percepción en robótica y por qué es fundamental en los sistemas autónomos?**

La percepción se refiere a la capacidad del robot para recolectar información de su entorno por medio de sensores, emulando los sentidos del ser humano. Esto es fundamental en sistemas autónomos ya que le permite orientarse, navegar espacios desconocidos con obstáculos, entre otros. Sin la percepción, el robot no puede tomar decisiones basadas en su entorno, perdiendo su autonomía.

**En el sensor ultrasónico HC-SR04 ¿Qué parámetro se mide para calcular la distancia?**

El parámetro medido es el tiempo que tarda en volver una onda ultrasónica como eco. La distancia es calculada con la fórmula d = t(µs)/58, donde t es el tiempo total que tarda en volver la onda en microsegundos y 58 es un factor derivado de la velocidad del sonido en el aire.

**¿Cómo influye el ruido en las mediciones del sensor ultrasónico y cómo podría reducirse?**

El ruido afecta a las mediciones alterando los datos y empeorando la precisión del sensor. Para reducir el ruido se pueden tomar varias medidas como tomar varias muestras y tomar el promedio de esta, aplicar un algoritmo de filtro de datos para filtrar el ruido o asegurarse de estar en una superficie adecuada, sin terreno irregular y en condiciones normales de entorno.

## Preguntas parte 2:

**Si el robot detecta el color rojo en el suelo ¿Qué acción debería tomar? ¿Por qué?** 

Al detectar el color rojo, el robot debería detenerse por completo, esto porque fue programado para reaccionar de esta manera al color rojo.

**Si el sensor ultrasónico detecta valores erráticos ¿Qué estrategias podrías aplicar para mejorar la precisión?**

Las estrategias para mejorar la precisión del sensor ultrasónico son la aplicación de filtrado de datos, como el filtro de media movil, filtro de media ponderada y filtro pasa bajo. También, se puede verificar que el entorno del robot sea adecuado con temperaturas y vientos normales, con terreno estable y no movil.

**Si tuvieras que integrar un nuevo sensor para mejorar la navegación del robot ¿Cuál eligirías y por qué?**

Eligiríamos el sensor LIDAR, por su robustez al ruido y porque le da al robot mayor autonomía al darle una visión más precisa y en 360º de su entorno, generando mapas 2D/2D, permitiendole mejorar su navegación.

**¿Cuál es el tiempo de respuesta del robot al detectar un cambio de color?**
El tiempo de respuesta del robot depende del tiempo de integración del sensor TCS34725 y del ciclo de lectura del programa. En este caso, el sensor tiene un tiempo de integración de 154 milisegundos y el programa incluye una espera (delay) de 1000 milisegundos entre lecturas. Por lo tanto, el tiempo de respuesta total es aproximadamente de 1.15 segundos. Este es el tiempo que tarda el robot en captar el nuevo color.
