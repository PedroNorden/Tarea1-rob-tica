# Tarea1-robótica

Vicente Rosales<br>
Manuel Aguilera<br>
Alex Alfaro<br>
Eliseo Guarda<br>
Pedro Nordenflycht<br>

## Videos demostración

Parte 1: https://www.youtube.com/watch?v=PZLfdyxwgxA <br>
Parte 2: https://www.youtube.com/watch?v=-RQUt73RMwE

## Preguntas parte 1:

**¿Qué función cumplen los sensores, actuadores y controladores en el robot?**

Los controladores funcionan como el cerebro del robot, dando las instrucciones para que este funcione. Los actuadores funcionan como el músculo y permiten transformar energía en fuerza, así permitiendo que el robot pueda moverse o realizar trabajos. Finalmente, los sensores funcionan como los órganos de los sentidos del robot, permitiendole procesar su entorno, tomar decisiones y actuar de manera autónoma.

**¿Cómo se puede estimar la velocidad sin encoders?**

Se puede estimar por medio de la velocidad enviada por PWM. Por medio de sensores LIDAR o ultrasónicos, también es posible estimar la velocidad según el cambio de posición relativa y el tiempo entre estas mediciones.

**¿Cómo afecta la falta de encoders a la precisión del movimiento?**

Sin un encoder, solo podemos esperar que el motor haga el movimiento esperado, lo que es improbable con distancias largas por imperfecciones en las ruedas o en la ruta. Por lo tanto, la acumulación de errores es mayor y no se puede saber la posición exacta sin uso de sensores externos.

**¿Qué es PWM y cómo ayuda a controlar la velocidad de los motores?**

Pulse Width Modulation o PWM es un pulso enviado desde el controlador hacia el motor para controlar el tiempo de encendido y apagado de este. Así, transformando la señal digital a análoga, permite controlar cuanto tiempo está prendido el motor para cambiar la aceleración y por consecuencia la velocidad.

**¿Cómo afecta el control de velocidad a la precisión de la navegación sin encoders?**

Al controlar la velocidad sin encoders, se pasan por alto las imperfecciones de la ruta o de las ruedas, por lo que el control de velocidad por medio de PWM, por ejemplo, va a ser constante y no se adapta a estas situaciones que cambian la distancia recorrida. Por esto, la precisión de la navegación es poco confiable.


## Preguntas parte 2

**¿Cómo se calcula la velocidad del robot sin encoders usando PWM?**

Se puede estimar la velocidad usando PWMs al enviar pulsos que hacen avanzar al robot, para luego calcular cuando tiempo tardó en recorrer una distancia específica. Este mismo experimento puede ser realizado varias veces para calibrar al robot, así finalmente se va a tener una estimación acertada de cual es la velocidad del robot al moverse usando PWM.

**¿Cómo factores afectan la trayectoria y velocidad del robot al cambiar los intervalos de tiempo?**

Estos cambian ya que si se tiene un intervalo muy grande, se hacen recorrecciones muy tarde, alterando la trayectoria del robot. Por otro lado, si se tiene un intervalo muy pequeño, se hacen muchas recorrecciones que frenan constantemente al robot, alterando la velocida, y lo hacen más propenso a caer en errores ocasionales, alterando la trayectoria.

**¿Cúales son las ventajas y desventajas de usar un IMU para ajustar la dirección en lugar de encoders?**

Al no depender de las ruedas, el IMU puede hacer ajustes independiente de las condiciones del terreno o ruedas. Por esto mismo, el IMU no calcula el desplazamiento lineal directamente, sino que con estimaciones, por lo que los ajustes pueden ser imprecisos.

**Qué efecto tiene la inclinación o el giro en el movimiento del robot, y cómo se corrige con el IMU?**

La inclinación y el giro le dan inestabilidad al robot, cambiando su orientación y alterando su trayectoria. El IMU puede detectar giros o inclinaciones no deseadas para cambiar la velocidad o detener el robot por completo para mantenerlo seguro y seguir con la trayectoria posteriormente.
