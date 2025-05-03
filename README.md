# Tarea1-robótica

Vicente Rosales
Manuel Aguilera
Alex Alfaro
Eliseo Guarda
Pedro Nordenflycht

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
