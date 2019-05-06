DC motor arduino library
=====

Esta librería facilita el control de un motor de corriente continua conectado a dos pines, siendo uno de ellos obligatoriamente una salida analógica con PWM:
* parada, movimiento hacía adelante y hacía atrás.
* movimiento con velocidad variable.

Para ello, la librería implementa la clase DcMotor, por lo que habrá que crear un objeto de esta clase para cada motor DC. La clase DcMotor implementa los siguientes métodos:
* begin(directionPin, speedPin), incializa el motor DC indicando los pines donde está conectado, siento el segundo de ellos una salda analógica con PWM.
* begin(directionPin, speedPin, [DIRECT, INVERSE]), incializa el motor DC indicando los pines donde está conectado, siento el segundo de ellos una salda analógica con PWM; en el tercer parámetro se indica si el motor está conectado en polarización directa o inversa.
* stop(), detiene el movimiento del motor.
* goForward(), mueve el motor en modo de avance a velocidad máxima.
* goBackward(), mueve el motor en modo de retroceso a velocidad máxima.
* move([FORWARD, BACKWARD], speed), mueve el motor indicando si en modo avance o retroceso y a una velocidad concreta entre 0 y 255, velocidad mínima y máxima, respectivaemente.
* move(speed), mueve el motor en base una velocidad entre -255 y 255; siendo los valores positivos en modo avance y los negativos en modo retroceso y la velocidad en valor absoluto, siendo 0 la velocidad mínima y 255 la velocidad máxima.

Esta librería trae implementados los siguientes ejemplos (en todos ellos, el motor está conectado a los pines 6 y 7):
* stopandrun.ino, detiene y mueve el motor en modo avance cada 1 segundo.
* variablespeed.ino, incrementa la velocidad del motor de 0 a 255, espera 1 segundo, decrementa la velocidad de 255 a 0, espera un segundo y empieza de nuevo. Siempre en modo avance.
* inverse.ino, igual que el ejemplo de stopandrun pero con el motor en polarización inversa.
