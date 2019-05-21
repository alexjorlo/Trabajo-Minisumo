
#include <DcMotor.h>



#include <Counter.h>
#include <interrupt_pins.h>



#include <MCP23008.h>

// direcciones I2C de los componentes
#define DIR_I2C_GESTION   0x27
#define DIR_I2C_SENSORES  0x20

// estados para la programación basada en autómatas
#define ARRANCAR   0
#define PARAR   1
#define BUSCAR 2



// pines del arduino
#define MOTOR_DER_DIR     12
#define MOTOR_DER_VEL     6
#define MOTOR_IZQ_DIR     4
#define MOTOR_IZQ_VEL     5


// instancia de la placa de control
MCP23008 gestionI2C;
MCP23008 sensoresI2C;




// variables
unsigned char estado = PARAR;
int error = 0, error_anterior;
int sensor_error[4] = { -4, -2, 2, 4};
#include <DcMotor.h>


DcMotor motorizquierdo;
DcMotor motorderecho;

int sensorldoderecho = A1; //conectado al pin A1 por ser analogico
int sensorldoizquierdo = A2; //conectado al pin A2 por ser analogico
int sensorarribaderecho = A0; //conectado al pin A0 por ser analogico
int sensorarribaizquierdo = A3; //conectado al pin A3 por ser analogico
int sensorblanconegroizquierdo = 8; //conectado al pin 8 por ser digital
int sensorblanconegroderecho = 1; //conectado al pin 1 por ser digital




void setup() {
  motorizquierdo.begin(12, 6, INVERSE);
  motorderecho.begin(4, 5);



  gestionI2C.begin(DIR_I2C_GESTION);//los motores
  gestionI2C.pinMode(0x0F);
  gestionI2C.setPullup(0x0F);

  sensoresI2C.begin(DIR_I2C_SENSORES);
  sensoresI2C.pinMode(0xFF);
  sensoresI2C.setPullup(0x00);

  pinMode (sensorldoderecho, INPUT); //decimos que son entradas
  pinMode (sensorldoizquierdo, INPUT);
  pinMode (sensorarribaderecho, INPUT);
  pinMode (sensorarribaizquierdo, INPUT);//los seis sensores*/
  pinMode (sensorblanconegroizquierdo, INPUT);
  pinMode (sensorblanconegroderecho, INPUT);
  Serial.begin(9600);

}

void loop() { //aqui ahcemos un switch con todos los estados
  switch (estado) {
    case ARRANCAR:



      digitalWrite(LED_BUILTIN, HIGH);
      gestionI2C.write(4, HIGH);
      motorizquierdo.move( 80);
      motorderecho.move(-175);
      break;
    case BUSCAR:


      if ((digitalRead(sensorblanconegroderecho) == HIGH) || (digitalRead(sensorblanconegroizquierdo) == HIGH)) { //Aqui es si con los dos sensores detecta negro que haga los demas casos
        Serial.println("aaa");




        if ((analogRead(sensorarribaderecho) >= 50) || (analogRead(sensorarribaizquierdo) >= 50)  ) {//si me detectan los de alante voy alante
          motorderecho.goBackward();
          motorizquierdo.goForward();

        }

        else if (analogRead(sensorldoderecho) >= 60  ) {//si detecto a la derecha gito a la derecha
          motorderecho.goBackward();
          motorizquierdo.goBackward();




        }


        if ( analogRead(sensorldoizquierdo) >= 60 ) { //si detecto a la izquierda giro a la izquierda
          motorderecho.goForward();
          motorizquierdo.goForward();



        }





      }

      if ( (digitalRead(sensorblanconegroizquierdo) == LOW) and (digitalRead(sensorblanconegroderecho) == HIGH)) { // si detecta con el sensor de alante que vaya hacia atras
        motorderecho.goForward();
        motorizquierdo.goBackward();

        delay(500); // Le metemos medio segundo de reloj para que se estabilice en la pista

        motorderecho.goForward();//luego ahcemos que gire
        motorizquierdo.goForward();
        delay(200);
      }
      else if ((digitalRead(sensorblanconegroderecho) == LOW) and (digitalRead(sensorblanconegroizquierdo) == HIGH)) { // por si se sale a la IZQUIERDA
        motorderecho.goBackward();
        motorizquierdo.goForward();
        delay(500);//Le metemos medio segundo de reloj para que se estabilice en la pista
        motorderecho.goBackward();//luego ahcemos que gire
        motorizquierdo.goBackward();
        delay(500);
      } else if ((digitalRead(sensorblanconegroderecho)) == LOW and (digitalRead(sensorblanconegroizquierdo) == LOW)) {// este es por si lee con los dos en blanco
        motorderecho.goBackward();
        motorizquierdo.goForward();
        delay(500);
        motorderecho.goBackward();
        motorizquierdo.goBackward();
        delay(500);
      }



      // Le metemos medio segundo de reloj para que se estabilice en la pista




      break;

    case PARAR:
      gestionI2C.write(4, LOW);
      motorderecho.stop();         // detiene los dos motores
      motorizquierdo.stop();
      digitalWrite(LED_BUILTIN, LOW);

      break;
  }
  if (bigButtonPulsed()) {

    if (estado == PARAR) {

      estado = ARRANCAR;
      delay(5000);
    }
    if (estado == ARRANCAR) {

      estado = BUSCAR;

    } else {

      estado = PARAR;
    }
  }

}

unsigned char estado_anterior_boton = HIGH;

bool bigButtonPulsed() {

  // lectura del estado del botón
  char estado_boton = gestionI2C.read(3);
  // identificación de un flanco ascendente en base al estado anterior y al actual
  if ((estado_anterior_boton == HIGH) && (estado_boton == LOW)) {
    estado_anterior_boton = estado_boton;
    return true;
  }
  else {
    estado_anterior_boton = estado_boton;
    return false;
  }



}
