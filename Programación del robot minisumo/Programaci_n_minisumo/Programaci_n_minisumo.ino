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

  pinMode (sensorldoderecho, INPUT);
  pinMode (sensorldoizquierdo, INPUT);
  pinMode (sensorarribaderecho, INPUT);
  pinMode (sensorarribaizquierdo, INPUT);//los seis sensores*/
  pinMode (sensorblanconegroizquierdo, INPUT);
  pinMode (sensorblanconegroderecho, INPUT);
  Serial.begin(9600);

}

void loop() {
  switch (estado) {
    case ARRANCAR:


      //delay(5000);
      digitalWrite(LED_BUILTIN, HIGH);
      gestionI2C.write(4, HIGH);
      motorizquierdo.move( 80);
      motorderecho.move(-175);



      if ((digitalRead(sensorblanconegroderecho) == HIGH) || (digitalRead(sensorblanconegroizquierdo) == HIGH)) { //Aqui es si con los dos sensores detecta negro que haga los demas casos
        //motorderecho.move(-100);
        // motorizquierdo.move(-100);
        if ((analogRead(sensorarribaderecho) <= 300) and (analogRead(sensorarribaizquierdo) <= 300) and (analogRead(sensorldoderecho) <= 300) and (analogRead(sensorldoizquierdo) <= 300)) {
          motorderecho.move(175);
          motorizquierdo.move(150);
        } 
        if ((analogRead(sensorarribaderecho) >= 305) and (analogRead(sensorarribaizquierdo) < 305)) {
          motorderecho.move(-175);
          motorizquierdo.move(0);

        } 
        if ((analogRead(sensorarribaderecho) <= 305) and (analogRead(sensorarribaizquierdo) >= 305)) {
          motorderecho.move(0);
          motorizquierdo.move(175);
        } 
        if ((analogRead(sensorarribaderecho) >= 305 and analogRead(sensorarribaizquierdo) >= 305)) {
          motorderecho.move(-175);
          motorizquierdo.move(100);

        }



      } 
      if ( (digitalRead(sensorblanconegroizquierdo) == LOW)) { // si detecta con el sensor de alante que vaya hacia atras
        motorderecho.goForward();
        motorizquierdo.goBackward();
        delay(2000); // Le metemos dos segundos de reloj para que se estabilice en la pista
        
        Serial.println("Entra por aqui");
      } else if ((digitalRead(sensorblanconegroderecho) == LOW)) { // por si se sale a la IZQUIERDA
        motorderecho.goBackward();
        motorizquierdo.goForward();
        delay(2000);                  // Le metemos dos segundos de reloj para que se estabilice en la pista
      }



      break;
    //case EVITAR_SALIR:
    // ¡
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
    } else {
      estado = PARAR;
    }
  }







  /**
    if (digitalRead(sensorblanconegroderecho) == 1 || digitalRead(sensorblanconegroizquierdo) == 1) {
    motorderecho.move(100);
    motorizquierdo.move(100);
    Serial.println(sensorblanconegroderecho);
    delay(3000);
    Serial.println(sensorblanconegroizquierdo);
    delay(3000);

    } else if (digitalRead(sensorblanconegroderecho) == 1) { // por si se sale a la derecha
    motorderecho.move(100);
    motorizquierdo.move(0);



    } else if (digitalRead(sensorblanconegroizquierdo) == 1) {
    motorderecho.move(0);
    motorizquierdo.move(100);



    }

    if (analogRead(sensorarribaderecho) == 300 and (sensorarribaizquierdo) == 300) {
    motorderecho.move(100);
    motorizquierdo.move(100);


    } else if (analogRead(sensorldoderecho) == 300) {
    motorderecho.move(100);
    motorizquierdo.move(0);


    } else if (analogRead(sensorldoldoizquierdo) == 300) {
    motorderecho.move(0);
    motorizquierdo.move(1000);

    }

  */
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
