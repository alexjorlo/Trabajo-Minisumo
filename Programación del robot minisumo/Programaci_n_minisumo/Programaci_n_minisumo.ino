#include <DcMotor.h>



#include <Counter.h>
#include <interrupt_pins.h>

#include <DcMotor.h>

#include <MCP23008.h>

// direcciones I2C de los componentes
#define DIR_I2C_GESTION   0x27
#define DIR_I2C_SENSORES  0x20

// estados para la programación basada en autómatas
#define PARADO            0
#define BUSCAR            1
#define ENBESTIRDERECHA   2
#define ENBESTIRIZQUIERDA 3

// pines del arduino
#define MOTOR_DER_DIR     12
#define MOTOR_DER_VEL     6
#define MOTOR_IZQ_DIR     4
#define MOTOR_IZQ_VEL     5

// constantes para el control PD y el ajuste de la velocidad de los motores
/**#define KP                12
#define KD                60 */
#define VELOCIDAD         140

// instancia de la placa de control
MCP23008 gestionI2C;
MCP23008 sensoresI2C;


int Sizq = 0;
int Sder = 0;
int Qder = 0;
int Qizq = 0;

// variables
unsigned char estado = PARADO;
int error = 0, error_anterior;
int sensor_error[4] = {-4, -2, 2, 4};
#include <DcMotor.h>


DcMotor motorizquierdo;
DcMotor motorderecho;

int sensorldoderecho=A1; //conectado al pin A1 por ser analogico
int sensorldoldoizquierdo=A2;//conectado al pin A2 por ser analogico
int sensorarribaderecho=A0;//conectado al pin A0 por ser analogico
int sensorarribaizquierdo=A3;//conectado al pin A3 por ser analogico
int sensorblanconegroizquierdo=8;//conectado al pin 8 por ser digital
int sensorblanconegroderecho=1;//conectado al pin 1 por ser digital

long distancia;
long tiempo;


void setup() {
  motorizquierdo.begin(12,6);
  motorderecho.begin(5,4);

  

  gestionI2C.begin(DIR_I2C_GESTION);//los motores
  gestionI2C.pinMode(0x0F);
  gestionI2C.setPullup(0x0F);

  sensoresI2C.begin(DIR_I2C_SENSORES);
  sensoresI2C.pinMode(0xFF);
  sensoresI2C.setPullup(0x00);
 
 pinMode (sensorldoderecho, OUTPUT);
 pinMode (sensorldoldoizquierdo, OUTPUT);
 pinMode (sensorarribaderecho, OUTPUT);
 pinMode (sensorarribaizquierdo, OUTPUT);//los seis sensores*/
 pinMode (sensorblanconegroizquierdo, OUTPUT);
 pinMode (sensorblanconegroderecho, OUTPUT);

}

void loop() {
switch (estado) {
case PARADO:
parar(); delay(5000);              //cuando lo tengamos parado
break;
case BUSCAR:
sensores();                   //comenzamos a buscar
adelante();
break;

case EMBESTIRDERECHA:
sensores();
derecha();
break;

case EMBESTIRIZQUIERDA:
sensores();
izquierda();
break;





}

  
  
  
  
  
  
  
  
  sensores();
  if((Sder<=300)&&(Sizq<=300)){adelante();} 
  if((Sder<=300)&&(Sizq>=300)){izquierda();}
  if((Sder<=300)&&(Sizq<=300)){derecha();}
  if((Sder>=300)&&(Sizq>=300)){adelante();}
  if((Qder>=300)||(Qizq>=300)){atras();delay(200);derecha(); delay(200);}

  void sensores (){
 Sder = analogRead(sensorldoderecho); 
 delay(1);   
 Sizq = analogRead(sensorldoldoizquierdo); 
 delay(1);   
 Qder = analogRead(sensorarribaderecho); 
 delay(1);   
 Qizq = analogRead(sensorarribaizquierdo); 
 delay(1);   
}
void adelante(){
digitalWrite(MOTOR_DER_VEL, HIGH);
digitalWrite(MOTOR_DER_DIR, LOW);
digitalWrite(MOTOR_IZQ_DIR, HIGH);
digitalWrite(MOTOR_IZQ_VEL, LOW);
}
void atras(){
digitalWrite(MOTOR_DER_VEL, LOW);
digitalWrite(MOTOR_DER_DIR, HIGH);
digitalWrite(MOTOR_IZQ_DIR, LOW);
digitalWrite(MOTOR_IZQ_VEL, HIGH);
}
void parar(){
digitalWrite(MOTOR_DER_VEL, LOW);
digitalWrite(MOTOR_DER_DIR, LOW);
digitalWrite(MOTOR_IZQ_DIR, LOW);
digitalWrite(MOTOR_IZQ_VEL, LOW);
}

void derecha(){
digitalWrite(MOTOR_DER_VEL, LOW);
digitalWrite(MOTOR_DER_DIR, LOW);
digitalWrite(MOTOR_IZQ_DIR, HIGH);
digitalWrite(MOTOR_IZQ_VEL, LOW);
}

void izquierda(){
digitalWrite(MOTOR_DER_VEL, HIGH);
digitalWrite(MOTOR_DER_DIR, LOW);
digitalWrite(MOTOR_IZQ_DIR, LOW);
digitalWrite(MOTOR_IZQ_VEL, LOW);
}




 
   /** case PARADO:
      // comprobación del botón de cambio de estado
      if (bigButtonPulsed() == true) {
        gestionI2C.write(4, HIGH);
        estado = CARRERA;                          // cambia al estado de carrera
        delay(5000);       
      
      }
      break;
    case BUSCAR:
    if(sensorldoderecho or sensorldoldoizquierdo or sensorarribaderecho or sensorarribaizquierdo<150){
      // comprobación del botón de cambio de estado 
      if (bigButtonPulsed() == true) {
        gestionI2C.write(4, LOW);
        estado = PARADO;                           // cambia al estado de parada
        motorderecho.stop();         // detiene los dos motores
        motorizquierdo.stop();
        break;                                    // se rompe el switch para que el control PD no active los motores de nuevo
      }

      case ENVESTIR:
      // lectura de sensores
      char values = sensoresI2C.read();

     /*

      // cálculo del error
      error_anterior = error;                     // se guarda el error anterior para el control derivativo
      error = 0;                                  // error y número de sensores en negro se inician en 0
      int num_negros = 0;
      for (int i = 0; i < 4; i++) {
        if (bitRead(values, i) == HIGH) {         // si el sensor ha detectado negro,
          error += sensor_error[i];               // se suma el error correspondiente a ese sensor y
          num_negros++;                           // se incrementa el número de sensores que detectan negro
        }
      }
      if (num_negros != 0) {                      // sólo si se ha detectado algún negro,
        error /= num_negros;                      // se aplica la corrección al error para el caso de uno o dos sensores detectando negro
      } 

      // aplicación del control PD
      int desfase = KP * error + KD* (error - error_anterior);*/
/**
      // cálculo y limitación de las velocidades
      int velocidad_der = VELOCIDAD; /*+ desfase;    // se calcula la nueva velocidad para el motor derecho y
      if (velocidad_der < 0) {                    // se comprueba que la velocidad esté dentro del rango de 0 a 255.
        velocidad_der = 0;
      }
      else if (velocidad_der > 140) {
        velocidad_der =140;
      }
      int velocidad_izq = VELOCIDAD;  /*-*desfase;    // se calcula la nueva velocidad para el motor izquierdo y
      if (velocidad_izq < 0) {                    // se comprueba que la velocidad esté dentro del rango de 0 a 255.
        velocidad_izq = 0;
      }
      else if (velocidad_izq > 140) {
        velocidad_izq = 140;
      }

      //actualización de velocidad en motores
      
     
      motorizquierdo.move(BACKWARD,velocidad_izq);
      motorderecho.move(velocidad_der);

      break;
  }
  delay(5);                                       // tiempo de espera hasta la próxima aplicación del control PD

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
  }*/
  
}
}

  
  
