#include <Counter.h>
#include <interrupt_pins.h>

#include <DcMotor.h>

#include <MCP23008.h>

// Aquí declararemos los pines de cada uno de los componentes
int Motor1=10;  //conectado al pin 10
int Motor2=11; //conectado al pin 11

int sensoranalog1=5; //conectado al pin 5
int sensoranalog2=12;//conectado al pin 12
int sensordigital1=8;//conectado al pin 8
int sensordigital2=1;//conectado al pin 1

long distancia;
long tiempo;


void setup() {
 pinMode (Motor1, OUTPUT);
 pinMode (Motor2, OUTPUT); //los dos motores

 pinMode (sensoranalog1, OUTPUT);
 pinMode (sensoranalog2, OUTPUT);
 pinMode (sensordigital1, OUTPUT);
 pinMode (sensordigital2, OUTPUT); //los cuatro sensores

}

void loop() {
  int botonestado1=digitalRead (sensordigital2);//lectura de los pines
  Serial.println(botonestado1);

  int botonestado2=digitalRead (sensordigital1);//lectura de los pines
  Serial.println(botonestado2);

  distancia= int(0.017*tiempo);//formula para la distancia
  Serial.println("Distancia "); //saber la distancia
Serial.println(distancia);
Serial.println(" cm");
delay(100);
if(distancia< 25)
{
  
}
else{
  
}
int adelante () { //para ir hacia delante
digitalWrite (Motor1, HIGH);
digitalWrite (Motor2, HIGH);
}

int atras atras(){
  //digitalWrite (Motor1, LOW);
//digitalWrite (Motor2, HIGH); Como se haría
}
int izquierda(){
digitalWrite (Motor1, HIGH);
digitalWrite (Motor2, LOW);
}
int derecha(){
digitalWrite (Motor1, LOW);
digitalWrite (Motor2, HIGH);
}
int detectar(){
  
}

  
  

}
