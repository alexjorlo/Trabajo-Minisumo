int IR = A0; //Entrada digital conectada al sensor infrarrojo

void setup()
{

 pinMode( IR , INPUT) ; //Sensor infrarrojo como entrada
}

void loop()
{
  int valor = 0;
  valor = digitalRead(IR) ; //leemos el valor del sensor infrarrojo
 

 
  if (valor == HIGH) {
      Serial.println("negro");
  }else{
    Serial.println("blanco");
  }
}
