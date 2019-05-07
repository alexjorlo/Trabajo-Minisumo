int sensor=A0;
int sensorValue;
int tiempo;
int distancia;
void setup() {
  Serial.begin(9600);

}

void loop() {
   distancia= int(0.017*tiempo);
  sensorValue = analogRead(sensor);
  Serial.println(sensorValue);
   if (sensorValue >=10) 
   {
      Serial.println("Objt cerca");
   }
   else 
   {
      Serial.println("Objt lejos");
   }
  //Serial.println(distancia);
 // if(sensor<=10){
  //  Serial.println("El objetivo esta cerca");
  //}else{
 //   Serial.println("El objetivo esta lejos");
  //}

}
