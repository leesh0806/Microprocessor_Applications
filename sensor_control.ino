
#define sensor1 A0
#define sensor2 A1
#define sensor3 A2
#define sensor4 A3

 int analog_value;


void setup() {
  
  Serial.begin(9600);
  
  
  
}

void loop() {

  analog_value = analogRead(sensor1);
  Serial.print(analog_value);
  Serial.print(" ");    //sensor1 value
  
  analog_value = analogRead(sensor2);
  Serial.print(analog_value);
  Serial.print(" ");    //sensor2 value

  analog_value = analogRead(sensor3);
  Serial.print(analog_value);
  Serial.print(" ");    //sensor3 value
  
  analog_value = analogRead(sensor4);
  Serial.print(analog_value);
  Serial.print(" ");    //sensor4 value

}
