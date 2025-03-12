
#define motor1 5
#define motor2 6
#define motor3 9
#define motor4 10



void setup() {

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);


}

void loop() {

   motor_controller(0,0,200,250);
   delay(1000);



}


void motor_controller(int a, int b, int m1speed, int m2speed){
  
  if(a=!0){
    analogWrite(motor1, m1speed);     
    analogWrite(motor2, 0);
  }
  
  else{ 
    analogWrite(motor1, 0);
    analogWrite(motor2, m1speed);
  }

  if(b=!0){
    analogWrite(motor3, m2speed);
    analogWrite(motor4, 0);
  }

  else{
    analogWrite(motor3, 0);
    analogWrite(motor4, m2speed);



  }



}








