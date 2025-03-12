#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "UART0.h"
#include <ADC.h>
#include <Wire.h>      //I2C를 사용을 위한 헤더파일
#include <hd44780.h>   //LCD 사용제어 헤더파일
#include <hd44780ioClass/hd44780_I2Cexp.h>  //I2C LCD사용을 위한 헤더파일
#include <Stepper.h>            //스텝모터 헤더파일
#include <Adafruit_TCS34725.h>  //컬러센서 헤더파일
#include <Servo.h>

#define set_bit(value, bit) ( _SFR_BYTE(value) |= _BV(bit) )
#define clear_bit(value, bit) ( _SFR_BYTE(value) &= ~_BV(bit) )

hd44780_I2Cexp lcd;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);         //클래스의 인스턴스
Servo servo1;
Servo servo2;
Stepper stepper(steps, 6, 5, 1, 0);

void PORT_init() {
  set_bit(DDRD, 3); //3번(PD3) 출력 12EN
  set_bit(DDRD, 2); //2번(PD2) 출력  1A
  set_bit(DDRD, 4); //4번(PD4) 출력  2A
  set_bit(DDRB, 3); //11번(PB3) 출력 34EN
  set_bit(DDRB, 0); // 8번(PB0) 출력 3A
  set_bit(DDRB, 4); //12번(PB4) 출력 4A
  set_bit(DDRB, 5); //13번 펌프모터
}  

void timercounter_pwm() {  
   TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21) ;   // 비반전 모드, 8비트 고속 PWM 모드, OC2A 핀, OC2B 핀 사용
   TCCR2B = (1 << CS20) | (1 << CS21);                    // 분주비 32 (사용자 설정에 따라 변경 가능)  
} 

void pwm11(uint8_t value) {
  OCR2A = value;                            // PWM 출력 설정
}

void pwm3(uint8_t value) {
  OCR2B = value;                            // PWM 출력 설정
}

//void servoInit() {                      //컵 부분 서보모터 설정        AVR MODE 서보모터 제어 설명
  // 핀 설정
  //DDRB |= (1 << SERVO_PIN1);
  //DDRB |= (1 << SERVO_PIN2);
  // Fast PWM 모드 설정
  //TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  //TCCR1B |= (1 << WGM13) | (1 << CS11) | (1 << WGM12);

  // ICR1 레지스터 설정
  //ICR1 = 39999;  // 주기: 20ms
//}

//void servocupWrite(double angle) {         //컵 부분 서보모터 구동
  // 서보모터 설정
  //double pulseWidth = 1000 + ((angle * 1000) / 45);
  //OCR1A = pulseWidth;
  //_delay_ms(10);
//}

//void servograbWrite(double angle) {       //약 내리기 서보모터 구동
  // 서보모터 설정
  //double pulseWidth = 1000 + ((angle * 1000) / 45);
  //OCR1B = pulseWidth;
  //_delay_ms(10);
//}

 double Kp = 0.25;                             //변수 선언
 double P_control;
 double Kd = 0.1;
 int last_error = 0;
 int derivative;
 int L_PWM, R_PWM;
 int error;
 int l_sensor_val;
 int r_sensor_val;
 int avg_PWM = 180;
 int ll_sensor_val;
 int rr_sensor_val;
 int angle;
 int servoPin1=9;
 int servoPin2=10;
 const int button_pin = 7;
 bool program_started = false;  //프로그램 시작 여부
 int steps = 2048;
 
   void left_motor_f(int a){                   //왼쪽 모터 직진
   clear_bit(PORTD, 2); //2번(PD2) LOW
   set_bit(PORTD, 4); //4번(PD4) HIGH
   pwm3(a);
}

   void left_motor_b(int a){                   //왼쪽 모터 후진
    set_bit(PORTD, 2); //2번(PD2)  high
    clear_bit(PORTD, 4); //4번(PD4) low
    pwm3(a);
   }

   void right_motor_f(int a){                  //오른쪽 모터 직진
   clear_bit(PORTB, 0); //8번(PB0) LOW
   set_bit(PORTB, 4); //12번(PB4) HIGH
   pwm11(a);
}

   void right_motor_b(int a){                  //오른쪽 모터 후진
   set_bit(PORTB, 0); //8번(PB0) high
   clear_bit(PORTB, 4); //12번(PB4) low
   pwm11(a);
}
   int speed_limit(int val, int minVal, int maxVal){         //속도 최소 최대
    if(val < minVal){
      return minVal;
    }
    else if(val > maxVal){
      return maxVal;
    }
    else{

      return val;
    }

  }

   int motor_stop() {                   //모터 정지
      left_motor_f(0);
      right_motor_f(0);
   }

   void ADCF(){                        //적외선센서값 초기화 및 읽기
    ADC_init(0);
    rr_sensor_val = read_ADC();                     
    ADC_init(1);
    l_sensor_val = read_ADC();
    ADC_init(2);
    r_sensor_val = read_ADC();
    ADC_init(3);
    ll_sensor_val = read_ADC();
    //UART0_print("\n");
    //UART0_print(rr_sensor_val);
    //UART0_print(" ");
    //UART0_print(ll_sensor_val);
    //UART0_print(" ");
    //UART0_print(r_sensor_val);
    //UART0_print(" ");
    //UART0_print(l_sensor_val);
    //UART0_print(" ");
    //UART0_print("\n");
   }

  void startProgram() {                                             //버튼 누르면 시작 시 lcd출력
  program_started = true;                                           // 프로그램 시작 시 수행해야 할 동작을 여기에 작성
  lcd.setCursor (0,0);
  lcd.print("mission start!");
  lcd.setCursor(1,1);
  lcd.print(" ");
  }

  void front() {                                                    //pd제어 직진 사용자 함수
    ADCF();
    error = (r_sensor_val) - (l_sensor_val);
    P_control = error * Kp;      //p제어

    //integral += error; // 적분 값 업데이트   integral = integral + error
    //double I_control = Ki * integral; // 적분 제어 

    derivative = error - last_error;
    P_control += Kd * derivative;
    last_error = error;

    R_PWM = speed_limit(avg_PWM + P_control, 0, 255);
    L_PWM = speed_limit(avg_PWM - P_control, 0, 255);
    //모터 속도 출력
     left_motor_f(L_PWM);
     right_motor_f(R_PWM);
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////이 위에는 사용자 정의 함수로 건들지 마시오.
void setup(){
    lcd.begin(16, 2);
    PORT_init(); //포트설정
    //UART0_init();
    timercounter_pwm();
    tcs.begin();
    servo2.attach(servoPin2);
    servo1.attach(servoPin1);
    servo2.write(95);  
    servo1.write(0);  
   
  DDRD &= ~(1 << button_pin);     // 버튼 핀을 입력으로 설정
  PORTD |= (1 << button_pin);     // 내부 풀업 저항 사용
  stepper.setSpeed(12);

}
void loop(){
    uint16_t red, green, blue, clear;
    tcs.getRawData(&red, &green, &blue, &clear);
  if (!program_started) {
    if ((PIND & (1 << button_pin))) {  // 버튼이 눌렸을 때
      startProgram();  // 프로그램 시작 함수 호출
     _delay_ms(1000);
    // 프로그램이 시작된 후의 동작
    // 필요시 추가코드 작성
    }
  }
    else {
     front();
      if(rr_sensor_val < 500)     //우회전을 하기 위한 단계
      {
      left_motor_f(200);
      right_motor_f(190);
      _delay_ms(1500);
      motor_stop();
      _delay_ms(500);
      while(r_sensor_val >= 500)        //하얀바닥시 계속 우회전
      {
         left_motor_f(200);
         right_motor_b(200);
         ADC_init(1);
         r_sensor_val = read_ADC();
      }
      while(r_sensor_val < 500 | l_sensor_val < 500)        //검은바닥 감지시 회전을 멈춤
      {
       ADC_init(1);
        l_sensor_val = read_ADC();
       ADC_init(2);
        r_sensor_val = read_ADC();
      }
         motor_stop();
      }
      while(ll_sensor_val < 500)                       //왼쪽 교차로 센서 감지시 정지 하고 lcd에 'Mission complete'문장 출력
      {
          motor_stop();
          _delay_ms(500);
           lcd.clear();
           lcd.setCursor (0,0);
           lcd.print("Mission complete");
           lcd.setCursor(1,1);
           lcd.print(" ");
      }
       
      if(green < 1000 & blue < 1000) //여기 부터가 레드 부분 미션 실행 코드
      {
        motor_stop();
        _delay_ms(500);
        lcd.clear();            
        lcd.setCursor (0,0);
        lcd.print("RED");            //lcd 표시
        lcd.setCursor(1,1);
        lcd.print(" ");
       _delay_ms(1000);
       for(int i = 1; i<=1; i++)
       {
        //stepper.step(steps);   //스텝모터
       
        _delay_ms(500);      //펌프모터
        set_bit(PORTB, 5);    
        _delay_ms(3000);     
        clear_bit(PORTB, 5);  

       for(angle=94; angle>55; angle--)   //내리기
       {        
        servo2.write(angle);
        _delay_ms(10);
       }
       _delay_ms(1000);
       for(angle=55; angle<94; angle++)   //올리기
       { 
        servo2.write(angle);
        _delay_ms(10);
       }
        _delay_ms(1000); 

       for(angle=0; angle<100; angle++)    //컵 내리기
       {             
        servo1.write(angle);
       _delay_ms(30);
       }
       _delay_ms(3000);


       for(angle=100; angle>0; angle--)      //컵 올리기
       {              
        servo1.write(angle);
        _delay_ms(30);
       }
        _delay_ms(500);
        left_motor_f(200);
        right_motor_f(200);
        _delay_ms(500);                       //조금 직진
      }
     }  //여기 까지가 레드 부분 미션 실행 코드
     
if(red < 1000 & blue < 1000) //여기 부터가 그린 부분 미션 실행 코드
      {
        motor_stop();
        _delay_ms(500);
        lcd.clear();
        lcd.setCursor (0,0);
        lcd.print("green");         //lcd 표시
        lcd.setCursor(1,1);
        lcd.print(" ");
        _delay_ms(1000);
     for(int i = 1; i<=1; i++)
     {
       stepper.step(2048);         //스텝모터
       stepper.step(690);
       _delay_ms(500); 

       set_bit(PORTB, 5);          //펌프모터
       _delay_ms(3000);
       clear_bit(PORTB, 5);
        
       for(angle=94; angle>55; angle--)           //내리기
       {                                        
        servo2.write(angle);
        _delay_ms(10);
       }
        _delay_ms(1000);
        
       for(angle=55; angle<94; angle++)           //올리기
       { 
        servo2.write(angle);
         _delay_ms(10);
       }
         _delay_ms(1000); 

       for(angle=0; angle<100; angle++)           //컵 내리기
       {                                          
        servo1.write(angle);
        _delay_ms(30);
       }

       _delay_ms(3000);

       for(angle=100; angle>0; angle--)           //컵 올리기
       {         
        servo1.write(angle);
        _delay_ms(30);
       }
       _delay_ms(500);
       left_motor_f(200);
       right_motor_f(200);
       _delay_ms(500);                            //조금 직진
    }
   }  //여기 까지가 그린 부분 미션 실행 코드

if(red < 1000 & green < 1200) //여기 부터가 블루 부분 미션 실행 코드
     {
        motor_stop();
        _delay_ms(500);
        //다른 임무 수행
         lcd.clear();
         lcd.setCursor (0,0);
         lcd.print("blue");
        lcd.setCursor(1,1);
         lcd.print(" ");
         _delay_ms(1000);
       for(int i = 1; i<=1; i++)
        {
          stepper.step(2048);   //스텝모터
          stepper.step(690);
          _delay_ms(500); 

          set_bit(PORTB, 5);    //펌프모터
          _delay_ms(3000);
          clear_bit(PORTB, 5);
        
          for(angle=94; angle>55; angle--)     //내리기
           {        
             servo2.write(angle);
              _delay_ms(10);
           }
              _delay_ms(1000);
        
          for(angle=55; angle<94; angle++)           //올리기
          { 
           servo2.write(angle);
             _delay_ms(10);
          }
             _delay_ms(1000); 

          for(angle=0; angle<100; angle++)           //컵 내리기
         {             
            servo1.write(angle);
            _delay_ms(30);
         }
           _delay_ms(3000);
          for(angle=100; angle>0; angle--)            //컵 올리기
        {              
           servo1.write(angle);
            _delay_ms(30);
        }
              _delay_ms(500);
              left_motor_f(200);
              right_motor_f(200);
              _delay_ms(500);                          //조금 직진
        }
       }  //여기 까지가 블루 부분 미션 실행 코드
    }
}
