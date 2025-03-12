#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "UART0.h"
#include <ADC.h>
#define set_bit(value, bit) ( _SFR_BYTE(value) |= _BV(bit) )
#define clear_bit(value, bit) ( _SFR_BYTE(value) &= ~_BV(bit) )

int motor12EN = 3 ;
int motor34EN = 9 ;

void PORT_init() {
  set_bit(DDRD, 0); //왼쪽 led
  set_bit(DDRD, 1); //오른쪽 led
  set_bit(DDRD, 3); //3번(PD3) 출력 12EN
  set_bit(DDRD, 5); //5번(PD5) 출력  1A
  set_bit(DDRD, 6); //6번(PD6) 출력  2A
  set_bit(DDRB, 1); //9번(PB1) 출력 34EN
  set_bit(DDRB, 2); //10번(PB2) 출력 3A
  set_bit(DDRB, 3); //11번(PB3) 출력 4A
  //모터 방향 설정
}   //2A랑 4A가 HIGH 이면 직진


 
 double Kp = 0.1025;
 double P_control;
 double Kd = 0.2;
 int last_error = 0;
 int derivative;
 int L_PWM, R_PWM;
 int error;
 int l_sensor_val;
 int r_sensor_val;
 int avg_PWM = 150;

void setup(){

  PORT_init(); //포트설정
  UART0_init();
  _delay_ms(3000);
}

void loop(){
   ADC_init(0);
   l_sensor_val = read_ADC();
   ADC_init(1);
   r_sensor_val = read_ADC();
   UART0_print(l_sensor_val);
   UART0_print(" ");
   UART0_print(r_sensor_val);
   UART0_write('\n');

    error = (l_sensor_val) - (r_sensor_val);
    P_control = error * Kp;      //p제어

    derivative = error - last_error;
    P_control += Kd * derivative;
    last_error = error;
    
    R_PWM = speed_limit(avg_PWM + P_control, 0, 210);
    L_PWM = speed_limit(avg_PWM - P_control, 0, 210);

    //모터 속도 출력
    
    left_motor_f(L_PWM);
    right_motor_f(R_PWM);
    
}



   void left_motor_f(int a){
   clear_bit(PORTD, 5); //5번(PD5) LOW
   set_bit(PORTD, 6); //6번(PD6) HIGH
   analogWrite(motor12EN, a);
  

}

   void right_motor_f(int a){
   clear_bit(PORTB, 2); //5번(PD5) LOW
   set_bit(PORTB, 3); //6번(PD6) HIGH
   analogWrite(motor34EN, a);
  

}

   int speed_limit(int val, int minVal, int maxVal){ 
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

//흰색 바닥 989 989 
//전진상태 959 968
// 왼쪽 센서 검은색 362 965
// 오른쪽 센서 검은색 978 743 

// l sensor 750  r sensor 950;

 /* error = l_sensor_val - r_sensor_val;
  P_control = Kp * error;      // 비례 제어

  integral += error;           // 적분 값 업데이트
  double I_control = Ki * integral;   // 적분 제어

  derivative = error - last_error;
  double D_control = Kd * derivative;   // 미분 제어

  last_error = error;

  double PID_control = P_control + I_control + D_control;   // PID 제어 값 계산

  R_PWM = speed_limit(avg_PWM + PID_control, 0, 210);
  L_PWM = speed_limit(avg_PWM - PID_control, 0, 210);

*/
