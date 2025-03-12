#define F_CPU 16000000UL
#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <string.h>
#define IR0   !(PINC & 0X01)
#define IR1   !(PINC & 0X02)

int main(){
   
  _delay_ms(255);
  _delay_ms(255);
 DDRC = 0b00000000;
 PINC = 0b00000000;
 DDRD = 0b11111111;
 PORTD =0b00000000;

  DDRB = 0b11111111;


 while(1){

 
 if (IR0 == 1)
 {
    PORTD =0b00010101;
    _delay_ms(3);
    PORTD =0b00010101;
    _delay_ms(1);

  //PORTD =0b00000001;
 }

  else if (IR1 == 1)
 {
  PORTD =0b00101010;
  _delay_ms(3);
  PORTD =0b00111010;
  _delay_ms(1);
 }
else

 PORTD =0b00101000;
 _delay_ms(4);

 

 }


}



//DDR(D,B,C) == pinMode
//PORT(D,B,C) == digitalWrite
//PIN(D,B,C) == READ
//체결 연결 left led 검: gnd, 빨: 5v 회색: 디지털 포트 0
//체결 연결 right led 검 gnd, 빨: 5V 색: 디지털 포트 1
//체결 연결 






