#define F_CPU 16000000UL
#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include <string.h>


int main(){

 DDRB = 0x20;
 PORTB = (0x20 | 0x01);
 

while(1){

 if(PINB & 0x01){
   PORTB &= ~0x20;
  


 }
 
  else{

  
  PORTB |= 0x20;

  }




}

  return 0;


}