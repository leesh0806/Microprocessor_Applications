#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define set_bit(value, bit) ( _SFR_BYTE(value) |= _BV(bit))
#define clear_bit(value, bit) ( _SFR_BYTE(value) &= ~_BV(bit))

volatile uint8_t state = 0;

ISR(PCINT0_vect) {
  state = ! state;
}

void PCINT0_init(void) {
  PCICR |= (1 << PCIE0);  //pincange in
  PCMSK0 |= (1 << PCINT0);
  sei();
}

int main(void) {
  PCINT0_init();
  clear_bit(DDRB, 0);
  set_bit(DDRB, 5);
 

  while(1) {
    if(state) {
      set_bit(PORTB, 5);
     
    }
         
    else {
      clear_bit(PORTB, 5);
       _delay_ms(50); //debouncing
    }
 
  }
  
  return 0;
}