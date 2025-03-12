#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define SWITCH_PIN PB0
#define LED_PIN PB5

int main(void) {
    DDRB |= (1 << LED_PIN);   // LED 핀을 출력으로 설정
    PORTB |= (1 << SWITCH_PIN);  // 스위치 핀을 내부 pull-up으로 설정

    while (1) {
        if (!(PINB & (1 << SWITCH_PIN))) { // 스위치가 눌리면
            PORTB |= (1 << LED_PIN); // LED 켜기
        } else {
            PORTB &= ~(1 << LED_PIN); // LED 끄기
        }
    }
    return 0;
}