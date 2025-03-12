#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "UART0.h"

#define buttonPin 0      // 버튼 핀 번호
#define ledPin 5         // LED 핀 번호

int main() {
  DDRB &= ~(1 << buttonPin);   // 버튼 핀을 입력으로 설정
  DDRB |= (1 << ledPin);       // LED 핀을 출력으로 설정
  PORTB |= (1 << buttonPin);   // 내부 풀업 저항 활성화
  UART0_init();          // 시리얼 통신 시작


while(1) {
  int buttonState = (PINB & (1 << buttonPin)); // 버튼 상태 읽기

  static int lastButtonState = buttonState; // 마지막 버튼 상태 (0 또는 1)
  if (buttonState != lastButtonState) { // 버튼 상태가 변했을 때
    if (buttonState == 0) {             // 버튼이 눌렸을 때
      PORTB |= (1 << ledPin);           // LED 켜기
        UART0_print("on\n");
    } else {                            // 버튼이 떨어졌을 때
      PORTB &= ~(1 << ledPin);          // LED 끄기
      UART0_print("off\n");
    }
    lastButtonState = buttonState;      // 마지막 버튼 상태 업데이트
  }
  
  _delay_ms(50); // 디바운싱 처리용 딜레이
}

}