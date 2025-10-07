/*// This is the program for makeing the song for when the washing machine is done
//doing the program
//the buzzer pin can be changed depend on which pin you plug it
//

#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

#define BUZZER_PIN  2   // PC1

void GPIO_Init_Custom(void) {
    // Enable clock for GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // PC1 = output
    GPIOC->MODER &= ~(0b11 << (BUZZER_PIN * 2));
    GPIOC->MODER |=  (0b01 << (BUZZER_PIN * 2));
}

void delay(volatile uint32_t d) {
    while(d--);
}

void beep(uint32_t frequency, uint32_t duration) {
    uint32_t period = 16000000 / frequency; // SYSCLK=16MHz assumed
    uint32_t half_period = period / 2;
    uint32_t cycles = (frequency * duration) / 1000; // duration in ms

    for(uint32_t i=0; i<cycles; i++) {
        GPIOC->ODR ^= (1 << BUZZER_PIN);  // toggle PC1
        delay(half_period);
    }
}

int main(void) {
    GPIO_Init_Custom();

    //the note better use with 1k up if you gen the chat add the 0 at the
    //end of the note
       beep(7840, 200);  delay(50);    // G5
       beep(9880, 200);  delay(50);    // B5
       beep(10470, 300); delay(80);    // C6
       beep(13190, 300); delay(60);    // E6
       beep(15680, 250); delay(60);    // G6

       beep(13970, 250); delay(50);    // F6
       beep(11750, 250); delay(50);    // D6
       beep(10470, 300); delay(100);   // C6

       // Second phrase (softer descent)
       beep(7840, 200);  delay(50);    // G5
       beep(9880, 200);  delay(50);    // B5
       beep(11750, 250); delay(60);    // D6
       beep(13190, 250); delay(60);    // E6
       beep(10470, 300); delay(100);   // C6

       // Ending gentle cadence
       beep(7840, 200);  delay(50);    // G5
       beep(6590, 200);  delay(50);    // E5
       beep(5230, 400);  delay(100);   // C5


       delay(4000); // wait before repeating
}


*/
