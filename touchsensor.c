#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

#define TOUCH_PIN   0   // PC0 (Sensor D0 output)
#define LED_PIN     5   // PA5 (Onboard LED on Nucleo)

void GPIO_Init_Custom(void) {
    // Enable clock for GPIOA and GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // for LED (PA5)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // for Touch sensor (PC0)

    // PC0 = input (touch sensor)
    GPIOC->MODER &= ~(0b11 << (TOUCH_PIN * 2));

    // PA5 = output (LED)
    GPIOA->MODER &= ~(0b11 << (LED_PIN * 2));
    GPIOA->MODER |=  (0b01 << (LED_PIN * 2));  // output mode
}

int main(void) {
    GPIO_Init_Custom();

    while(1) {
        // Read touch sensor from PC0 (Active LOW)
        if((GPIOC->IDR & (1 << TOUCH_PIN))) {  // LOW = touched
            GPIOA->ODR |= (1 << LED_PIN);       // turn on LED (PA5)
        } else {
            GPIOA->ODR &= ~(1 << LED_PIN);      // turn off LED
        }
    }
}
