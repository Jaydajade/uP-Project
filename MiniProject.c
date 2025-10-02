/*This is the code to make the 7-segment showing the number 0-9 when we adjust the
 * voltage adjustment*/

/*#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

// -------------------- GPIO --------------------
void GPIO_Init(void) {
    // 7-segment outputs
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER8_Pos) | (0b01 << GPIO_MODER_MODER9_Pos);

    GPIOB->MODER &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODER10_Pos);

    GPIOC->MODER &= ~(GPIO_MODER_MODER7);
    GPIOC->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);
}

// -------------------- 7-segment display --------------------
void Display_Number(uint8_t num) {
    // clear
    GPIOC->ODR &= ~(1 << 7);
    GPIOA->ODR &= ~(1 << 8);
    GPIOA->ODR &= ~(1 << 9);
    GPIOB->ODR &= ~(1 << 10);

    switch (num) {
        case 0: break;
        case 1: GPIOC->ODR |= (1 << 7); break;
        case 2: GPIOA->ODR |= (1 << 8); break;
        case 3: GPIOA->ODR |= (1 << 8); GPIOC->ODR |= (1 << 7); break;
        case 4: GPIOB->ODR |= (1 << 10); break;
        case 5: GPIOB->ODR |= (1 << 10); GPIOC->ODR |= (1 << 7); break;
        case 6: GPIOA->ODR |= (1 << 8); GPIOB->ODR |= (1 << 10); break;
        case 7: GPIOA->ODR |= (1 << 8); GPIOB->ODR |= (1 << 10); GPIOC->ODR |= (1 << 7); break;
        case 8: GPIOA->ODR |= (1 << 9); break;
        case 9: GPIOA->ODR |= (1 << 9); GPIOC->ODR |= (1 << 7); break;
        default: break;
    }
}

// -------------------- ADC Init --------------------
void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // PA4 analog
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos);

    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SMPR2 |= ADC_SMPR2_SMP4;
    ADC1->SQR1 = 0;                       // one conversion
    ADC1->SQR3 = 4;                       // channel 4
}

// -------------------- Read ADC --------------------
uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

// -------------------- Main --------------------
int main(void) {
    GPIO_Init();
    ADC1_Init();

    while (1) {
        uint16_t adc_val = ADC1_Read();
        uint8_t number = (adc_val * 10) / 4096;   // map 0-4095 → 0-9
        Display_Number(number);
    }
}

*/
