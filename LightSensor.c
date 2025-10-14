#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define STM32F411xE
#include "stm32f4xx.h"
#define THRESHOLD 133333

#define VREF			3.3f
#define VCC				3.3f
#define ADC_MAXERS 		4095.0f
#define RX				10000.0f
#define SLOPE			-0.6875
#define OFFET			5.1276f
#define R0				10000.0f
#define T0				298.15f
#define BETA			3950.0f


char stringOut[50];

void vdg_UART_TxString(char strOut[]){
	for (uint8_t idx =0; strOut[idx] != '\0' ; idx++){
			while ((USART2-> SR & USART_SR_TXE) == 0);
			USART2 -> DR = strOut[idx];
		}
}



int main(void) {
	GPIO_Init_Custom(); //Initiate Touch Sensor Function


	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;

	GPIOA->MODER &=  ~(GPIO_MODER_MODER5);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5);

	//set up GPIO PA2,PA3
	GPIOA -> MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
	GPIOA -> MODER |= (0b10 << GPIO_MODER_MODER2_Pos) + (0b10 << GPIO_MODER_MODER3_Pos);
	GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
	GPIOA -> AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL2_Pos) + (0b0111 << GPIO_AFRL_AFSEL3_Pos);

	//Set up UASRT
	USART2 -> CR1 |= USART_CR1_UE;
	USART2 -> CR1 &= ~USART_CR1_M;
	USART2 -> CR2 &= ~USART_CR2_STOP;
	USART2 -> BRR = 139;
	USART2 -> CR1 |= USART_CR1_TE;

	//ADC channel 1 setup
	GPIOA -> MODER &= ~(GPIO_MODER_MODER0);
	GPIOA -> MODER |= (0b11 << GPIO_MODER_MODER0_Pos);

	//ADC channel 0 setup
	GPIOA -> MODER &= ~(GPIO_MODER_MODER1);
	GPIOA -> MODER |= (0b11 << GPIO_MODER_MODER1_Pos);


	ADC1 -> CR2 |= ADC_CR2_ADON;
	ADC1 -> SMPR2 |= ADC_SMPR2_SMP0;
	ADC1 -> SQR1 &= ~(ADC_SQR1_L);
	ADC1 -> SQR1 |= (1 << ADC_SQR1_L_Pos);
	ADC1 -> SQR3 &= ~(ADC_SQR3_SQ1);
	ADC1 -> SQR3 |= (1 << ADC_SQR3_SQ1_Pos );

	//Enable FPU
	SCB -> CPACR |= (0b1111 << 20);
	__asm volatile("dsb");
	__asm volatile("isb");

	while(1){
		ADC1 -> CR2 |= ADC_CR2_SWSTART;
		while((ADC1->SR & ADC_SR_EOC) == 0);

		float adc_voltage = (ADC1->DR * VREF) / ADC_MAXERS;
		float r_ldr = RX * adc_voltage / (VREF - adc_voltage);
		float lightintensity = pow(10,(log10(r_ldr) - OFFET)/SLOPE) ;

		float r_ntc = RX * adc_voltage/ ( VCC - adc_voltage);
		float temperature = ((BETA * T0)/(T0*log(r_ntc/R0 )+BETA)) - 273.15f;

		sprintf(stringOut, "Light intensity = %d  Lux\n", (uint32_t)lightintensity);
		vdg_UART_TxString(stringOut);
		sprintf(stringOut, "Temperature = %d millidegree Celcius\n", (int32_t)(temperature * 1000.0f));
		vdg_UART_TxString(stringOut);

		if(lightintensity < 20){
			GPIOA->ODR |= GPIO_ODR_OD5;
		}else{
			GPIOA->ODR &= ~GPIO_ODR_OD5;
		}
		for (uint32_t iter=0; iter< THRESHOLD; iter++){
								}
	}
}


