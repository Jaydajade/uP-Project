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

//Touch Sensor Part
#define TOUCH_PIN   0   // PC0 (Touch Sensor D0)
#define LED_PIN     5   // PA5 (Onboard LED)
#define THRESHOLD   100000

//Buzzer_pin
#define BUZZER_PIN  1   // PC1


char stringOut[50];

//Buzzer Function

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

// Melody Function

void Melody(void) {
	// Samsung washing machine end tune (Die Forelle - Schubert)
	// Buzzer tone version by ChatGPT (GPT-5)

	beep(7840, 180);  delay(50);    // G5
	beep(9880, 180);  delay(50);    // B5
	beep(10470, 220); delay(60);    // C6
	beep(13190, 250); delay(60);    // E6
	beep(15680, 250); delay(80);    // G6

	beep(13970, 220); delay(50);    // F6
	beep(11750, 200); delay(50);    // D6
	beep(10470, 250); delay(100);   // C6

	beep(7840, 180);  delay(50);    // G5
	beep(9880, 180);  delay(50);    // B5
	beep(11750, 200); delay(60);    // D6
	beep(13190, 220); delay(60);    // E6
	beep(10470, 250); delay(100);   // C6

	beep(7840, 200);  delay(50);    // G5
	beep(6590, 200);  delay(50);    // E5
	beep(5230, 350);  delay(120);   // C5



	delay(4000); // wait before repeating
}
//Touch Function
void GPIO_Init_Custom(void) {
    // เปิด clock GPIOA และ GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // LED (PA5)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // Touch sensor (PC0)

    //Buzzer Pin
    // PC1 = output
   GPIOC->MODER &= ~(0b11 << (BUZZER_PIN * 2));
   GPIOC->MODER |=  (0b01 << (BUZZER_PIN * 2));

    // PC0 = Input mode
    GPIOC->MODER &= ~(0b11 << (TOUCH_PIN * 2));

    // PA5 = Output mode
    GPIOA->MODER &= ~(0b11 << (LED_PIN * 2));
    GPIOA->MODER |=  (0b01 << (LED_PIN * 2));
}


//Showing things on coolterm
void vdg_UART_TxString(char strOut[]){
	for (uint8_t idx =0; strOut[idx] != '\0' ; idx++){
			while ((USART2-> SR & USART_SR_TXE) == 0);
			USART2 -> DR = strOut[idx];
		}
}

int main(void) {
	GPIO_Init_Custom(); //Initiate Touch Sensor Function
	Melody();

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

		//foor touch sensor
		uint8_t touchState = (GPIOC->IDR & (1 << TOUCH_PIN)) ? 1 : 0;

		        // แสดงค่าใน CoolTerm
		 sprintf(stringOut, "Touch = %d\r\n", touchState);
		 vdg_UART_TxString(stringOut);

		        // ถ้าแตะ → LED ติด, ถ้าไม่แตะ → LED ดับ
		if(touchState == 1){
		      GPIOA->ODR |= (1 << LED_PIN);
		}else{
		      GPIOA->ODR &= ~(1 << LED_PIN);
		}

		for (uint32_t iter=0; iter< THRESHOLD; iter++){
								}
	}
}


