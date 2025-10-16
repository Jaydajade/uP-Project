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

/* ===== Manual clock ===== */
#define CPU_HZ 16000000u
uint32_t SystemCoreClock = CPU_HZ;
static volatile uint32_t g_ms = 0;
void SysTick_Handler(void) {
	g_ms++;
}

static inline void systick_init_ms(void) {
	SysTick->LOAD = (CPU_HZ / 1000u) - 1u;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
			| SysTick_CTRL_ENABLE_Msk;
}

/* ===== Config ===== */
static const uint32_t DEBOUNCE_DELAY = 500;
static const uint32_t MULTI_WINDOW = 250000;
static const uint32_t BLINK_INTERVAL = 8000;
static const uint32_t BTN_SHORT_DELAY = 160000;
static const uint32_t COUNTDOWN_STEP_MS = 1000;

/* ===== Utils ===== */
#define BIT(n) (1u << (n))
static inline void delay_cycles(volatile uint32_t n) {
	while (n--)
		__NOP();
}
static inline uint8_t read_btn_idr(volatile uint32_t *idr, uint32_t bit) {
	return (*idr & bit) ? 1u : 0u;
}
static inline uint8_t falling_edge(uint8_t *last, uint8_t now,
		uint32_t debounce) {
	uint8_t hit = (*last == 1u && now == 0u);
	*last = now;
	if (hit)
		delay_cycles(debounce);
	return hit;
}

/* ===== 7-segment: PC7, PA8, PA9, PB10 ===== */
static inline void seg_off_all(void) {
	GPIOC->BSRR = GPIO_BSRR_BR7;
	GPIOA->BSRR = GPIO_BSRR_BR8 | GPIO_BSRR_BR9;
	GPIOB->BSRR = GPIO_BSRR_BR10;
}

static inline void seg_show(uint8_t num) {
	seg_off_all();
	switch (num) {
	case 0:
		break;
	case 1:
		GPIOC->BSRR = GPIO_BSRR_BS7;
		break;
	case 2:
		GPIOA->BSRR = GPIO_BSRR_BS8;
		break;
	case 3:
		GPIOA->BSRR = GPIO_BSRR_BS8;
		GPIOC->BSRR = GPIO_BSRR_BS7;
		break;
	case 4:
		GPIOB->BSRR = GPIO_BSRR_BS10;
		break;
	case 5:
		GPIOB->BSRR = GPIO_BSRR_BS10;
		GPIOC->BSRR = GPIO_BSRR_BS7;
		break;
	case 6:
		GPIOA->BSRR = GPIO_BSRR_BS8;
		GPIOB->BSRR = GPIO_BSRR_BS10;
		break;
	case 7:
		GPIOA->BSRR = GPIO_BSRR_BS8;
		GPIOB->BSRR = GPIO_BSRR_BS10;
		GPIOC->BSRR = GPIO_BSRR_BS7;
		break;
	case 8:
		GPIOA->BSRR = GPIO_BSRR_BS9;
		break;
	case 9:
		GPIOA->BSRR = GPIO_BSRR_BS9;
		GPIOC->BSRR = GPIO_BSRR_BS7;
		break;
	default:
		break;
	}
}

/* ===== ADC1 CH4 @ PA4 ===== */
static inline void adc1_init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	GPIOA->MODER |= (0b11 << GPIO_MODER_MODE4_Pos);
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4;

	ADC1->CR1 = 0;
	ADC1->CR2 = 0;
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP4;
	ADC1->SMPR2 |= ADC_SMPR2_SMP4_2;
	ADC1->SQR1 = 0;
	ADC1->SQR3 = 4;
	ADC1->CR2 |= ADC_CR2_EOCS | ADC_CR2_ADON;
	delay_cycles(3000);
}
static inline uint16_t adc1_read(void) {
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while (!(ADC1->SR & ADC_SR_EOC))
		;
	return ADC1->DR;
}
static inline uint8_t adc_to_digit(uint16_t v) {
	uint8_t d = (uint8_t) ((v * 10u) / 4096u);
	return (d > 9) ? 9 : d;
}

/* ===== GPIO init ===== */
static inline void gpio_init_all(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
			| RCC_AHB1ENR_GPIOCEN;

	GPIOA->MODER &= ~GPIO_MODER_MODE10;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
	GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos);

	GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);
	GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD3_Pos)
			| (0b01 << GPIO_PUPDR_PUPD4_Pos) | (0b01 << GPIO_PUPDR_PUPD5_Pos);

	GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODE5_Pos)
			| (0b01 << GPIO_MODER_MODE6_Pos) | (0b01 << GPIO_MODER_MODE7_Pos);

	GPIOB->MODER &= ~GPIO_MODER_MODE6;
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE6_Pos);

	GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODE8_Pos)
			| (0b01 << GPIO_MODER_MODE9_Pos);
	GPIOB->MODER &= ~GPIO_MODER_MODE10;
	GPIOB->MODER |= (0b01 << GPIO_MODER_MODE10_Pos);
	GPIOC->MODER &= ~GPIO_MODER_MODE7;
	GPIOC->MODER |= (0b01 << GPIO_MODER_MODE7_Pos);

	GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
	GPIOB->BSRR = GPIO_BSRR_BR6;
	seg_off_all();
}

/* ===== LED helpers ===== */
static inline void leds_all_off(void) {
	GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
}
static inline void led_on(uint8_t pin) {
	GPIOA->BSRR = BIT(pin);
}
static inline void led_toggle(uint8_t pin) {
	GPIOA->ODR ^= BIT(pin);
}

//Touch Sensor Part
#define TOUCH_PIN   0   // PC0 (Touch Sensor D0)
#define LED_PIN     5   // PA5 (Onboard LED)
#define THRESHOLD   100000

//Buzzer_pin
#define BUZZER_PIN  1   // PC1

char stringOut[50];

//Buzzer Function

void delay(volatile uint32_t d) {
	while (d--)
		;
}

void beep(uint32_t frequency, uint32_t duration) {
	uint32_t period = 16000000 / frequency; // SYSCLK=16MHz assumed
	uint32_t half_period = period / 2;
	uint32_t cycles = (frequency * duration) / 1000; // duration in ms

	for (uint32_t i = 0; i < cycles; i++) {
		GPIOC->ODR ^= (1 << BUZZER_PIN);  // toggle PC1
		delay(half_period);
	}
}

// Melody Function

void Melody(void) {
	// Samsung washing machine end tune (Die Forelle - Schubert)
	// Buzzer tone version by ChatGPT (GPT-5)

	beep(7840, 180);
	delay(50);    // G5
	beep(9880, 180);
	delay(50);    // B5
	beep(10470, 220);
	delay(60);    // C6
	beep(13190, 250);
	delay(60);    // E6
	beep(15680, 250);
	delay(80);    // G6

	beep(13970, 220);
	delay(50);    // F6
	beep(11750, 200);
	delay(50);    // D6
	beep(10470, 250);
	delay(100);   // C6

	beep(7840, 180);
	delay(50);    // G5
	beep(9880, 180);
	delay(50);    // B5
	beep(11750, 200);
	delay(60);    // D6
	beep(13190, 220);
	delay(60);    // E6
	beep(10470, 250);
	delay(100);   // C6

	beep(7840, 200);
	delay(50);    // G5
	beep(6590, 200);
	delay(50);    // E5
	beep(5230, 350);
	delay(120);   // C5

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
	GPIOC->MODER |= (0b01 << (BUZZER_PIN * 2));

	// PC0 = Input mode
	GPIOC->MODER &= ~(0b11 << (TOUCH_PIN * 2));

	// PA5 = Output mode
	GPIOA->MODER &= ~(0b11 << (LED_PIN * 2));
	GPIOA->MODER |= (0b01 << (LED_PIN * 2));
}

//Showing things on coolterm
void vdg_UART_TxString(char strOut[]) {
	for (uint8_t idx = 0; strOut[idx] != '\0'; idx++) {
		while ((USART2->SR & USART_SR_TXE) == 0)
			;
		USART2->DR = strOut[idx];
	}
}

int main(void) {
	GPIO_Init_Custom(); //Initiate Touch Sensor Function
	gpio_init_all();
	adc1_init();
	systick_init_ms();

	uint8_t master_on = 0;
	uint8_t pa10_last = 1, pb5_last = 1, pb3_last = 1, pb4_last = 1;

	uint8_t active_led = 0;
	uint8_t held_led = 0;
	uint8_t pb4_zero_mode = 0;

	uint8_t click_count = 0;
	uint32_t multi_timer = 0;
	uint32_t blink_timer = 0;

	uint8_t cd_active = 0;
	int8_t cd_value = 0;
	uint32_t cd_next_ms = 0;

	uint8_t seg_digit_live = 0;

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	GPIOA->MODER &= ~(GPIO_MODER_MODER5);
	GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5);

	//set up GPIO PA2,PA3
	GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
	GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos)
			+ (0b10 << GPIO_MODER_MODER3_Pos);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
	GPIOA->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL2_Pos)
			+ (0b0111 << GPIO_AFRL_AFSEL3_Pos);

	//Set up UASRT
	USART2->CR1 |= USART_CR1_UE;
	USART2->CR1 &= ~USART_CR1_M;
	USART2->CR2 &= ~USART_CR2_STOP;
	USART2->BRR = 139;
	USART2->CR1 |= USART_CR1_TE;

	//ADC channel 1 setup
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->MODER |= (0b11 << GPIO_MODER_MODER0_Pos);

	//ADC channel 0 setup
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	GPIOA->MODER |= (0b11 << GPIO_MODER_MODER1_Pos);

	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->SMPR2 |= ADC_SMPR2_SMP0;
	ADC1->SQR1 &= ~(ADC_SQR1_L);
	ADC1->SQR1 |= (1 << ADC_SQR1_L_Pos);
	ADC1->SQR3 &= ~(ADC_SQR3_SQ1);
	ADC1->SQR3 |= (1 << ADC_SQR3_SQ1_Pos);

	//Enable FPU
	SCB->CPACR |= (0b1111 << 20);
	__asm volatile("dsb");
	__asm volatile("isb");

	while (1) {
		GPIO_Init_Custom();

		uint8_t nowA10 = read_btn_idr(&GPIOA->IDR, BIT(10));
		if (falling_edge(&pa10_last, nowA10, DEBOUNCE_DELAY)) {
			master_on ^= 1u;
			if (master_on) {
				GPIOB->BSRR = GPIO_BSRR_BS6;
			} else {
				GPIOB->BSRR = GPIO_BSRR_BR6;
				leds_all_off();
				seg_off_all();
				active_led = held_led = 0;
				cd_active = 0;
				pb4_zero_mode = 0;
			}
		}
		if (!master_on)
			continue;

		/* PB5: multi-click เลือกไฟ */
		uint8_t now5 = read_btn_idr(&GPIOB->IDR, BIT(5));
		if (falling_edge(&pb5_last, now5, DEBOUNCE_DELAY)) {
			click_count++;
			multi_timer = MULTI_WINDOW;
		}
		if (multi_timer > 0) {
			multi_timer--;
			if (multi_timer == 0 && click_count > 0) {
				leds_all_off();
				uint8_t n = (click_count > 3) ? 3 : click_count;
				active_led = (n == 1) ? 5 : (n == 2) ? 6 : 7;
				blink_timer = 0;
				click_count = 0;
			}
		}

		if (active_led && !held_led && !cd_active) {
			if (blink_timer == 0) {
				led_toggle(active_led);
				blink_timer = BLINK_INTERVAL;
			} else
				blink_timer--;
		}

		/* PB4 toggle แสดงเลข 0 ↔ กลับเลขเดิม */
		uint8_t now4 = read_btn_idr(&GPIOB->IDR, BIT(4));
		if (falling_edge(&pb4_last, now4, DEBOUNCE_DELAY)) {
			if (!cd_active) {
				pb4_zero_mode ^= 1;
				if (pb4_zero_mode) {
					seg_show(0);
					leds_all_off();
				} else
					seg_show(seg_digit_live);
			}
		}
		pb4_last = now4;

		/* PB3 toggle countdown */
		uint8_t now3 = read_btn_idr(&GPIOB->IDR, BIT(3));
		if (falling_edge(&pb3_last, now3, DEBOUNCE_DELAY)) {
			if (!cd_active && active_led && !pb4_zero_mode) {
				led_on(active_led);
				held_led = active_led;
				active_led = 0;
				cd_value = (int8_t) seg_digit_live;
				cd_active = 1;
				cd_next_ms = g_ms + COUNTDOWN_STEP_MS;
			} else if (cd_active) {
				cd_active = 0;
				active_led = held_led;
				held_led = 0;
				blink_timer = 0;
			}
		}

		/* Countdown */
		if (cd_active) {
			seg_show((uint8_t) cd_value);
			if ((int32_t) (g_ms - cd_next_ms) >= 0) {
				if (cd_value > 0) {
					cd_value--;
					cd_next_ms += COUNTDOWN_STEP_MS;
				} else {
					// ==== เมื่อถึง 0 ====
					seg_show(0);
					delay_cycles(CPU_HZ * 5 / 8);  // ค้างเลข 0 ประมาณ 5 วิ
					leds_all_off(); // ดับไฟทั้งหมด
					Melody(); //เพลงติด
					// กลับไปโหมดกระพริบไฟเดิม
					cd_active = 0;
					active_led = held_led;  // คืนไฟที่เลือกไว้ก่อน countdown
					held_led = 0;
					blink_timer = 0;
				}
			}
			continue;
		}

		/* โหมดปกติ */
		if (!pb4_zero_mode) {
			seg_digit_live = adc_to_digit(adc1_read());
			seg_show(seg_digit_live);
		}
	}

	ADC1->CR2 |= ADC_CR2_SWSTART;
	while ((ADC1->SR & ADC_SR_EOC) == 0)
		;

	float adc_voltage = (ADC1->DR * VREF) / ADC_MAXERS;
	float r_ldr = RX * adc_voltage / (VREF - adc_voltage);
	float lightintensity = pow(10, (log10(r_ldr) - OFFET) / SLOPE);

	float r_ntc = RX * adc_voltage / ( VCC - adc_voltage);
	float temperature = ((BETA * T0) / (T0 * log(r_ntc / R0) + BETA)) - 273.15f;

	sprintf(stringOut, "Light intensity = %d  Lux\n",
			(uint32_t) lightintensity);
	vdg_UART_TxString(stringOut);
	sprintf(stringOut, "Temperature = %d millidegree Celcius\n",
			(int32_t) (temperature * 1000.0f));
	vdg_UART_TxString(stringOut);

	if (lightintensity < 20) {
		//มืดแปลว่ามีสิ่งของติดในเครื่อง ให้หยุดทำงานชั่วคราว
		cd_active = 0;
		active_led = held_led;
		held_led = 0;
		blink_timer = 0;
	}

	//ยังไม่แน่ใจว่าค่าเท่าไหร่
	if (temperature < 20) {
		//ถ้าอุณหภูมิน้ำสูงเกินไปให้หยุดการทำงานชั่วคราว
		cd_active = 0;
		active_led = held_led;
		held_led = 0;
		blink_timer = 0;
	}
	//foor touch sensor
	uint8_t touchState = (GPIOC->IDR & (1 << TOUCH_PIN)) ? 1 : 0;

	// แสดงค่าใน CoolTerm
	sprintf(stringOut, "Touch = %d\r\n", touchState);
	vdg_UART_TxString(stringOut);

	// ถ้าแตะเซ็นเซอร์คือมีคนมาเปิดฝาให้หยุดการทำงานชั่วคราว , ถ้าไม่แตะก็คือฝายังปิดอยู่ทำงานต่อได้
	if (touchState == 1) {
		//เข้าใจว่าด้านล่างนี้เป็นคำสั่งที่ทำให้เครื่องหยุดชั่วคราว
		cd_active = 0;
		active_led = held_led;
		held_led = 0;
		blink_timer = 0;
	}

	for (uint32_t iter = 0; iter < THRESHOLD; iter++) {
	}
}

