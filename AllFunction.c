#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* ===== Configuration ===== */
#define CPU_HZ 16000000u
#define VREF 3.3f
#define VCC 3.3f
#define ADC_MAXERS 4095.0f
#define RX 10000.0f
#define SLOPE -0.6875
#define OFFET 5.1276f
#define R0 10000.0f
#define T0 298.15f
#define BETA 3950.0f

// Touch Sensor
#define TOUCH_PIN 0
#define LED_PIN 5
#define BUZZER_PIN 1

// System timing
#define DEBOUNCE_DELAY 500
#define MULTI_WINDOW 250000
#define BLINK_INTERVAL 8000
#define BTN_SHORT_DELAY 160000
#define COUNTDOWN_STEP_MS 1000
#define THRESHOLD 100000

// Temperature and light thresholds
#define TEMP_THRESHOLD 20.0f
#define LIGHT_THRESHOLD 20.0f

/* ===== Global Variables ===== */
uint32_t SystemCoreClock = CPU_HZ;
static volatile uint32_t g_ms = 0;
char stringOut[100];

// Sensor data (shared between main loop and interrupt check)
static volatile float g_temperature = 25.0f;
static volatile float g_lightintensity = 100.0f;
static volatile uint8_t g_touchState = 0;
static volatile uint8_t g_interrupt_occurred = 0;
static volatile int8_t g_paused_countdown = -1;

/* ===== Utility Macros ===== */
#define BIT(n) (1u << (n))

/* ===== SysTick Handler ===== */
void SysTick_Handler(void){
    g_ms++;
}

static inline void systick_init_ms(void){
    SysTick->LOAD = (CPU_HZ/1000u) - 1u;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

/* ===== Delay Function ===== */
static inline void delay_cycles(volatile uint32_t n){
    while(n--) __NOP();
}

void delay(volatile uint32_t d) {
    while(d--);
}

/* ===== Buzzer Functions ===== */
void beep(uint32_t frequency, uint32_t duration) {
    uint32_t period = 16000000 / frequency;
    uint32_t half_period = period / 2;
    uint32_t cycles = (frequency * duration) / 1000;

    for(uint32_t i=0; i<cycles; i++) {
        GPIOC->ODR ^= (1 << BUZZER_PIN);
        delay(half_period);
    }
}

void Melody(void) {
    // Samsung washing machine end tune
    beep(7840, 180); delay(50);
    beep(9880, 180); delay(50);
    beep(10470, 220); delay(60);
    beep(13190, 250); delay(60);
    beep(15680, 250); delay(80);
    beep(13970, 220); delay(50);
    beep(11750, 200); delay(50);
    beep(10470, 250); delay(100);
    beep(7840, 180); delay(50);
    beep(9880, 180); delay(50);
    beep(11750, 200); delay(60);
    beep(13190, 220); delay(60);
    beep(10470, 250); delay(100);
    beep(7840, 200); delay(50);
    beep(6590, 200); delay(50);
    beep(5230, 350); delay(120);
    delay(4000);
}

/* ===== 7-Segment Display ===== */
static inline void seg_off_all(void){
    GPIOC->BSRR = GPIO_BSRR_BR7;
    GPIOA->BSRR = GPIO_BSRR_BR8 | GPIO_BSRR_BR9;
    GPIOB->BSRR = GPIO_BSRR_BR10;
}

static inline void seg_show(uint8_t num){
    seg_off_all();
    switch(num){
        case 0: break;
        case 1: GPIOC->BSRR = GPIO_BSRR_BS7; break;
        case 2: GPIOA->BSRR = GPIO_BSRR_BS8; break;
        case 3:
            GPIOA->BSRR = GPIO_BSRR_BS8;
            GPIOC->BSRR = GPIO_BSRR_BS7;
            break;
        case 4: GPIOB->BSRR = GPIO_BSRR_BS10; break;
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
        case 8: GPIOA->BSRR = GPIO_BSRR_BS9; break;
        case 9:
            GPIOA->BSRR = GPIO_BSRR_BS9;
            GPIOC->BSRR = GPIO_BSRR_BS7;
            break;
        default: break;
    }
}

/* ===== Button Reading ===== */
static inline uint8_t read_btn_idr(volatile uint32_t *idr, uint32_t bit){
    return (*idr & bit) ? 1u : 0u;
}

static inline uint8_t falling_edge(uint8_t *last, uint8_t now, uint32_t debounce){
    uint8_t hit = (*last == 1u && now == 0u);
    *last = now;
    if (hit) delay_cycles(debounce);
    return hit;
}

/* ===== LED Helpers ===== */
static inline void leds_all_off(void){
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
}

static inline void led_on(uint8_t pin){
    GPIOA->BSRR = BIT(pin);
}

static inline void led_toggle(uint8_t pin){
    GPIOA->ODR ^= BIT(pin);
}

/* ===== UART Functions ===== */
void vdg_UART_TxString(char strOut[]){
    for (uint8_t idx =0; strOut[idx] != '\0' ; idx++){
        while ((USART2->SR & USART_SR_TXE) == 0);
        USART2->DR = strOut[idx];
    }
}

/* ===== ADC Functions ===== */
static inline void adc1_init(void){
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // PA4 for potentiometer (ADC1 CH4)
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODE4_Pos);
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4;

    // PA0 for LDR (ADC1 CH0)
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODE0_Pos);

    // PA1 for NTC (ADC1 CH1)
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODE1_Pos);

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;
    ADC1->SMPR2 |= ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP4_2;
    ADC1->SQR1 = 0;
    ADC1->CR2 |= ADC_CR2_EOCS | ADC_CR2_ADON;
    delay_cycles(3000);
}

static inline uint16_t adc1_read_channel(uint8_t channel){
    ADC1->SQR3 = channel;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

static inline uint8_t adc_to_digit(uint16_t v){
    uint8_t d = (uint8_t)((v * 10u) / 4096u);
    return (d > 9) ? 9 : d;
}

/* ===== Sensor Reading Functions ===== */
void read_sensors(void){
    // Read LDR (Channel 1)
    uint16_t adc_ldr = adc1_read_channel(1);
    float adc_voltage = (adc_ldr * VREF) / ADC_MAXERS;
    float r_ldr = RX * adc_voltage / (VREF - adc_voltage);
    g_lightintensity = pow(10, (log10(r_ldr) - OFFET) / SLOPE);

    // Read NTC (Channel 1 - same pin alternating read)
    uint16_t adc_ntc = adc1_read_channel(1);
    adc_voltage = (adc_ntc * VREF) / ADC_MAXERS;
    float r_ntc = RX * adc_voltage / (VCC - adc_voltage);
    g_temperature = ((BETA * T0) / (T0 * log(r_ntc / R0) + BETA)) - 273.15f;

    // Read Touch Sensor
    g_touchState = (GPIOC->IDR & (1 << TOUCH_PIN)) ? 1 : 0;
}

void print_sensor_data(void){
    sprintf(stringOut, "Light: %d Lux\r\n", (uint32_t)g_lightintensity);
    vdg_UART_TxString(stringOut);

    sprintf(stringOut, "Temp: %d mC\r\n", (int32_t)(g_temperature * 1000.0f));
    vdg_UART_TxString(stringOut);

    sprintf(stringOut, "Touch: %d\r\n", g_touchState);
    vdg_UART_TxString(stringOut);
}

/* ===== Check Interrupt Conditions ===== */
uint8_t check_interrupt_conditions(void){
    return (g_touchState == 1) ||
           (g_lightintensity < LIGHT_THRESHOLD) ||
           (g_temperature < TEMP_THRESHOLD);
}

/* ===== GPIO Initialization ===== */
static inline void gpio_init_all(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;

    // Button inputs (PA10, PB3, PB4, PB5)
    GPIOA->MODER &= ~GPIO_MODER_MODE10;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos);

    GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD3_Pos) |
                    (0b01 << GPIO_PUPDR_PUPD4_Pos) |
                    (0b01 << GPIO_PUPDR_PUPD5_Pos);

    // LED outputs (PA5, PA6, PA7, PB6)
    GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODE5_Pos) |
                    (0b01 << GPIO_MODER_MODE6_Pos) |
                    (0b01 << GPIO_MODER_MODE7_Pos);
    GPIOB->MODER &= ~GPIO_MODER_MODE6;
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODE6_Pos);

    // 7-segment outputs (PA8, PA9, PB10, PC7)
    GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODE8_Pos) |
                    (0b01 << GPIO_MODER_MODE9_Pos);
    GPIOB->MODER &= ~GPIO_MODER_MODE10;
    GPIOB->MODER |= (0b01 << GPIO_MODER_MODE10_Pos);
    GPIOC->MODER &= ~GPIO_MODER_MODE7;
    GPIOC->MODER |= (0b01 << GPIO_MODER_MODE7_Pos);

    // Touch sensor input (PC0)
    GPIOC->MODER &= ~(0b11 << (TOUCH_PIN * 2));

    // Buzzer output (PC1)
    GPIOC->MODER &= ~(0b11 << (BUZZER_PIN * 2));
    GPIOC->MODER |= (0b01 << (BUZZER_PIN * 2));

    // Initialize all outputs to off
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
    GPIOB->BSRR = GPIO_BSRR_BR6;
    seg_off_all();
}

/* ===== UART Initialization ===== */
static inline void uart_init(void){
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2, PA3 for UART
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos) |
                    (0b10 << GPIO_MODER_MODER3_Pos);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL2_Pos) |
                     (0b0111 << GPIO_AFRL_AFSEL3_Pos);

    USART2->CR1 |= USART_CR1_UE;
    USART2->CR1 &= ~USART_CR1_M;
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->BRR = 139;
    USART2->CR1 |= USART_CR1_TE;
}

/* ===== Main Program ===== */
int main(void){
    // Enable FPU
    SCB->CPACR |= (0b1111 << 20);
    __asm volatile("dsb");
    __asm volatile("isb");

    // Initialize all peripherals
    gpio_init_all();
    uart_init();
    adc1_init();
    systick_init_ms();



    // State variables
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
    uint32_t sensor_read_counter = 0;

    while(1){
        // Read sensors periodically
        if(sensor_read_counter++ >= 5000){
            read_sensors();
            sensor_read_counter = 0;
            if(master_on){
                print_sensor_data();
            }
        }

        // Master ON/OFF button (PA10)
        uint8_t nowA10 = read_btn_idr(&GPIOA->IDR, BIT(10));
        if (falling_edge(&pa10_last, nowA10, DEBOUNCE_DELAY)){
            master_on ^= 1u;
            if (master_on){
                GPIOB->BSRR = GPIO_BSRR_BS6;
            }else{
                GPIOB->BSRR = GPIO_BSRR_BR6;
                leds_all_off();
                seg_off_all();
                active_led = held_led = 0;
                cd_active = 0;
                pb4_zero_mode = 0;
                g_interrupt_occurred = 0;
                g_paused_countdown = -1;
            }
        }

        if (!master_on) continue;

        /* ===== Check interrupt conditions ===== */
        if(cd_active && check_interrupt_conditions()){
            if(!g_interrupt_occurred){
                // First time interrupt detected - pause countdown
                g_interrupt_occurred = 1;
                g_paused_countdown = cd_value;  // เก็บค่า countdown ปัจจุบัน (เช่น 5)

                sprintf(stringOut, "*** INTERRUPT! Paused at: %d ***\r\n", cd_value);
                vdg_UART_TxString(stringOut);

                // แสดงค่าที่หยุดทันทีบน 7-segment
                seg_show((uint8_t)cd_value);
            }

            // Stop countdown and return to blinking mode
            cd_active = 0;
            active_led = held_led;
            held_led = 0;
            blink_timer = 0;

            // แสดงค่าที่หยุดค้างไว้ตลอดเวลาระหว่าง interrupt
            seg_show((uint8_t)g_paused_countdown);

            // LED ยังคงกระพริบต่อ
            if (active_led && !held_led){
                if (blink_timer == 0){
                    led_toggle(active_led);
                    blink_timer = BLINK_INTERVAL;
                } else {
                    blink_timer--;
                }
            }

            continue;
        }

        // If interrupt condition cleared, allow countdown to resume
        if(g_interrupt_occurred && !check_interrupt_conditions()){
            sprintf(stringOut, "*** Interrupt cleared ***\r\n");
            vdg_UART_TxString(stringOut);
            g_interrupt_occurred = 0;
            // Note: countdown stays paused until user restarts with PB3
        }

        /* ===== Show paused countdown value when interrupted ===== */
        if(g_paused_countdown >= 0 && !cd_active && !pb4_zero_mode){
            seg_show((uint8_t)g_paused_countdown);

            // Continue with LED blinking
            if (active_led && !held_led){
                if (blink_timer == 0){
                    led_toggle(active_led);
                    blink_timer = BLINK_INTERVAL;
                } else {
                    blink_timer--;
                }
            }
        }

        /* ===== PB5: Multi-click LED selection ===== */
        uint8_t now5 = read_btn_idr(&GPIOB->IDR, BIT(5));
        if (falling_edge(&pb5_last, now5, DEBOUNCE_DELAY)){
            click_count++;
            multi_timer = MULTI_WINDOW;
        }

        if (multi_timer > 0){
            multi_timer--;
            if (multi_timer == 0 && click_count > 0){
                leds_all_off();
                uint8_t n = (click_count > 3) ? 3 : click_count;
                active_led = (n==1) ? 5 : (n==2) ? 6 : 7;
                blink_timer = 0;
                click_count = 0;
            }
        }

        /* ===== LED blinking when active ===== */
        if (active_led && !held_led && !cd_active && g_paused_countdown < 0){
            if (blink_timer == 0){
                led_toggle(active_led);
                blink_timer = BLINK_INTERVAL;
            } else {
                blink_timer--;
            }
        }

        /* ===== PB4: Toggle show zero ===== */
        uint8_t now4 = read_btn_idr(&GPIOB->IDR, BIT(4));
        if (falling_edge(&pb4_last, now4, DEBOUNCE_DELAY)){
            if (!cd_active){
                pb4_zero_mode ^= 1;
                if (pb4_zero_mode){
                    seg_show(0);
                    leds_all_off();
                } else {
                    seg_show(seg_digit_live);
                }
            }
        }
        pb4_last = now4;

        /* ===== PB3: Toggle countdown ===== */
        uint8_t now3 = read_btn_idr(&GPIOB->IDR, BIT(3));
        if (falling_edge(&pb3_last, now3, DEBOUNCE_DELAY)){
            if (!cd_active && active_led && !pb4_zero_mode){
                // Start countdown
                led_on(active_led);
                held_led = active_led;
                active_led = 0;

                // If resuming from interrupt, use paused value
                if(g_paused_countdown >= 0){
                    cd_value = g_paused_countdown;
                    g_paused_countdown = -1;
                    sprintf(stringOut, "*** Resuming from: %d ***\r\n", cd_value);
                    vdg_UART_TxString(stringOut);
                } else {
                    cd_value = (int8_t)seg_digit_live;
                }

                cd_active = 1;
                cd_next_ms = g_ms + COUNTDOWN_STEP_MS;
            } else if (cd_active){
                // Stop countdown
                cd_active = 0;
                active_led = held_led;
                held_led = 0;
                blink_timer = 0;
                g_paused_countdown = -1;
            }
        }

        /* ===== Countdown Logic ===== */
        if (cd_active){
            seg_show((uint8_t)cd_value);

            if ((int32_t)(g_ms - cd_next_ms) >= 0){
                if (cd_value > 0){
                    cd_value--;
                    cd_next_ms += COUNTDOWN_STEP_MS;
                } else {
                    // ==== Countdown reached 0 ====
                    sprintf(stringOut, "*** COUNTDOWN COMPLETE! ***\r\n");
                    vdg_UART_TxString(stringOut);

                    seg_show(0);

                    // Play melody
                    Melody();

                    delay_cycles(CPU_HZ * 2);

                    leds_all_off();

                    // Return to blinking mode
                    cd_active = 0;
                    active_led = held_led;
                    held_led = 0;
                    blink_timer = 0;
                    g_paused_countdown = -1;
                }
            }
            continue;
        }

        /* ===== Normal mode: Read potentiometer and update display ===== */
        if (!pb4_zero_mode && g_paused_countdown < 0){
            // Only read potentiometer if not paused from interrupt
            seg_digit_live = adc_to_digit(adc1_read_channel(4));
            seg_show(seg_digit_live);
        } else if (pb4_zero_mode) {
            // Show zero when PB4 mode is active
            seg_show(0);
        }
    }
}
