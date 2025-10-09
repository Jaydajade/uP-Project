#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* ==== Config ==== */
static const uint32_t DEBOUNCE_DELAY   = 500;
static const uint32_t MULTI_WINDOW     = 250000;
static const uint32_t BLINK_INTERVAL   = 8000;

/* ==== Utils ==== */
static inline void delay_cycles(volatile uint32_t n){ while(n--) __NOP(); }
static inline uint8_t read_btn(volatile uint32_t *idr, uint32_t mask){ return (*idr & mask) ? 1u : 0u; }
static inline uint8_t falling_edge(uint8_t *last, uint8_t now, uint32_t debounce){
    uint8_t hit = (*last == 1u && now == 0u);
    *last = now;
    if (hit) delay_cycles(debounce);
    return hit;
}

/* ==== 7-seg: PC7, PA8, PA9, PB10 ==== */
static inline void seg_off_all(void){
    GPIOC->BSRR = GPIO_BSRR_BR7;
    GPIOA->BSRR = GPIO_BSRR_BR8 | GPIO_BSRR_BR9;
    GPIOB->BSRR = GPIO_BSRR_BR10;
}
static inline void seg_show(uint8_t num){
    seg_off_all();
    switch(num){
        case 0: break;
        case 1: GPIOC->ODR |= (1<<7); break;
        case 2: GPIOA->ODR |= (1<<8); break;
        case 3: GPIOA->ODR |= (1<<8); GPIOC->ODR |= (1<<7); break;
        case 4: GPIOB->ODR |= (1<<10); break;
        case 5: GPIOB->ODR |= (1<<10); GPIOC->ODR |= (1<<7); break;
        case 6: GPIOA->ODR |= (1<<8); GPIOB->ODR |= (1<<10); break;
        case 7: GPIOA->ODR |= (1<<8); GPIOB->ODR |= (1<<10); GPIOC->ODR |= (1<<7); break;
        case 8: GPIOA->ODR |= (1<<9); break;
        case 9: GPIOA->ODR |= (1<<9); GPIOC->ODR |= (1<<7); break;
        default: break;
    }
}

/* ==== ADC1 CH4 @ PA4 ==== */
static inline void adc1_init(void){
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos);   // PA4 analog
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4;
    ADC1->CR2   |= ADC_CR2_ADON;
    ADC1->SMPR2 |= ADC_SMPR2_SMP4;
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 4; // ch4
}
static inline uint16_t adc1_read(void){
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}
static inline uint8_t adc_to_digit(uint16_t v){
    uint8_t d = (uint8_t)((v * 10u) / 4096u);
    return (d > 9) ? 9 : d;
}

/* ==== GPIO init ==== */
static inline void gpio_init_all(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // PA10 (pull-up)
    GPIOA->MODER &= ~GPIO_MODER_MODE10;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR |=  (0b01 << GPIO_PUPDR_PUPD10_Pos);

    // PB3, PB4, PB5 (pull-up)
    GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |=  (0b01 << GPIO_PUPDR_PUPD3_Pos) |
                     (0b01 << GPIO_PUPDR_PUPD4_Pos) |
                     (0b01 << GPIO_PUPDR_PUPD5_Pos);

    // LEDs PA5/6/7
    GPIOA->MODER   &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER   |=  (0b01 << GPIO_MODER_MODE5_Pos) |
                       (0b01 << GPIO_MODER_MODE6_Pos) |
                       (0b01 << GPIO_MODER_MODE7_Pos);
    GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);

    // PB6 master LED
    GPIOB->MODER   &= ~GPIO_MODER_MODE6;
    GPIOB->MODER   |=  (0b01 << GPIO_MODER_MODE6_Pos);

    // 7-seg: PC7, PA8, PA9, PB10
    GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOA->MODER |=  (0b01 << GPIO_MODER_MODE8_Pos) | (0b01 << GPIO_MODER_MODE9_Pos);
    GPIOB->MODER &= ~GPIO_MODER_MODE10;
    GPIOB->MODER |=  (0b01 << GPIO_MODER_MODE10_Pos);
    GPIOC->MODER &= ~GPIO_MODER_MODE7;
    GPIOC->MODER |=  (0b01 << GPIO_MODER_MODE7_Pos);

    // clear outputs
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
    GPIOB->BSRR = GPIO_BSRR_BR6;
    seg_off_all();
}

/* ==== LED helpers ==== */
static inline void leds_all_off(void){
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
}
static inline void led_on(uint8_t pin){
    if(pin==5)      GPIOA->BSRR = GPIO_BSRR_BS5;
    else if(pin==6) GPIOA->BSRR = GPIO_BSRR_BS6;
    else if(pin==7) GPIOA->BSRR = GPIO_BSRR_BS7;
}
static inline void led_toggle(uint8_t pin){
    if(pin==5)      GPIOA->ODR ^= GPIO_ODR_OD5;
    else if(pin==6) GPIOA->ODR ^= GPIO_ODR_OD6;
    else if(pin==7) GPIOA->ODR ^= GPIO_ODR_OD7;
}

/* ==== Main ==== */
int main(void){
    gpio_init_all();
    adc1_init();

    uint8_t  master_on = 0;
    uint8_t  pa10_last = 1, pb5_last = 1, pb3_last = 1, pb4_last = 1;

    uint8_t  active_led = 0;
    uint8_t  held_led   = 0;

    uint8_t  click_count = 0;
    uint32_t multi_timer = 0;
    uint32_t blink_timer = 0;

    /* NEW: โหมดสลับการแสดงผล 0/ADC ด้วย PB4 */
    uint8_t  show_zero = 0;   // 0 = โชว์ ADC, 1 = ค้างเลข 0

    while(1){
        /* Master: PA10 */
        {
            uint8_t now = read_btn(&GPIOA->IDR, GPIO_IDR_ID10);
            if (falling_edge(&pa10_last, now, DEBOUNCE_DELAY)){
                master_on ^= 1u;
                if (master_on){
                    GPIOB->BSRR = GPIO_BSRR_BS6;
                }else{
                    GPIOB->BSRR = GPIO_BSRR_BR6;
                    leds_all_off();
                    active_led = 0; held_led = 0;
                    click_count = 0; multi_timer = 0; blink_timer = 0;
                    seg_off_all();
                    show_zero = 0;  // รีเซ็ตโหมดโชว์ 0 เมื่อปิด master
                }
            }
        }
        if (!master_on) continue;

        /* PB4: toggle โหมดโชว์ 0 */
        {
            uint8_t now = read_btn(&GPIOB->IDR, GPIO_IDR_ID4);
            if (falling_edge(&pb4_last, now, DEBOUNCE_DELAY)){
                show_zero ^= 1u;  // กดครั้งแรก -> 1 (โชว์ 0), กดครั้งต่อมา -> 0 (กลับโชว์ ADC)
            }
        }

        /* HOLD mode: PB3 เพื่อ resume */
        if (held_led != 0){
            led_on(held_led);
            uint8_t now = read_btn(&GPIOB->IDR, GPIO_IDR_ID3);
            if (falling_edge(&pb3_last, now, DEBOUNCE_DELAY)){
                active_led = held_led;
                held_led   = 0;
                blink_timer = 0;
            }
            /* แสดง 7-seg */
            if (show_zero) seg_show(0);
            else           seg_show(adc_to_digit(adc1_read()));
            continue;
        }

        /* PB5: multi-click เลือก LED */
        {
            uint8_t now = read_btn(&GPIOB->IDR, GPIO_IDR_ID5);
            if (falling_edge(&pb5_last, now, DEBOUNCE_DELAY)){
                click_count++;
                multi_timer = MULTI_WINDOW;
            }
            if (multi_timer > 0){
                multi_timer--;
                if (multi_timer == 0 && click_count > 0){
                    leds_all_off();
                    uint8_t n = (click_count > 3) ? 3 : click_count;
                    active_led = (n==1)?5:(n==2)?6:7;
                    blink_timer = 0;
                    click_count = 0;
                }
            }
        }

        /* กระพริบ LED */
        if (active_led != 0){
            if (blink_timer == 0){
                led_toggle(active_led);
                blink_timer = BLINK_INTERVAL;
            }else{
                blink_timer--;
            }
        }

        /* PB3: เข้าสู่ hold (ถ้ายังไม่ hold) */
        {
            uint8_t now = read_btn(&GPIOB->IDR, GPIO_IDR_ID3);
            if (falling_edge(&pb3_last, now, DEBOUNCE_DELAY)){
                if (active_led != 0){
                    led_on(active_led);
                    held_led = active_led;
                    active_led = 0;
                    blink_timer = 0;
                }
            }
        }

        /* 7-seg แสดงตามโหมด */
        if (show_zero) seg_show(0);
        else           seg_show(adc_to_digit(adc1_read()));
    }
}
