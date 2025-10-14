#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* ==== Manual clock config (no system_stm32f4xx.c needed) ==== */
#define CPU_HZ 16000000u                 // HSI 16 MHz
uint32_t SystemCoreClock = CPU_HZ;       // prevent undefined reference

static volatile uint32_t g_ms = 0;
void SysTick_Handler(void){ g_ms++; }

static inline void systick_init_ms(void){
    SysTick->LOAD  = (CPU_HZ/1000u) - 1u;   // 1 ms tick
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk
                   | SysTick_CTRL_TICKINT_Msk
                   | SysTick_CTRL_ENABLE_Msk;
}

/* ==== Config ==== */
static const uint32_t DEBOUNCE_DELAY   = 500;
static const uint32_t MULTI_WINDOW     = 250000;
static const uint32_t BLINK_INTERVAL   = 8000;
static const uint32_t BTN_SHORT_DELAY  = 160000;
static const uint32_t COUNTDOWN_STEP_MS = 1000;

/* ==== Utils ==== */
#define BIT(n) (1u << (n))
static inline void delay_cycles(volatile uint32_t n){ while(n--) __NOP(); }
static inline uint8_t read_btn_idr(volatile uint32_t *idr, uint32_t bit){ return (*idr & bit) ? 1u : 0u; }
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
        case 1: GPIOC->BSRR = GPIO_BSRR_BS7; break;
        case 2: GPIOA->BSRR = GPIO_BSRR_BS8; break;
        case 3: GPIOA->BSRR = GPIO_BSRR_BS8; GPIOC->BSRR = GPIO_BSRR_BS7; break;
        case 4: GPIOB->BSRR = GPIO_BSRR_BS10; break;
        case 5: GPIOB->BSRR = GPIO_BSRR_BS10; GPIOC->BSRR = GPIO_BSRR_BS7; break;
        case 6: GPIOA->BSRR = GPIO_BSRR_BS8; GPIOB->BSRR = GPIO_BSRR_BS10; break;
        case 7: GPIOA->BSRR = GPIO_BSRR_BS8; GPIOB->BSRR = GPIO_BSRR_BS10; GPIOC->BSRR = GPIO_BSRR_BS7; break;
        case 8: GPIOA->BSRR = GPIO_BSRR_BS9; break;
        case 9: GPIOA->BSRR = GPIO_BSRR_BS9; GPIOC->BSRR = GPIO_BSRR_BS7; break;
        default: break;
    }
}

/* ==== ADC1 CH4 @ PA4 ==== */
static inline void adc1_init(void){
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODE4_Pos);
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4;

    ADC1->CR1 = 0; ADC1->CR2 = 0;
    ADC1->SMPR2 |= ADC_SMPR2_SMP4_2;
    ADC1->SQR1 = 0; ADC1->SQR3 = 4;
    ADC1->CR2 |= ADC_CR2_EOCS | ADC_CR2_ADON;
    delay_cycles(3000);
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

    GPIOA->MODER &= ~GPIO_MODER_MODE10;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPD10_Pos);

    GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD3_Pos)
                  | (0b01 << GPIO_PUPDR_PUPD4_Pos)
                  | (0b01 << GPIO_PUPDR_PUPD5_Pos);

    GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODE5_Pos)
                  | (0b01 << GPIO_MODER_MODE6_Pos)
                  | (0b01 << GPIO_MODER_MODE7_Pos);

    GPIOB->MODER &= ~GPIO_MODER_MODE6;
    GPIOB->MODER |=  (0b01 << GPIO_MODER_MODE6_Pos);

    GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOA->MODER |=  (0b01 << GPIO_MODER_MODE8_Pos) | (0b01 << GPIO_MODER_MODE9_Pos);
    GPIOB->MODER &= ~GPIO_MODER_MODE10;
    GPIOB->MODER |=  (0b01 << GPIO_MODER_MODE10_Pos);
    GPIOC->MODER &= ~GPIO_MODER_MODE7;
    GPIOC->MODER |=  (0b01 << GPIO_MODER_MODE7_Pos);

    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
    GPIOB->BSRR = GPIO_BSRR_BR6;
    seg_off_all();
}

/* ==== LED helpers ==== */
static inline void leds_all_off(void){
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
}
static inline void led_on(uint8_t pin){
    GPIOA->BSRR = BIT(pin);
}
static inline void led_toggle(uint8_t pin){
    GPIOA->ODR ^= BIT(pin);
}

/* ==== Main ==== */
int main(void){
    gpio_init_all();
    adc1_init();
    systick_init_ms();

    uint8_t master_on = 0;
    uint8_t pa10_last = 1, pb5_last = 1, pb3_last = 1, pb4_last = 1;

    uint8_t active_led = 0;
    uint8_t held_led   = 0;

    uint8_t click_count = 0;
    uint32_t multi_timer = 0;
    uint32_t blink_timer = 0;

    uint8_t  cd_active = 0;
    int8_t   cd_value  = 0;
    uint32_t cd_next_ms = 0;

    uint8_t seg_digit = 0;

    while(1){
        uint8_t nowA = read_btn_idr(&GPIOA->IDR, BIT(10));
        if (falling_edge(&pa10_last, nowA, DEBOUNCE_DELAY)){
            master_on ^= 1u;
            if (master_on) GPIOB->BSRR = GPIO_BSRR_BS6;
            else {
                GPIOB->BSRR = GPIO_BSRR_BR6;
                leds_all_off();
                seg_off_all();
                active_led = held_led = click_count = 0;
                cd_active = 0;
            }
        }
        if (!master_on) continue;

        if (!cd_active){
            uint8_t now4 = read_btn_idr(&GPIOB->IDR, BIT(4));
            if (pb4_last == 1 && now4 == 0){
                delay_cycles(BTN_SHORT_DELAY);
                if (read_btn_idr(&GPIOB->IDR, BIT(4)) == 0){
                    seg_show(0);
                    delay_cycles(8*BTN_SHORT_DELAY);
                    while(read_btn_idr(&GPIOB->IDR, BIT(4)) == 0){}
                }
            }
            pb4_last = now4;
        }

        if (held_led) led_on(held_led);

        uint8_t now5 = read_btn_idr(&GPIOB->IDR, BIT(5));
        if (falling_edge(&pb5_last, now5, DEBOUNCE_DELAY)){
            click_count++;
            multi_timer = MULTI_WINDOW;
        }
        if (multi_timer>0){
            multi_timer--;
            if (multi_timer==0 && click_count>0){
                leds_all_off();
                uint8_t n = (click_count>3)?3:click_count;
                active_led = (n==1)?5:(n==2)?6:7;
                blink_timer=0;
                click_count=0;
            }
        }

        if (active_led && !held_led && !cd_active){
            if (blink_timer==0){ led_toggle(active_led); blink_timer=BLINK_INTERVAL; }
            else blink_timer--;
        }

        uint8_t now3 = read_btn_idr(&GPIOB->IDR, BIT(3));
        if (falling_edge(&pb3_last, now3, DEBOUNCE_DELAY)){
            if (!cd_active && active_led){
                led_on(active_led);
                held_led=active_led; active_led=0;
                cd_value=seg_digit; cd_active=1; cd_next_ms=g_ms+COUNTDOWN_STEP_MS;
            }else if (cd_active){
                cd_active=0; active_led=held_led; held_led=0; blink_timer=0;
            }
        }

        if (cd_active){
            seg_show((uint8_t)cd_value);
            if ((int32_t)(g_ms - cd_next_ms)>=0){
                if (cd_value>0){ cd_value--; cd_next_ms+=COUNTDOWN_STEP_MS; }
                else cd_active=0;
            }
            continue;
        }

        seg_digit = adc_to_digit(adc1_read());
        seg_show(seg_digit);
    }
}