#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* ===== Utils ===== */
static inline void delay_cycles(volatile uint32_t n) { while (n--) __NOP(); }

/* ===== 7-seg (PC7, PA8, PA9, PB10) ===== */
static inline void Display_Number(uint8_t num) {
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

/* ===== ADC1 CH4 @ PA4 ===== */
static inline void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos);     // PA4 analog
    ADC1->CR2   |= ADC_CR2_ADON;
    ADC1->SMPR2 |= ADC_SMPR2_SMP4;                       // long sample
    ADC1->SQR1 = 0;                                      // 1 conversion
    ADC1->SQR3 = 4;                                      // channel 4
}
static inline uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

static inline void Display_Off(void) {
    GPIOC->BSRR = GPIO_BSRR_BR7;
    GPIOA->BSRR = GPIO_BSRR_BR8 | GPIO_BSRR_BR9;
    GPIOB->BSRR = GPIO_BSRR_BR10;
}

int main(void) {
    /* ===== Clocks ===== */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    /* ===== PA10 button (pull-up) ===== */
    GPIOA->MODER  &= ~GPIO_MODER_MODE10;
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR  |=  (0b01 << GPIO_PUPDR_PUPD10_Pos);

    /* ===== PB3/PB5 buttons (pull-up) ===== */
    GPIOB->MODER  &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE5);
    GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD5);
    GPIOB->PUPDR  |=  (0b01 << GPIO_PUPDR_PUPD3_Pos) | (0b01 << GPIO_PUPDR_PUPD5_Pos);

    /* ===== PB4 button (pull-up) ===== */
    GPIOB->MODER  &= ~GPIO_MODER_MODE4;
    GPIOB->PUPDR  &= ~GPIO_PUPDR_PUPD4;
    GPIOB->PUPDR  |=  (0b01 << GPIO_PUPDR_PUPD4_Pos);

    /* ===== LEDs: PA5/6/7 (active-high) ===== */
    GPIOA->MODER   &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER   |=  (0b01 << GPIO_MODER_MODE5_Pos) | (0b01 << GPIO_MODER_MODE6_Pos) | (0b01 << GPIO_MODER_MODE7_Pos);
    GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
    GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);

    /* ===== PB6: master LED ===== */
    GPIOB->MODER   &= ~GPIO_MODER_MODE6;
    GPIOB->MODER   |=  (0b01 << GPIO_MODER_MODE6_Pos);
    GPIOB->OTYPER  &= ~GPIO_OTYPER_OT6;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6;
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPD6;

    /* ===== 7-seg pins: PC7, PA8, PA9, PB10 ===== */
    GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOA->MODER |=  (0b01 << GPIO_MODER_MODE8_Pos) | (0b01 << GPIO_MODER_MODE9_Pos);
    GPIOB->MODER &= ~GPIO_MODER_MODE10;
    GPIOB->MODER |=  (0b01 << GPIO_MODER_MODE10_Pos);
    GPIOC->MODER &= ~GPIO_MODER_MODE7;
    GPIOC->MODER |=  (0b01 << GPIO_MODER_MODE7_Pos);

    /* Clear LEDs */
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
    GPIOB->BSRR = GPIO_BSRR_BR6;

    ADC1_Init();

    /* ===== Timing (ปรับได้) ===== */
    const uint32_t DEBOUNCE_DELAY = 500;
    const uint32_t MULTI_WINDOW   = 250000;
    const uint32_t BLINK_INTERVAL = 8000;

    /* ===== States ===== */
    uint8_t  pa10_last  = 1;
    uint8_t  pb6_state  = 0;

    uint8_t  pb5_last    = 1;
    uint8_t  click_count = 0;
    uint32_t multi_timer = 0;
    uint8_t  active_led  = 0;     // 0,5,6,7
    uint32_t blink_timer = 0;

    uint8_t  pb3_last = 1;
    uint8_t  held_led = 0;        // 0 = none, else 5/6/7

    uint8_t  display_frozen = 0;  // NEW: flag to freeze display at 0
    uint8_t  pb4_last = 1;

    while (1) {
        /* --- PA10: master toggle --- */
        {
            uint8_t btn = (GPIOA->IDR & GPIO_IDR_ID10) ? 1 : 0;
            if (pa10_last == 1 && btn == 0) {
                pb6_state ^= 1;
                if (pb6_state) {
                    GPIOB->BSRR = GPIO_BSRR_BS6;  // master ON
                } else {
                    GPIOB->BSRR = GPIO_BSRR_BR6;  // master OFF
                    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
                    active_led = 0; held_led = 0;
                    click_count = 0; multi_timer = 0; blink_timer = 0;
                    display_frozen = 0;  // unfreeze when master off
                }
                delay_cycles(DEBOUNCE_DELAY);
            }
            pa10_last = btn;
        }

        /* --- ถ้า master OFF: บังคับดับทุกดวงแล้ววนรอ PA10 --- */
        if (!pb6_state) {
            GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
            active_led = 0; held_led = 0;
            click_count = 0; multi_timer = 0; blink_timer = 0;

            Display_Off();
            /* แสดง 7-seg ได้ตามต้องการ: (ตัวอย่างอ่านทุกลูป) */
//            if (!display_frozen) {
//                uint16_t v  = ADC1_Read();
//                uint8_t  num = (uint8_t)((v * 10U) / 4096U);
//                if (num > 9) num = 9;
//                Display_Number(num);
//            }

            continue;
        }

        /* --- โหมด HOLD: กด PB3 อีกครั้งเพื่อ "ปลด hold → กลับมากระพริบ" --- */
        if (held_led != 0) {
            // ย้ำให้ไฟค้าง
            if (held_led == 5)      GPIOA->BSRR = GPIO_BSRR_BS5;
            else if (held_led == 6) GPIOA->BSRR = GPIO_BSRR_BS6;
            else                    GPIOA->BSRR = GPIO_BSRR_BS7;

            // อนุญาตเฉพาะ PB3 เพื่อ "ปลด hold"
            uint8_t btn_pb3 = (GPIOB->IDR & GPIO_IDR_ID3) ? 1 : 0;
            if (pb3_last == 1 && btn_pb3 == 0) {
                active_led  = held_led;   // กลับมากระพริบดวงเดิม
                held_led    = 0;
                blink_timer = 0;          // เริ่ม toggle ทันที
                delay_cycles(DEBOUNCE_DELAY);
            }
            pb3_last = btn_pb3;

            // แสดง 7-seg ได้ตามปกติ
            if (!display_frozen) {
                uint16_t v  = ADC1_Read();
                uint8_t  num = (uint8_t)((v * 10U) / 4096U);
                if (num > 9) num = 9;
                Display_Number(num);
            }

            continue; // กันปุ่มอื่นระหว่าง hold
        }

        /* --- PB5: single/double/triple click → เลือกดวงสำหรับกระพริบ --- */
        {
            uint8_t btn_pb5 = (GPIOB->IDR & GPIO_IDR_ID5) ? 1 : 0;
            if (pb5_last == 1 && btn_pb5 == 0) {
                click_count++;
                multi_timer = MULTI_WINDOW;
                display_frozen = 0;  // unfreeze when interacting with LEDs
                delay_cycles(DEBOUNCE_DELAY);
            }
            pb5_last = btn_pb5;

            if (multi_timer > 0) {
                multi_timer--;
                if (multi_timer == 0 && click_count > 0) {
                    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
                    uint8_t n = (click_count > 3) ? 3 : click_count;
                    active_led  = (n == 1) ? 5 : (n == 2) ? 6 : 7;
                    blink_timer = 0;           // ให้เริ่มกระพริบทันที
                    click_count = 0;
                }
            }
        }

        /* --- กระพริบ LED ที่ active --- */
        if (active_led != 0) {
            if (blink_timer == 0) {
                if (active_led == 5)      GPIOA->ODR ^= GPIO_ODR_OD5;
                else if (active_led == 6) GPIOA->ODR ^= GPIO_ODR_OD6;
                else                      GPIOA->ODR ^= GPIO_ODR_OD7;
                blink_timer = BLINK_INTERVAL;
            } else {
                blink_timer--;
            }
        }

        /* --- PB3: เข้าสู่ hold (เมื่อยังไม่ hold) --- */
        {
            uint8_t btn_pb3 = (GPIOB->IDR & GPIO_IDR_ID3) ? 1 : 0;
            if (pb3_last == 1 && btn_pb3 == 0) {
                if (active_led != 0) {
                    if (active_led == 5)      GPIOA->BSRR = GPIO_BSRR_BS5;
                    else if (active_led == 6) GPIOA->BSRR = GPIO_BSRR_BS6;
                    else                      GPIOA->BSRR = GPIO_BSRR_BS7;
                    held_led   = active_led;   // เข้า hold
                    active_led = 0;            // หยุดกระพริบ
                    blink_timer = 0;
                }
                delay_cycles(DEBOUNCE_DELAY);
            }
            pb3_last = btn_pb3;
        }

        /* --- PB4 reset number --- */
        {
            uint8_t btn_pb4 = (GPIOB->IDR & GPIO_IDR_ID4) ? 1 : 0;
            if (pb4_last == 1 && btn_pb4 == 0) {
                Display_Number(0);
                display_frozen = 1;  // freeze display at 0
                delay_cycles(DEBOUNCE_DELAY);
            }
            pb4_last = btn_pb4;
        }

        /* --- 7-seg: อ่าน ADC แล้วโชว์ (ถ้าไม่ frozen) --- */
        if (!display_frozen) {
            uint16_t v  = ADC1_Read();
            uint8_t  num = (uint8_t)((v * 10U) / 4096U);
            if (num > 9) num = 9;
            Display_Number(num);
        }
    }
}
