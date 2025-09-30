#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

static inline void delay_cycles(volatile uint32_t n) {
    while (n--) __NOP();
}

int main(void) {
    // ==== Enable clocks =======================================================
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // ==== PA10: Button (input + pull-up) =====================================
    GPIOA->MODER  &= ~GPIO_MODER_MODE10;                  // 00 = input
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR  |=  (0b01 << GPIO_PUPDR_PUPD10_Pos);    // 01 = pull-up

    // ==== PB5: Button (input + pull-up) ======================================
    GPIOB->MODER  &= ~GPIO_MODER_MODE5;
    GPIOB->PUPDR  &= ~GPIO_PUPDR_PUPD5;
    GPIOB->PUPDR  |=  (0b01 << GPIO_PUPDR_PUPD5_Pos);

    // ==== PA5/PA6/PA7: LEDs (output push-pull) ================================
    // PA5
    GPIOA->MODER   &= ~GPIO_MODER_MODE5;
    GPIOA->MODER   |=  (0b01 << GPIO_MODER_MODE5_Pos);
    GPIOA->OTYPER  &= ~GPIO_OTYPER_OT5;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5;
    GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPD5;

    // PA6
    GPIOA->MODER   &= ~GPIO_MODER_MODE6;
    GPIOA->MODER   |=  (0b01 << GPIO_MODER_MODE6_Pos);
    GPIOA->OTYPER  &= ~GPIO_OTYPER_OT6;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6;
    GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPD6;

    // PA7
    GPIOA->MODER   &= ~GPIO_MODER_MODE7;
    GPIOA->MODER   |=  (0b01 << GPIO_MODER_MODE7_Pos);
    GPIOA->OTYPER  &= ~GPIO_OTYPER_OT7;
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7;
    GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPD7;

    // ==== PB6: LED (output push-pull) ========================================
    GPIOB->MODER   &= ~GPIO_MODER_MODE6;
    GPIOB->MODER   |=  (0b01 << GPIO_MODER_MODE6_Pos);
    GPIOB->OTYPER  &= ~GPIO_OTYPER_OT6;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6;
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPD6;

    // ดับไฟทั้งหมดก่อน
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
    GPIOB->BSRR = GPIO_BSRR_BR6;

    // ==== Timing params =======================================================
    const uint32_t DEBOUNCE_DELAY = 2000;
    const uint32_t MULTI_WINDOW   = 900000;
    const uint32_t BLINK_INTERVAL = 250000;

    // ==== State variables =====================================================
    // สำหรับ PA10 -> PB6 (toggle)
    uint8_t  pa10_last  = 1;  // ปุ่มว่าง (pull-up)
    uint8_t  pb6_state  = 0;  // 0=off, 1=on

    // สำหรับ PB5 -> PA5/6/7 (multi-click)
    uint8_t  pb5_last    = 1;
    uint8_t  click_count = 0;
    uint32_t multi_timer = 0;
    uint8_t  active_led  = 0;      // 0=none, 5/6/7
    uint32_t blink_timer = 0;

    while (1) {
        // ===== 1) PA10 toggle PB6 ============================================
        {
            uint8_t btn_pa10 = (GPIOA->IDR & GPIO_IDR_ID10) ? 1 : 0;

            if (pa10_last == 1 && btn_pa10 == 0) {
                // falling edge -> toggle PB6
                pb6_state ^= 1;

                if (pb6_state) {
                    GPIOB->BSRR = GPIO_BSRR_BS6;
                } else {
                    GPIOB->BSRR = GPIO_BSRR_BR6;
                    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;

                    active_led  = 0;
                    click_count = 0;
                    multi_timer = 0;
                    blink_timer = 0;
                }

                // กันเด้งเล็กน้อย (ถ้าต้องการมากกว่านี้ เพิ่มค่า)
                delay_cycles(DEBOUNCE_DELAY);
            }

            pa10_last = btn_pa10;
        }

        // ===== 2) PB5 single/double/triple click =============================
        {
            uint8_t btn_pb5 = (GPIOB->IDR & GPIO_IDR_ID5) ? 1 : 0;

            // ตรวจจับขอบตก = คลิกใหม่
            if (pb5_last == 1 && btn_pb5 == 0) {
                click_count++;
                multi_timer = MULTI_WINDOW;
                delay_cycles(DEBOUNCE_DELAY);
            }
            pb5_last = btn_pb5;

            // เมื่อหน้าต่างเวลาหมด -> ตัดสินจำนวนคลิก
            if (multi_timer > 0) {
                multi_timer--;
                if (multi_timer == 0 && click_count > 0) {
                    // ดับทุกดวงก่อน
                    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;

                    // จำกัดสูงสุด 3 คลิก
                    uint8_t n = (click_count > 3) ? 3 : click_count;

                    if (n == 1) {
                        active_led = 5;
                    } else if (n == 2) {
                        active_led = 6;
                    } else {
                        active_led = 7;
                    }

                    blink_timer = BLINK_INTERVAL;
                    click_count = 0;
                }
            }

            // กระพริบ LED ที่เลือก
            if (active_led != 0) {
                if (blink_timer == 0) {
                    if (active_led == 5) {
                        GPIOA->ODR ^= GPIO_ODR_OD5;
                    } else if (active_led == 6) {
                        GPIOA->ODR ^= GPIO_ODR_OD6;
                    } else {
                        GPIOA->ODR ^= GPIO_ODR_OD7;
                    }
                    blink_timer = BLINK_INTERVAL;
                } else {
                    blink_timer--;
                }
            }
        }
    }
}
