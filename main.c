#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

static inline void delay_cycles(volatile uint32_t n) {
    while (n--) __NOP();
}

int main(void) {
    // ==== Enable clocks =======================================================
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // ไม่ใช้ก็ไม่เป็นไร แต่คงไว้ตามฟอร์แมตเดิม
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // ไม่ใช้ก็ไม่เป็นไร แต่คงไว้ตามฟอร์แมตเดิม

    // ==== PA10: Button (input + pull-up, priority สูงสุด) ====================
    GPIOA->MODER  &= ~GPIO_MODER_MODE10;                  // 00 = input
    GPIOA->PUPDR  &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR  |=  (0b01 << GPIO_PUPDR_PUPD10_Pos);    // 01 = pull-up

    // ==== PB3: Button (input + pull-up) ======================================
    GPIOB->MODER  &= ~GPIO_MODER_MODE3;
    GPIOB->PUPDR  &= ~GPIO_PUPDR_PUPD3;
    GPIOB->PUPDR  |=  (0b01 << GPIO_PUPDR_PUPD3_Pos);

    // ==== PB5: Button (input + pull-up) ======================================
    GPIOB->MODER  &= ~GPIO_MODER_MODE5;
    GPIOB->PUPDR  &= ~GPIO_PUPDR_PUPD5;
    GPIOB->PUPDR  |=  (0b01 << GPIO_PUPDR_PUPD5_Pos);

    // ==== PA5/PA6/PA7: LEDs (output push-pull, active-high) ==================
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

    // ==== PB6: LED (master indicator, output push-pull, active-high) =========
    GPIOB->MODER   &= ~GPIO_MODER_MODE6;
    GPIOB->MODER   |=  (0b01 << GPIO_MODER_MODE6_Pos);
    GPIOB->OTYPER  &= ~GPIO_OTYPER_OT6;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6;
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPD6;

    // ดับไฟทั้งหมดก่อน
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
    GPIOB->BSRR = GPIO_BSRR_BR6;

    // ==== Timing params =======================================================
    const uint32_t DEBOUNCE_DELAY = 2000;     // กันเด้ง (ปรับตามความเหมาะสม)
    const uint32_t MULTI_WINDOW   = 900000;   // หน้าต่างเวลานับคลิก PB5
    const uint32_t BLINK_INTERVAL = 250000;   // คาบกระพริบ

    // ==== State variables =====================================================
    // PA10 -> PB6 (master)
    uint8_t  pa10_last  = 1;   // pull-up: not pressed = 1
    uint8_t  pb6_state  = 0;   // master: 0=off, 1=on

    // PB5 -> PA5/6/7 (multi-click)
    uint8_t  pb5_last    = 1;
    uint8_t  click_count = 0;
    uint32_t multi_timer = 0;
    uint8_t  active_led  = 0;         // 0=none, 5/6/7=กำลังกระพริบขานั้น
    uint32_t blink_timer = 0;

    // PB3 -> Hold/Lock
    uint8_t  pb3_last = 1;
    uint8_t  held_led = 0;            // 0=ไม่ค้าง, 5/6/7=ไฟที่ค้างอยู่

    while (1) {
        // ===== 1) PA10: Priority สูงสุด → Toggle Master PB6 =================
        {
            uint8_t btn_pa10 = (GPIOA->IDR & GPIO_IDR_ID10) ? 1 : 0;
            if (pa10_last == 1 && btn_pa10 == 0) {     // falling edge
                pb6_state ^= 1;
                if (pb6_state) {
                    // Master ON
                    GPIOB->BSRR = GPIO_BSRR_BS6;
                } else {
                    // Master OFF → ดับทุกดวง + เคลียร์สถานะทั้งหมด
                    GPIOB->BSRR = GPIO_BSRR_BR6;
                    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
                    active_led  = 0;
                    held_led    = 0;
                    click_count = 0;
                    multi_timer = 0;
                    blink_timer = 0;
                }
                delay_cycles(DEBOUNCE_DELAY);
            }
            pa10_last = btn_pa10;
        }

        // ===== 2) ถ้า Master OFF → คุมให้ดับตลอดและข้าม PB5/PB3 ============
        if (!pb6_state) {
            GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;
            active_led  = 0;
            held_led    = 0;
            click_count = 0;
            multi_timer = 0;
            blink_timer = 0;
            continue;  // ข้ามรอบนี้ไป รอ PA10 เท่านั้น
        }

        // ===== 3) ถ้าอยู่โหมด HOLD → ค้างไฟนิ่ง และเพิกเฉย PB5/PB3 =========
        if (held_led != 0) {
            // ตอกย้ำให้ไฟที่ hold ติดค้าง (กันกรณี ODR ถูกเปลี่ยน)
            if (held_led == 5)      GPIOA->BSRR = GPIO_BSRR_BS5;
            else if (held_led == 6) GPIOA->BSRR = GPIO_BSRR_BS6;
            else                    GPIOA->BSRR = GPIO_BSRR_BS7;

            // ไม่อ่าน PB5/PB3 ในโหมด hold (ปุ่มอื่นไม่มีผล ยกเว้น PA10 ที่จัดการไปแล้ว)
            // หมายเหตุ: หากต้องการ “กด PB3 อีกครั้งเพื่อละจาก hold กลับมากระพริบ”
            // ให้ย้าย logic resume มาใส่ในสาขานี้ได้ในอนาคต
            continue;
        }

        // ===== 4) PB5: Single/Double/Triple click → เลือก LED สำหรับกระพริบ =
        {
            uint8_t btn_pb5 = (GPIOB->IDR & GPIO_IDR_ID5) ? 1 : 0;
            if (pb5_last == 1 && btn_pb5 == 0) {   // falling edge
                click_count++;
                multi_timer = MULTI_WINDOW;
                delay_cycles(DEBOUNCE_DELAY);
            }
            pb5_last = btn_pb5;

            if (multi_timer > 0) {
                multi_timer--;
                if (multi_timer == 0 && click_count > 0) {
                    // ดับทุกดวงก่อนเลือกดวงใหม่
                    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR7;

                    uint8_t n = (click_count > 3) ? 3 : click_count;
                    active_led = (n == 1) ? 5 : (n == 2) ? 6 : 7;

                    blink_timer = BLINK_INTERVAL;
                    click_count = 0;
                }
            }
        }

        // ===== 5) กระพริบ LED ที่ active ====================================
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

        // ===== 6) PB3: Hold/Lock (ค้างไฟที่กำลังกระพริบให้ติดค้าง) =========
        {
            uint8_t btn_pb3 = (GPIOB->IDR & GPIO_IDR_ID3) ? 1 : 0;
            if (pb3_last == 1 && btn_pb3 == 0) {
                if (active_led != 0) {
                    // ค้างไฟดวงปัจจุบันให้ติดค้าง
                    if (active_led == 5)      GPIOA->BSRR = GPIO_BSRR_BS5;
                    else if (active_led == 6) GPIOA->BSRR = GPIO_BSRR_BS6;
                    else                      GPIOA->BSRR = GPIO_BSRR_BS7;

                    held_led   = active_led;   // เข้าสู่โหมด hold
                    active_led = 0;            // หยุดกระพริบ
                    blink_timer = 0;
                }
                // (ไม่ทำ resume ในเวอร์ชันนี้ตามโจทย์: hold แล้วปุ่มอื่นไม่มีผล ยกเว้น PA10)
                delay_cycles(DEBOUNCE_DELAY);
            }
            pb3_last = btn_pb3;
        }
    }
}
