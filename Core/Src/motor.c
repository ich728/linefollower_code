/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : TB6612 雙輪 DC 馬達驅動 + Skywalker ESC 負壓風扇
  *
  *   PWMA = PA1 (TIM2_CH2), PWMB = PA3 (TIM2_CH4)
  *   同屬 TIM2，單次更新兩個 CCR 保證同步輸出無相位抖動
  *   ESC  = PB8 (TIM4_CH3), DIR = PB1
  ******************************************************************************
  */
#include "pinout.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

/* ======================== TIM2 PWM 初始化 (馬達) ================== */
void Motor_TIM_Init(void)
{
    /* TIM2 clock enable */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* PSC=15, ARR=999 → 16MHz/16/1000 = 1kHz */
    TIM2->PSC  = 15;
    TIM2->ARR  = 999;
    TIM2->CNT  = 0;

    /* CH2 (PA1): PWM mode 1, preload enable */
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
    TIM2->CCMR1 |=  (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCER  |=  TIM_CCER_CC2E;
    TIM2->CCR2  =  0;

    /* CH4 (PA3): PWM mode 1, preload enable */
    TIM2->CCMR2 &= ~(TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE);
    TIM2->CCMR2 |=  (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM2->CCER  |=  TIM_CCER_CC4E;
    TIM2->CCR4  =  0;

    /* Enable TIM2 */
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* ======================== 馬達初始化 ======================== */
void Motor_Init(void)
{
    /* 初始狀態：煞車 + 致能 */
    Motor_Brake();
    Motor_Enable();
}

/* ======================== PWM 輸出 ======================== */
void Motor_SetPWM(uint16_t left, uint16_t right)
{
    if (left  > PWM_MAX) left  = PWM_MAX;
    if (right > PWM_MAX) right = PWM_MAX;

    TIM2->CCR2 = left;
    TIM2->CCR4 = right;
}

/* ======================== 速度設定 (含方向) ======================== */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    uint16_t pwm_l, pwm_r;

    /* 左輪方向 + duty */
    if (left_speed >= 0) {
        Motor_LeftFwd();
        pwm_l = (uint16_t)left_speed;
    } else {
        Motor_LeftRev();
        pwm_l = (uint16_t)(-left_speed);
    }

    /* 右輪方向 + duty */
    if (right_speed >= 0) {
        Motor_RightFwd();
        pwm_r = (uint16_t)right_speed;
    } else {
        Motor_RightRev();
        pwm_r = (uint16_t)(-right_speed);
    }

    /* 同步寫入雙輪 PWM */
    Motor_SetPWM(pwm_l, pwm_r);
}

/* ======================== ESC 負壓風扇 (同 Demo: TIM4_CH3, PB8) ====== */
void Fan_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC  = 159;
    TIM4->ARR  = 1999;
    TIM4->CCR3 = 110;                  /* 110 ticks = 1100µs */
    TIM4->CCMR2 = (6 << 4) | (1 << 3); /* CH3 PWM mode 1, preload */
    TIM4->CCER  = TIM_CCER_CC3E;
    TIM4->CR1   = TIM_CR1_CEN;
}

/* us: 1100-2000 µs → ticks = us/10 */
void Fan_SetSpeed(uint16_t us)
{
    if (us < 1100) us = 1100;
    if (us > 1250) us = 1250;  /* 上限保護 */
    TIM4->CCR3 = us / 10;
}
