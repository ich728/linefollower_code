/**
  ******************************************************************************
  * @file           : encoder.c
  * @brief          : TIM1/TIM8 quadrature encoder speed and odometry
  ******************************************************************************
  */
#include "encoder.h"
#include "stm32f4xx.h"

#define MM_PER_COUNT \
    (ENCODER_WHEEL_CIRCUMFERENCE_MM / ENCODER_COUNTS_PER_WHEEL_REV)

/* Low-pass factor for the 200 Hz speed estimate. Higher = faster response. */
#define SPEED_FILTER_ALPHA  0.25f

static EncoderData_t encoder_data;
static uint16_t prev_left_count;
static uint16_t prev_right_count;

static void Encoder_TimerInit(TIM_TypeDef *timer)
{
    timer->CR1 = 0;
    timer->PSC = 0;
    timer->ARR = 0xFFFF;
    timer->CNT = 0;

    /*
     * CC1/CC2 are inputs mapped to TI1/TI2.
     * IC filter 4 suppresses short glitches on Hall encoder wiring.
     */
    timer->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos)
                 | (4U << TIM_CCMR1_IC1F_Pos)
                 | (1U << TIM_CCMR1_CC2S_Pos)
                 | (4U << TIM_CCMR1_IC2F_Pos);
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;
    timer->EGR = TIM_EGR_UG;
    timer->CR1 = TIM_CR1_CEN;
}

void Encoder_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN;
    (void)RCC->APB2ENR;

    Encoder_TimerInit(TIM1);
    Encoder_TimerInit(TIM8);
    Encoder_Reset();
}

void Encoder_Reset(void)
{
    TIM1->CNT = 0;
    TIM8->CNT = 0;
    prev_left_count = 0;
    prev_right_count = 0;
    encoder_data = (EncoderData_t){0};
}

void Encoder_Update(float dt_s)
{
    uint16_t left_count = (uint16_t)TIM1->CNT;
    uint16_t right_count = (uint16_t)TIM8->CNT;

    /*
     * Converting the unsigned subtraction to int16_t handles 16-bit timer
     * wraparound, provided a wheel moves less than 32768 counts per update.
     */
    int16_t left_delta =
        (int16_t)(left_count - prev_left_count) * ENCODER_LEFT_SIGN;
    int16_t right_delta =
        (int16_t)(right_count - prev_right_count) * ENCODER_RIGHT_SIGN;

    prev_left_count = left_count;
    prev_right_count = right_count;

    encoder_data.left.delta_counts = left_delta;
    encoder_data.right.delta_counts = right_delta;
    encoder_data.left.total_counts += left_delta;
    encoder_data.right.total_counts += right_delta;

    encoder_data.left.distance_mm =
        (float)encoder_data.left.total_counts * MM_PER_COUNT;
    encoder_data.right.distance_mm =
        (float)encoder_data.right.total_counts * MM_PER_COUNT;

    if (dt_s > 0.0001f) {
        float left_raw = (float)left_delta * MM_PER_COUNT / dt_s;
        float right_raw = (float)right_delta * MM_PER_COUNT / dt_s;

        encoder_data.left.speed_mm_s += SPEED_FILTER_ALPHA
            * (left_raw - encoder_data.left.speed_mm_s);
        encoder_data.right.speed_mm_s += SPEED_FILTER_ALPHA
            * (right_raw - encoder_data.right.speed_mm_s);
    }
}

const EncoderData_t *Encoder_GetData(void)
{
    return &encoder_data;
}
