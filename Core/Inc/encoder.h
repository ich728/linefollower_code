/**
  ******************************************************************************
  * @file           : encoder.h
  * @brief          : TIM1/TIM8 quadrature encoder speed and odometry
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Mechanical parameters from doc/development_plan.md:
 *   wheel diameter = 24 mm
 *   motor encoder  = 1000 counts/rev
 *   gear ratio     = 10:1
 *
 * If the encoder's "1000 PPR" specification excludes quadrature x4 counting,
 * this value must be changed to the measured timer counts per wheel revolution.
 */
#define ENCODER_COUNTS_PER_WHEEL_REV  10000.0f
#define ENCODER_WHEEL_CIRCUMFERENCE_MM   75.398f

/* Change either sign to -1 if forward wheel motion produces negative counts. */
#define ENCODER_LEFT_SIGN   1
#define ENCODER_RIGHT_SIGN -1

typedef struct {
    int16_t delta_counts;
    int32_t total_counts;
    float speed_mm_s;
    float distance_mm;
} EncoderWheel_t;

typedef struct {
    EncoderWheel_t left;
    EncoderWheel_t right;
} EncoderData_t;

void Encoder_Init(void);
void Encoder_Reset(void);
void Encoder_Update(float dt_s);
const EncoderData_t *Encoder_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
