/**
  ******************************************************************************
  * @file           : speed_control.h
  * @brief          : Independent left/right wheel speed PI controllers
  ******************************************************************************
  */
#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Initial bench-safe tuning values.
 *
 * Feed-forward converts target mm/s to an approximate PWM duty. Tune this
 * first from straight-line measurements, then tune KP and finally KI.
 */
#define SPEED_KP                 0.18f
#define SPEED_KI                 0.25f
#define SPEED_FF_LEFT            0.60f
#define SPEED_FF_RIGHT           0.60f
#define SPEED_INTEGRAL_LIMIT   120.0f
#define SPEED_MIN_ACTIVE_PWM      60
#define SPEED_MAX_PWM            350

typedef struct {
    float integral;
    float output_pwm;
} SpeedPI_t;

typedef struct {
    SpeedPI_t left;
    SpeedPI_t right;
} SpeedController_t;

void SpeedControl_Init(SpeedController_t *controller);
void SpeedControl_Reset(SpeedController_t *controller);
void SpeedControl_Update(SpeedController_t *controller,
                         float target_left_mm_s,
                         float target_right_mm_s,
                         float measured_left_mm_s,
                         float measured_right_mm_s,
                         float dt_s,
                         int16_t *pwm_left,
                         int16_t *pwm_right);

#ifdef __cplusplus
}
#endif

#endif /* __SPEED_CONTROL_H */
