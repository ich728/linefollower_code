/**
  ******************************************************************************
  * @file           : speed_control.c
  * @brief          : Independent left/right wheel feed-forward + PI control
  ******************************************************************************
  */
#include "speed_control.h"

static float clampf(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static float absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int16_t SpeedPI_Update(SpeedPI_t *pi,
                              float target_mm_s,
                              float measured_mm_s,
                              float feedforward_gain,
                              float dt_s)
{
    if (absf(target_mm_s) < 1.0f) {
        pi->integral = 0.0f;
        pi->output_pwm = 0.0f;
        return 0;
    }

    float error = target_mm_s - measured_mm_s;
    float candidate_integral = clampf(
        pi->integral + error * dt_s,
        -SPEED_INTEGRAL_LIMIT,
        SPEED_INTEGRAL_LIMIT);

    float feedforward = feedforward_gain * target_mm_s;
    float candidate_output = feedforward
                           + SPEED_KP * error
                           + SPEED_KI * candidate_integral;

    /*
     * Conditional integration: keep integrating unless the output is
     * saturated and the current error would push it further into saturation.
     */
    uint8_t saturating_high =
        candidate_output > SPEED_MAX_PWM && error > 0.0f;
    uint8_t saturating_low =
        candidate_output < -SPEED_MAX_PWM && error < 0.0f;
    if (!saturating_high && !saturating_low) {
        pi->integral = candidate_integral;
    }

    float output = feedforward
                 + SPEED_KP * error
                 + SPEED_KI * pi->integral;

    /*
     * Do not reverse a wheel merely to correct a forward-speed overshoot.
     * Direction changes must come from a direction-changing target command.
     */
    if (target_mm_s > 0.0f) {
        output = clampf(output, 0.0f, SPEED_MAX_PWM);
    } else {
        output = clampf(output, -SPEED_MAX_PWM, 0.0f);
    }

    /* Compensate the motor driver's low-duty dead zone. */
    if (output > 0.0f && output < SPEED_MIN_ACTIVE_PWM) {
        output = SPEED_MIN_ACTIVE_PWM;
    } else if (output < 0.0f && output > -SPEED_MIN_ACTIVE_PWM) {
        output = -SPEED_MIN_ACTIVE_PWM;
    }

    pi->output_pwm = output;
    return (int16_t)output;
}

void SpeedControl_Init(SpeedController_t *controller)
{
    SpeedControl_Reset(controller);
}

void SpeedControl_Reset(SpeedController_t *controller)
{
    controller->left = (SpeedPI_t){0};
    controller->right = (SpeedPI_t){0};
}

void SpeedControl_Update(SpeedController_t *controller,
                         float target_left_mm_s,
                         float target_right_mm_s,
                         float measured_left_mm_s,
                         float measured_right_mm_s,
                         float dt_s,
                         int16_t *pwm_left,
                         int16_t *pwm_right)
{
    *pwm_left = SpeedPI_Update(
        &controller->left,
        target_left_mm_s,
        measured_left_mm_s,
        SPEED_FF_LEFT,
        dt_s);
    *pwm_right = SpeedPI_Update(
        &controller->right,
        target_right_mm_s,
        measured_right_mm_s,
        SPEED_FF_RIGHT,
        dt_s);
}
