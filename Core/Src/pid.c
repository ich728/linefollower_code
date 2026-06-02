/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : 泛用 PID 控制器實作
  *
  *   支援位置模式和速度模式（透過 derivative on measurement）
  *   內含積分抗飽和 (clamping) 和輸出限幅
  ******************************************************************************
  */
#include "pid.h"
#include <string.h>

/**
  * @brief  初始化 PID 控制器
  */
void PID_Init(PID_t *p, float kp, float ki, float kd,
              float i_lim, float o_lim)
{
    memset(p, 0, sizeof(PID_t));
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
    p->integral_limit = i_lim;
    p->output_limit   = o_lim;
}

/**
  * @brief  計算 PID 輸出（位置模式）
  *
  *   u(t) = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt
  *
  *   - D 項作用在 error 上（標準形式）
  *   - 積分項有 clamping 防止 windup
  *   - 輸出有 hard limit
  */
float PID_Compute(PID_t *p, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* Proportional */
    float p_term = p->Kp * error;

    /* Integral (with anti-windup clamping) */
    p->integral += error * dt;
    if (p->integral > p->integral_limit) {
        p->integral = p->integral_limit;
    } else if (p->integral < -p->integral_limit) {
        p->integral = -p->integral_limit;
    }
    float i_term = p->Ki * p->integral;

    /* Derivative (on error) */
    float derivative = (dt > 0.0001f) ? (error - p->prev_error) / dt : 0.0f;
    p->prev_error = error;
    float d_term = p->Kd * derivative;

    /* Sum & clamp output */
    float output = p_term + i_term + d_term;
    if (output > p->output_limit) {
        output = p->output_limit;
    } else if (output < -p->output_limit) {
        output = -p->output_limit;
    }

    return output;
}

/**
  * @brief  重置 PID 狀態
  */
void PID_Reset(PID_t *p)
{
    p->integral   = 0.0f;
    p->prev_error = 0.0f;
}
