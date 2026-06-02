/**
  ******************************************************************************
  * @file           : pid.h
  * @brief          : 泛用 PID 控制器
  *
  *   可重複用於：位置 PID（線循跡）、速度 PID（馬達閉環）
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;          /* 積分累積值       */
    float prev_error;        /* 前次誤差 (D 項)  */
    float integral_limit;    /* 積分上限 (抗飽和) */
    float output_limit;      /* 輸出上限         */
} PID_t;

/**
  * @brief  初始化 PID 控制器
  * @param  p       PID 實例指標
  * @param  kp      比例增益
  * @param  ki      積分增益
  * @param  kd      微分增益
  * @param  i_lim   積分限幅（絕對值）
  * @param  o_lim   輸出限幅（絕對值）
  */
void PID_Init(PID_t *p, float kp, float ki, float kd,
              float i_lim, float o_lim);

/**
  * @brief  計算 PID 輸出
  * @param  p            PID 實例指標
  * @param  setpoint     目標值
  * @param  measurement  實際量測值
  * @param  dt           距離上次呼叫的時間 (秒)
  * @retval PID 輸出值（已限幅）
  */
float PID_Compute(PID_t *p, float setpoint, float measurement, float dt);

/**
  * @brief  重置 PID 狀態（清除積分和歷史誤差）
  */
void PID_Reset(PID_t *p);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
