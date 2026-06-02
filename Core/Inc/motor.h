/**
  ******************************************************************************
  * @file           : motor.h
  * @brief          : TB6612 雙輪驅動 + 負壓風扇 ESC 控制
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== PWM 參數 ======================== */
#define PWM_MAX        999    /* TIM2 ARR, 1kHz */
#define PWM_MIN        0
#define ESC_MIN_US     1000   /* 風扇停止       */
#define ESC_MAX_US     2000   /* 風扇全速       */
#define ESC_DEFAULT_US  1500  /* 風扇中等       */

/* ======================== 馬達初始化 ======================== */
void Motor_Init(void);
void Motor_TIM_Init(void);     /* TIM2 PWM + TIM14 ESC 時鐘配置 */

/* ======================== 馬達控制巨集 ======================== */
/* 左輪方向 (1 號 N30) */
#define Motor_LeftFwd()   do { PORT_MOTOR->BSRR = (uint32_t)PIN_AIN2 << 16; \
                                PORT_MOTOR->BSRR = PIN_AIN1; } while(0)
#define Motor_LeftRev()   do { PORT_MOTOR->BSRR = (uint32_t)PIN_AIN1 << 16; \
                                PORT_MOTOR->BSRR = PIN_AIN2; } while(0)

/* 右輪方向 (2 號 N30) */
#define Motor_RightFwd()  do { PORT_MOTOR->BSRR = (uint32_t)PIN_BIN2 << 16; \
                                PORT_MOTOR->BSRR = PIN_BIN1; } while(0)
#define Motor_RightRev()  do { PORT_MOTOR->BSRR = (uint32_t)PIN_BIN1 << 16; \
                                PORT_MOTOR->BSRR = PIN_BIN2; } while(0)

/* 雙輪同步 */
#define Motor_Forward()   do { Motor_LeftFwd();  Motor_RightFwd();  } while(0)
#define Motor_Reverse()   do { Motor_LeftRev();  Motor_RightRev();  } while(0)

/* 煞車：四路 H 橋下臂全開 */
#define Motor_Brake()     do { PORT_MOTOR->BSRR = PIN_AIN1 | PIN_AIN2 | \
                                                   PIN_BIN1 | PIN_BIN2; } while(0)

/* 滑行：四路 H 橋全關 */
#define Motor_Coast()     do { PORT_MOTOR->BSRR = (uint32_t)(PIN_AIN1 | PIN_AIN2 | \
                                                   PIN_BIN1 | PIN_BIN2) << 16; } while(0)

/* STBY 致能 */
#define Motor_Enable()    (PORT_MOTOR->BSRR = PIN_STBY)
#define Motor_Standby()   (PORT_MOTOR->BSRR = (uint32_t)PIN_STBY << 16)

/* ======================== PWM 輸出 ======================== */
/**
  * @brief  設定左右輪 PWM duty
  * @param  left   左輪 duty (0-999)
  * @param  right  右輪 duty (0-999)
  */
void Motor_SetPWM(uint16_t left, uint16_t right);

/**
  * @brief  設定左右輪速度（含方向自動判斷）
  * @param  left_speed   正值=前進，負值=後退
  * @param  right_speed  正值=前進，負值=後退
  */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

/* ======================== 風扇 ESC 控制 ======================== */
void Fan_Init(void);

/**
  * @brief  設定風扇轉速
  * @param  us  脈衝寬度 (1000-2000 µs)
  */
void Fan_SetSpeed(uint16_t us);

/**
  * @brief  設定風扇方向
  * @param  fwd  1=正向, 0=反向
  */
#define Fan_SetDir(fwd)  do { if (fwd) PORT_ESC_DIR->BSRR = PIN_ESC_DIR; \
                               else PORT_ESC_DIR->BSRR = (uint32_t)PIN_ESC_DIR << 16; \
                             } while(0)

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
