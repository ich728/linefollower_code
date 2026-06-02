/**
  ******************************************************************************
  * @file           : sensors.h
  * @brief          : 8 通道灰階感測器 + 線位置計算
  ******************************************************************************
  */
#ifndef __SENSORS_H
#define __SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 特殊返回值 */
#define LINE_LOST   999.0f   /* 全白：斷線 / 完全離線 */
#define LINE_FULL  -999.0f   /* 全黑：起點區或異常     */

/**
  * @brief  讀取 8 通道灰階感測器原始值
  * @retval 8-bit 值，bit0=CH1(最左) ... bit7=CH8(最右)，1=黑線
  */
uint8_t Gray_Read(void);

/**
  * @brief  計算線位置誤差（加權平均法）
  * @param  gray  8-bit 灰階原始值
  * @retval 誤差值 (mm)，0 = 黑線正中，負 = 偏左，正 = 偏右
  *         回傳 LINE_LOST 表示全白（斷線）
  *         回傳 LINE_FULL 表示全黑
  */
float Line_GetError(uint8_t gray);

/**
  * @brief  檢查是否偵測到黑線
  * @param  gray  8-bit 灰階原始值
  * @retval 1 = 至少一個通道看到黑，0 = 全白
  */
static inline uint8_t Line_Detected(uint8_t gray)
{
    return (gray != 0x00) ? 1 : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H */
