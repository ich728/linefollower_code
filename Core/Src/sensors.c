/**
  ******************************************************************************
  * @file           : sensors.c
  * @brief          : 8 通道灰階感測器讀取 + 線位置加權平均計算
  *
  *   感測器實體排列 (左→右)，間距 12mm：
  *     CH1  CH2  CH3  CH4  CH5  CH6  CH7  CH8
  *     PE4  PE5  PE6  PE7  PE8  PE10 PE12 PE13
  *   權重位置 (mm)：-42 -30 -18 -6 +6 +18 +30 +42
  ******************************************************************************
  */
#include "main.h"
#include "pinout.h"
#include "sensors.h"

/* 感測器權重位置 (mm)，中心 = 0 */
static const int8_t sensor_pos[8] = {
    -42,  /* CH1 (PE4)  最左  */
    -30,  /* CH2 (PE5)        */
    -18,  /* CH3 (PE6)        */
     -6,  /* CH4 (PE7)        */
      6,  /* CH5 (PE8)        */
     18,  /* CH6 (PE10)       */
     30,  /* CH7 (PE12)       */
     42   /* CH8 (PE13) 最右  */
};

/**
  * @brief  讀取 8 通道灰階（單次 GPIOE IDR 並列讀取）
  *
  *   PE4-13 散佈在 PORTE 上，被 PE9/PE11 (編碼器) 打斷。
  *   逐 bit 判斷組合成 8-bit：
  *     bit0 (LSB) = CH1 (PE4) 最左
  *     bit7 (MSB) = CH8 (PE13) 最右
  */
uint8_t Gray_Read(void)
{
    uint32_t r = PORT_GRAY->IDR;
    uint8_t gray = 0;

    /* 感測器 active-LOW: 黑線=0, 白色=1, 取反 */
    if (!(r & PIN_GRAY_CH1))  gray |= 0x01;   /* PE4  → bit 0 */
    if (!(r & PIN_GRAY_CH2))  gray |= 0x02;   /* PE5  → bit 1 */
    if (!(r & PIN_GRAY_CH3))  gray |= 0x04;   /* PE6  → bit 2 */
    if (!(r & PIN_GRAY_CH4))  gray |= 0x08;   /* PE7  → bit 3 */
    if (!(r & PIN_GRAY_CH5))  gray |= 0x10;   /* PE8  → bit 4 */
    if (!(r & PIN_GRAY_CH6))  gray |= 0x20;   /* PE10 → bit 5 */
    if (!(r & PIN_GRAY_CH7))  gray |= 0x40;   /* PE12 → bit 6 */
    if (!(r & PIN_GRAY_CH8))  gray |= 0x80;   /* PE13 → bit 7 */

    return gray;
}

/**
  * @brief  邊緣檢測法計算線位置
  *
  *   取最左和最右看到黑線的感測器中點 = 線中心
  *   解決加權平均在單/多感測器時結果不一致的問題
  *   0 = 正中, 負 = 偏左, 正 = 偏右
  */
float Line_GetError(uint8_t gray)
{
    if (gray == 0x00) return LINE_LOST;
    if (gray == 0xFF) return LINE_FULL;

    /* 找最左和最右的 ON 感測器 */
    int left  = 8;   /* 初始: 最右+1 */
    int right = -1;  /* 初始: 最左-1 */

    for (int i = 0; i < 8; i++) {
        if (gray & (1 << i)) {
            if (i < left)  left  = i;
            if (i > right) right = i;
        }
    }

    /* 線中心 = 左右邊界中點 */
    return (float)(sensor_pos[left] + sensor_pos[right]) / 2.0f;
}
