/**
  ******************************************************************************
  * @file           : pinout.h
  * @brief          : 循跡機器人完整腳位定義
  *
  *   PORTA[0:7] — TB6612 馬達驅動全組 (近端 R14-R17)
  *   PORTB      — ESC 方向、IMU 通訊、按鍵
  *   PORTC      — 右編碼器 (TIM8 Quadrature)
  *   PORTD      — OLED 顯示 (遠端 R17-R19)
  *   PORTE      — 灰階感測器 8ch + 左編碼器 (TIM1 Quadrature)
  ******************************************************************************
  */
#ifndef __PINOUT_H
#define __PINOUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* =========================== PORTA: 馬達專區 ============================ */
#define PIN_AIN1      GPIO_PIN_0   /* PA0  近 R17 內  左輪 H 橋 A1        */
#define PIN_PWMA      GPIO_PIN_1   /* PA1  近 R17 外  左輪 PWM  TIM2_CH2   */
#define PIN_AIN2      GPIO_PIN_2   /* PA2  近 R16 內  左輪 H 橋 A2        */
#define PIN_PWMB      GPIO_PIN_3   /* PA3  近 R16 外  右輪 PWM  TIM2_CH4   */
#define PIN_STBY      GPIO_PIN_4   /* PA4  近 R15 內  馬達致能 STBY        */
#define PIN_BIN1      GPIO_PIN_5   /* PA5  近 R15 外  右輪 H 橋 B1        */
#define PIN_BIN2      GPIO_PIN_6   /* PA6  近 R14 內  右輪 H 橋 B2        */

#define PORT_MOTOR    GPIOA

/* =========================== PORTB: 控制 ============================== */
#define PIN_ESC       GPIO_PIN_8   /* PB8  遠 R22 內  負壓風扇 TIM4_CH3    */
#define PIN_ESC_DIR   GPIO_PIN_1   /* PB1  近 R12 外  風扇方向             */
#define PIN_IMU_SCK   GPIO_PIN_3   /* PB3  遠 R20 內  IMU SPI 時鐘         */
#define PIN_IMU_MOSI  GPIO_PIN_5   /* PB5  遠 R20 外  IMU SPI 數據         */
#define PIN_IMU_MISO  GPIO_PIN_6   /* PB6  遠 R21 內  IMU SPI 回讀         */
#define PIN_IMU_CS    GPIO_PIN_7   /* PB7  遠 R21 外  IMU 片選             */
#define PIN_BTN       GPIO_PIN_15  /* PB15 遠 R4  內  按鍵                 */

#define PORT_IMU      GPIOB
#define PORT_ESC_DIR  GPIOB
#define PORT_ESC_FAN  GPIOB
#define PORT_BTN      GPIOB

/* =========================== PORTC: 右編碼器 =========================== */
#define PIN_E2A       GPIO_PIN_6   /* PC6  遠 R8  外  右編碼器 A  TIM8_CH1 */
#define PIN_E2B       GPIO_PIN_7   /* PC7  遠 R9  內  右編碼器 B  TIM8_CH2 */

#define PORT_ENC_R    GPIOC

/* =========================== PORTD: OLED ============================== */
#define PIN_OLED_SCK  GPIO_PIN_3   /* PD3  遠 R17 外  OLED SPI 時鐘        */
#define PIN_OLED_MOSI GPIO_PIN_4   /* PD4  遠 R18 內  OLED SPI 數據        */
#define PIN_OLED_DC   GPIO_PIN_5   /* PD5  遠 R18 外  OLED 命令/數據       */
#define PIN_OLED_RES  GPIO_PIN_6   /* PD6  遠 R19 內  OLED 復位            */

#define PORT_OLED     GPIOD

/* =========================== PORTE: 灰階 + 左編碼器 ==================== */
#define PIN_GRAY_CH1  GPIO_PIN_4   /* PE4  近 R21 內  灰階 CH1 (最左)      */
#define PIN_GRAY_CH2  GPIO_PIN_5   /* PE5  近 R21 外  灰階 CH2             */
#define PIN_GRAY_CH3  GPIO_PIN_6   /* PE6  近 R20 內  灰階 CH3             */
#define PIN_GRAY_CH4  GPIO_PIN_7   /* PE7  近 R11 內  灰階 CH4             */
#define PIN_GRAY_CH5  GPIO_PIN_8   /* PE8  近 R11 外  灰階 CH5             */
#define PIN_E1A       GPIO_PIN_9   /* PE9  近 R10 內  左編碼器 A  TIM1_CH1 */
#define PIN_GRAY_CH6  GPIO_PIN_10  /* PE10 近 R10 外  灰階 CH6             */
#define PIN_E1B       GPIO_PIN_11  /* PE11 近 R9  內  左編碼器 B  TIM1_CH2 */
#define PIN_GRAY_CH7  GPIO_PIN_12  /* PE12 近 R9  外  灰階 CH7             */
#define PIN_GRAY_CH8  GPIO_PIN_13  /* PE13 近 R8  內  灰階 CH8 (最右)      */

#define PORT_GRAY     GPIOE
#define PORT_ENC_L    GPIOE

/* =========================== 灰階遮罩 (所有 8ch) ======================= */
#define GRAY_MASK    (PIN_GRAY_CH1 | PIN_GRAY_CH2 | PIN_GRAY_CH3 | \
                      PIN_GRAY_CH4 | PIN_GRAY_CH5 | PIN_GRAY_CH6 | \
                      PIN_GRAY_CH7 | PIN_GRAY_CH8)

#ifdef __cplusplus
}
#endif

#endif /* __PINOUT_H */
