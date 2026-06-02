/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 學界循跡機器人 — Phase 1 MVP
  *                   PID 線循跡 + 雙輪差速 + 負壓風扇
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "pinout.h"
#include "sensors.h"
#include "motor.h"
#include "pid.h"
#include <stdio.h>

/* USER CODE BEGIN PD */
/* ======================== 控制參數 ======================== */
#define LOOP_MS         10       /* 控制迴圈週期 (100Hz)          */
#define BASE_SPEED      500      /* 基底速度 PWM duty (0-999)     */
#define FAN_SPEED_US    1500     /* 風扇固定轉速 µs              */
#define TARGET_DIST_MM  10000    /* 一圈總長 mm（現場調整）       */

/* PID 參數 (初始值，透過 OLED + 按鍵可調) */
#define KP_INIT         1.5f
#define KI_INIT         0.02f
#define KD_INIT         12.0f
#define I_LIMIT         300.0f
#define OUTPUT_LIMIT    300.0f

/* 出界判定閾值 */
#define OOB_ERROR_MM    40.0f    /* 線誤差超過此值視為偏離       */
#define OOB_COUNT_MAX   5        /* 連續 5 次 (~50ms) 觸發出界   */

/* ======================== 狀態機 ======================== */
typedef enum {
    STATE_IDLE,          /* 等待按鍵啟動                        */
    STATE_RUNNING,       /* 循跡中                              */
    STATE_FINISHED,      /* 完成一圈                            */
    STATE_OOB            /* 出界                                */
} State_t;

/* ======================== OLED 5×8 字型 ======================== */
static const uint8_t font5x8[91][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00},
  {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
  {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
  {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
  {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
  {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
  {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
  {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
  {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14},
  {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E},
  {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41},
  {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32},
  {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
  {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F},
  {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E},
  {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
  {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
  {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
  {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03},
  {0x61,0x51,0x49,0x45,0x43}, {0x00,0x00,0x7F,0x41,0x41},
  {0x02,0x04,0x08,0x10,0x20}, {0x41,0x41,0x7F,0x00,0x00},
  {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
  {0x00,0x00,0x03,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78},
  {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
  {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18},
  {0x08,0x7E,0x09,0x01,0x02}, {0x08,0x14,0x54,0x54,0x3C},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00},
  {0x20,0x40,0x44,0x3D,0x00}, {0x00,0x7F,0x10,0x28,0x44},
  {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78},
  {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
  {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C},
  {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C},
  {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
  {0x44,0x64,0x54,0x4C,0x44},
};
/* USER CODE END PD */

/* USER CODE BEGIN 0 */
/* ======================== OLED (SPI2, PD3-6) ======================== */
static uint8_t oled_buf[128][8];

static void spi2_write(uint8_t d)
{
    while (!(SPI2->SR & SPI_SR_TXE));
    *(volatile uint8_t *)&SPI2->DR = d;
    while (SPI2->SR & SPI_SR_BSY);
}

#define OLED_DC_LO()  (PORT_OLED->BSRR = (uint32_t)PIN_OLED_DC << 16)
#define OLED_DC_HI()  (PORT_OLED->BSRR = PIN_OLED_DC)
#define OLED_RES_LO() (PORT_OLED->BSRR = (uint32_t)PIN_OLED_RES << 16)
#define OLED_RES_HI() (PORT_OLED->BSRR = PIN_OLED_RES)

static void oled_cmd(uint8_t c) { OLED_DC_LO(); spi2_write(c); }
static void oled_dat(uint8_t d) { OLED_DC_HI(); spi2_write(d); }

static void OLED_Init(void)
{
    OLED_RES_LO(); HAL_Delay(10); OLED_RES_HI(); HAL_Delay(10);
    oled_cmd(0xAE); oled_cmd(0xD5); oled_cmd(0x50); oled_cmd(0xA8); oled_cmd(0x3F);
    oled_cmd(0xD3); oled_cmd(0x00); oled_cmd(0x40); oled_cmd(0x8D); oled_cmd(0x14);
    oled_cmd(0x20); oled_cmd(0x02); oled_cmd(0xA1); oled_cmd(0xC8); oled_cmd(0xDA); oled_cmd(0x12);
    oled_cmd(0x81); oled_cmd(0xEF); oled_cmd(0xD9); oled_cmd(0xF1); oled_cmd(0xDB); oled_cmd(0x30);
    oled_cmd(0xA4); oled_cmd(0xA6); oled_cmd(0xAF);
}

static void OLED_Clear(void)
{
    for (int p = 0; p < 8; p++)
        for (int c = 0; c < 128; c++)
            oled_buf[c][p] = 0;
}

static void OLED_Flush(void)
{
    for (int p = 0; p < 8; p++) {
        oled_cmd(0xB0 | p); oled_cmd(0x00); oled_cmd(0x10);
        for (int c = 0; c < 128; c++) oled_dat(oled_buf[c][p]);
    }
}

static void OLED_DrawChar(char ch, uint8_t col, uint8_t page)
{
    int idx = (int)(unsigned char)ch - 32;
    if (idx < 0 || idx > 90) idx = 0;
    if (col > 122) return;
    const uint8_t *g = font5x8[idx];
    for (int i = 0; i < 5; i++) oled_buf[col + i][page] = g[i];
    oled_buf[col + 5][page] = 0;
}

static void OLED_Str(const char *s, uint8_t col, uint8_t page)
{
    while (*s) { OLED_DrawChar(*s++, col, page); col += 6; if (col > 122) break; }
}

/* ======================== OLED 除錯頁面 ======================== */
static void OLED_ShowDebug(State_t state, float error, uint16_t speed,
                           uint8_t gray, uint32_t elapsed_ms)
{
    OLED_Clear();

    /* Row 0: 狀態 */
    switch (state) {
        case STATE_IDLE:     OLED_Str("READY  Press BTN", 0, 0); break;
        case STATE_RUNNING:  OLED_Str("RUNNING", 0, 0);          break;
        case STATE_FINISHED: OLED_Str("FINISHED!", 0, 0);        break;
        case STATE_OOB:      OLED_Str("OUT OF BOUNDS!", 0, 0);   break;
    }

    /* Row 2: 誤差 bar */
    char buf[24];
    if (error > 800.0f) {
        snprintf(buf, sizeof(buf), "Line: LOST");
    } else if (error < -800.0f) {
        snprintf(buf, sizeof(buf), "Line: FULL");
    } else {
        snprintf(buf, sizeof(buf), "E:%+5.1fmm S:%3u", (double)error, speed);
    }
    OLED_Str(buf, 0, 2);

    /* Row 4: 灰階 8-bit pattern */
    char gbuf[12];
    for (int i = 0; i < 8; i++)
        gbuf[7 - i] = (gray & (1 << i)) ? '1' : '0';
    gbuf[8] = '\0';
    OLED_Str("Gray:", 0, 4);
    OLED_Str(gbuf, 36, 4);

    /* Row 6: 時間 */
    uint32_t sec = elapsed_ms / 1000;
    uint32_t ms  = elapsed_ms % 1000;
    snprintf(buf, sizeof(buf), "T:%2lu.%03lu", (unsigned long)sec, (unsigned long)ms);
    OLED_Str(buf, 0, 6);

    OLED_Flush();
}
/* USER CODE END 0 */

/* USER CODE BEGIN PV */
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
    /* HAL 初始化 */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* SPI2 (OLED) — PD3=SCK, PD4=MOSI */
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM
              | (2 << 3)          /* BR: fPCLK/8 = 2MHz @ 16MHz */
              | SPI_CR1_SPE;

    /* TIM2 PWM (馬達) + TIM14 ESC (風扇) */
    Motor_TIM_Init();
    Fan_Init();

    /* TIM1 + TIM8 Quadrature Encoder */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN;
    TIM1->CCMR1 = 0x4141;  /* CC1/CC2 = input, mapped to TI1/TI2 */
    TIM1->CCER  = 0x0011;  /* CC1E, CC2E */
    TIM1->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;
    TIM1->CR1   = TIM_CR1_CEN;
    TIM8->CCMR1 = 0x4141;
    TIM8->CCER  = 0x0011;
    TIM8->SMCR  = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;
    TIM8->CR1   = TIM_CR1_CEN;

    /* 馬達初始化 */
    Motor_Init();
    Fan_SetSpeed(FAN_SPEED_US);
    Fan_SetDir(1);

    /* PID 初始化 */
    PID_t pid_pos;
    PID_Init(&pid_pos, KP_INIT, KI_INIT, KD_INIT, I_LIMIT, OUTPUT_LIMIT);

    /* OLED */
    OLED_Init();
    OLED_Clear();
    OLED_ShowDebug(STATE_IDLE, 0.0f, 0, 0x00, 0);

    /* ======================== 主迴圈 ======================== */
    State_t state = STATE_IDLE;
    uint32_t last_loop   = HAL_GetTick();
    uint32_t start_time  = 0;
    uint32_t elapsed_ms  = 0;
    uint8_t  oob_cnt     = 0;

    while (1)
    {
        /* ---- 100Hz timing ---- */
        uint32_t now = HAL_GetTick();
        if (now - last_loop < LOOP_MS) continue;
        float dt = (float)(now - last_loop) / 1000.0f;
        last_loop = now;

        /* ---- 按鍵 (含 debounce) ---- */
        uint8_t btn_raw = (PORT_BTN->IDR & PIN_BTN) ? 1 : 0;
        static uint8_t btn_debounce_cnt;
        static uint8_t btn_state;

        if (btn_raw == btn_state) {
            btn_debounce_cnt = 0;
        } else {
            btn_debounce_cnt++;
            if (btn_debounce_cnt >= 3) {  /* 30ms debounce @ 100Hz */
                btn_state = btn_raw;
                btn_debounce_cnt = 0;
                /* 按下事件 */
                if (btn_state) {
                    if (state == STATE_IDLE) {
                        state = STATE_RUNNING;
                        start_time = now;
                        elapsed_ms = 0;
                        PID_Reset(&pid_pos);
                    }
                }
            }
        }

        /* ---- 狀態機 ---- */
        switch (state) {

        case STATE_IDLE:
            Motor_Brake();
            break;

        case STATE_RUNNING: {
            elapsed_ms = now - start_time;

            /* 讀取灰階 */
            uint8_t gray = Gray_Read();
            float error = Line_GetError(gray);

            /* 出界偵測 */
            if (error == LINE_LOST || (error > -800.0f && error < 800.0f
                                       && (error > OOB_ERROR_MM || error < -OOB_ERROR_MM))) {
                oob_cnt++;
                if (oob_cnt > OOB_COUNT_MAX) {
                    state = STATE_OOB;
                    Motor_Brake();
                    break;
                }
            } else {
                if (oob_cnt > 0) oob_cnt--;
            }

            /* PID 計算 */
            float steering = PID_Compute(&pid_pos, 0.0f, error, dt);

            /* 馬達輸出 */
            int16_t pwm_l = (int16_t)BASE_SPEED + (int16_t)steering;
            int16_t pwm_r = (int16_t)BASE_SPEED - (int16_t)steering;
            Motor_SetSpeed(pwm_l, pwm_r);
            break;
        }

        case STATE_FINISHED:
        case STATE_OOB:
            Motor_Brake();
            break;
        }

        /* ---- OLED 更新 (10Hz) ---- */
        static uint32_t last_oled;
        if (now - last_oled >= 100) {
            last_oled = now;
            uint8_t gray = Gray_Read();
            float error = Line_GetError(gray);
            OLED_ShowDebug(state, error, BASE_SPEED, gray, elapsed_ms);
        }
    }
}

/* ======================== 系統時脈 ======================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    o.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    o.HSIState            = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV1;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/* ======================== GPIO 初始化 (新 pinout) ======================== */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* ---- PORTA: 馬達 + ESC ---- */
    g.Pin   = PIN_AIN1 | PIN_AIN2 | PIN_STBY | PIN_BIN1 | PIN_BIN2;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PORT_MOTOR, &g);

    /* PWMA (PA1) — TIM2_CH2, AF1 */
    g.Pin       = PIN_PWMA;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(PORT_MOTOR, &g);

    /* PWMB (PA3) — TIM2_CH4, AF1 */
    g.Pin       = PIN_PWMB;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(PORT_MOTOR, &g);

    /* ESC_PWM (PA7) — TIM14_CH1, AF9 */
    g.Pin       = PIN_ESC;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Alternate = GPIO_AF9_TIM14;
    HAL_GPIO_Init(PORT_ESC_FAN, &g);

    /* ---- PORTB: 控制 ---- */
    /* ESC_DIR (PB1) */
    g.Pin   = PIN_ESC_DIR;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;
    HAL_GPIO_Init(PORT_ESC_DIR, &g);

    /* IMU Soft SPI: PB3=SCK, PB5=MOSI, PB7=CS (output) */
    g.Pin   = PIN_IMU_SCK | PIN_IMU_MOSI | PIN_IMU_CS;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PORT_IMU, &g);

    /* IMU MISO (PB6) — input */
    g.Pin   = PIN_IMU_MISO;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(PORT_IMU, &g);

    /* BTN (PB15) — input */
    g.Pin   = PIN_BTN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLDOWN;
    HAL_GPIO_Init(PORT_BTN, &g);

    /* ---- PORTC: 右編碼器 (TIM8) ---- */
    g.Pin       = PIN_E2A | PIN_E2B;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLDOWN;
    g.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(PORT_ENC_R, &g);

    /* ---- PORTD: OLED ---- */
    /* PD3(SCK), PD4(MOSI) — SPI2, AF5 */
    g.Pin       = PIN_OLED_SCK | PIN_OLED_MOSI;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PORT_OLED, &g);

    /* PD5(DC), PD6(RES) — GPIO OUT */
    g.Pin       = PIN_OLED_DC | PIN_OLED_RES;
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Alternate = 0;
    HAL_GPIO_Init(PORT_OLED, &g);

    /* ---- PORTE: 灰階 (input) + 左編碼器 (TIM1) ---- */
    /* 灰階 8ch (input pull-down) */
    g.Pin       = PIN_GRAY_CH1 | PIN_GRAY_CH2 | PIN_GRAY_CH3 | PIN_GRAY_CH4
                | PIN_GRAY_CH5 | PIN_GRAY_CH6 | PIN_GRAY_CH7 | PIN_GRAY_CH8;
    g.Mode      = GPIO_MODE_INPUT;
    g.Pull      = GPIO_PULLDOWN;
    g.Alternate = 0;
    HAL_GPIO_Init(PORT_GRAY, &g);

    /* 左編碼器 PE9, PE11 — TIM1_CH1/CH2, AF1 */
    g.Pin       = PIN_E1A | PIN_E1B;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLDOWN;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(PORT_ENC_L, &g);

    /* 初始輸出狀態 */
    HAL_GPIO_WritePin(PORT_MOTOR, PIN_STBY, GPIO_PIN_SET);  /* STBY = HIGH */
    OLED_DC_LO();
    OLED_RES_HI();
    Fan_SetDir(1);
    Motor_Coast();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
