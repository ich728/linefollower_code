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
#define LOOP_MS         5        /* 控制迴圈週期 (200Hz)          */
#define MOTOR_DIAGNOSTIC_MODE 0  /* 1=低速左右輪/編碼器診斷       */
#define OLED_ENABLED         0  /* 實機未安裝 OLED，禁止阻塞刷新   */
#define ESC_DIAGNOSTIC_MODE  0  /* 禁止開機按住按鍵進入隱藏模式   */
#define DIAG_PWM              120
#define DIAG_RUN_MS          1000U
#define FIXED_SPEED     200      /* 降速以減弱慣性                 */
#define RIGHT_TRIM      25       /* 右輪 PWM 補償                  */

/* PID */
#define KP_INIT         1.35f    /* 提高彎道跟隨能力               */
#define KI_INIT         0.00f    /* 暫不積分，避免出彎後積分甩尾   */
#define KD_INIT         0.015f   /* 小 D 抑制左右擺動              */
#define I_LIMIT         100.0f
#define OUTPUT_LIMIT    150.0f   /* 允許急彎有足夠左右輪差         */

/* 出界判定閾值 */
#define OOB_ERROR_MM    40.0f    /* 線誤差超過此值視為偏離       */
#define LINE_LOST_GRACE_COUNT 64 /* 约 320ms 沿最后方向寻找黑线    */
#define LINE_LOST_SPEED       70 /* 急弯失线期间更低速搜索         */
#define LINE_LOST_STEER_GAIN 1.00f
#define SHARP_TURN_ERROR_MM  30.0f
#define SHARP_TURN_GAIN       1.25f
#define SHARP_TURN_LIMIT    175.0f
#define SENSOR_LEFT_HALF_MASK   0x0FU /* CH1-CH4 */
#define SENSOR_RIGHT_HALF_MASK  0xF0U /* CH5-CH8 */
#define SENSOR_CENTER_MASK      0x18U /* CH4-CH5 */
#define ZIGZAG_SPEED             125
#define ZIGZAG_STEERING        115.0f
#define ZIGZAG_MAX_COUNT         80U  /* 单次强制急转最多约 400ms */
#define ZIGZAG_MIN_COUNT          6U  /* 至少保持约 30ms */
#define GAP_ENTRY_ERROR_MM       12.0f /* 接近中心时全白判为直线断线 */
#define GAP_CROSS_SPEED           180
#define GAP_MAX_ENCODER_COUNTS  24000U /* 双轮平均计数安全上限 */
#define GAP_MAX_COUNT             200U /* 最多约 1 秒 */

/* ======================== 狀態機 ======================== */
typedef enum {
    STATE_IDLE,          /* 等待按鍵啟動                        */
    STATE_COUNTDOWN,     /* 倒數 3 秒後啟動                     */
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
static int32_t  g_enc_diff;    /* 編碼器差值 */
static float    g_steering;     /* PID 輸出供 OLED */

/* ======================== OLED (SPI2, PD3-6) ======================== */
static uint8_t oled_buf[128][8];

#define OLED_DC_LO()  (PORT_OLED->BSRR = (uint32_t)PIN_OLED_DC << 16)
#define OLED_DC_HI()  (PORT_OLED->BSRR = PIN_OLED_DC)
#define OLED_RES_LO() (PORT_OLED->BSRR = (uint32_t)PIN_OLED_RES << 16)
#define OLED_RES_HI() (PORT_OLED->BSRR = PIN_OLED_RES)
#define OLED_SCK_LO() (PORT_OLED->BSRR = (uint32_t)PIN_OLED_SCK << 16)
#define OLED_SCK_HI() (PORT_OLED->BSRR = PIN_OLED_SCK)
#define OLED_MOSI_LO() (PORT_OLED->BSRR = (uint32_t)PIN_OLED_MOSI << 16)
#define OLED_MOSI_HI() (PORT_OLED->BSRR = PIN_OLED_MOSI)

/* 軟體 SPI (bit-bang) — PD4 沒有硬體 SPI2_MOSI */
static void oled_spi_dly(void) { for (volatile int i = 0; i < 5; i++); }

static void oled_spi_write(uint8_t d)
{
    for (int i = 7; i >= 0; i--) {
        if (d & (1 << i))
            OLED_MOSI_HI();
        else
            OLED_MOSI_LO();
        oled_spi_dly();
        OLED_SCK_HI();
        oled_spi_dly();
        OLED_SCK_LO();
        oled_spi_dly();
    }
}

static void oled_cmd(uint8_t c) { OLED_DC_LO(); oled_spi_write(c); }
static void oled_dat(uint8_t d) { OLED_DC_HI(); oled_spi_write(d); }

static void OLED_Init(void)
{
    OLED_RES_LO(); HAL_Delay(10); OLED_RES_HI(); HAL_Delay(10);
    oled_cmd(0xAE); oled_cmd(0xD5); oled_cmd(0x50); oled_cmd(0xA8); oled_cmd(0x3F);
    oled_cmd(0xD3); oled_cmd(0x00); oled_cmd(0x40); oled_cmd(0x8D); oled_cmd(0x14);
    oled_cmd(0x20); oled_cmd(0x02); oled_cmd(0xA1); oled_cmd(0xC8); oled_cmd(0xDA); oled_cmd(0x12);
    oled_cmd(0x81); oled_cmd(0xEF); oled_cmd(0xD9); oled_cmd(0xF1); oled_cmd(0xDB); oled_cmd(0x30);
    oled_cmd(0xA4); oled_cmd(0xA6);
    /* 清空 GDDRAM 再開顯示，防止開機亂碼 */
    for (int p = 0; p < 8; p++) {
        oled_cmd(0xB0 | p); oled_cmd(0x00); oled_cmd(0x10);
        for (int c = 0; c < 128; c++) oled_dat(0x00);
    }
    oled_cmd(0xAF);
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
        case STATE_COUNTDOWN: {
            char cd[16];
            int sec = 3 - (int)(elapsed_ms / 1000);
            snprintf(cd, sizeof(cd), "START IN %d", sec);
            OLED_Str(cd, 12, 0);
            break;
        }
        case STATE_RUNNING:  OLED_Str("RUNNING", 0, 0);          break;
        case STATE_FINISHED: OLED_Str("FINISHED!", 0, 0);        break;
        case STATE_OOB:      OLED_Str("OUT OF BOUNDS!", 0, 0);   break;
    }

    /* Row 2: steering + speed */
    char buf[24];
    int st_int = (int)error;
    snprintf(buf, sizeof(buf), "St:%+4d Sp:%3u", st_int, speed);
    OLED_Str(buf, 0, 2);

    /* Row 4: 灰階 8-bit pattern */
    char gbuf[12];
    for (int i = 0; i < 8; i++)
        gbuf[7 - i] = (gray & (1 << i)) ? '1' : '0';
    gbuf[8] = '\0';
    OLED_Str("Gray:", 0, 4);
    OLED_Str(gbuf, 36, 4);

    /* Row 6: 時間 + 編碼器 */
    uint32_t sec = elapsed_ms / 1000;
    uint32_t ms  = elapsed_ms % 1000;
    snprintf(buf, sizeof(buf), "T:%2lu.%03lu d:%+4ld", (unsigned long)sec, (unsigned long)ms, (long)g_enc_diff);
    OLED_Str(buf, 0, 6);

    OLED_Flush();
}

/* ======================== 馬達/編碼器診斷模式 ======================== */
static void Motor_Diagnostic_Run(void)
{
    uint8_t test_step = 0;
    uint8_t btn_last = 0;

    Motor_SetPWM(0, 0);
    Motor_Brake();
    OLED_Clear();
    OLED_Str("MOTOR DIAG", 30, 0);
    OLED_Str("WHEELS UP!", 30, 2);
    OLED_Str("Press BTN", 36, 4);
    OLED_Str("1:L 2:R 3:BOTH", 12, 6);
    OLED_Flush();

    while (1) {
        uint8_t btn = (PORT_BTN->IDR & PIN_BTN) ? 1 : 0;
        uint8_t released = !btn && btn_last;
        btn_last = btn;

        if (!released) {
            HAL_Delay(10);
            continue;
        }

        HAL_Delay(30);
        test_step = (uint8_t)((test_step % 3U) + 1U);
        TIM1->CNT = 0;
        TIM8->CNT = 0;

        OLED_Clear();
        if (test_step == 1) {
            OLED_Str("CMD LEFT", 36, 0);
            OLED_Str("ONLY LEFT?", 30, 3);
            Motor_SetSpeed(DIAG_PWM, 0);
        } else if (test_step == 2) {
            OLED_Str("CMD RIGHT", 33, 0);
            OLED_Str("ONLY RIGHT?", 27, 3);
            Motor_SetSpeed(0, DIAG_PWM);
        } else {
            OLED_Str("CMD BOTH", 36, 0);
            OLED_Str("BOTH FORWARD?", 21, 3);
            Motor_SetSpeed(DIAG_PWM, DIAG_PWM);
        }
        OLED_Flush();

        HAL_Delay(DIAG_RUN_MS);
        Motor_SetPWM(0, 0);
        Motor_Brake();

        int32_t count_l = (int16_t)(uint16_t)TIM1->CNT;
        int32_t count_r = (int16_t)(uint16_t)TIM8->CNT;
        char line[24];

        OLED_Clear();
        OLED_Str("TEST DONE", 36, 0);
        snprintf(line, sizeof(line), "TIM1 L:%+5ld", (long)count_l);
        OLED_Str(line, 6, 2);
        snprintf(line, sizeof(line), "TIM8 R:%+5ld", (long)count_r);
        OLED_Str(line, 6, 4);
        OLED_Str("Press next", 33, 6);
        OLED_Flush();
    }
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

    /* TIM2 PWM (馬達) */
    Motor_TIM_Init();
    /* 負壓風扇：TIM4 不啟動, PB8 = GPIO OUT LOW */

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

#if MOTOR_DIAGNOSTIC_MODE && OLED_ENABLED
    OLED_Init();
    Motor_Diagnostic_Run();
#endif

    /* ── ESC 調試模式：開機按住按鍵進入 ── */
#if ESC_DIAGNOSTIC_MODE
    if (PORT_BTN->IDR & PIN_BTN) {
        /* 重設 PB8 為 TIM4 AF2 PWM */
        GPIO_InitTypeDef gt = {0};
        gt.Pin       = PIN_ESC;
        gt.Mode      = GPIO_MODE_AF_PP;
        gt.Pull      = GPIO_NOPULL;
        gt.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(PORT_ESC_FAN, &gt);

        /* 啟動 TIM4 ESC PWM */
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        TIM4->PSC  = 159;   TIM4->ARR  = 1999;
        TIM4->CCR3 = 110;   /* 1100µs */
        TIM4->CCMR2 = (6 << 4) | (1 << 3);
        TIM4->CCER  = TIM_CCER_CC3E;
        TIM4->CR1   = TIM_CR1_CEN;

        OLED_Clear();
        OLED_Str("ESC CAL MODE", 0, 0);
        OLED_Str("Release BTN", 0, 2);
        OLED_Str("then press:",  0, 4);
        OLED_Str("1940 <-> 1100", 0, 6);
        OLED_Flush();

        /* 放開按鍵 */
        while (PORT_BTN->IDR & PIN_BTN) HAL_Delay(10);

        uint16_t esc_us = 1100;
        uint8_t  btn_last = 0;
        char buf[24];

        OLED_Clear();
        snprintf(buf, sizeof(buf), "ESC: %4u us", esc_us);
        OLED_Str(buf, 12, 3);
        OLED_Flush();

        while (1) {
            uint8_t btn = (PORT_BTN->IDR & PIN_BTN) ? 1 : 0;
            uint8_t rising = btn && !btn_last;

            if (rising) {
                /* 按鍵: 1940 ↔ 1100 切換 */
                esc_us = (esc_us == 1100) ? 1940 : 1100;
                TIM4->CCR3 = esc_us / 10;

                OLED_Clear();
                snprintf(buf, sizeof(buf), "ESC: %4u us", esc_us);
                OLED_Str(buf, 12, 3);
                OLED_Flush();
            }

            btn_last = btn;
            HAL_Delay(10);
        }
    }
#endif
    /* PID 初始化 */
    PID_t pid_pos;
    PID_Init(&pid_pos, KP_INIT, KI_INIT, KD_INIT, I_LIMIT, OUTPUT_LIMIT);

    /* ===== OLED 啟動提示 ===== */
#if OLED_ENABLED
    OLED_Init();
    OLED_Clear();

    /* 就緒 */
    OLED_Clear();
    OLED_Str("READY", 36, 1);
    OLED_Str("Press BTN", 24, 3);
    OLED_Flush();
#endif

    /* ======================== 主迴圈 ======================== */
    State_t state = STATE_IDLE;
    uint32_t last_loop   = HAL_GetTick();
    uint32_t start_time  = 0;
    uint32_t elapsed_ms  = 0;
    uint8_t  line_lost_cnt = 0;
    float last_valid_steering = 0.0f;
    int8_t zigzag_dir = 0;       /* -1=左急转, +1=右急转 */
    uint8_t zigzag_count = 0;
    uint8_t gap_mode = 0;
    uint16_t gap_count = 0;
    uint32_t gap_encoder_counts = 0;
    float last_valid_error = 0.0f;

    /* 編碼器 */
    uint16_t enc_l_prev = (uint16_t)TIM1->CNT;
    uint16_t enc_r_prev = (uint16_t)TIM8->CNT;

    while (1)
    {
        /* ---- 100Hz timing ---- */
        uint32_t now = HAL_GetTick();
        if (now - last_loop < LOOP_MS) continue;
        float dt = (float)(now - last_loop) / 1000.0f;
        last_loop = now;

        /* ---- 按鍵 (含 debounce + 長短按) ---- */
        uint8_t btn_raw = (PORT_BTN->IDR & PIN_BTN) ? 1 : 0;
        static uint8_t btn_db_cnt;
        static uint8_t btn_state;
        static uint8_t btn_prev;

        /* Debounce */
        if (btn_raw == btn_state) {
            btn_db_cnt = 0;
        } else {
            btn_db_cnt++;
            if (btn_db_cnt >= 3) {  /* 30ms debounce @ 100Hz */
                btn_state = btn_raw;
                btn_db_cnt = 0;
            }
        }

        /* 按鍵放開觸發 (falling edge) */
        uint8_t btn_falling = !btn_state && btn_prev;

        if (btn_falling && state == STATE_IDLE) {
            state = STATE_COUNTDOWN;
            start_time = now;
            PID_Reset(&pid_pos);
            line_lost_cnt = 0;
            last_valid_steering = 0.0f;
            zigzag_dir = 0;
            zigzag_count = 0;
            gap_mode = 0;
            gap_count = 0;
            gap_encoder_counts = 0;
            last_valid_error = 0.0f;
            enc_l_prev = (uint16_t)TIM1->CNT;
            enc_r_prev = (uint16_t)TIM8->CNT;
        }

        btn_prev = btn_state;

        /* ---- 狀態機 ---- */
        switch (state) {

        case STATE_IDLE:
            Motor_Brake();
            break;

        case STATE_COUNTDOWN: {
            /* 倒數期間預讀感測器, 避免起跑 derivative kick */
            uint8_t g = Gray_Read();
            float e = Line_GetError(g);
            if (e < 800.0f && e > -800.0f) {
                PID_Compute(&pid_pos, 0.0f, e, dt);  /* 預熱 prev_error */
            }

            elapsed_ms = now - start_time;
            if (elapsed_ms >= 3000) {
                state = STATE_RUNNING;
                start_time = now;
                elapsed_ms = 0;
            }
            Motor_Brake();
            break;
        }

        case STATE_RUNNING: {
            elapsed_ms = now - start_time;

            /* 讀取灰階 → 加權平均 → error */
            uint8_t gray = Gray_Read();
            float   error = Line_GetError(gray);

            /* 每周期读取双编码器增量，int16 转换可处理 16-bit 回绕。 */
            uint16_t enc_l_now = (uint16_t)TIM1->CNT;
            uint16_t enc_r_now = (uint16_t)TIM8->CNT;
            int16_t enc_l_delta = (int16_t)(enc_l_now - enc_l_prev);
            int16_t enc_r_delta = (int16_t)(enc_r_now - enc_r_prev);
            enc_l_prev = enc_l_now;
            enc_r_prev = enc_r_now;
            uint32_t enc_l_abs =
                (uint32_t)((enc_l_delta < 0) ? -enc_l_delta : enc_l_delta);
            uint32_t enc_r_abs =
                (uint32_t)((enc_r_delta < 0) ? -enc_r_delta : enc_r_delta);

            /*
             * 锯齿路段特征：
             *   右半 CH5-CH8 全黑且左半未全黑 -> 强制右转
             *   左半 CH1-CH4 全黑且右半未全黑 -> 强制左转
             * 全黑 0xFF 不触发，避免把起终点标记误判为锯齿。
             */
            uint8_t left_half_black =
                (gray & SENSOR_LEFT_HALF_MASK) == SENSOR_LEFT_HALF_MASK;
            uint8_t right_half_black =
                (gray & SENSOR_RIGHT_HALF_MASK) == SENSOR_RIGHT_HALF_MASK;

            if (right_half_black && !left_half_black) {
                if (zigzag_dir != 1) {
                    zigzag_dir = 1;
                    zigzag_count = 0;
                }
            } else if (left_half_black && !right_half_black) {
                if (zigzag_dir != -1) {
                    zigzag_dir = -1;
                    zigzag_count = 0;
                }
            }

            if (zigzag_dir != 0) {
                if (zigzag_count < ZIGZAG_MAX_COUNT) zigzag_count++;

                /*
                 * 强制转向后，中央重新捕获窄线且半区全黑特征消失，
                 * 才交还给普通 PD。下一折若出现相反半区全黑，会直接
                 * 切换方向。
                 */
                uint8_t normal_center_reacquired =
                    (gray & SENSOR_CENTER_MASK) != 0U &&
                    !left_half_black && !right_half_black &&
                    error > -18.0f && error < 18.0f;

                if ((zigzag_count >= ZIGZAG_MIN_COUNT &&
                     normal_center_reacquired) ||
                    zigzag_count >= ZIGZAG_MAX_COUNT) {
                    zigzag_dir = 0;
                    zigzag_count = 0;
                    if (error != LINE_LOST && error != LINE_FULL) {
                        pid_pos.prev_error = -error;
                    }
                }
            }

            /*
             * 赛规直线断线：进入全白前，黑线必须在中央附近且不处于
             * 锯齿模式。断线期间固定直行，并以双轮平均编码器计数和
             * 1 秒超时作为双重安全限制。
             */
            if (error == LINE_LOST && zigzag_dir == 0 && !gap_mode) {
                float last_error_abs =
                    (last_valid_error >= 0.0f)
                    ? last_valid_error : -last_valid_error;
                if (last_error_abs <= GAP_ENTRY_ERROR_MM) {
                    gap_mode = 1;
                    gap_count = 0;
                    gap_encoder_counts = 0;
                    PID_Reset(&pid_pos);
                }
            }

            if (gap_mode) {
                if (error != LINE_LOST && error != LINE_FULL) {
                    gap_mode = 0;
                    gap_count = 0;
                    gap_encoder_counts = 0;
                    pid_pos.prev_error = -error;
                } else {
                    if (gap_count < GAP_MAX_COUNT) gap_count++;
                    gap_encoder_counts += (enc_l_abs + enc_r_abs) / 2U;
                    if (gap_count >= GAP_MAX_COUNT ||
                        gap_encoder_counts >= GAP_MAX_ENCODER_COUNTS) {
                        Motor_SetPWM(0, 0);
                        Motor_Brake();
                        PID_Reset(&pid_pos);
                        g_steering = 0.0f;
                        state = STATE_OOB;
                        break;
                    }
                }
            }

            /*
             * 感测器靠近车轮，弯道中可能短暂全白。此时沿用最后有效
             * 转向并降速寻找黑线；持续约 180ms 仍未找回才停车。
             */
            if (error == LINE_LOST && zigzag_dir == 0 && !gap_mode) {
                if (line_lost_cnt < LINE_LOST_GRACE_COUNT) line_lost_cnt++;
                if (line_lost_cnt >= LINE_LOST_GRACE_COUNT) {
                    Motor_SetPWM(0, 0);
                    Motor_Brake();
                    PID_Reset(&pid_pos);
                    g_steering = 0.0f;
                    state = STATE_OOB;
                    break;
                }
            } else {
                if (line_lost_cnt > 0 && error != LINE_FULL) {
                    /* 重新捕获黑线时同步 D 项，避免瞬时反向冲击。 */
                    pid_pos.prev_error = -error;
                }
                line_lost_cnt = 0;
            }

            float steering = 0.0f;
            float e_abs = (error > 0) ? error : -error;

            if (gap_mode) {
                steering = 0.0f;
            } else if (zigzag_dir > 0) {
                steering = -ZIGZAG_STEERING;
            } else if (zigzag_dir < 0) {
                steering = ZIGZAG_STEERING;
            } else if (error == LINE_LOST) {
                steering = last_valid_steering * LINE_LOST_STEER_GAIN;
            } else if (error == LINE_FULL) {
                steering = 0.0f;
            } else {
                steering = PID_Compute(&pid_pos, 0.0f, error, dt);
                /*
                 * 黑线进入外侧感测器时视为急弯。只在边缘区域增加转向，
                 * 避免改变直线与普通弯道的手感。
                 */
                if (e_abs >= SHARP_TURN_ERROR_MM) {
                    steering *= SHARP_TURN_GAIN;
                    if (steering > SHARP_TURN_LIMIT) {
                        steering = SHARP_TURN_LIMIT;
                    } else if (steering < -SHARP_TURN_LIMIT) {
                        steering = -SHARP_TURN_LIMIT;
                    }
                }
                last_valid_steering = steering;
                last_valid_error = error;
            }

            /* 速度曲線: 陡降, 大彎更慢 = 更多修正時間 */
            float ratio = 1.0f - e_abs * 0.014f;
            if (ratio < 0.35f) ratio = 0.35f;
            uint16_t cur_speed = (uint16_t)((float)FIXED_SPEED * ratio);
            if (gap_mode) {
                cur_speed = GAP_CROSS_SPEED;
            } else if (zigzag_dir != 0) {
                cur_speed = ZIGZAG_SPEED;
            } else if (error == LINE_LOST) {
                cur_speed = LINE_LOST_SPEED;
            }

            /* 馬達輸出 */
            int16_t pwm_l = (int16_t)cur_speed - (int16_t)steering;
            int16_t pwm_r = (int16_t)cur_speed + (int16_t)steering + RIGHT_TRIM;
            /* 首輪彎道測試不允許單輪反轉，避免直接甩出賽道。 */
            if (pwm_l < 0) pwm_l = 0;
            if (pwm_r < 0) pwm_r = 0;
            g_steering = steering;  /* 供 OLED */
            Motor_SetSpeed(pwm_l, pwm_r);
            break;
        }

        case STATE_FINISHED:
        case STATE_OOB:
            Motor_Brake();
            break;
        }

        /* ---- OLED 更新 (10Hz) ---- */
#if OLED_ENABLED
        static uint32_t last_oled;
        if (now - last_oled >= 100) {
            last_oled = now;
            uint8_t gray = Gray_Read();
            float error = Line_GetError(gray);
            OLED_ShowDebug(state, g_steering, FIXED_SPEED, gray, elapsed_ms);
        }
#endif
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

    /* ---- PORTA: 馬達 ---- */
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

    /* ---- PORTB: 控制 ---- */
    /* ESC_PWM (PB8) — GPIO OUT LOW (負壓風扇禁用) */
    g.Pin       = PIN_ESC;
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Alternate = 0;
    HAL_GPIO_Init(PORT_ESC_FAN, &g);

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

    /* ---- PORTD: OLED (軟體 SPI, 全 GPIO OUT) ---- */
    g.Pin       = PIN_OLED_SCK | PIN_OLED_MOSI | PIN_OLED_DC | PIN_OLED_RES;
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
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
    Motor_Coast();
    HAL_GPIO_WritePin(PORT_ESC_FAN, PIN_ESC, GPIO_PIN_RESET);  /* 強制 PB8 LOW */
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
