# 循跡機器人 — 最終腳位定義表

> 版本: 1.0 | 日期: 2026-06-02 | 以 `PINOUT.md` + `stm32_pinout_definition.md` 為唯一依據

---

## 設計原則

| # | 原則 | 說明 |
|---|------|------|
| 1 | TB6612 全組連續 | AIN1, PWMA, AIN2, PWMB, STBY, BIN1, BIN2, ESC_PWM 集中在 PA[0:7] |
| 2 | 雙編碼器硬體 Quadrature | TIM1 (32-bit) + TIM8 (32-bit)，無軟體方向判斷 |
| 3 | 8ch 灰階單 Port 讀取 | 全部在 PORTE，一次 IDR + bit 移位 |
| 4 | 近端 = 馬達+感測，遠端 = 通訊+顯示 | 接線邏輯清晰，除錯互不干擾 |
| 5 | 僅使用排針上已引出的腳位 | 不虛構不存在於 `stm32_pinout_definition.md` 的腳位 |

---

## 完整腳位分配表

### PORTA [0:7] — 🏎️ 馬達專區（近端 R14-R17，連續 8 腳）

| Pin | 功能 | 模式 | 定時器 | AF | 位置 |
|-----|------|------|--------|-----|------|
| PA0 | **AIN1** (左輪 H 橋) | GPIO OUT | — | — | 近 R17 內 |
| PA1 | **PWMA** (左輪 PWM) | AF PP | TIM2_CH2 | AF1 | 近 R17 外 |
| PA2 | **AIN2** (左輪 H 橋) | GPIO OUT | — | — | 近 R16 內 |
| PA3 | **PWMB** (右輪 PWM) | AF PP | TIM2_CH4 | AF1 | 近 R16 外 |
| PA4 | **STBY** (馬達致能) | GPIO OUT | — | — | 近 R15 內 |
| PA5 | **BIN1** (右輪 H 橋) | GPIO OUT | — | — | 近 R15 外 |
| PA6 | **BIN2** (右輪 H 橋) | GPIO OUT | — | — | 近 R14 內 |
| PA7 | **ESC_PWM** (負壓風扇) | AF PP | TIM14_CH1 | AF9 | 近 R14 外 |

> PWMA/PWMB 同屬 **TIM2**，單次 CCR 更新即可同步輸出，無相位抖動。
> ESC_PWM 使用 TIM14（獨立 16-bit），不與馬達 PWM 競爭。

### PORTB — 控制與通訊輔助

| Pin | 功能 | 模式 | 定時器 | AF | 位置 |
|-----|------|------|--------|-----|------|
| PB1 | **ESC_DIR** (風扇方向) | GPIO OUT | — | — | 近 R12 外 |
| PB3 | **IMU_SCK** | GPIO OUT | — | — | 遠 R20 內 |
| PB5 | **IMU_MOSI** | GPIO OUT | — | — | 遠 R20 外 |
| PB6 | **IMU_MISO** | GPIO IN | — | — | 遠 R21 內 |
| PB7 | **IMU_CS** | GPIO OUT | — | — | 遠 R21 外 |
| PB15 | **BTN** (按鍵) | GPIO IN | — | — | 遠 R4 內 |

### PORTC — 右編碼器

| Pin | 功能 | 模式 | 定時器 | AF | 位置 |
|-----|------|------|--------|-----|------|
| PC6 | **E2A** (右編碼器 A) | AF PP | TIM8_CH1 | AF3 | 遠 R8 外 |
| PC7 | **E2B** (右編碼器 B) | AF PP | TIM8_CH2 | AF3 | 遠 R9 內 |

### PORTD — OLED 顯示（遠端 R17-R19，集中插拔）

| Pin | 功能 | 模式 | 定時器 | AF | 位置 |
|-----|------|------|--------|-----|------|
| PD3 | **OLED_SCK** | AF PP | SPI2_SCK | AF5 | 遠 R17 外 |
| PD4 | **OLED_MOSI** | AF PP | SPI2_MOSI | AF5 | 遠 R18 內 |
| PD5 | **OLED_DC** | GPIO OUT | — | — | 遠 R18 外 |
| PD6 | **OLED_RES** | GPIO OUT | — | — | 遠 R19 內 |

> 四條線集中在 3 排之內，一個 4-pin 杜邦頭即可。

### PORTE — 灰階感測器 + 左編碼器（近端）

| Pin | 功能 | 模式 | 定時器 | AF | 位置 |
|-----|------|------|--------|-----|------|
| PE4 | **Gray CH1** | GPIO IN | — | — | 近 R21 內 |
| PE5 | **Gray CH2** | GPIO IN | — | — | 近 R21 外 |
| PE6 | **Gray CH3** | GPIO IN | — | — | 近 R20 內 |
| PE7 | **Gray CH4** | GPIO IN | — | — | 近 R11 內 |
| PE8 | **Gray CH5** | GPIO IN | — | — | 近 R11 外 |
| PE9 | **E1A** (左編碼器 A) | AF PP | TIM1_CH1 | AF1 | 近 R10 內 |
| PE10 | **Gray CH6** | GPIO IN | — | — | 近 R10 外 |
| PE11 | **E1B** (左編碼器 B) | AF PP | TIM1_CH2 | AF1 | 近 R9 內 |
| PE12 | **Gray CH7** | GPIO IN | — | — | 近 R9 外 |
| PE13 | **Gray CH8** | GPIO IN | — | — | 近 R8 內 |

> PE9/PE11 被左編碼器佔用，灰階繞過此二腳。排序從 PE4 (CH1) 到 PE13 (CH8)，物理位置由下往上對應感測器左到右。

---

## 定時器資源分配

| 定時器 | 用途 | 通道 | 解析度 | 備註 |
|--------|------|------|--------|------|
| **TIM1** | 左編碼器 Quadrature | CH1+CH2 | 32-bit | PE9, PE11 |
| **TIM2** | 雙輪 PWM | CH2 (PA1), CH4 (PA3) | 16-bit | PSC=15, ARR=999, 1kHz |
| **TIM4** | — (釋放) | — | — | 原 Demo 用於 ESC，現已移至 TIM14 |
| **TIM8** | 右編碼器 Quadrature | CH1+CH2 | 32-bit | PC6, PC7 |
| **TIM14** | ESC PWM (負壓風扇) | CH1 (PA7) | 16-bit | PSC=159, ARR=1999, 50Hz |

> TIM3, TIM4, TIM5 釋出備用。

---

## 灰階感測器讀取方法

8 個通道散佈在 PORTE 上，中間被 PE9/PE11（編碼器）打斷：

```c
static uint8_t gray_read(void)
{
    uint32_t r = GPIOE->IDR;
    uint8_t gray = 0;

    gray |= (r & (1<<4))  ? (1<<0) : 0;  // PE4  → CH1
    gray |= (r & (1<<5))  ? (1<<1) : 0;  // PE5  → CH2
    gray |= (r & (1<<6))  ? (1<<2) : 0;  // PE6  → CH3
    gray |= (r & (1<<7))  ? (1<<3) : 0;  // PE7  → CH4
    gray |= (r & (1<<8))  ? (1<<4) : 0;  // PE8  → CH5
    gray |= (r & (1<<10)) ? (1<<5) : 0;  // PE10 → CH6
    gray |= (r & (1<<12)) ? (1<<6) : 0;  // PE12 → CH7
    gray |= (r & (1<<13)) ? (1<<7) : 0;  // PE13 → CH8

    return gray;
}
```

> 一次 IDR 讀取 + 逐 bit 判斷，開銷極小。CH1=LSB (PE4), CH8=MSB (PE13)，對應感測器從左到右。

### 線位置計算（加權平均法）

8 個通道間距 12mm，黑線寬 18mm。給每個通道一個權重位置：

| CH | 權重位置 (mm) | 說明 |
|----|:----------:|------|
| CH1 | -42 | 最左 |
| CH2 | -30 | |
| CH3 | -18 | |
| CH4 | -6 | |
| CH5 | +6 | |
| CH6 | +18 | |
| CH7 | +30 | |
| CH8 | +42 | 最右 |

```c
// 加權平均求線位置（0 = 正中，負 = 偏左，正 = 偏右）
#define NUM_SENSORS 8
static const int8_t sensor_pos[NUM_SENSORS] = {-42, -30, -18, -6, 6, 18, 30, 42};

float line_position(uint8_t gray)
{
    int32_t sum = 0, count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (gray & (1 << i)) {
            sum += sensor_pos[i];
            count++;
        }
    }
    if (count == 0) return 999.0f;  // 全白：斷線
    if (count == 8) return -999.0f; // 全黑：異常
    return (float)sum / (float)count;
}
```

---

## TB6612 馬達控制巨集

```c
// 左輪 (1 號 N30)
#define MOTOR_L_FWD()  do { GPIOA->BSRR = (uint32_t)GPIO_PIN_2 << 16; GPIOA->BSRR = GPIO_PIN_0; } while(0)
#define MOTOR_L_REV()  do { GPIOA->BSRR = (uint32_t)GPIO_PIN_0 << 16; GPIOA->BSRR = GPIO_PIN_2; } while(0)
#define MOTOR_L_BRK()  do { GPIOA->BSRR = GPIO_PIN_0 | GPIO_PIN_2; } while(0)
#define MOTOR_L_CST()  do { GPIOA->BSRR = (uint32_t)(GPIO_PIN_0 | GPIO_PIN_2) << 16; } while(0)

// 右輪 (2 號 N30)
#define MOTOR_R_FWD()  do { GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16; GPIOA->BSRR = GPIO_PIN_5; } while(0)
#define MOTOR_R_REV()  do { GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16; GPIOA->BSRR = GPIO_PIN_6; } while(0)
#define MOTOR_R_BRK()  do { GPIOA->BSRR = GPIO_PIN_5 | GPIO_PIN_6; } while(0)
#define MOTOR_R_CST()  do { GPIOA->BSRR = (uint32_t)(GPIO_PIN_5 | GPIO_PIN_6) << 16; } while(0)

#define MOTOR_ENABLE()  (GPIOA->BSRR = GPIO_PIN_4)   // STBY = HIGH
#define MOTOR_STANDBY() (GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16)

// PWM 輸出
#define PWM_L(duty)  (TIM2->CCR2 = (duty))  // PA1, TIM2_CH2
#define PWM_R(duty)  (TIM2->CCR4 = (duty))  // PA3, TIM2_CH4
#define PWM_ESC(us)  (TIM14->CCR1 = (us))   // PA7, TIM14_CH1, 單位 µs
```

---

## ESC 負壓風扇控制

| 項目 | 規格 |
|------|------|
| 訊號腳位 | PA7 (TIM14_CH1) |
| 方向腳位 | PB1 (GPIO OUT) |
| 協定 | Futaba 標準，50Hz（20ms 週期） |
| 油門範圍 | 1000–2000 µs |
| 調速解析度 | 1 µs |
| TIM14 配置 | PSC=159, ARR=1999（16MHz / 160 = 100kHz, 100kHz / 2000 = 50Hz） |

```c
// TIM14 初始化 (ESC 50Hz PWM)
RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
TIM14->PSC = 159;      // 16MHz / 160 = 100kHz
TIM14->ARR = 1999;     // 100kHz / 2000 = 50Hz
TIM14->CCR1 = 1000;    // 初始 1000µs (OFF)
TIM14->CCMR1 = (6 << 4) | (1 << 3);  // PWM mode 1, preload
TIM14->CCER = TIM_CCER_CC1E;
TIM14->CR1 = TIM_CR1_CEN;
```

---

## 編碼器配置

| 項目 | 左編碼器 (TIM1) | 右編碼器 (TIM8) |
|------|:-----------:|:-----------:|
| Pin | PE9, PE11 | PC6, PC7 |
| PPR | 1000 pulse/r (馬達端) | 同 |
| 減速比 | 1:10 | 同 |
| 輪徑 | 24mm | 同 |
| 每 pulse 距離 | 0.00754 mm | 同 |
| 計數範圍 | 32-bit (±2¹⁷ mm) | 同 |

```c
// TIM1/TIM8 編碼器模式初始化
RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN;

// TIM1 (左) — PE9/PE11, AF1
TIM1->CCMR1 = 0x4141;  // CC1/CC2 = input, IC1→TI1, IC2→TI2
TIM1->CCER = 0x0011;   // CC1E, CC2E
TIM1->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;  // encoder mode 3
TIM1->CR1 = TIM_CR1_CEN;

// TIM8 (右) — PC6/PC7, AF3
TIM8->CCMR1 = 0x4141;
TIM8->CCER = 0x0011;
TIM8->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;
TIM8->CR1 = TIM_CR1_CEN;
```

### 里程計讀取

```c
int32_t enc_left(void)  { return (int32_t)TIM1->CNT; }
int32_t enc_right(void) { return (int32_t)TIM8->CNT; }

// 前次值差量計算
static int32_t  enc_l_prev, enc_r_prev;
static uint32_t dist_l_mm10, dist_r_mm10;  // 0.1mm 單位

void odometry_update(void)
{
    int32_t l_now = enc_left();
    int32_t r_now = enc_right();

    int32_t dl = l_now - enc_l_prev;
    int32_t dr = r_now - enc_r_prev;

    // 每 pulse = 0.00754mm → 累積時用 μmm 或 mm×10 避免浮點
    // 1000000 pulse ≈ 7539.8 mm → 用整數近似
    dist_l_mm10 += (int32_t)((int64_t)dl * 75398LL / 10000000LL);
    dist_r_mm10 += (int32_t)((int64_t)dr * 75398LL / 10000000LL);

    enc_l_prev = l_now;
    enc_r_prev = r_now;
}
```

---

## 腳位功能分區總覽

```
═══════════════════════════════════════════════════
  近端（左排針）— 馬達 · 感測 · 編碼器
═══════════════════════════════════════════════════
R5  PB13 (free)     │ PB14 (free)
R7  PE15 (free)     │ PE8  Gray CH5
R8  PE13 Gray CH8   │ PE14 (free)
R9  PE11 E1B ← 左ENC│ PE12 Gray CH7
R10 PE9  E1A ← 左ENC│ PE10 Gray CH6
R11 PE7  Gray CH4   │ PE6  Gray CH3 (R20內) …注意 PE6/PE7 交錯
R12 PB0  (free)     │ PB1  ESC_DIR
R13 PC4  (free)     │ PC5  (free)
R14 PA6  BIN2       │ PA7  ESC_PWM
R15 PA4  STBY       │ PA5  BIN1
R16 PA2  AIN2       │ PA3  PWMB
R17 PA0  AIN1       │ PA1  PWMA
R18 PC2  (free)     │ PC3  (free)
R19 PC0  (free)     │ PC1  (free)
R20 PE6  Gray CH3   │ PC13 (free)
R21 PE4  Gray CH1   │ PE5  Gray CH2
═══════════════════════════════════════════════════
  遠端（右排針）— 通訊 · 顯示 · 按鍵
═══════════════════════════════════════════════════
R4  PB15 BTN ← 按鍵 │ PD8  (free)
...
R8  PD15 (free)     │ PC6  E2A → 右ENC
R9  PC7  E2B → 右ENC│ PC8  (free)
...
R17 PD2  (free)     │ PD3  OLED_SCK
R18 PD4  OLED_MOSI  │ PD5  OLED_DC
R19 PD6  OLED_RES   │ PD7  (free)
R20 PB3  IMU_SCK    │ PB5  IMU_MOSI
R21 PB6  IMU_MISO   │ PB7  IMU_CS
R22 PB8  (free)     │ PB9  (free)
═══════════════════════════════════════════════════
```

---

## 變更記錄

| 日期 | 變更 | 原因 |
|------|------|------|
| 2026-06-02 | 初版 | 從零設計，以 doc 為唯一依據 |
| 2026-06-02 | TB6612 全組 PA[0:7] | 連續接線 |
| 2026-06-02 | ESC_PWM PA7 | 近端集中 |
| 2026-06-02 | 按鍵 PB0→PB15 | 移到遠端 |
| 2026-06-02 | OLED PB13/PC3/PB14/PE15→PD3-6 | 遠端集中插拔 |
