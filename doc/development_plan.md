# 學界循跡機器人 — 韌體開發計劃

> 版本: 1.0 | 日期: 2026-06-02 | 全線方案 B

---

## 目錄

1. [專案概述](#1-專案概述)
2. [競賽規則分析](#2-競賽規則分析)
3. [硬體配置](#3-硬體配置)
4. [系統架構](#4-系統架構)
5. [控制演算法設計](#5-控制演算法設計)
6. [腳位分配總表](#6-腳位分配總表)
7. [模組設計](#7-模組設計)
8. [狀態機](#8-狀態機)
9. [開發階段劃分](#9-開發階段劃分)
10. [測試計劃](#10-測試計劃)
11. [風險評估](#11-風險評估)
12. [參考資源](#12-參考資源)

---

## 1. 專案概述

### 目標

開發一台能在競賽場地（1.2m × 5m 白底黑線）上，以最短時間完成一圈循跡的自主機器人。

### 核心技術選擇（全線方案 B）

| 面向 | 選擇 | 說明 |
|------|------|------|
| PID 架構 | **串級 PID** | 外環位置 PID → 內環雙速度 PID |
| 線位置計算 | **加權平均法** | 8 通道數位灰階，間距 12mm |
| 速度策略 | **基於誤差動態調速** | error → speed_factor 線性映射 |
| 斷線處理 | **IMU 陀螺航向鎖定** | 出線鎖定 Z 軸角度 → 直走通過 |
| 起/終點 | **編碼器距離推算** | 累積脈衝 → 到達目標距離停止 |
| 負壓風扇 | **基於速度動態** | 速度越快 → 下壓力越大 |
| 控制頻率 | **200Hz (5ms)** | 16MHz HSI 完全夠用 |
| 狀態機 | **含異常處理** | RUNNING + LINE_LOST + OOB |
| 程式架構 | **模組化 10 檔案** | 每個子系統獨立 .c/.h |
| PID 調校 | **OLED 即時顯示 + 按鍵調參** | 不需重編譯 |

### 詳細方案說明

參考文件：
- 腳位定義：[doc/pinout_final.md](pinout_final.md)
- 方案對比：[doc/brainstorm.md](brainstorm.md)
- 比賽規則：[doc/com_rules.txt](com_rules.txt)

---

## 2. 競賽規則分析

### 場地規格

| 參數 | 數值 | 影響 |
|------|------|------|
| 場地大小 | 1.2m × 5m | 一圈最長約 10m |
| 黑線寬度 | 18mm | 8ch × 12mm = 96mm 覆蓋寬度，充裕 |
| 線間距 | ≥ 15cm | 之字形不會太擠 |
| 最小弧半徑 | 7.5cm | 需要快速 PID 響應 |
| 角度 | ≥ 90° | 可能存在直角彎 |
| 斷線 | 10cm（直線段） | 約 42 個 encoder pulse |
| 出界 | 03:00:01 | 致命，必須避免 |

### 技術挑戰難度

```
🔴 高：≥90° 急彎、現場公布地圖、出界懲罰
🟡 中：10cm 斷線、之字形、R=7.5cm 彎道
🟢 低：無十字路口、起終點重合
```

### 設計約束推導

| 規則 | 推導出的設計要求 |
|------|-----------------|
| R=7.5cm 彎道 | 控制迴圈 ≥ 100Hz，PID D 項足夠大 |
| 現場公布地圖 | 不能 hardcode 路線，必須通用演算法 |
| 出界 = 失敗 | 保守速度策略，寧慢勿出 |
| 斷線在直線 | IMU 鎖頭策略可行（不需轉彎推算） |

---

## 3. 硬體配置

### MCU: STM32F407VGT6

- Core: ARM Cortex-M4 @ 16MHz (HSI, 後續可升 PLL 168MHz)
- Flash: 1MB, RAM: 192KB
- 定時器: TIM1 (32-bit), TIM2 (32-bit), TIM8 (32-bit), TIM14 (16-bit)

### 外設清單

| 模組 | 型號 | 接口 | 用途 |
|------|------|------|------|
| 馬達驅動 | TB6612FNG | GPIO + PWM | 雙 N30 DC 馬達 H 橋 |
| 左馬達 | N30, 1:10, 1000PPR | — | 左輪驅動 |
| 右馬達 | N30, 1:10, 1000PPR | — | 右輪驅動 |
| 左編碼器 | Hall encoder | TIM1 Quadrature | 左輪速度/里程 |
| 右編碼器 | Hall encoder | TIM8 Quadrature | 右輪速度/里程 |
| 負壓風扇 | 無刷 + Skywalker ESC | TIM14 50Hz PWM | 下壓力 |
| 灰階感測器 | 8ch 數位陣列 | GPIOE parallel | 黑線偵測 |
| IMU | LSM6DSR | Soft SPI | 6 軸姿態 |
| OLED | SSD1306 128×64 | SPI2 | 除錯顯示 |
| 按鍵 | Tactile switch | GPIO PB15 | 啟動/模式切換 |

### 運動學參數

| 參數 | 值 | 公式 |
|------|-----|------|
| 輪徑 | 24mm | — |
| 輪周長 | 75.398 mm | π × 24 |
| 編碼器 PPR | 1000 (馬達端) | — |
| 減速比 | 1:10 | — |
| 輪端 PPR | 10000 | 1000 × 10 |
| mm/pulse | 0.00754 | 75.398 / 10000 |
| pulse/mm | 132.63 | 10000 / 75.398 |

---

## 4. 系統架構

### 檔案結構

```
Core/
├── Src/
│   ├── main.c              # 系統初始化 + 主迴圈 + 狀態機
│   ├── gpio_init.c         # GPIO / AF / Clock 統一初始化
│   ├── motor.c             # TB6612 雙輪驅動 (FWD/REV/BRK/COAST/PWM)
│   ├── encoder.c           # TIM1/TIM8 Quadrature 讀取 + 里程計
│   ├── sensors.c           # 灰階讀取 + 線位置誤差計算
│   ├── imu.c               # LSM6DSR Soft SPI 驅動 + 角度讀取
│   ├── pid.c               # 泛用 PID 控制器 (位置模式 / 速度模式)
│   ├── fan.c               # ESC 負壓風扇控制 (TIM14 50Hz PWM)
│   └── oled.c              # SSD1306 OLED 顯示
├── Inc/
│   ├── main.h
│   ├── motor.h
│   ├── encoder.h
│   ├── sensors.h
│   ├── imu.h
│   ├── pid.h
│   ├── fan.h
│   ├── oled.h
│   └── pinout.h            # 所有腳位巨集定義
└── Makefile
```

### 控制迴圈架構（200Hz）

```
                        ┌──────────────────────────────┐
                  5ms   │        主控制迴圈 200Hz        │
                        │                              │
  GPIOE IDR ──→ gray_read() ──→ line_position() ──→ error
                                                        │
  TIM1 CNT ──→ enc_left()  ──→ odometry_update() ──→ distance
  TIM8 CNT ──→ enc_right() ──┘                          │
                                                        │
                 ┌──── error ──→ [Position PID] ──→ speed_diff
                 │                                    │  │
                 │   enc_L ──→ [Velocity PID L] ──→ PWM_L (TIM2_CH2)
                 │   enc_R ──→ [Velocity PID R] ──→ PWM_R (TIM2_CH4)
                 │                                    │
                 └── |error| ──→ speed_profile() ──→ base_speed
                      |error| ──→ fan_control() ──→ ESC (TIM14_CH1)
                                                        │
  IMU SPI ──→ imu_read() ──→ heading (斷線備援)        │
                        └──────────────────────────────┘

  OLED 更新: 10Hz (100ms, 非同步)
```

### 資料流

```
感測器 (PORTE IDR)  ──→ 灰階 8-bit
                           │
                    line_position()
                           │
                    error (mm, float)
                           │
              ┌────────────┼────────────┐
              ↓            ↓            ↓
       Position PID   Speed Curve   Fan Control
              │            │            │
         speed_diff    base_speed    ESC PWM
              │            │
              ├────────────┤
              ↓            ↓
        L_target    R_target
              │            │
       Velocity PID  Velocity PID
              │            │
           PWM_L        PWM_R
```

---

## 5. 控制演算法設計

### 5.1 線位置計算（加權平均法）

感測器實體排列（從左到右）：

```
CH1   CH2   CH3   CH4   CH5   CH6   CH7   CH8
 ←── 12mm 間隔 ──→ 總覆蓋 84mm
-42   -30   -18    -6    +6   +18   +30   +42  (mm, 中心=0)
```

```c
static const int8_t sensor_weight[8] = {-42, -30, -18, -6, 6, 18, 30, 42};

float line_position(uint8_t gray)
{
    int32_t w_sum = 0;
    int32_t count = 0;

    for (int i = 0; i < 8; i++) {
        if (gray & (1 << i)) {
            w_sum += sensor_weight[i];
            count++;
        }
    }

    if (count == 0) return LINE_LOST;    // 全白 → 斷線
    if (count == 8) return LINE_FULL;    // 全黑 → 起點區

    return (float)w_sum / (float)count;  // mm
}
```

### 5.2 位置 PID（外環，200Hz）

輸入：`line_error` (mm)，目標 = 0（黑線正中）
輸出：`speed_diff`（左右輪目標速差，PWM 單位）

```c
PID_t pid_position;

// 初始化
PID_Init(&pid_position, KP_POS, KI_POS, KD_POS, I_LIMIT, OUTPUT_LIMIT);

// 每 5ms 呼叫
float speed_diff = PID_Compute(&pid_position, 0.0f, line_error, 0.005f);
```

### 5.3 動態速度曲線

```c
#define MAX_SPEED     700    // PWM duty (0-999)
#define MIN_SPEED     200    // 彎道最低速度
#define K_SPEED       0.015f // 速度衰減係數

uint16_t calc_base_speed(float error)
{
    float factor = 1.0f - fabsf(error) * K_SPEED;
    if (factor < 0.3f) factor = 0.3f;   // 下限 30%
    return (uint16_t)(MAX_SPEED * factor);
}
```

| error | factor | base_speed | 情境 |
|------:|:------:|:----------:|------|
| 0 | 1.00 | 700 | 正中直線 |
| ±10 | 0.85 | 595 | 微偏 |
| ±20 | 0.70 | 490 | 彎道 |
| ±42 | 0.37→0.30 | 210 | 急彎 / 感測器邊緣 |

### 5.4 速度 PID（內環 ×2，200Hz）

輸入：`target_speed` (PWM 單位)，`current_speed` (encoder delta / dt)
輸出：`pwm_output` (0-999)

```c
// 左輪
PID_t pid_vel_L, pid_vel_R;
PID_Init(&pid_vel_L, KP_VEL, KI_VEL, 0.0f, I_LIMIT, PWM_LIMIT);

// 目標速度 = base_speed ± speed_diff
float target_L = base_speed + speed_diff;
float target_R = base_speed - speed_diff;

// 當前速度 = encoder delta in PWM units
float current_L = enc_delta_L * ENC_TO_PWM;

// PID 計算
uint16_t pwm_L = (uint16_t)PID_Compute(&pid_vel_L, target_L, current_L, 0.005f);
```

**串級調校順序（由內而外）：**

```
Step 1: KP_VEL → 讓速度追蹤穩定（無震盪、無延遲）
Step 2: KI_VEL → 消除速度穩態誤差（小值即可）
Step 3: KP_POS → 讓線位置追蹤響應快（逐步加大直到微震）
Step 4: KD_POS → 消除過衝（加大到過衝消失）
Step 5: KI_POS → 消除穩態偏移（最後才加）
```

### 5.5 斷線處理 — IMU 航向鎖定

```c
static float lock_heading;
static uint8_t line_state = ON_LINE;

// 出線偵測：連續 3 次 (15ms) 全白
static uint8_t white_count = 0;

if (line_error == LINE_LOST) {
    white_count++;
    if (white_count >= 3 && line_state == ON_LINE) {
        line_state = LINE_LOST;
        lock_heading = imu_get_yaw();  // 鎖定當前 IMU Z 軸角度
        white_count = 0;
    }
} else {
    white_count = 0;
    if (line_state == LINE_LOST) {
        line_state = LINE_REACQUIRING;
    }
}

// LINE_LOST 狀態：用 heading error 取代 line error
if (line_state == LINE_LOST) {
    float heading_err = imu_get_yaw() - lock_heading;
    // 用 heading_err 做位置 PID 輸入（模擬線誤差）
    line_error = heading_err * HEADING_TO_LINE_SCALE;  // deg → mm
}

// LINE_REACQUIRING：平滑過渡
if (line_state == LINE_REACQUIRING) {
    // 前 3 個週期：50% heading + 50% line
    // 之後回歸正常
    line_state = ON_LINE;
}
```

### 5.6 負壓風扇 — 動態控制

```c
// speed_factor: 0.3 (急彎) ∼ 1.0 (全速直線)
// ESC 範圍: 1000µs (OFF) ∼ 2000µs (最大)
uint16_t calc_fan_pwm(float speed_factor)
{
    // 風扇轉速與行進速度成正比
    uint16_t fan_us = 1000 + (uint16_t)((1.0f - speed_factor) * 500 + 500);
    // speed_factor=1.0 → 1500µs (中等)
    // speed_factor=0.3 → 1850µs (高轉速)
    if (fan_us < 1000) fan_us = 1000;
    if (fan_us > 2000) fan_us = 2000;
    return fan_us;
}
```

### 5.7 起/終點偵測

```c
#define TARGET_DISTANCE_MM  10000   // 賽前量測一圈長度（需現場調整）

if (total_distance_mm >= TARGET_DISTANCE_MM) {
    state = FINISHED;
    MOTOR_L_BRK();
    MOTOR_R_BRK();
    PWM_ESC(1000);  // 風扇停止
}
```

---

## 6. 腳位分配總表

> 完整細節見 [pinout_final.md](pinout_final.md)

### PORTA [0:7] — TB6612 + ESC（近端 R14-R17）

| Pin | 功能 | 模式 | 定時器 |
|-----|------|------|--------|
| PA0 | AIN1 (左輪 H 橋) | GPIO OUT | — |
| PA1 | PWMA (左輪 PWM) | AF PP | TIM2_CH2 |
| PA2 | AIN2 (左輪 H 橋) | GPIO OUT | — |
| PA3 | PWMB (右輪 PWM) | AF PP | TIM2_CH4 |
| PA4 | STBY (致能) | GPIO OUT | — |
| PA5 | BIN1 (右輪 H 橋) | GPIO OUT | — |
| PA6 | BIN2 (右輪 H 橋) | GPIO OUT | — |
| PA7 | ESC_PWM (風扇) | AF PP | TIM14_CH1 |

### 雙編碼器 — 32-bit Quadrature

| 編碼器 | Pin | 定時器 | 位置 |
|--------|-----|--------|------|
| 左 E1A/B | PE9 / PE11 | TIM1 | 近端 |
| 右 E2A/B | PC6 / PC7 | TIM8 | 遠端 |

### 其餘

| 功能 | Pin | 位置 |
|------|-----|------|
| 灰階 CH1-8 | PE4,5,6,7,8,10,12,13 | 近端 |
| ESC 方向 | PB1 | 近端 |
| OLED | PD3,4,5,6 | 遠端 |
| IMU | PB3,5,6,7 | 遠端 |
| 按鍵 | PB15 | 遠端 |

---

## 7. 模組設計

### 7.1 PID 模組（泛用設計）

```c
// pid.h
typedef enum { PID_MODE_POSITION, PID_MODE_VELOCITY } PID_Mode;

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float prev_measurement;  // for velocity mode
    float integral_limit;
    float output_limit;
    PID_Mode mode;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float i_lim, float o_lim, PID_Mode mode);
float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt);
void PID_Reset(PID_t *pid);
```

```c
// pid.c
float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    if (pid->mode == PID_MODE_VELOCITY) {
        // 速度模式：error = 目標速度 - 實際速度
        // derivative on measurement (not error) to avoid derivative kick
        float derivative = -(measurement - pid->prev_measurement) / dt;
        pid->prev_measurement = measurement;

        pid->integral += error * dt;
        if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

        float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
        if (output > pid->output_limit)  output = pid->output_limit;
        if (output < -pid->output_limit) output = -pid->output_limit;
        return output;
    } else {
        // 位置模式：標準 PID
        float derivative = (error - pid->prev_error) / dt;
        pid->prev_error = error;

        pid->integral += error * dt;
        if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

        float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
        if (output > pid->output_limit)  output = pid->output_limit;
        if (output < -pid->output_limit) output = -pid->output_limit;
        return output;
    }
}
```

### 7.2 馬達模組

```c
// motor.h
void Motor_Init(void);
void Motor_SetPWM(uint16_t left, uint16_t right);
void Motor_Forward(void);
void Motor_Reverse(void);
void Motor_Brake(void);
void Motor_Coast(void);
```

### 7.3 編碼器模組

```c
// encoder.h
void Encoder_Init(void);
int32_t Encoder_GetLeft(void);    // 讀 TIM1 CNT
int32_t Encoder_GetRight(void);   // 讀 TIM8 CNT
void Odometry_Update(void);       // 更新累積距離
int32_t Odometry_GetDistance(void); // 回傳 0.1mm 單位
float Encoder_GetSpeedL(void);    // PWM 單位
float Encoder_GetSpeedR(void);
```

### 7.4 感測器模組

```c
// sensors.h
#define LINE_LOST   999.0f
#define LINE_FULL  -999.0f

uint8_t Gray_Read(void);
float Line_GetError(void);       // 回傳 mm 誤差（0=正中）
uint8_t Line_IsLost(void);       // 是否斷線
```

### 7.5 IMU 模組

```c
// imu.h
int IMU_Init(void);
int IMU_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
int IMU_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);
float IMU_GetYaw(void);          // Z 軸積分角度（度）
void IMU_ResetYaw(void);
```

### 7.6 風扇模組

```c
// fan.h
void Fan_Init(void);             // TIM14 50Hz, 預設 1000µs
void Fan_SetSpeed(uint16_t us);  // 1000-2000 µs
void Fan_SetDirection(uint8_t fwd); // PB1 控制
```

### 7.7 OLED 模組

```c
// oled.h
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowString(const char *s, uint8_t col, uint8_t page);
void OLED_ShowPIDPage(float kp, float ki, float kd, float error, int speed);
void OLED_Flush(void);
```

---

## 8. 狀態機

```
                    ┌─────────────┐
                    │ CALIBRATION │  開機 / 重置
                    └──────┬──────┘
                           │ 校準完成
                    ┌──────▼──────┐
                    │    IDLE     │  等待按鍵
                    └──────┬──────┘
                           │ 按鍵按下
                    ┌──────▼──────┐
              ┌─────│   RUNNING   │─────┐
              │     └──────┬──────┘     │
              │            │            │
        全白>15ms     距離達標      error>40mm
        連續3次        │            持續50ms
              │            │            │
      ┌───────▼───┐  ┌───▼────┐  ┌───▼───┐
      │ LINE_LOST │  │FINISHED│  │  OOB   │
      └─────┬─────┘  └────────┘  └───────┘
            │ 看到線              (停車)
      ┌─────▼────────┐
      │LINE_REACQUIRE│ 平滑過渡 3 週期
      └──────┬───────┘
             │ 過渡完成 → RUNNING
```

### 狀態行為表

| 狀態 | 馬達 | PID | 風扇 | OLED |
|------|:---:|:---:|:---:|------|
| CALIBRATION | BRAKE | OFF | 1000µs | "CAL..." |
| IDLE | BRAKE | OFF | 1000µs | "READY\nPress BTN" |
| RUNNING | 串級控制 | ON | 動態 | error + speed |
| LINE_LOST | 航向鎖定 | heading PID | 維持 | "LOST!" |
| LINE_REACQUIRING | 平滑混合 | 50/50 mix | 維持 | "REACQ" |
| FINISHED | BRAKE | OFF | 1000µs | "DONE!\nTime: XX.XX" |
| OOB | BRAKE | OFF | 1000µs | "OOB!\n03:00:01" |

---

## 9. 開發階段劃分

### Phase 1: MVP — 能跑完一圈（1-2 天）

**目標**：基礎循跡，能完成一整圈不脫線

| # | 任務 | 檔案 | 預計耗時 |
|---|------|------|:------:|
| 1.1 | GPIO 初始化（全腳位） | `gpio_init.c` | 2h |
| 1.2 | 灰階讀取 + 線位置計算 | `sensors.c` | 1h |
| 1.3 | 雙輪馬達驅動 + PWM | `motor.c` | 1h |
| 1.4 | 單層位置 PID | `pid.c` | 2h |
| 1.5 | 主迴圈 100Hz + 狀態機 (4 狀態) | `main.c` | 2h |
| 1.6 | 固定速度 + 固定風扇 | `main.c` | 0.5h |
| 1.7 | 場地測試 + PID 粗調 | — | 3h |

**Phase 1 驗收標準**：0.3 m/s 穩定完成一圈，不脫線

---

### Phase 2: 穩定 — 加入完整功能（2-3 天）

**目標**：加入所有方案 B 功能，提高穩定性和速度

| # | 任務 | 檔案 | 預計耗時 |
|---|------|------|:------:|
| 2.1 | 雙編碼器 Quadrature + 里程計 | `encoder.c` | 2h |
| 2.2 | 速度 PID 內環（雙輪） | `pid.c` | 2h |
| 2.3 | 串級 PID 整合 | `main.c` | 2h |
| 2.4 | IMU 驅動 + 斷線航向鎖定 | `imu.c` | 3h |
| 2.5 | 動態速度曲線 | `main.c` | 1h |
| 2.6 | 動態風扇控制 | `fan.c` | 1h |
| 2.7 | 增強狀態機（+ LINE_LOST + OOB） | `main.c` | 1h |
| 2.8 | OLED 即時顯示 + 按鍵調參 | `oled.c` | 2h |
| 2.9 | 控制頻率升到 200Hz | `main.c` | 1h |
| 2.10 | 場地測試 + 串級 PID 精調 | — | 4h |

**Phase 2 驗收標準**：0.5-0.7 m/s 穩定，斷線通過成功 > 90%，OLED 參數可調

---

### Phase 3: 競賽優化（1-2 天）

**目標**：追求最快圈速

| # | 任務 | 預計耗時 |
|---|------|:------:|
| 3.1 | IMU 預判彎道減速 | 2h |
| 3.2 | 預判式風扇控制 | 1h |
| 3.3 | 起/終點自動停止微調 | 1h |
| 3.4 | 速度上限逐步提升測試 | 2h |
| 3.5 | 多張地圖測試（自製 3 張） | 3h |
| 3.6 | PLL 168MHz 升級（可選） | 1h |

**Phase 3 驗收標準**：> 0.8 m/s，2 輪取最佳，穩定完成

---

## 10. 測試計劃

### 10.1 單元測試（每模組獨立）

| 模組 | 測試方法 | 通過條件 |
|------|----------|----------|
| 灰階 | OLED 顯示 8-bit 二進制，手動在線上移動 | 每個通道獨立亮/滅，無雜訊 |
| 馬達 | PWM duty=300, 觀察輪子旋轉 | 正反轉正確，PWM 變化平穩 |
| 編碼器 | 手轉輪子，OLED 顯示 CNT | 正轉遞增、反轉遞減，無跳動 |
| IMU | OLED 顯示 6 軸數值 | 靜止時 gyro ≈ 0，轉動時角度變化正確 |
| ESC/風扇 | 逐步增加 PWM，聽風扇轉速 | 1000µs 停止，2000µs 全速 |
| OLED | 顯示測試字串 | 字元清晰無殘影 |

### 10.2 整合測試

| # | 測試 | 方法 | 通過條件 |
|---|------|------|----------|
| 1 | 直線 50cm | 直線來回 | error < ±5mm 穩態 |
| 2 | R=15cm 圓弧 | 定速過彎 | 不脫線，無蛇行 |
| 3 | R=7.5cm 最小弧 | 最低速過彎 | 不脫線（允許減速） |
| 4 | 90° 直角 | 低速過直角 | 內側輪不離開黑線 |
| 5 | 之字形 (15cm 間距) | 中速通過 | 不出界，不震盪 |
| 6 | 10cm 斷線 | 直線上模擬斷線 | 成功通過 ≥ 90% |
| 7 | 全圈測試 | 自製測試地圖 | 2 分鐘內完成一圈 |

### 10.3 競賽模擬測試

1. 自製 3 張不同地圖（不預先告知自己）
2. 每張地圖跑 2 輪，取最佳
3. 記錄每輪時間、脫線次數、出界次數

---

## 11. 風險評估

| # | 風險 | 可能性 | 影響 | 緩解措施 |
|---|------|:---:|:---:|------|
| 1 | 灰階感測器在急彎時完全偏離線 | 中 | 高 | 降低彎道速度 + 增大 PID D 項 |
| 2 | IMU 陀螺儀積分漂移影響斷線 | 低 | 中 | 10cm 短距漂移極小；備案用純 encoder |
| 3 | 負壓風扇震動干擾 IMU | 中 | 中 | 軟體低通濾波；IMU 遠離風扇安裝 |
| 4 | 編碼器距離累積誤差 | 低 | 低 | 一圈 < 10m，誤差 < 10cm |
| 5 | 電池電壓下降影響馬達性能 | 中 | 中 | 速度 PID 閉環補償；滿電起跑 |
| 6 | 輪子打滑（起跑/急彎） | 中 | 高 | 風扇下壓力；平滑加速曲線 |
| 7 | 現場地圖有未預期特徵 | 中 | 高 | 通用演算法（無 hardcode）；保守速度 |
| 8 | 16MHz 效能不足 | 低 | 低 | 200Hz 下每週期 550µs ≪ 5000µs；可升 PLL |

---

## 12. 參考資源

| 資源 | 連結 |
|------|------|
| STM32-Line-Follower-with-PID (GitHub) | [deepwiki.com](https://deepwiki.com/sametoguten/STM32-Line-Follower-with-PID) |
| Cascaded PID Tuning Guide (Synapticon) | [synapticon.com](https://doc.synapticon.com/circulo_safe_motion/tutorials/tuning_guides/cascaded_position_controller.html) |
| Pololu Line Follower Blog | [pololu.com](https://www.pololu.com/blog/486/davids-line-following-robot-that-learns-the-course) |
| Semreh Line-Follower Robot (Hackaday) | [hackaday.io](https://hackaday.io/project/202208/logs?sort=oldest) |
| STM32F407 正交編碼器應用指南 | [CSDN](https://blog.csdn.net/yuan19997/article/details/159512043) |
| PUT-PTM/STM_LineFollower (GitHub) | [github.com](https://github.com/PUT-PTM/STM_LineFollower) |
| Differential Drive Kinematics | [zbotic.in](https://zbotic.in/differential-drive-robot-kinematics-speed-control-guide/) |

---

## 附錄 A: 預設 PID 參數起點

| 參數 | 建議起始值 | 說明 |
|------|:--------:|------|
| KP_POS | 1.5 | 位置環 P（先調） |
| KI_POS | 0.02 | 位置環 I（最後加） |
| KD_POS | 12.0 | 位置環 D（第二調） |
| KP_VEL | 0.8 | 速度環 P |
| KI_VEL | 0.1 | 速度環 I |
| MAX_SPEED | 700 | PWM duty (0-999) |
| MIN_SPEED | 200 | 彎道下限 |
| K_SPEED | 0.015 | 速度衰減率 |
| FAN_BASE | 1500 | ESC µs (1500 = 中速) |

> 以上僅為起點。實際值需在場地測試中按串級調校步驟調整。

## 附錄 B: 比賽日檢查清單

- [ ] 滿電電池 ×2（備用一顆）
- [ ] 自製測試地圖（賽前熱身用）
- [ ] OLED 顯示正常（即時看 error / speed）
- [ ] 按鍵功能正常（啟動 / 停止 / 調參）
- [ ] 下載線 + 筆電（現場微調 Kp/Ki/Kd）
- [ ] 備用輪胎
- [ ] 量測一圈距離（編碼器 target 設定）
- [ ] ESC 校準完成
- [ ] 風扇運轉正常
- [ ] 灰階感測器乾淨無塵
