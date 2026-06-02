# 循跡機器人 — 技術頭腦風暴

> 版本: 1.0 | 日期: 2026-06-02 | 每個技術面向提供 2-3 個方案選擇

---

## 目錄

1. [PID 控制架構](#1-pid-控制架構)
2. [線位置計算方法](#2-線位置計算方法)
3. [速度策略](#3-速度策略)
4. [斷線處理](#4-斷線處理)
5. [起點/終點偵測](#5-起點終點偵測)
6. [負壓風扇控制](#6-負壓風扇控制)
7. [控制迴圈頻率](#7-控制迴圈頻率)
8. [狀態機設計](#8-狀態機設計)
9. [程式碼架構](#9-程式碼架構)
10. [PID 調校策略](#10-pid-調校策略)

---

## 1. PID 控制架構

### 方案 A：單層位置 PID（最簡）

```
感測器誤差 → [Kp + Ki∫ + Kd·d/dt] → 直接輸出左右差速量
```

| 優點 | 缺點 |
|------|------|
| 程式碼最少（~30 行） | 無速度閉環，受電池電壓影響 |
| 調參直覺（只調 3 個參數） | 直線時可能有穩態誤差 |
| 16MHz MCU 完全夠用 | 加減速不平滑 |

```c
error = line_position();                         // 線位置誤差
float steering = Kp * error + Ki * integral + Kd * derivative;
int16_t pwm_l = base_speed + steering;           // 左輪 = 基底 + 修正
int16_t pwm_r = base_speed - steering;           // 右輪 = 基底 - 修正
```

---

### 方案 B：串級 PID — 外環位置 + 內環速度（推薦）

```
線誤差 → [位置 PID] → 目標速度差 → [速度 PID ×2] → PWM 輸出
              ↑                            ↑
         感測器誤差                    編碼器回授
```

| 優點 | 缺點 |
|------|------|
| 速度閉環，不受負載/電壓影響 | 需要 2+1 個 PID（約 100 行程式碼） |
| 加減速平滑可控 | 調參較複雜（6-9 個參數） |
| 競賽級標準架構 | 需要編碼器中斷或高頻讀取 |

```
[外環 Position PID — 50Hz]         [內環 Velocity PID ×2 — 200Hz]
                                    
 error_line ──→ Kp_pos ──→ target_speed_diff ──→ L: Kp_vel ──→ PWM_L
                Ki_pos                            L: Ki_vel
                Kd_pos                            R: Kp_vel ──→ PWM_R
                                                   R: Ki_vel
                                                      ↑
                                               encoder_feedback
```

參考：[STM32-Line-Follower-with-PID (DeepWiki)](https://deepwiki.com/sametoguten/STM32-Line-Follower-with-PID)

---

### 方案 C：增量式 PID

| 優點 | 缺點 |
|------|------|
| 無積分飽和（windup）問題 | 需要保存歷史狀態 |
| 輸出平滑，適合 PWM | 穩態誤差可能需要額外補償 |
| 適合 MCU 計算（只用最近幾次誤差） | 調參與位置式不同，需重新學習 |

```c
// Δu = Kp*(e_k - e_{k-1}) + Ki*e_k + Kd*(e_k - 2*e_{k-1} + e_{k-2})
float delta_u = Kp * (error - last_error)
              + Ki * error
              + Kd * (error - 2*last_error + prev_error);
output += delta_u;
```

---

### 建議

| 場景 | 建議方案 |
|------|----------|
| 快速原型 / 先求能跑 | **方案 A** |
| 競賽追求穩定和速度 | **方案 B**（串級 PID） |
| 編碼器故障備援 | 方案 A（無編碼器依賴） |

> 🏆 **推薦方案 B**：串級 PID 是競賽級標準，內環速度控制消除電壓波動影響，外環位置控制保持精準循跡。

---

## 2. 線位置計算方法

### 方案 A：加權平均法（數位感測器）

每個感測器只有 0/1（白/黑），給每個通道一個位置權重：

```c
// 感測器位置 (mm)：CH1 最左，CH8 最右，中心=0
static const int8_t pos[8] = {-42, -30, -18, -6, 6, 18, 30, 42};

float calc_error(uint8_t gray) {
    int32_t sum = 0, cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (gray & (1 << i)) { sum += pos[i]; cnt++; }
    }
    if (cnt == 0) return LINE_LOST;   // 全白：斷線
    if (cnt == 8) return LINE_FULL;   // 全黑：異常
    return (float)sum / cnt;           // 單位 mm，0=正中
}
```

| 優點 | 缺點 |
|------|------|
| 計算極快（整數運算） | 只用 0/1，丟失灰階資訊 |
| 適合數位感測器 | 邊緣通道可能來回跳動 |

---

### 方案 B：內插法（需要多 bit 感測器讀數）

若感測器能回傳多級灰階值，而非僅 0/1：

```c
// 找最大亮度位置 → 以相鄰通道做二次內插
int16_t raw[8];  // 0-4095 ADC 讀數
// 二次內插：peak = i + (raw[i-1] - raw[i+1]) / (2*(raw[i-1] + raw[i+1] - 2*raw[i]))
```

| 優點 | 缺點 |
|------|------|
| 亞 mm 級精度 | 需要 ADC 讀取（每個通道 ~10µs） |
| 平滑連續 | 程式碼較複雜 |

---

### 方案 C：兩階段法（粗定位 + 細定位）

先用加權平均粗算位置，再根據中心通道的具體灰階值微調：

```c
// Phase 1: 粗定位（加權平均）→ integer position
// Phase 2: 細定位（中心通道灰度值）→ fractional offset
float fine_pos = coarse_pos + gray_offset;
```

| 優點 | 缺點 |
|------|------|
| 速度快 + 精度高 | 需要感測器支援多級讀數 |
| 適合混合感測器 | 兩段程式碼維護成本 |

---

### 建議

| 場景 | 建議方案 |
|------|----------|
| 使用現有 8 通道數位灰階（PINOUT 定義） | **方案 A** |
| 未來升級更高精度感測器 | 方案 B 或 C |

> 🏆 **推薦方案 A**：符合現有硬體（數位並列輸入），計算最快，配合 PID 已足夠。

---

## 3. 速度策略

### 方案 A：固定基底速度

```c
base_speed = 600;  // 固定 PWM duty (0-999)
```

| 優點 | 缺點 |
|------|------|
| 最簡單 | 彎道可能過快出界 |
| 調參只調 PID | 直線浪費時間 |

---

### 方案 B：基於誤差動態調速（線性）

```c
// 誤差越大 → 速度越低
float speed_factor = 1.0f - fabsf(error) * K_speed;
if (speed_factor < MIN_SPEED_FACTOR) speed_factor = MIN_SPEED_FACTOR;
base_speed = MAX_SPEED * speed_factor;
```

| error | speed_factor | 說明 |
|-------|:-----------:|------|
| 0 | 1.00 (全速) | 正中直線 |
| ±10mm | 0.80 | 微偏 |
| ±20mm | 0.60 | 彎道 |
| ±35mm | 0.30 (下限) | 急彎 |

| 優點 | 缺點 |
|------|------|
| 實現簡單 | 彎道減速滯後（看到彎才慢） |

---

### 方案 C：IMU 預判 + 誤差複合調速（推薦）

```c
// 組合：當前誤差 + 誤差變化率 + IMU Z 軸角速度
float predict = fabsf(error) * K1 + fabsf(derivative) * K2 + fabsf(gz) * K3;
float speed_factor = 1.0f - predict;
```

| 優點 | 缺點 |
|------|------|
| 彎道提前減速（IMU 預判） | 需要 IMU 讀取正常 |
| 直線即時加速 | 三個係數交互作用，調校複雜 |
| 對急彎（≥90°）反應最快 | 程式碼較多 |

參考：[Pololu - David's line following robot that learns the course](https://www.pololu.com/blog/486/davids-line-following-robot-that-learns-the-course)

---

### 建議

| 階段 | 建議方案 |
|------|----------|
| 開發初期、調 PID | **方案 A** |
| PID 穩定後 | **方案 B** |
| 追求競賽最佳成績 | **方案 C**（IMU 預判） |

> 🏆 **推薦漸進式**：先用 A 調好 PID → 升級到 B → 最後用 C 做競賽優化。

---

## 4. 斷線處理

### 方案 A：慣性直走

```
偵測到全白 → PWM 鎖定最後值 → 直走直到重新看到線
```

| 優點 | 缺點 |
|------|------|
| 最簡單，零額外程式碼 | 若有方向偏差會越走越偏 |
| 10cm 短斷線通常夠用 | 無法處理彎道上的斷線（雖然規則說斷線在直線） |

```c
if (line_lost) {
    // 保持最後的 PWM 輸出，不做 PID 修正
    pwm_l = last_pwm_l;
    pwm_r = last_pwm_r;
} else {
    // 正常 PID 控制
    last_pwm_l = pwm_l;
    last_pwm_r = pwm_r;
}
```

---

### 方案 B：IMU 陀螺儀航向鎖定

```
出線前記錄陀螺儀 Z 軸角度 → 斷線期間 PID 鎖定該角度 → 恢復後平滑過渡
```

| 優點 | 缺點 |
|------|------|
| 維持出線前的行進方向 | 需要 IMU 正常運作 |
| 10cm 直線斷線可靠 | 陀螺儀積分漂移（但 10cm 短距不顯著） |

```c
static float lock_heading;
static uint8_t line_lost;

if (!line_lost && all_white) {
    line_lost = 1;
    lock_heading = imu_yaw();   // 記錄當前航向
}
if (line_lost) {
    float heading_err = imu_yaw() - lock_heading;
    // 用 heading_err 取代 line_error 做 PID 控制
    pwm_l = base - K_heading * heading_err;
    pwm_r = base + K_heading * heading_err;
}
if (line_lost && line_detected) {
    line_lost = 0;
    // 平滑過渡：漸進恢復 line PID 權重
}
```

---

### 方案 C：編碼器里程計 + IMU 融合推算（推薦）

```
出線時 → 記錄 (x, y, θ)
斷線期間 → 編碼器推算距離 + IMU 維持方向 → 持續更新估計位置
以直線延伸方向前進 10cm → 預期重新入線
```

| 優點 | 缺點 |
|------|------|
| 最精確（融合編碼器 + IMU） | 實作最複雜 |
| 可計算已走距離，超過 15cm 未入線則停車 | 需要里程計正常累積 |

> 規則確認：斷線 10cm，在直線段。10cm 對應約 42 個 encoder pulse（1000ppr × 10:1 / (π×24mm) × 100mm）。

---

### 建議

| 場景 | 建議方案 |
|------|----------|
| 先求能過關 | **方案 A**（10cm 短距夠用） |
| 確保穩定性 | **方案 B**（加入 IMU 鎖頭） |
| 追求零失誤 | **方案 C**（融合推算） |

> 🏆 **推薦方案 B**：規則明確保斷線在直線，加上 IMU 鎖頭已足夠穩妥。方案 C 可作為進階優化。

---

## 5. 起點/終點偵測

### 方案 A：編碼器距離推算（開環）

```c
// 賽前量一圈總長 → 設定 target_distance
// 起跑後累積編碼器脈衝 → 到達 target 自動停止
if (total_distance_mm >= target_distance_mm) {
    state = FINISHED;
    motor_stop();
}
```

| 優點 | 缺點 |
|------|------|
| 最簡單，無需額外感測器 | 距離誤差累積（輪子打滑） |
| 編碼器已經有讀取 | 必須賽前知道圈長 |

---

### 方案 B：起點全黑區偵測 + 計圈

```
起點放置與賽道其餘部分不同的特徵（如全黑方塊）→ 第一次經過忽略（起跑），第二次經過觸發停止
```

需要全部 8 通道同時看到黑線的情況來觸發：

```c
if (gray == 0xFF) {  // 全部 8 通道都看到黑
    lap_count++;
    if (lap_count >= 2) state = FINISHED;
}
```

| 優點 | 缺點 |
|------|------|
| 無需知道圈長 | 需在起點放置額外標記 |
| 精確（物理觸發） | 比賽現場地圖未知，可能不允許標記 |

---

### 方案 C：起跑線特徵 + 編碼器輔助（推薦）

```
1. 起跑時記錄編碼器為 0
2. 前 N cm 內抑制停止觸發（防止起跑誤觸發）
3. N cm 後，每當 8 通道全黑 → 判定回到起點
4. 備援：若編碼器超過預期長度 150% 仍未觸發 → 強制停止
```

| 優點 | 缺點 |
|------|------|
| 物理觸發 + 里程保護 | 需起點有可辨識特徵 |
| 雙重保險 | 若起點無特徵則需方案 A |

---

### 建議

| 場景 | 建議方案 |
|------|----------|
| 起點無特徵標記 | **方案 A** |
| 起點有全黑區 | **方案 C**（最穩） |
| 不確定 | **方案 A + C 備援**：編碼器為主要判斷，全黑區輔助確認 |

> 🏆 **推薦方案 A**：最單純，比賽規則未提及起點有特殊標記。加上一圈總長不會超過 ~10m，編碼器累積誤差 < 1%。

---

## 6. 負壓風扇控制

### 方案 A：固定轉速

```c
PWM_ESC(1500);  // 固定 1500µs，全程恆定下壓力
```

| 優點 | 缺點 |
|------|------|
| 最簡單 | 直線浪費電力 |
| 無控制延遲 | 彎道可能下壓力不足 |

---

### 方案 B：基於速度動態調整

```c
// 速度越快 → 風扇轉速越高（需要更多下壓力過彎）
uint16_t fan_us = 1000 + (uint16_t)(speed_factor * 1000);
// speed_factor=0.3 → fan=1300µs
// speed_factor=1.0 → fan=2000µs
```

| 優點 | 缺點 |
|------|------|
| 彎道時更多下壓力 | 風扇響應有延遲（ESC 軟斜坡） |
| 直線時省電 | 需校準 speed→fan 曲線 |

---

### 方案 C：預判式動態控制（推薦）

```c
// 結合當前誤差 + 誤差變化率 + IMU 角速度 → 預判彎道 → 提前加速風扇
float corner_severity = fabsf(error) * K_fan1
                      + fabsf(error_derivative) * K_fan2
                      + fabsf(gz) * K_fan3;
uint16_t fan_us = 1000 + (uint16_t)(corner_severity * 1000);
if (fan_us > 2000) fan_us = 2000;
```

| 優點 | 缺點 |
|------|------|
| 彎道前就增加下壓力 | 最複雜 |
| 最大化過彎極限 | 三個係數需調校 |

參考：[Semreh Line-Follower Robot (Hackaday)](https://hackaday.io/project/202208/logs?sort=oldest) — 巴西隊伍，使用無刷馬達提供下壓力，從鐵盃賽第二名進化到 RoboChallenge 第一名。

---

### 建議

| 階段 | 建議方案 |
|------|----------|
| 開發初期 | **方案 A**（固定 60% 轉速） |
| PID 穩定後 | **方案 B** |
| 競賽優化 | **方案 C** |

> 🏆 **推薦漸進式**：ESC 有軟斜坡延遲（~200ms），預判式（方案 C）最有價值，先用方案 A/B 驗證風扇效果。

---

## 7. 控制迴圈頻率

### 頻率需求分析

| 情境 | 所需頻率 | 計算 |
|------|:------:|------|
| 直線 @ 1m/s，感測器間距 12mm | 84 Hz | 1000mm/s ÷ 12mm |
| 最小彎 R=75mm @ 0.5m/s | 106 Hz | 500mm/s ÷ (2π×75mm/360°×10°) |
| 競賽級 recommendation | 100-200 Hz | [STM32 Line Follower 參考](https://deepwiki.com/sametoguten/STM32-Line-Follower-with-PID) |

### 16MHz 下的時間預算

| 操作 | 耗時 |
|------|:----:|
| 灰階 GPIOE IDR 讀取 + bit 移位 | ~2 µs |
| 線位置加權平均計算 | ~10 µs |
| PID 計算（位置 + 雙速度） | ~30 µs |
| 編碼器讀取 + 里程計更新 | ~5 µs |
| IMU SPI 讀取（12 bytes） | ~500 µs |
| OLED 更新（128×8 bytes SPI） | ~2000 µs |
| **總計（不含 OLED）** | **~550 µs** |

### 方案 A：10ms / 100Hz

```c
// SysTick 或 TIM 中斷驅動
if (HAL_GetTick() - last_tick >= 10) {
    last_tick = HAL_GetTick();
    control_loop();   // PID + sensor + motor update
}
OLED 更新降頻到 10Hz (100ms)
```

| 優點 | 缺點 |
|------|------|
| 16MHz 完全夠用 | 急彎可能不夠快 |
| OLED 不影響控制速率 | 100Hz 對 R=75mm 彎道邊界值 |

---

### 方案 B：5ms / 200Hz（推薦）

```
控制迴圈 200Hz：感測器 + PID + PWM（不含 OLED）
OLED 非同步更新，獨立計時
```

| 優點 | 缺點 |
|------|------|
| 涵蓋所有彎道情境 | 需確保 16MHz 能在 5ms 內完成 |
| 16MHz 下 ~550µs ≪ 5000µs | 需要非阻塞 SPI（IMU） |

---

### 方案 C：非同步多速率

```
感測器讀取 + 線位置計算：1kHz（最快）
PID 位置環：200Hz
PID 速度環：500Hz（在位置環內）
OLED：10Hz
IMU：208Hz（感測器原生速率）
```

| 優點 | 缺點 |
|------|------|
| 各子系統獨立最優頻率 | 程式架構複雜 |
| IMU 不拖慢主迴圈 | 建議上 FreeRTOS 或用 DMA |

---

### 建議

| 場景 | 建議方案 |
|------|----------|
| 先求能跑 | **方案 A**（100Hz） |
| 正式開發 | **方案 B**（200Hz） |
| 追求極限 | **方案 C**（多速率） |

> 🏆 **推薦方案 B**：200Hz 是這個硬體配置的甜蜜點——夠快且 16MHz 綽綽有餘。

---

## 8. 狀態機設計

### 方案 A：簡潔線性狀態

```
[CALIBRATION] → [IDLE] → [RUNNING] → [FINISHED]
                     ↑                      │
                     └──── 按鍵重置 ────────┘
```

| 狀態 | 觸發條件 | 行為 |
|------|----------|------|
| CALIBRATION | 開機 | ESC 校準 / 感測器基準 |
| IDLE | 校準完成 | 等待按鍵啟動 |
| RUNNING | 按鍵按下 | PID 循跡、里程計累積 |
| FINISHED | 距離達標 / 按鍵 | 停車、顯示成績 |

---

### 方案 B：含異常處理的增強狀態機（推薦）

```
                 ┌── LINE_LOST ──┐
                 │    (斷線)     │
[CALIB] → [IDLE] → [RUNNING] → [FINISHED]
                 │              │
                 └── OOB ───────┘
                    (出界)
```

| 狀態 | 子狀態 | 觸發 |
|------|--------|------|
| RUNNING | ON_LINE | 正常循跡 |
| RUNNING | LINE_LOST | 全白 > 5ms |
| RUNNING | LINE_REACQUIRING | 斷線後剛看到線（平滑過渡） |
| OOB | — | 任何輪子出界（IMU 或感測器判斷） |
| FINISHED | — | 距離達標 |

---

### 建議

> 🏆 **推薦方案 B**：多兩個狀態（LINE_LOST / OOB）就能大幅提高穩定性。OOB 判斷可在感測器極端偏離時觸發（如連續 50ms error > 40mm）。

---

## 9. 程式碼架構

### 方案 A：單檔案（繼承現有 main.c 風格）

```
main.c (~800 行)
├── GPIO/PWM/Encoder 初始化
├── IMU 驅動
├── OLED 驅動
├── 灰階讀取
├── PID 控制
├── 主迴圈
└── 中斷服務
```

| 優點 | 缺點 |
|------|------|
| 編譯快、無依賴 | 難維護、難分工 |
| 適合原型階段 | 改一個功能可能牽動全域 |

---

### 方案 B：模組化拆分（推薦）

```
Src/
├── main.c              # 初始化 + 主迴圈 + 狀態機
├── gpio_init.c/h       # MX_GPIO_Init（統一初始化）
├── motor.c/h           # TB6612 雙輪驅動 + PWM
├── encoder.c/h         # TIM1/TIM8 編碼器讀取 + 里程計
├── sensors.c/h         # 灰階讀取 + 線位置計算
├── imu.c/h             # LSM6DSR SPI 驅動
├── pid.c/h             # PID 控制器（可重用多個實例）
├── fan.c/h             # ESC 負壓風扇控制
├── oled.c/h            # SSD1306 OLED 顯示
└── state_machine.c/h   # 狀態機邏輯
```

| 優點 | 缺點 |
|------|------|
| 模組獨立測試 | 需要 header 管理 |
| 團隊可分工 | 編譯時間略增 |
| PID 模組可複用（位置 PID 和速度 PID）| |

### PID 模組泛用設計

```c
// pid.h
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error, prev_prev_error;
    float integral_limit, output_limit;
    uint8_t mode;  // POSITION_MODE or VELOCITY_MODE (delta)
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float i_lim, float o_lim);
float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt);
void PID_Reset(PID_t *pid);
```

---

### 方案 C：搭配簡單排程器

在方案 B 基礎上，加入一個輕量排程器（非 RTOS）：

```c
// 任務註冊
typedef struct {
    void (*func)(void);
    uint32_t period_ms;
    uint32_t last_run;
} Task_t;

Task_t tasks[] = {
    { .func = sensors_update,    .period_ms = 5   },  // 200Hz
    { .func = pid_position,      .period_ms = 5   },  // 200Hz
    { .func = pid_velocity,      .period_ms = 2   },  // 500Hz (within position)
    { .func = imu_read,          .period_ms = 5   },  // 200Hz
    { .func = odometry_update,   .period_ms = 10  },  // 100Hz
    { .func = oled_update,       .period_ms = 100 },  // 10Hz
    { .func = state_machine,     .period_ms = 5   },  // 200Hz
};
```

| 優點 | 缺點 |
|------|------|
| 精確控制每個模組頻率 | 需確保任務總時間 < 最小週期 |
| 不需 RTOS 開銷 | 無搶佔，一個任務卡住會拖垮全部 |

---

### 建議

> 🏆 **推薦方案 B**：模組化拆分是正道。PID 泛用設計讓同一套程式碼驅動位置環和速度環。方案 C 的排程器可在方案 B 穩定後加入。

---

## 10. PID 調校策略

### 方案 A：手動試誤法

```
1. Kp 從 0 開始，每次 +0.1，直到循跡開始震盪
2. Kp 退回震盪值的 60-70%
3. Kd 從 0 開始，每次 +0.5，直到過衝消失
4. Ki 從 0 開始，每次 +0.01，直到穩態誤差消失
```

| 步驟 | 觀察 | 調整 |
|------|------|------|
| 只調 Kp | 蛇行震盪 | Kp 太高，降低 |
| 加入 Kd | 過衝 → 回拉 | Kd 消除 overshoot |
| 加入 Ki | 輕微偏移不歸位 | Ki 消除 steady-state error |

---

### 方案 B：OLED 即時顯示 + 電位器調參

在 OLED 上即時顯示誤差曲線，用可變電阻（ADC）動態調 Kp/Ki/Kd：

```
OLED 顯示格式：
Kp:1.2 Ki:0.05 Kd:8.0
E:████░░░░░░  +12mm
Speed: 60%
```

| 優點 | 缺點 |
|------|------|
| 調參直覺快速 | 需要額外硬體（電位器）或按鍵組合 |
| 不需重編譯 | 佔用 ADC 腳位 |

---

### 方案 C：Ziegler-Nichols 閉環震盪法

```
1. Ki=0, Kd=0
2. 增加 Kp 直到系統持續震盪（臨界增益 Ku，臨界週期 Tu）
3. 查表計算：
   P-only:  Kp = 0.5 * Ku
   PI:      Kp = 0.45 * Ku, Ki = 0.54 * Ku / Tu
   PID:     Kp = 0.6 * Ku,  Ki = 1.2 * Ku / Tu, Kd = 0.075 * Ku * Tu
```

| 優點 | 缺點 |
|------|------|
| 系統化，有理論基礎 | 可能震到出界 |
| 不需猜參數範圍 | 需要測量 Tu（週期） |

---

### 建議

> 🏆 **推薦方案 B（OLED 顯示 + 按鍵調參）**：不需額外硬體，OLED 已有。用按鍵組合（單擊=選參數、長按=增/減值）動態調整，每次跑完一圈看 OLED 上的誤差紀錄，迭代速度遠超重編譯。

---

## 總結：推薦技術路線圖

| 階段 | 目標 | 技術選擇 |
|------|------|----------|
| **Phase 1** MVP | 能循跡跑完一圈 | 方案 A：單層 PID + 固定速度 + 固定風扇 + 簡單狀態機 |
| **Phase 2** 穩定 | 穩定不脫線 | 方案 B：串級 PID + 動態調速 + IMU 斷線 + 模組化架構 |
| **Phase 3** 競賽 | 追求最快圈速 | 方案 B/C：IMU 預判調速 + 預判風扇 + 200Hz + OLED 調參 |

---

*本文件由技術調研 + 頭腦風暴產生，每個方案均附帶優缺點對比和建議。*
*參考來源：[DeepWiki STM32-Line-Follower](https://deepwiki.com/sametoguten/STM32-Line-Follower-with-PID), [Synapticon Cascaded PID Guide](https://doc.synapticon.com/circulo_safe_motion/tutorials/tuning_guides/cascaded_position_controller.html), [Pololu Line Follower Blog](https://www.pololu.com/blog/486/davids-line-following-robot-that-learns-the-course), [Hackaday Semreh Robot](https://hackaday.io/project/202208/logs?sort=oldest)*
