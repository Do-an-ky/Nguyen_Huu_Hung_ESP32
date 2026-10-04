// Mock tối thiểu của Arduino-ESP32 / FreeRTOS để biên dịch NGUYÊN VĂN các hàm lõi của firmware
// Mpu6050_VIP.ino (FW 3.0.0) trên máy tính. Không sửa bất kỳ dòng nào của firmware.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <string>
#include <algorithm>
#include <functional>
#include <vector>
#include <deque>
#include <stdexcept>

using std::min;
using std::max;
typedef uint8_t byte;
#define PROGMEM
#define PI 3.14159265358979f
#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
typedef std::string String;
class WiFiManager {};
class WiFiManagerParameter {};

// ---------- thời gian mô phỏng ----------
extern uint32_t g_now_ms;
inline uint32_t millis() { return g_now_ms; }
inline void delay(uint32_t ms) { g_now_ms += ms; }
inline void delayMicroseconds(uint32_t) {}

// ---------- GPIO / buzzer ----------
extern int g_level[64];
extern int g_button_level;            // LOW = đang nhấn
extern uint16_t g_buzzer_hz;
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int v) { g_level[pin] = v; }
inline int digitalRead(int pin) { return pin == 4 ? g_button_level : g_level[pin]; }
inline void tone(int, unsigned f) { g_buzzer_hz = (uint16_t)f; }
inline void noTone(int) { g_buzzer_hz = 0; }

// ---------- I2C ----------
extern bool g_i2c_ok;
struct TwoWire {
  void begin(int, int) {}
  void setClock(uint32_t) {}
  void beginTransmission(int) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission() { return g_i2c_ok ? 0 : 2; }
  void end() {}
};
extern TwoWire Wire;

// ---------- MPU6050_light (giả lập: giá trị do bộ phát tín hiệu cung cấp) ----------
struct Sample { float ax, ay, az, gx, gy, gz; };
extern std::function<Sample(uint32_t)> g_gen;
extern Sample g_last_sample;
extern int g_mpu_begin_status;
class MPU6050 {
 public:
  MPU6050(TwoWire&) {}
  byte begin(byte, byte) { return (byte)g_mpu_begin_status; }
  void update() { g_last_sample = g_gen ? g_gen(millis()) : Sample{0, 0, 1, 0, 0, 0}; }
  float getAccX() { return g_last_sample.ax; }
  float getAccY() { return g_last_sample.ay; }
  float getAccZ() { return g_last_sample.az; }
  float getGyroX() { return g_last_sample.gx; }
  float getGyroY() { return g_last_sample.gy; }
  float getGyroZ() { return g_last_sample.gz; }
};

// ---------- FreeRTOS ----------
typedef void* SemaphoreHandle_t;
struct Q { std::deque<std::vector<uint8_t>> d; size_t item; size_t cap; };
typedef Q* QueueHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
#define pdTRUE 1
#define pdMS_TO_TICKS(x) (x)
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return (void*)1; }
inline int xSemaphoreTake(SemaphoreHandle_t, int) { return 1; }
inline int xSemaphoreGive(SemaphoreHandle_t) { return 1; }
inline QueueHandle_t xQueueCreate(size_t len, size_t item) { auto* q = new Q; q->item = item; q->cap = len; return q; }
inline int xQueueSend(QueueHandle_t q, const void* p, int) {
  if (q->d.size() >= q->cap) return 0;
  q->d.emplace_back((const uint8_t*)p, (const uint8_t*)p + q->item);
  return 1;
}
inline int xQueueReceive(QueueHandle_t q, void* p, int) {
  if (q->d.empty()) return 0;
  memcpy(p, q->d.front().data(), q->item);
  q->d.pop_front();
  return 1;
}

// ---------- ESP ----------
struct RestartException : std::runtime_error { RestartException() : std::runtime_error("esp_restart") {} };
extern int g_wdt_resets;
inline void esp_task_wdt_reset() { g_wdt_resets++; }
inline void esp_restart() { throw RestartException(); }

// ---------- Serial ----------
extern std::vector<std::string> g_logs;
struct SerialMock {
  void println(const char* s) {
    char b[512];
    snprintf(b, sizeof(b), "[%7lu ms] %s", (unsigned long)millis(), s);
    g_logs.push_back(b);
  }
  void printf(const char*, ...) {}
};
extern SerialMock Serial;
