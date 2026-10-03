/*
 * HE THONG PHAT HIEN TE NGA - ESP32 + MPU6050        (FW 3.0.0 "VIP")
 *
 * Nen tang: v4 (FW 2.1.0, thuat toan 4 giai doan). GIU NGUYEN giao thuc voi app Flutter:
 *   - cac node Firebase: /devices/<id>/status, /events, /command
 *   - cac truong JSON cu (state, accTotal, gyroTotal, tiltAngle, acked, timestamp, rssi, uptime, fw, lastSeen)
 *   - FallEvent (trigger, peakAcc, peakGyro, tiltChange, freefallMs, impactToRestMs)
 *   - giao thuc lenh {cmd, id}: ACK / RESET / TEST
 *
 * Chuoi quyet dinh (TAT CA deu bat buoc):
 *   [1] Roi tu do   : tong thoi gian acc < 0.6 g >= 100 ms
 *   [2] Va cham+Xoay: acc > 1.5 g sat sau pha roi (<= 500 ms) hoac > 2.6 g; xoay dinh >= 50 / 100 deg/s
 *   [3] Nam yen     : bo tich luy "ro ri" >= 2.0 s / 2.5 s
 *   [4] Doi tu the  : goc huong trong luc sau/truoc nga >= 45 / 55 do
 *
 * NOVEL SO VOI v4 (2.1.0) -> 3.0.0:
 *   1. Giam sat canh bien: kiem tra I2C + loc mau rac (raw=-1/0 => ~0 g bi nham la roi tu do),
 *      tu khoi tao lai MPU + giai phong bus I2C (9 xung SCL), bao loi bang LED nhay nhanh,
 *      khong con while(1) khi MPU khong ket noi (thu lai 5 lan roi tu khoi dong lai).
 *   2. Khong con "mu" sau khi cap dien: Wi-Fi/cong cau hinh chay KHONG CHAN trong netTask,
 *      loop() lay mau ngay sau khi hieu chuan xong.
 *   3. netTask khong con bi dua vao WDT (WDT panic tu netTask co the reset ca chip va mat
 *      trang thai ALARM). Thay bang: handshake TLS toi da 10 s + loop() giam sat netTask,
 *      chi khoi dong lai khi dang IDLE.
 *   4. Pre-alarm (PRE_ALARM_MS): sau khi xac nhan nga, coi keu CUC BO truoc, nguoi dung giu nut
 *      1,5 s de huy => KHONG gui len ung dung. Het gio moi leo thang (status ALARM + event).
 *      SOS va TEST leo thang NGAY. Trong pre-alarm app thay trang thai CHECKING.
 *   5. Nut huy phai GIU (BTN_CANCEL_HOLD_MS) o moi giai doan => nga de len nut khong huy nham.
 *   6. Lenh cu tren Firebase khong con bi chay lai sau moi lan khoi dong (baseline id).
 *   7. Event giu dung thoi diem xay ra (timestamp tru di do tre gui), khong bi chan vinh vien
 *      boi loi 4xx, co backoff rieng.
 *   8. Coi cuc bo chi goi tone() khi doi tan so (khong cau hinh lai LEDC moi 10 ms), tu tat sau
 *      ALARM_BUZZER_MAX_MS.
 *   9. Tuy chon do pin (ENABLE_BATTERY) + canh bao pin yeu + truong "battery" trong status.
 *  10. I2C mac dinh 100 kHz (on dinh voi module clone/breadboard); lay mau 100 Hz van du.
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

// ======================= CẤU HÌNH HỆ THỐNG =======================
#define FW_VERSION "3.0.0"
#define FIREBASE_HOST "https://doanky-972e0-default-rtdb.firebaseio.com"
#define FIREBASE_API_KEY "AIzaSyAuQmHl2CMPMOvob7WXty1Xkvr9z79VzZY"

// Định danh thiết bị: mặc định, có thể đổi trong cổng cấu hình Wi-Fi.
#define DEFAULT_DEVICE_ID "device01"
char gDeviceId[24] = DEFAULT_DEVICE_ID;

// Bật kiểm tra chứng chỉ máy chủ (khuyến nghị cho bản triển khai thật).
// Khi bật, dán chứng chỉ gốc GTS Root R1 của Google vào FIREBASE_ROOT_CA.
#define TLS_VERIFY 0
static const char FIREBASE_ROOT_CA[] PROGMEM = R"CERT(
-----BEGIN CERTIFICATE-----
   (Dán nội dung chứng chỉ gốc GTS Root R1 tại đây khi bật TLS_VERIFY)
-----END CERTIFICATE-----
)CERT";

#define CPU_FREQ_MHZ 160  // 80 tiết kiệm pin hơn nhưng TLS chậm hơn; 160 là mức an toàn

// ======================= CHÂN KẾT NỐI =======================
#define PIN_LED 5
#define PIN_BUZZER 18
#define PIN_BUTTON 4
#define PIN_SDA 21
#define PIN_SCL 22

// ======================= ĐO PIN (TÙY CHỌN) =======================
// Chia áp 2 điện trở bằng nhau từ Vbat vào chân ADC1 (GPIO32..39). KHÔNG dùng ADC2 (xung đột Wi-Fi).
#define ENABLE_BATTERY 0
#define PIN_BATTERY 34
#define BATT_DIVIDER 2.0f  // Vbat = Vadc * BATT_DIVIDER
#define BATT_EMPTY_MV 3300
#define BATT_FULL_MV 4150
#define BATT_LOW_PCT 15  // dưới mức này: bíp nhắc mỗi 60 s khi IDLE
#define BATT_CHIRP_MS 60000

// ======================= CẤU HÌNH CẢM BIẾN =======================
// MPU6050_light: acc_config 3 = ±16 g, gyro_config 3 = ±2000 °/s
#define MPU_ACC_CONFIG 3
#define MPU_GYRO_CONFIG 3
#define MPU_DLPF_CFG 0x03  // thanh ghi CONFIG(0x1A) = 44 Hz
#define MPU_REG_CONFIG 0x1A
#define I2C_CLOCK_HZ 100000  // nâng lên 400000 chỉ khi đã kiểm chứng ổn định trên mạch thật

// ======================= NGƯỠNG THUẬT TOÁN (4 GIAI ĐOẠN) =======================
#ifndef ENABLE_ROTATION_CHECK
#define ENABLE_ROTATION_CHECK 1  // giai đoạn 2b: bắt buộc có xoay quanh va chạm
#endif
#ifndef ENABLE_ORIENT_CHECK
#define ENABLE_ORIENT_CHECK 1  // giai đoạn 4 : bắt buộc đổi tư thế
#endif

#define SAMPLE_PERIOD_MS 10  // 100 Hz

// --- Giai đoạn 1: rơi tự do ---
#define FREEFALL_THRESHOLD 0.60f   // g
#define MIN_FREEFALL_DURATION 100  // ms : TỔNG thời gian mất trọng lượng tối thiểu
#define TIME_IMPACT_MAX 1000       // ms : hạn chờ va chạm kể từ lúc bắt đầu rơi
#define IMPACT_GAP_MAX_MS 500      // ms : va chạm phải đến sát sau mẫu mất trọng lượng cuối

// --- Giai đoạn 2: va chạm + xoay ---
#define IMPACT_THRESHOLD 1.50f       // g
#define IMPACT_HARD_THRESHOLD 2.60f  // g  : va chạm mạnh, vào thẳng pha kiểm tra
#define GYRO_THRESHOLD 50.0f         // °/s: đường rơi tự do
#define GYRO_THRESHOLD_HARD 100.0f   // °/s: đường va chạm mạnh
#define PRE_IMPACT_BUF_SAMPLES 30    // mẫu: nhìn lại 300 ms trước va chạm mạnh

// --- Giai đoạn 3: nằm yên ---
#define MIN_REST_ACC 0.70f  // g  : dải LỌC TRỌNG LỰC
#define MAX_REST_ACC 1.30f
#define POST_REST_MIN_ACC 0.65f  // g  : dải "nằm yên" sau va chạm
#define POST_REST_MAX_ACC 1.35f
#define REST_GYRO_THRESHOLD 40.0f   // °/s
#define SETTLE_MS 500               // ms : bỏ qua pha nảy/lăn sau va chạm
#define REST_REQUIRED_MS 2000       // ms : đường rơi tự do
#define REST_REQUIRED_MS_HARD 2500  // ms : đường va chạm mạnh
#define MOTION_PENALTY 3            // mỗi ms chuyển động trừ 3 ms nằm yên
#define MOTION_CANCEL_MS 600        // ms : chuyển động LIÊN TỤC => huỷ chuỗi
#define CHECK_MAX_MS 8000           // ms : hạn tối đa của pha kiểm tra

// --- Giai đoạn 4: đổi tư thế ---
#define ORIENT_CHANGE_THRESHOLD 45.0f       // độ
#define ORIENT_CHANGE_THRESHOLD_HARD 55.0f  // độ
#define POSTURE_STABLE_DEG 15.0f            // độ : hướng trọng lực đã lọc phải bám sát mẫu hiện tại

#define GRAV_ALPHA_SLOW 0.01f
#define GRAV_ALPHA_FAST 0.10f

#define CALIBRATION_SAMPLES 100
#define CALIB_MAX_ATTEMPTS 3
#define CALIB_MOVE_GYRO_LIMIT 20.0f

// --- Pre-alarm / báo động tại chỗ ---
#define PRE_ALARM_MS 10000            // ms : cửa sổ huỷ cục bộ trước khi gửi lên app (0 = gửi ngay)
#define ALARM_BUZZER_MAX_MS 300000UL  // còi tự tắt sau 5 phút (đèn vẫn nháy)

// --- Nút nhấn ---
#define BTN_DEBOUNCE_SAMPLES 3   // 3 mẫu (30 ms) mới tính là nhấn
#define BTN_SOS_HOLD_MS 2000     // giữ nút 2 s ở IDLE => SOS
#define BTN_CANCEL_HOLD_MS 1500  // giữ nút 1,5 s mới huỷ (chống ngã đè lên nút)

// --- Giám sát cảm biến ---
#define I2C_FAIL_REINIT_SAMPLES 50     // 50 mẫu lỗi liên tiếp (~0,5 s) => khởi tạo lại MPU
#define SENSOR_FAULT_ABORT_MS 2000     // lỗi > 2 s khi đang kiểm tra => bỏ chuỗi, về IDLE
#define SENSOR_FAULT_RESTART_MS 30000  // lỗi > 30 s khi IDLE => khởi động lại

#define ENABLE_DATA_LOG true
#define ENABLE_CSV_STREAM false  // true: in CSV mỗi mẫu để phân tích

// ======================= CẤU HÌNH MẠNG =======================
#define WDT_TIMEOUT_SECONDS 20         // chỉ giám sát loop()
#define LOOP_STALL_LIMIT_MS 2000       // vòng lấy mẫu treo quá mức này => khởi động lại
#define NET_STALL_RESTART_MS 180000UL  // netTask treo quá mức này VÀ đang IDLE => khởi động lại
#define TLS_HANDSHAKE_TIMEOUT_S 10
#define WIFI_RECONNECT_INTERVAL 5000
#define WIFI_CONNECT_TIMEOUT_S 10  // chờ kết nối Wi-Fi lúc khởi động (chỉ chặn netTask)
#define PORTAL_TIMEOUT_S 180
#define HTTP_CONNECT_TIMEOUT_MS 5000
#define HTTP_TIMEOUT_MS 6000
#define HEARTBEAT_MS 10000
#define RETRY_BASE_MS 2000  // lùi theo cấp số nhân: 2s,4s,8s... tối đa 30s
#define RETRY_MAX_MS 30000
#define EVENT_MAX_4XX_TRIES 5  // lỗi 4xx "vĩnh viễn" thử tối đa chừng này lần rồi bỏ
#define CMD_POLL_ALARM_MS 2000
#define CMD_POLL_IDLE_MS 15000
#define BATT_UPDATE_MS 30000
static const unsigned long TOKEN_LIFETIME_MS = 3500000UL;

// ======================= KIỂU DỮ LIỆU =======================
enum FallState : uint8_t { STATE_IDLE = 0,
                           STATE_FREE_FALL,
                           STATE_CHECK,
                           STATE_ALARM };
enum PubState : uint8_t { PUB_IDLE = 0,
                          PUB_CHECKING = 1,
                          PUB_ALARM = 2 };
enum TriggerType : uint8_t { TRIG_FREEFALL = 0,
                             TRIG_HARD_IMPACT = 1,
                             TRIG_SOS = 2,
                             TRIG_TEST = 3 };
enum Command : uint8_t { CMD_NONE = 0,
                         CMD_ACK,
                         CMD_RESET,
                         CMD_TEST };

struct StatusSnapshot {  // loop() ghi, netTask đọc
  uint8_t pubState;
  bool acked;
  float acc;
  float gyro;
  float tilt;
  uint32_t seq;
};

struct FallEvent {  // loop() -> netTask
  uint8_t trigger;
  float peakAcc;
  float peakGyro;
  float tiltChange;
  uint16_t freefallMs;
  uint16_t impactToRestMs;
  uint32_t atMs;  // millis() lúc xảy ra, để gửi đúng timestamp dù gửi trễ
};

// ======================= BIẾN TOÀN CỤC =======================
MPU6050 mpu(Wire);
FallState currentState = STATE_IDLE;

static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
static StatusSnapshot gSnapshot = { PUB_IDLE, false, 1.0f, 0.0f, 0.0f, 0 };
static QueueHandle_t gCmdQueue = nullptr;    // netTask -> loop
static QueueHandle_t gEventQueue = nullptr;  // loop -> netTask
static SemaphoreHandle_t gSerialMutex = nullptr;
static volatile uint32_t gLoopHeartbeat = 0;  // loop tăng mỗi chu kỳ
static volatile uint32_t gNetHeartbeat = 0;   // netTask tăng mỗi vòng
static volatile int8_t gBatteryPct = -1;      // -1 = không rõ

// hiệu chuẩn
float accOffset = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float gRefX = 0, gRefY = 0, gRefZ = 1;

// mẫu hiện tại
float curAx = 0, curAy = 0, curAz = 1;

// hướng trọng lực đã lọc và ảnh chụp trước khi ngã
float gAvgX = 0, gAvgY = 0, gAvgZ = 1;
float gSnapX = 0, gSnapY = 0, gSnapZ = 1;

// biến của một sự kiện
uint32_t freeFallTime = 0, impactTime = 0;
uint32_t restAccumMs = 0, motionRunMs = 0;
uint16_t freefallDurationMs = 0;
float peakAcc = 0, peakGyro = 0;
bool hardImpactPath = false;
uint32_t lastLowTime = 0, ffLowMs = 0;
float rotPeakGyro = 0;
bool rotationVerified = false;

// điều khiển cục bộ
bool localAlarmOn = false, buzzerOn = false;
bool alarmAcked = false;
bool alarmEscalated = false;  // false = đang pre-alarm (chỉ báo tại chỗ)
uint8_t alarmTrigger = TRIG_FREEFALL;
uint32_t alarmStartMs = 0, escalateMs = 0;
static FallEvent gAlarmEvent;  // sự kiện chờ gửi khi leo thang
static uint16_t gToneNow = 0;

// nút nhấn
uint8_t btnLowCount = 0;
bool btnHeldHandled = false;
bool btnReleasedInAlarm = false;
uint32_t btnPressStart = 0;

// giám sát cảm biến
static uint16_t gBadSamples = 0;
static uint32_t gFaultSinceMs = 0;

// xác thực Firebase (chỉ netTask dùng sau setup)
String idToken = "", refreshToken = "";
unsigned long tokenObtainedTime = 0;

// Wi-Fi (chỉ netTask dùng sau setup)
static WiFiManager* gWm = nullptr;
static WiFiManagerParameter* gDevParam = nullptr;
static volatile bool gDevIdDirty = false;
static bool gForcePortal = false;

// ======================= TIỆN ÍCH =======================
void logLine(const char* s) {
  if (gSerialMutex && xSemaphoreTake(gSerialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println(s);
    xSemaphoreGive(gSerialMutex);
  } else {
    Serial.println(s);
  }
}

void logf(const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  logLine(buf);
}

float vecNorm(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

float vecAngleDeg(float ax, float ay, float az, float bx, float by, float bz) {
  float na = vecNorm(ax, ay, az), nb = vecNorm(bx, by, bz);
  if (na < 0.05f || nb < 0.05f) return 0;
  float c = (ax * bx + ay * by + az * bz) / (na * nb);
  c = constrain(c, -1.0f, 1.0f);
  return acosf(c) * 180.0f / PI;
}

void publishSnapshot(uint8_t pubState, bool acked, float acc, float gyro, float tilt, bool bumpSeq) {
  portENTER_CRITICAL(&gMux);
  if (bumpSeq && (gSnapshot.pubState != pubState || gSnapshot.acked != acked)) gSnapshot.seq++;
  gSnapshot.pubState = pubState;
  gSnapshot.acked = acked;
  gSnapshot.acc = acc;
  gSnapshot.gyro = gyro;
  gSnapshot.tilt = tilt;
  portEXIT_CRITICAL(&gMux);
}

StatusSnapshot readSnapshot() {
  StatusSnapshot s;
  portENTER_CRITICAL(&gMux);
  s = gSnapshot;
  portEXIT_CRITICAL(&gMux);
  return s;
}

// Trạng thái công bố cho app: pre-alarm vẫn là CHECKING, chỉ khi leo thang mới là ALARM.
static uint8_t pubStateNow() {
  switch (currentState) {
    case STATE_IDLE: return PUB_IDLE;
    case STATE_ALARM: return alarmEscalated ? PUB_ALARM : PUB_CHECKING;
    default: return PUB_CHECKING;
  }
}

// ======================= CẢM BIẾN =======================
void mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Giải phóng bus I2C bị slave giữ SDA (9 xung SCL + STOP).
static void i2cBusClear() {
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT);
  digitalWrite(PIN_SCL, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(10);
  }
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SDA, HIGH);
  delayMicroseconds(10);
  pinMode(PIN_SDA, INPUT_PULLUP);
}

static bool initSensor() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(50);
  byte status = mpu.begin(MPU_GYRO_CONFIG, MPU_ACC_CONFIG);
  if (status != 0) {
    logf("LOI: Khong ket noi duoc MPU6050! Ma loi: %d", status);
    return false;
  }
  mpuWriteRegister(MPU_REG_CONFIG, MPU_DLPF_CFG);  // DLPF 44 Hz - chống chồng phổ
  return true;
}

static void recoverSensor() {
  logLine("Cam bien loi lien tiep - giai phong I2C va khoi tao lai MPU...");
  Wire.end();
  i2cBusClear();
  if (initSensor()) logLine("Khoi tao lai MPU thanh cong.");
}

// Đọc I2C hỏng thường ra raw = -1 hoặc 0 ở cả 3 trục => ~0 g (dễ nhầm là rơi tự do).
static bool sampleIsGarbage(float x, float y, float z) {
  return fabsf(x) < 0.001f && fabsf(y) < 0.001f && fabsf(z) < 0.001f && fabsf(x - y) < 0.00002f && fabsf(y - z) < 0.00002f;
}

// true = mẫu hợp lệ và đã cập nhật curAx/Ay/Az.
bool readSensor() {
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() != 0) return false;  // MPU không phản hồi
  mpu.update();
  float x = mpu.getAccX(), y = mpu.getAccY(), z = mpu.getAccZ();
  if (sampleIsGarbage(x, y, z)) return false;
  curAx = x;
  curAy = y;
  curAz = z;
  return true;
}

float computeAccTotal() {
  return vecNorm(curAx, curAy, curAz) + accOffset;
}

float computeGyroTotal() {
  float gx = mpu.getGyroX() - gyroBiasX;
  float gy = mpu.getGyroY() - gyroBiasY;
  float gz = mpu.getGyroZ() - gyroBiasZ;
  return vecNorm(gx, gy, gz);
}

// Góc lệch tư thế so với tư thế lúc hiệu chuẩn (hiển thị trên app)
float computePostureTilt() {
  return vecAngleDeg(gAvgX, gAvgY, gAvgZ, gRefX, gRefY, gRefZ);
}

void updateGravity(float acc) {
  if (acc < MIN_REST_ACC || acc > MAX_REST_ACC) return;
  float a = (currentState == STATE_IDLE) ? GRAV_ALPHA_SLOW : GRAV_ALPHA_FAST;
  gAvgX += a * (curAx - gAvgX);
  gAvgY += a * (curAy - gAvgY);
  gAvgZ += a * (curAz - gAvgZ);
}

bool calibrateSensor() {
  logLine("Dang hieu chuan cam bien... Giu yen thiet bi!");
  for (int attempt = 1; attempt <= CALIB_MAX_ATTEMPTS; attempt++) {
    float sumMag = 0, sax = 0, say = 0, saz = 0;
    float sgx = 0, sgy = 0, sgz = 0, sumGyroMag = 0;
    int valid = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      if (readSensor()) {
        float gx = mpu.getGyroX(), gy = mpu.getGyroY(), gz = mpu.getGyroZ();
        sumMag += vecNorm(curAx, curAy, curAz);
        sax += curAx;
        say += curAy;
        saz += curAz;
        sgx += gx;
        sgy += gy;
        sgz += gz;
        sumGyroMag += vecNorm(gx, gy, gz);
        valid++;
      }
      delay(20);
    }

    if (valid < (CALIBRATION_SAMPLES * 8) / 10) {
      logLine("Qua nhieu mau cam bien loi khi hieu chuan.");
      if (attempt < CALIB_MAX_ATTEMPTS) continue;
      return false;
    }

    float n = (float)valid;
    if (sumGyroMag / n > CALIB_MOVE_GYRO_LIMIT && attempt < CALIB_MAX_ATTEMPTS) {
      logLine("Thiet bi dang chuyen dong khi hieu chuan - thu lai...");
      continue;
    }

    gyroBiasX = sgx / n;
    gyroBiasY = sgy / n;
    gyroBiasZ = sgz / n;
    accOffset = 1.0f - (sumMag / n);
    if (fabsf(accOffset) > 0.3f) {
      logLine("Offset gia toc bat thuong - bo qua bu offset.");
      accOffset = 0;
    }
    gAvgX = sax / n;
    gAvgY = say / n;
    gAvgZ = saz / n;
    gRefX = gAvgX;
    gRefY = gAvgY;
    gRefZ = gAvgZ;
    logf("Hieu chuan xong. accOffset=%.4f | gyroBias=%.2f,%.2f,%.2f",
         accOffset, gyroBiasX, gyroBiasY, gyroBiasZ);
    return true;
  }
  return false;
}

// ======================= CẢNH BÁO TẠI CHỖ =======================
static void setTone(uint16_t f) {  // chỉ cấu hình lại khi đổi tần số
  if (f == gToneNow) return;
  gToneNow = f;
  if (f) tone(PIN_BUZZER, f);
  else noTone(PIN_BUZZER);
}

void startLocalAlarm() {
  localAlarmOn = true;
  buzzerOn = true;
}

void muteBuzzer() {
  buzzerOn = false;
  setTone(0);
}

void stopLocalAlarm() {
  localAlarmOn = false;
  buzzerOn = false;
  setTone(0);
  digitalWrite(PIN_LED, LOW);
}

// Pre-alarm : bíp nhanh 150/150 ms (2 kHz), đèn nháy nhanh.
// Leo thang : 300 ms bật / 200 ms tắt (3 kHz); đã tiếp nhận => tắt còi, đèn nháy 1 Hz.
void updateAlarmOutputs(uint32_t now) {
  if (!localAlarmOn) return;

  if (!alarmEscalated) {
    const uint32_t ph = (now - alarmStartMs) % 300;
    if (buzzerOn) setTone(ph < 150 ? 2000 : 0);
    digitalWrite(PIN_LED, ph < 150 ? HIGH : LOW);
    return;
  }

  if (buzzerOn && (now - escalateMs) > ALARM_BUZZER_MAX_MS) muteBuzzer();
  if (buzzerOn) {
    const uint32_t ph = (now - escalateMs) % 500;
    setTone(ph < 300 ? 3000 : 0);
  }
  if (alarmAcked) digitalWrite(PIN_LED, ((now / 500) % 2) == 0 ? HIGH : LOW);
  else digitalWrite(PIN_LED, HIGH);
}

// ======================= NÚT NHẤN =======================
// Giữ >= BTN_CANCEL_HOLD_MS : huỷ chuỗi xác nhận / pre-alarm / báo động.
// Giữ >= BTN_SOS_HOLD_MS khi IDLE : kích hoạt SOS thủ công.
void updateButton(uint32_t now) {
  bool down = (digitalRead(PIN_BUTTON) == LOW);
  if (down) {
    if (btnLowCount < 255) btnLowCount++;
    if (btnLowCount == BTN_DEBOUNCE_SAMPLES) btnPressStart = now;
  } else {
    btnLowCount = 0;
    btnHeldHandled = false;
  }
}

bool buttonPressed() {
  return btnLowCount >= BTN_DEBOUNCE_SAMPLES;
}

bool buttonHeldFor(uint32_t now, uint32_t ms) {
  return buttonPressed() && (now - btnPressStart >= ms);
}

bool buttonHeldForSos(uint32_t now) {
  if (!buttonPressed() || btnHeldHandled) return false;
  if (now - btnPressStart >= BTN_SOS_HOLD_MS) {
    btnHeldHandled = true;
    return true;
  }
  return false;
}

// ======================= CHUYỂN TRẠNG THÁI =======================
void setState(FallState s) {
  currentState = s;
  StatusSnapshot cur = readSnapshot();
  publishSnapshot(pubStateNow(), alarmAcked, cur.acc, cur.gyro, cur.tilt, true);
}

void returnToIdle(const char* why) {
  if (buttonPressed()) btnHeldHandled = true;  // nút còn giữ: không kích SOS oan
  stopLocalAlarm();
  alarmAcked = false;
  alarmEscalated = false;
  hardImpactPath = false;
  restAccumMs = motionRunMs = 0;
  setState(STATE_IDLE);
  logLine(why);
}

// Leo thang: công bố ALARM + đẩy sự kiện lên Firebase.
void escalateAlarm() {
  if (alarmEscalated) return;
  alarmEscalated = true;
  escalateMs = millis();
  StatusSnapshot s = readSnapshot();
  publishSnapshot(PUB_ALARM, alarmAcked, s.acc, s.gyro, s.tilt, true);
  if (gEventQueue) xQueueSend(gEventQueue, &gAlarmEvent, 0);  // không chặn vòng lấy mẫu
  logLine("[ALARM] Leo thang len ung dung.");
}

void enterAlarm(uint8_t trigger, float tiltChange, uint16_t impactToRestMs) {
  const bool immediate = (trigger == TRIG_SOS || trigger == TRIG_TEST || PRE_ALARM_MS == 0);

  alarmAcked = false;
  alarmEscalated = false;
  btnReleasedInAlarm = !buttonPressed();  // đang giữ nút (SOS) => chờ nhả
  alarmTrigger = trigger;
  alarmStartMs = millis();
  escalateMs = alarmStartMs;

  gAlarmEvent.trigger = trigger;
  gAlarmEvent.peakAcc = peakAcc;
  gAlarmEvent.peakGyro = peakGyro;
  gAlarmEvent.tiltChange = tiltChange;
  gAlarmEvent.freefallMs = freefallDurationMs;
  gAlarmEvent.impactToRestMs = impactToRestMs;
  gAlarmEvent.atMs = alarmStartMs;

  setState(STATE_ALARM);
  startLocalAlarm();  // cảnh báo tại chỗ TRƯỚC, không chờ mạng
  logf("[ALARM] trigger=%u peakAcc=%.2fg peakGyro=%.1f tilt=%.1f (%s)",
       trigger, peakAcc, peakGyro, tiltChange, immediate ? "gui ngay" : "pre-alarm");

  if (immediate) escalateAlarm();
}

// ======================= MÁY TRẠNG THÁI 4 GIAI ĐOẠN =======================
static float gGyroHist[PRE_IMPACT_BUF_SAMPLES];
static uint8_t gGyroHistIdx = 0;

static void pushGyroHistory(float g) {
  gGyroHist[gGyroHistIdx] = g;
  gGyroHistIdx = (uint8_t)((gGyroHistIdx + 1) % PRE_IMPACT_BUF_SAMPLES);
}

static float maxGyroHistory() {
  float m = 0;
  for (uint8_t i = 0; i < PRE_IMPACT_BUF_SAMPLES; i++) m = fmaxf(m, gGyroHist[i]);
  return m;
}

void enterCheckPhase(uint32_t now, bool hardPath) {
  impactTime = now;
  restAccumMs = 0;
  motionRunMs = 0;
  hardImpactPath = hardPath;
  rotationVerified = false;
  setState(STATE_CHECK);
  logf("[Giai doan 2] Impact! duong=%s", hardPath ? "va cham manh" : "sau roi tu do");
}

void runStateMachine(uint32_t now, uint32_t dt, float acc, float gyro) {
  pushGyroHistory(gyro);

  switch (currentState) {

    // ---------- GIAI ĐOẠN 1: chờ rơi tự do / va chạm mạnh ----------
    case STATE_IDLE:
      {
        if (buttonHeldForSos(now)) {  // SOS thủ công
          peakAcc = acc;
          peakGyro = gyro;
          freefallDurationMs = 0;
          gSnapX = gAvgX;
          gSnapY = gAvgY;
          gSnapZ = gAvgZ;
          enterAlarm(TRIG_SOS, 0.0f, 0);
          break;
        }
        if (acc < FREEFALL_THRESHOLD) {  // đường 1: rơi tự do
          freeFallTime = now;
          lastLowTime = now;
          ffLowMs = dt;
          gSnapX = gAvgX;
          gSnapY = gAvgY;
          gSnapZ = gAvgZ;  // tư thế TRƯỚC khi ngã
          peakAcc = acc;
          peakGyro = gyro;
          rotPeakGyro = gyro;
          setState(STATE_FREE_FALL);
          logLine("[Giai doan 1] Free-fall!");
        } else if (acc > IMPACT_HARD_THRESHOLD) {  // đường 2: va chạm mạnh trực tiếp
          gSnapX = gAvgX;
          gSnapY = gAvgY;
          gSnapZ = gAvgZ;
          freefallDurationMs = 0;
          rotPeakGyro = fmaxf(maxGyroHistory(), gyro);
          peakAcc = acc;
          peakGyro = rotPeakGyro;
          enterCheckPhase(now, true);
        }
        break;
      }

    // ---------- GIAI ĐOẠN 1 -> 2: đang rơi, chờ va chạm ----------
    case STATE_FREE_FALL:
      {
        peakAcc = fmaxf(peakAcc, acc);
        peakGyro = fmaxf(peakGyro, gyro);
        rotPeakGyro = fmaxf(rotPeakGyro, gyro);

        if (buttonHeldFor(now, BTN_CANCEL_HOLD_MS)) {
          returnToIdle("Nguoi dung huy bang nut nhan.");
          break;
        }

        if (acc < FREEFALL_THRESHOLD) {
          ffLowMs += dt;
          lastLowTime = now;
        }

        if (acc > IMPACT_THRESHOLD) {
          const uint32_t span = lastLowTime - freeFallTime;
          const uint32_t gap = now - lastLowTime;
          freefallDurationMs = (uint16_t)min<uint32_t>(ffLowMs, 65535);
#if ENABLE_DATA_LOG
          logf("[DATA] freefall=%lums (span=%lums gap=%lums) impactAcc=%.2fg peakGyro=%.1fdeg/s",
               (unsigned long)ffLowMs, (unsigned long)span, (unsigned long)gap, acc, peakGyro);
#else
          (void)span;
#endif
          if (ffLowMs < MIN_FREEFALL_DURATION) returnToIdle("Bo qua - roi tu do qua ngan.");
          else if (gap > IMPACT_GAP_MAX_MS) returnToIdle("Bo qua - va cham khong lien ngay sau roi.");
          else enterCheckPhase(now, false);
        } else if (now - freeFallTime > TIME_IMPACT_MAX) {
          returnToIdle("Bo qua - het thoi gian cho va cham.");
        }
        break;
      }

    // ---------- GIAI ĐOẠN 2b + 3 + 4: xoay -> nằm yên -> đổi tư thế ----------
    case STATE_CHECK:
      {
        peakAcc = fmaxf(peakAcc, acc);
        peakGyro = fmaxf(peakGyro, gyro);

        if (buttonHeldFor(now, BTN_CANCEL_HOLD_MS)) {
          returnToIdle("Nguoi dung huy bang nut nhan.");
          break;
        }

        uint32_t elapsed = now - impactTime;

        if (elapsed < SETTLE_MS) {  // pha nảy/lăn: chỉ ghi nhận xoay
          rotPeakGyro = fmaxf(rotPeakGyro, gyro);
          break;
        }
        if (elapsed > CHECK_MAX_MS) {
          returnToIdle("Bo qua - het thoi gian kiem tra.");
          break;
        }

#if ENABLE_ROTATION_CHECK
        if (!rotationVerified) {
          rotationVerified = true;
          const float gyroNeed = hardImpactPath ? GYRO_THRESHOLD_HARD : GYRO_THRESHOLD;
          logf("[Giai doan 2] Xoay: dinh=%.1f can>=%.1f deg/s", rotPeakGyro, gyroNeed);
          if (rotPeakGyro < gyroNeed) {
            returnToIdle("Bo qua - khong xoay du.");
            break;
          }
        }
#endif

        const bool resting = (acc >= POST_REST_MIN_ACC && acc <= POST_REST_MAX_ACC && gyro < REST_GYRO_THRESHOLD);
        if (resting) {
          restAccumMs += dt;
          motionRunMs = 0;
        } else {
          const uint32_t penalty = dt * MOTION_PENALTY;
          restAccumMs = (restAccumMs > penalty) ? (restAccumMs - penalty) : 0;
          motionRunMs += dt;
          if (motionRunMs >= MOTION_CANCEL_MS) {
            returnToIdle("Bo qua - nguoi dung van dang chuyen dong.");
            break;
          }
        }

        const uint32_t restNeed = hardImpactPath ? REST_REQUIRED_MS_HARD : REST_REQUIRED_MS;
        if (restAccumMs >= restNeed) {
          float settleDev = vecAngleDeg(curAx, curAy, curAz, gAvgX, gAvgY, gAvgZ);
          if (settleDev > POSTURE_STABLE_DEG) break;

          float tiltChange = vecAngleDeg(gAvgX, gAvgY, gAvgZ, gSnapX, gSnapY, gSnapZ);
          const float orientNeed = hardImpactPath ? ORIENT_CHANGE_THRESHOLD_HARD : ORIENT_CHANGE_THRESHOLD;
          logf("[Giai doan 3-4] nam yen %lums, doi tu the=%.1f do (can>=%.1f)",
               (unsigned long)restAccumMs, tiltChange, orientNeed);
#if ENABLE_ORIENT_CHECK
          if (tiltChange < orientNeed) {
            returnToIdle("Bo qua - tu the khong thay doi.");
            break;
          }
#endif
          enterAlarm(hardImpactPath ? TRIG_HARD_IMPACT : TRIG_FREEFALL,
                     tiltChange, (uint16_t)min<uint32_t>(elapsed, 65535));
        }
        break;
      }

    // ---------- BÁO ĐỘNG (pre-alarm -> leo thang) ----------
    // Nút phải được NHẢ ra ít nhất một lần rồi mới GIỮ lại để huỷ; nếu không, SOS (giữ nút 2 s)
    // sẽ tự huỷ ngay vì nút vẫn đang bị giữ.
    case STATE_ALARM:
      {
        if (!alarmEscalated && (now - alarmStartMs) >= (uint32_t)PRE_ALARM_MS) escalateAlarm();

        if (!buttonPressed()) btnReleasedInAlarm = true;
        else if (btnReleasedInAlarm && buttonHeldFor(now, BTN_CANCEL_HOLD_MS)) {
          returnToIdle(alarmEscalated ? "Da huy bao dong bang nut nhan."
                                      : "Da huy trong pre-alarm - KHONG gui len ung dung.");
        }
        break;
      }
  }
}

// Xử lý lệnh nhận từ ứng dụng (do netTask đẩy vào hàng đợi)
void processCommands() {
  Command c;
  while (gCmdQueue && xQueueReceive(gCmdQueue, &c, 0) == pdTRUE) {
    switch (c) {
      case CMD_ACK:
        if (currentState == STATE_ALARM && alarmEscalated && !alarmAcked) {
          alarmAcked = true;
          muteBuzzer();
          StatusSnapshot s = readSnapshot();
          publishSnapshot(PUB_ALARM, true, s.acc, s.gyro, s.tilt, true);
          logLine("Nguoi than da tiep nhan - tat coi.");
        }
        break;
      case CMD_RESET:
        if (currentState == STATE_ALARM) returnToIdle("Da ket thuc canh bao tu ung dung.");
        break;
      case CMD_TEST:
        if (currentState == STATE_IDLE) {
          peakAcc = 0;
          peakGyro = 0;
          freefallDurationMs = 0;
          enterAlarm(TRIG_TEST, 0.0f, 0);
        }
        break;
      default: break;
    }
  }
}

// ======================= TIỆN ÍCH MẠNG =======================
static WiFiClientSecure gSecure;

String jsonGetString(const String& s, const char* key) {
  String k = String("\"") + key + "\"";
  int i = s.indexOf(k);
  if (i < 0) return "";
  int c = s.indexOf(':', i + k.length());
  if (c < 0) return "";
  int q1 = s.indexOf('"', c + 1);
  if (q1 < 0) return "";
  int q2 = s.indexOf('"', q1 + 1);
  if (q2 < 0) return "";
  return s.substring(q1 + 1, q2);
}

double jsonGetNumber(const String& s, const char* key, double def) {
  String k = String("\"") + key + "\"";
  int i = s.indexOf(k);
  if (i < 0) return def;
  int c = s.indexOf(':', i + k.length());
  if (c < 0) return def;
  int j = c + 1;
  while (j < (int)s.length() && (s[j] == ' ' || s[j] == '"')) j++;
  int st = j;
  while (j < (int)s.length() && (isdigit(s[j]) || s[j] == '-' || s[j] == '.' || s[j] == 'e')) j++;
  if (j == st) return def;
  return s.substring(st, j).toDouble();
}

int httpsRequest(const String& url, const char* method, const char* body,
                 const char* contentType, String& resp) {
  HTTPClient http;
  http.setReuse(true);
  http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(gSecure, url)) return -1;
  if (contentType != nullptr) http.addHeader("Content-Type", contentType);

  int code;
  if (strcmp(method, "GET") == 0) code = http.GET();
  else code = http.sendRequest(method, (uint8_t*)body, body ? strlen(body) : 0);

  resp = (code > 0) ? http.getString() : String("");
  http.end();
  return code;
}

void loadDeviceConfig() {
  Preferences p;
  p.begin("fbcfg", true);
  String id = p.getString("devid", DEFAULT_DEVICE_ID);
  refreshToken = p.getString("rt", "");
  p.end();
  id.toCharArray(gDeviceId, sizeof(gDeviceId));
}

void saveRefreshToken() {
  Preferences p;
  p.begin("fbcfg", false);
  p.putString("rt", refreshToken);
  p.end();
}

void saveDeviceId(const char* id) {
  Preferences p;
  p.begin("fbcfg", false);
  p.putString("devid", id);
  p.end();
}

bool signInAnonymously() {
  String resp;
  String url = String("https://identitytoolkit.googleapis.com/v1/accounts:signUp?key=") + FIREBASE_API_KEY;
  int code = httpsRequest(url, "POST", "{\"returnSecureToken\":true}", "application/json", resp);
  if (code != 200) {
    logf("Dang nhap Firebase that bai, ma: %d", code);
    return false;
  }
  String tok = jsonGetString(resp, "idToken");
  if (tok.length() == 0) return false;
  idToken = tok;
  refreshToken = jsonGetString(resp, "refreshToken");
  tokenObtainedTime = millis();
  saveRefreshToken();
  logLine("Dang nhap an danh Firebase thanh cong.");
  return true;
}

bool refreshIdToken() {
  if (refreshToken.length() == 0) return false;
  String resp;
  String url = String("https://securetoken.googleapis.com/v1/token?key=") + FIREBASE_API_KEY;
  String body = String("grant_type=refresh_token&refresh_token=") + refreshToken;
  int code = httpsRequest(url, "POST", body.c_str(), "application/x-www-form-urlencoded", resp);
  if (code != 200) {
    logf("Lam moi token that bai, ma: %d", code);
    if (code == 400) {
      refreshToken = "";
      saveRefreshToken();
    }
    return false;
  }
  String tok = jsonGetString(resp, "id_token");
  if (tok.length() == 0) return false;
  idToken = tok;
  String newRt = jsonGetString(resp, "refresh_token");
  if (newRt.length() > 0 && newRt != refreshToken) {
    refreshToken = newRt;
    saveRefreshToken();
  }
  tokenObtainedTime = millis();
  return true;
}

bool ensureValidToken() {
  if (idToken.length() > 0 && (millis() - tokenObtainedTime) < TOKEN_LIFETIME_MS) return true;
  if (refreshIdToken()) return true;
  return signInAnonymously();
}

#if ENABLE_BATTERY
static int readBatteryPercent() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_BATTERY);
  float vbat = (mv / 8.0f) * BATT_DIVIDER;
  int pct = (int)((vbat - BATT_EMPTY_MV) * 100.0f / (float)(BATT_FULL_MV - BATT_EMPTY_MV));
  return constrain(pct, 0, 100);
}
#endif

// Dựng payload bằng bộ đệm tĩnh (không dùng String => không phân mảnh heap)
void buildStatusPayload(const StatusSnapshot& s, char* out, size_t n) {
  static const char* NAMES[3] = { "IDLE", "CHECKING", "ALARM" };
  float a = isnan(s.acc) ? 0 : s.acc, g = isnan(s.gyro) ? 0 : s.gyro, t = isnan(s.tilt) ? 0 : s.tilt;
  time_t nowT = time(nullptr);
  long ts = (nowT > 100000) ? (long)nowT : -1;
  snprintf(out, n,
           "{\"state\":\"%s\",\"accTotal\":%.2f,\"gyroTotal\":%.1f,\"tiltAngle\":%.1f,"
           "\"acked\":%s,\"timestamp\":%ld,\"rssi\":%d,\"uptime\":%lu,\"fw\":\"%s\","
           "\"lastSeen\":{\".sv\":\"timestamp\"}}",
           NAMES[s.pubState > 2 ? 0 : s.pubState], a, g, t,
           s.acked ? "true" : "false", ts, (int)WiFi.RSSI(),
           (unsigned long)(millis() / 1000), FW_VERSION);
#if ENABLE_BATTERY
  size_t l = strlen(out);
  if (l > 0 && out[l - 1] == '}' && (l + 20) < n) {
    snprintf(out + l - 1, n - (l - 1), ",\"battery\":%d}", (int)gBatteryPct);
  }
#endif
}

bool sendStatus(const StatusSnapshot& s) {
  if (!ensureValidToken()) return false;
  char payload[384];
  buildStatusPayload(s, payload, sizeof(payload));
  String resp;
  String url = String(FIREBASE_HOST) + "/devices/" + gDeviceId + "/status.json?auth=" + idToken;
  int code = httpsRequest(url, "PUT", payload, "application/json", resp);
  if (code == 200) return true;
  logf("Gui Firebase LOI, ma: %d", code);
  if (code == 401 || code == 403) idToken = "";
  return false;
}

// Trả về mã HTTP (200 = thành công); -2 = chưa có token.
int sendEvent(const FallEvent& ev) {
  if (!ensureValidToken()) return -2;
  static const char* TRIG[4] = { "FREEFALL", "HARD_IMPACT", "SOS", "TEST" };
  char payload[320];
  time_t nowT = time(nullptr);
  long ts = -1;
  if (nowT > 100000) {  // timestamp = thời điểm XẢY RA, không phải lúc gửi
    long ageS = (long)((millis() - ev.atMs) / 1000UL);
    ts = (long)nowT - ageS;
  }
  snprintf(payload, sizeof(payload),
           "{\"trigger\":\"%s\",\"peakAcc\":%.2f,\"peakGyro\":%.1f,\"tiltChange\":%.1f,"
           "\"freefallMs\":%u,\"impactToRestMs\":%u,\"timestamp\":%ld,"
           "\"at\":{\".sv\":\"timestamp\"}}",
           TRIG[ev.trigger > 3 ? 0 : ev.trigger], ev.peakAcc, ev.peakGyro, ev.tiltChange,
           (unsigned)ev.freefallMs, (unsigned)ev.impactToRestMs, ts);
  String resp;
  String url = String(FIREBASE_HOST) + "/devices/" + gDeviceId + "/events.json?auth=" + idToken;
  int code = httpsRequest(url, "POST", payload, "application/json", resp);
  if (code == 401 || code == 403) idToken = "";
  return code;
}

// Giao thức lệnh idempotent: ứng dụng ghi {cmd, id}; thiết bị chỉ ĐỌC.
// Lần đọc thành công đầu tiên sau khi khởi động chỉ ghi nhớ id (baseline), KHÔNG thực thi lệnh cũ.
Command pollCommand(double& lastCmdId, bool& baselineDone) {
  if (!ensureValidToken()) return CMD_NONE;
  String resp;
  String url = String(FIREBASE_HOST) + "/devices/" + gDeviceId + "/command.json?auth=" + idToken;
  int code = httpsRequest(url, "GET", nullptr, nullptr, resp);
  if (code == 401 || code == 403) idToken = "";
  if (code != 200) return CMD_NONE;

  const bool hasCmd = (resp.length() >= 5 && resp != "null");
  const double id = hasCmd ? jsonGetNumber(resp, "id", 0) : 0;

  if (!baselineDone) {
    baselineDone = true;
    if (id > lastCmdId) lastCmdId = id;
    return CMD_NONE;
  }
  if (!hasCmd || id <= lastCmdId) return CMD_NONE;

  String cmd = jsonGetString(resp, "cmd");
  lastCmdId = id;

  if (cmd == "ACK") return CMD_ACK;
  if (cmd == "RESET") return CMD_RESET;
  if (cmd == "TEST") return CMD_TEST;
  return CMD_NONE;
}

// ======================= WI-FI (KHÔNG CHẶN) =======================
static void applyDeviceIdFromPortal() {
  gDevIdDirty = false;
  if (!gDevParam) return;
  const char* v = gDevParam->getValue();
  if (v && strlen(v) > 0 && strcmp(v, gDeviceId) != 0) {
    strncpy(gDeviceId, v, sizeof(gDeviceId) - 1);
    gDeviceId[sizeof(gDeviceId) - 1] = '\0';
    saveDeviceId(gDeviceId);
    logf("Doi device id -> %s", gDeviceId);
  }
}

// Gọi từ netTask. Chỉ chờ kết nối tối đa WIFI_CONNECT_TIMEOUT_S giây; nếu thất bại, mở cổng cấu hình
// ở chế độ KHÔNG CHẶN (xử lý bằng gWm->process() trong netTask) - loop() vẫn lấy mẫu bình thường.
static void startWiFi(bool forcePortal) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  gWm = new WiFiManager();
  gDevParam = new WiFiManagerParameter("devid", "Ma thiet bi (device id)", gDeviceId, sizeof(gDeviceId) - 1);
  gWm->addParameter(gDevParam);
  gWm->setSaveParamsCallback([]() {
    gDevIdDirty = true;
  });
  gWm->setSaveConfigCallback([]() {
    gDevIdDirty = true;
  });
  gWm->setConnectTimeout(WIFI_CONNECT_TIMEOUT_S);
  gWm->setConfigPortalTimeout(PORTAL_TIMEOUT_S);
  gWm->setConfigPortalBlocking(false);

  logLine("Dang ket noi WiFi (khong chan lay mau)...");
  bool connected = forcePortal
                     ? gWm->startConfigPortal("FallDetector-Setup", "12345678")
                     : gWm->autoConnect("FallDetector-Setup", "12345678");

  if (connected) logf("WiFi da ket noi! IP: %s", WiFi.localIP().toString().c_str());
  else logLine("Chua co WiFi - cong cau hinh dang mo; thiet bi van phat hien te nga cuc bo.");
}

// ======================= TÁC VỤ MẠNG (nhân 0) =======================
// KHÔNG đăng ký vào WDT: treo mạng không được phép reset cả chip (mất trạng thái ALARM).
// loop() giám sát netTask và chỉ khởi động lại khi đang IDLE.
void netTask(void* pv) {
  startWiFi(gForcePortal);

  unsigned long lastOk = 0, lastAttempt = 0, lastWifiRetry = 0, lastPoll = 0;
  unsigned long evNextTry = 0, lastBatt = 0;
  unsigned long backoff = RETRY_BASE_MS;
  bool lastFailed = false, ntpStarted = false;
  uint32_t lastSeq = 0xFFFFFFFF;
  uint32_t lastLoopBeat = 0, lastBeatChange = millis();
  double lastCmdId = 0;
  bool cmdBaselineDone = false;
  FallEvent pendingEvent;
  bool hasPendingEvent = false;
  uint8_t evTries = 0;

  for (;;) {
    gNetHeartbeat++;
    unsigned long now = millis();

    // 0) Giám sát chéo: vòng lấy mẫu (chức năng an toàn) phải luôn chạy
    uint32_t beat = gLoopHeartbeat;
    if (beat != lastLoopBeat) {
      lastLoopBeat = beat;
      lastBeatChange = now;
    } else if (now - lastBeatChange > LOOP_STALL_LIMIT_MS) {
      logLine("Vong lay mau bi treo - khoi dong lai thiet bi.");
      delay(100);
      esp_restart();
    }

    // 0b) Cổng cấu hình Wi-Fi (không chặn)
    if (gWm && gWm->getConfigPortalActive()) {
      gWm->process();
      if (gDevIdDirty) applyDeviceIdFromPortal();
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (gDevIdDirty) applyDeviceIdFromPortal();

    // 1) Bảo đảm có Wi-Fi
    if (WiFi.status() != WL_CONNECTED) {
      if (now - lastWifiRetry >= WIFI_RECONNECT_INTERVAL) {
        lastWifiRetry = now;
        logLine("Mat WiFi - dang thu ket noi lai...");
        WiFi.reconnect();
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // 2) Đồng bộ thời gian (một lần khi có mạng)
    if (!ntpStarted) {
      configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
      ntpStarted = true;
    }

#if ENABLE_BATTERY
    if (lastBatt == 0 || now - lastBatt >= BATT_UPDATE_MS) {
      lastBatt = now;
      gBatteryPct = (int8_t)readBatteryPercent();
    }
#else
    (void)lastBatt;
#endif

    // 3) Gửi trạng thái: khi đổi trạng thái, theo nhịp tim, hoặc gửi lại khi lỗi
    StatusSnapshot s = readSnapshot();
    bool changed = (s.seq != lastSeq);
    bool due = (lastOk == 0) || (now - lastOk >= HEARTBEAT_MS);
    bool canTry = !lastFailed || (now - lastAttempt >= backoff);

    if ((changed || due) && canTry) {
      lastAttempt = now;
      if (sendStatus(s)) {
        lastOk = millis();
        lastFailed = false;
        backoff = RETRY_BASE_MS;
        lastSeq = s.seq;
      } else {
        lastFailed = true;
        backoff = min<unsigned long>(backoff * 2, RETRY_MAX_MS);
      }
    }

    // 4) Đẩy nhật ký sự kiện ngã: lỗi mạng/5xx thử mãi; lỗi 4xx "vĩnh viễn" bỏ sau EVENT_MAX_4XX_TRIES
    if (!hasPendingEvent && gEventQueue) {
      hasPendingEvent = (xQueueReceive(gEventQueue, &pendingEvent, 0) == pdTRUE);
      if (hasPendingEvent) {
        evTries = 0;
        evNextTry = 0;
      }
    }
    if (hasPendingEvent && now >= evNextTry) {
      int code = sendEvent(pendingEvent);
      if (code == 200) {
        hasPendingEvent = false;
        evTries = 0;
      } else {
        if (evTries < 255) evTries++;
        bool permanent4xx = (code >= 400 && code < 500 && code != 401 && code != 403 && code != 408 && code != 429);
        if (permanent4xx && evTries >= EVENT_MAX_4XX_TRIES) {
          logf("Bo su kien - loi vinh vien, ma: %d", code);
          hasPendingEvent = false;
          evTries = 0;
        } else {
          unsigned long d = (unsigned long)RETRY_BASE_MS << (evTries < 4 ? evTries : 4);
          evNextTry = millis() + min<unsigned long>(d, RETRY_MAX_MS);
        }
      }
    }

    // 5) Đọc lệnh từ ứng dụng: dày khi đang cảnh báo, thưa khi bình thường
    unsigned long pollPeriod = (s.pubState == PUB_ALARM) ? CMD_POLL_ALARM_MS : CMD_POLL_IDLE_MS;
    if (now - lastPoll >= pollPeriod) {
      lastPoll = now;
      Command c = pollCommand(lastCmdId, cmdBaselineDone);
      if (c != CMD_NONE && gCmdQueue) xQueueSend(gCmdQueue, &c, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ======================= GIÁM SÁT TÁC VỤ MẠNG (gọi từ loop) =======================
static void checkNetTaskAlive(uint32_t now) {
  static uint32_t lastBeat = 0, lastChange = 0;
  if (lastChange == 0) lastChange = now;
  if (gNetHeartbeat != lastBeat) {
    lastBeat = gNetHeartbeat;
    lastChange = now;
  } else if (currentState == STATE_IDLE && (now - lastChange) > NET_STALL_RESTART_MS) {
    logLine("netTask bi treo qua lau (dang IDLE) - khoi dong lai thiet bi.");
    delay(100);
    esp_restart();
  }
}

// ======================= SETUP =======================
void setup() {
  setCpuFrequencyMhz(CPU_FREQ_MHZ);
  Serial.begin(115200);
  gSerialMutex = xSemaphoreCreateMutex();
  gCmdQueue = xQueueCreate(4, sizeof(Command));
  gEventQueue = xQueueCreate(8, sizeof(FallEvent));

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_LED, LOW);
  noTone(PIN_BUZZER);
#if ENABLE_BATTERY
  analogReadResolution(12);
  pinMode(PIN_BATTERY, INPUT);
#endif

  // Dải đo rộng: va chạm khi ngã thường đạt 3-8 g và 300-600 °/s. Thử lại thay vì kẹt vĩnh viễn.
  bool sensorOk = false;
  for (int i = 0; i < 5 && !sensorOk; i++) {
    sensorOk = initSensor();
    if (!sensorOk) {
      Wire.end();
      i2cBusClear();
      delay(200);
    }
  }
  if (!sensorOk) {
    logLine("MPU6050 khong phan hoi - LED nhay, tu khoi dong lai sau 5 s.");
    for (int i = 0; i < 25; i++) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(200);
    }
    esp_restart();
  }
  logf("Ket noi MPU6050 thanh cong (+-16g, +-2000 deg/s, DLPF 44Hz, I2C %d Hz).", I2C_CLOCK_HZ);

  if (!calibrateSensor()) {
    logLine("Hieu chuan THAT BAI - dung gia tri mac dinh (se on dinh dan nho bo loc trong luc).");
  }

  loadDeviceConfig();
  gForcePortal = (digitalRead(PIN_BUTTON) == LOW);             // giữ nút lúc bật nguồn => mở cổng cấu hình
  if (buttonPressed() || gForcePortal) btnHeldHandled = true;  // không để nút bật nguồn kích SOS

#if TLS_VERIFY
  gSecure.setCACert(FIREBASE_ROOT_CA);
#else
  gSecure.setInsecure();  // giới hạn đã nêu trong báo cáo (mục 3.3)
#endif
  gSecure.setHandshakeTimeout(TLS_HANDSHAKE_TIMEOUT_S);

  // WDT chỉ giám sát loop(): an toàn cục bộ không phụ thuộc mạng.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_deinit();
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  xTaskCreatePinnedToCore(netTask, "netTask", 12288, NULL, 1, NULL, 0);

  logLine("==================================================");
  logf("HE THONG PHAT HIEN TE NGA v%s - SAN SANG (device=%s)", FW_VERSION, gDeviceId);
  logLine("==================================================");
}

// ======================= LOOP (100 Hz, không gọi mạng) =======================
void loop() {
  esp_task_wdt_reset();
  gLoopHeartbeat++;

  static uint32_t nextTick = 0, lastNow = 0;
  uint32_t now = millis();
  if (nextTick == 0) {
    nextTick = now;
    lastNow = now;
  }
  uint32_t dt = now - lastNow;
  if (dt == 0 || dt > 200) dt = SAMPLE_PERIOD_MS;
  lastNow = now;

  updateButton(now);
  processCommands();
  checkNetTaskAlive(now);

  if (readSensor()) {
    // ----- mẫu hợp lệ -----
    if (gFaultSinceMs != 0) {  // vừa phục hồi sau lỗi cảm biến
      gFaultSinceMs = 0;
      if (!localAlarmOn) digitalWrite(PIN_LED, LOW);
      logLine("Cam bien da hoat dong tro lai.");
    }
    gBadSamples = 0;

    float acc = computeAccTotal();
    float gyro = computeGyroTotal();

    updateGravity(acc);
    runStateMachine(now, dt, acc, gyro);
    updateAlarmOutputs(now);

    // Công bố: IDLE -> giá trị tức thời; có sự cố -> giá trị ĐỈNH của sự kiện
    float pubAcc = (currentState == STATE_IDLE) ? acc : peakAcc;
    float pubGyro = (currentState == STATE_IDLE) ? gyro : peakGyro;
    publishSnapshot(pubStateNow(), alarmAcked, pubAcc, pubGyro, computePostureTilt(), false);

#if ENABLE_CSV_STREAM
    Serial.printf("%lu,%.3f,%.1f,%.1f,%u\n", (unsigned long)now, acc, gyro,
                  computePostureTilt(), (unsigned)currentState);
#endif
  } else {
    // ----- mẫu lỗi: bỏ qua, không đưa vào thuật toán -----
    if (gFaultSinceMs == 0) {
      gFaultSinceMs = now;
      logLine("Mau cam bien loi (I2C).");
    }
    if (++gBadSamples >= I2C_FAIL_REINIT_SAMPLES) {
      gBadSamples = 0;
      recoverSensor();
    }

    // Báo động đang chạy vẫn phải huỷ/leo thang được dù cảm biến lỗi.
    if (currentState == STATE_ALARM) runStateMachine(now, dt, 1.0f, 0.0f);
    updateAlarmOutputs(now);
    if (!localAlarmOn) digitalWrite(PIN_LED, ((now / 100) % 2) == 0 ? HIGH : LOW);  // nháy nhanh = lỗi cảm biến

    const uint32_t faultMs = now - gFaultSinceMs;
    if ((currentState == STATE_FREE_FALL || currentState == STATE_CHECK) && faultMs > SENSOR_FAULT_ABORT_MS) {
      returnToIdle("Bo chuoi kiem tra - cam bien loi.");
    }
    if (currentState == STATE_IDLE && faultMs > SENSOR_FAULT_RESTART_MS) {
      logLine("Cam bien hong keo dai - khoi dong lai thiet bi.");
      delay(100);
      esp_restart();
    }
  }

#if ENABLE_BATTERY
  {
    static uint32_t lastChirp = 0, chirpEnd = 0;
    if (chirpEnd != 0 && now >= chirpEnd) {
      if (!localAlarmOn) setTone(0);
      chirpEnd = 0;
    }
    if (!localAlarmOn && currentState == STATE_IDLE && gBatteryPct >= 0 && gBatteryPct < BATT_LOW_PCT && (lastChirp == 0 || now - lastChirp > BATT_CHIRP_MS)) {
      lastChirp = now;
      chirpEnd = now + 120;
      setTone(1500);
    }
  }
#endif

  nextTick += SAMPLE_PERIOD_MS;
  int32_t waitMs = (int32_t)(nextTick - millis());
  if (waitMs > 0) delay((uint32_t)waitMs);
  else nextTick = millis();
}
