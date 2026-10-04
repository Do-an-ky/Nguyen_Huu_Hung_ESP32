// ======================= CẤU HÌNH HỆ THỐNG =======================
#define FW_VERSION       "3.0.0"
#define FIREBASE_HOST    "https://doanky-972e0-default-rtdb.firebaseio.com"
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

#define CPU_FREQ_MHZ     160       // 80 tiết kiệm pin hơn nhưng TLS chậm hơn; 160 là mức an toàn

// ======================= CHÂN KẾT NỐI =======================
#define PIN_LED      5
#define PIN_BUZZER   18
#define PIN_BUTTON   4
#define PIN_SDA      21
#define PIN_SCL      22

// ======================= ĐO PIN (TÙY CHỌN) =======================
// Chia áp 2 điện trở bằng nhau từ Vbat vào chân ADC1 (GPIO32..39). KHÔNG dùng ADC2 (xung đột Wi-Fi).
#define ENABLE_BATTERY   0
#define PIN_BATTERY      34
#define BATT_DIVIDER     2.0f      // Vbat = Vadc * BATT_DIVIDER
#define BATT_EMPTY_MV    3300
#define BATT_FULL_MV     4150
#define BATT_LOW_PCT     15        // dưới mức này: bíp nhắc mỗi 60 s khi IDLE
#define BATT_CHIRP_MS    60000

// ======================= CẤU HÌNH CẢM BIẾN =======================
// MPU6050_light: acc_config 3 = ±16 g, gyro_config 3 = ±2000 °/s
#define MPU_ACC_CONFIG    3
#define MPU_GYRO_CONFIG   3
#define MPU_DLPF_CFG      0x03     // thanh ghi CONFIG(0x1A) = 44 Hz
#define MPU_REG_CONFIG    0x1A
#define I2C_CLOCK_HZ      100000   // nâng lên 400000 chỉ khi đã kiểm chứng ổn định trên mạch thật

// ======================= NGƯỠNG THUẬT TOÁN (4 GIAI ĐOẠN) =======================
#ifndef ENABLE_ROTATION_CHECK
#define ENABLE_ROTATION_CHECK       1       // giai đoạn 2b: bắt buộc có xoay quanh va chạm
#endif
#ifndef ENABLE_ORIENT_CHECK
#define ENABLE_ORIENT_CHECK         1       // giai đoạn 4 : bắt buộc đổi tư thế
#endif

#define SAMPLE_PERIOD_MS            10      // 100 Hz

// --- Giai đoạn 1: rơi tự do ---
#define FREEFALL_THRESHOLD          0.60f   // g
#define MIN_FREEFALL_DURATION        100    // ms : TỔNG thời gian mất trọng lượng tối thiểu
#define TIME_IMPACT_MAX             1000    // ms : hạn chờ va chạm kể từ lúc bắt đầu rơi
#define IMPACT_GAP_MAX_MS            500    // ms : va chạm phải đến sát sau mẫu mất trọng lượng cuối

// --- Giai đoạn 2: va chạm + xoay ---
#define IMPACT_THRESHOLD            1.50f   // g
#define IMPACT_HARD_THRESHOLD       2.60f   // g  : va chạm mạnh, vào thẳng pha kiểm tra
#define GYRO_THRESHOLD              50.0f   // °/s: đường rơi tự do
#define GYRO_THRESHOLD_HARD        100.0f   // °/s: đường va chạm mạnh
#define PRE_IMPACT_BUF_SAMPLES        30    // mẫu: nhìn lại 300 ms trước va chạm mạnh

// --- Giai đoạn 3: nằm yên ---
#define MIN_REST_ACC                0.70f   // g  : dải LỌC TRỌNG LỰC
#define MAX_REST_ACC                1.30f
#define POST_REST_MIN_ACC           0.65f   // g  : dải "nằm yên" sau va chạm
#define POST_REST_MAX_ACC           1.35f
#define REST_GYRO_THRESHOLD         40.0f   // °/s
#define SETTLE_MS                    500    // ms : bỏ qua pha nảy/lăn sau va chạm
#define REST_REQUIRED_MS            2000    // ms : đường rơi tự do
#define REST_REQUIRED_MS_HARD       2500    // ms : đường va chạm mạnh
#define MOTION_PENALTY                 3    // mỗi ms chuyển động trừ 3 ms nằm yên
#define MOTION_CANCEL_MS             600    // ms : chuyển động LIÊN TỤC => huỷ chuỗi
#define CHECK_MAX_MS                8000    // ms : hạn tối đa của pha kiểm tra

// --- Giai đoạn 4: đổi tư thế ---
#define ORIENT_CHANGE_THRESHOLD     45.0f   // độ
#define ORIENT_CHANGE_THRESHOLD_HARD 55.0f  // độ
#define POSTURE_STABLE_DEG          15.0f   // độ : hướng trọng lực đã lọc phải bám sát mẫu hiện tại

#define GRAV_ALPHA_SLOW            0.01f
#define GRAV_ALPHA_FAST            0.10f

#define CALIBRATION_SAMPLES          100
#define CALIB_MAX_ATTEMPTS             3
#define CALIB_MOVE_GYRO_LIMIT      20.0f

// --- Pre-alarm / báo động tại chỗ ---
#define PRE_ALARM_MS               10000    // ms : cửa sổ huỷ cục bộ trước khi gửi lên app (0 = gửi ngay)
#define ALARM_BUZZER_MAX_MS       300000UL  // còi tự tắt sau 5 phút (đèn vẫn nháy)

// --- Nút nhấn ---
#define BTN_DEBOUNCE_SAMPLES           3    // 3 mẫu (30 ms) mới tính là nhấn
#define BTN_SOS_HOLD_MS             2000    // giữ nút 2 s ở IDLE => SOS
#define BTN_CANCEL_HOLD_MS          1500    // giữ nút 1,5 s mới huỷ (chống ngã đè lên nút)

// --- Giám sát cảm biến ---
#define I2C_FAIL_REINIT_SAMPLES       50    // 50 mẫu lỗi liên tiếp (~0,5 s) => khởi tạo lại MPU
#define SENSOR_FAULT_ABORT_MS       2000    // lỗi > 2 s khi đang kiểm tra => bỏ chuỗi, về IDLE
#define SENSOR_FAULT_RESTART_MS    30000    // lỗi > 30 s khi IDLE => khởi động lại

#define ENABLE_DATA_LOG   true
#define ENABLE_CSV_STREAM false             // true: in CSV mỗi mẫu để phân tích

// ======================= CẤU HÌNH MẠNG =======================
#define WDT_TIMEOUT_SECONDS       20        // chỉ giám sát loop()
#define LOOP_STALL_LIMIT_MS     2000        // vòng lấy mẫu treo quá mức này => khởi động lại
#define NET_STALL_RESTART_MS  180000UL      // netTask treo quá mức này VÀ đang IDLE => khởi động lại
#define TLS_HANDSHAKE_TIMEOUT_S     10
#define WIFI_RECONNECT_INTERVAL 5000
#define WIFI_CONNECT_TIMEOUT_S     10       // chờ kết nối Wi-Fi lúc khởi động (chỉ chặn netTask)
#define PORTAL_TIMEOUT_S          180
#define HTTP_CONNECT_TIMEOUT_MS 5000
#define HTTP_TIMEOUT_MS         6000
#define HEARTBEAT_MS           10000
#define RETRY_BASE_MS           2000        // lùi theo cấp số nhân: 2s,4s,8s... tối đa 30s
#define RETRY_MAX_MS           30000
#define EVENT_MAX_4XX_TRIES        5        // lỗi 4xx "vĩnh viễn" thử tối đa chừng này lần rồi bỏ
#define CMD_POLL_ALARM_MS       2000
#define CMD_POLL_IDLE_MS       15000
#define BATT_UPDATE_MS         30000
static const unsigned long TOKEN_LIFETIME_MS = 3500000UL;

// ======================= KIỂU DỮ LIỆU =======================
enum FallState : uint8_t { STATE_IDLE = 0, STATE_FREE_FALL, STATE_CHECK, STATE_ALARM };
enum PubState  : uint8_t { PUB_IDLE = 0, PUB_CHECKING = 1, PUB_ALARM = 2 };
enum TriggerType : uint8_t { TRIG_FREEFALL = 0, TRIG_HARD_IMPACT = 1, TRIG_SOS = 2, TRIG_TEST = 3 };
enum Command : uint8_t { CMD_NONE = 0, CMD_ACK, CMD_RESET, CMD_TEST };

struct StatusSnapshot {          // loop() ghi, netTask đọc
  uint8_t  pubState;
  bool     acked;
  float    acc;
  float    gyro;
  float    tilt;
  uint32_t seq;
};

struct FallEvent {               // loop() -> netTask
  uint8_t  trigger;
  float    peakAcc;
  float    peakGyro;
  float    tiltChange;
  uint16_t freefallMs;
  uint16_t impactToRestMs;
  uint32_t atMs;                 // millis() lúc xảy ra, để gửi đúng timestamp dù gửi trễ
};

// ======================= BIẾN TOÀN CỤC =======================
MPU6050  mpu(Wire);
FallState currentState = STATE_IDLE;

static portMUX_TYPE  gMux = portMUX_INITIALIZER_UNLOCKED;
static StatusSnapshot gSnapshot = {PUB_IDLE, false, 1.0f, 0.0f, 0.0f, 0};
static QueueHandle_t  gCmdQueue   = nullptr;   // netTask -> loop
static QueueHandle_t  gEventQueue = nullptr;   // loop -> netTask
static SemaphoreHandle_t gSerialMutex = nullptr;
static volatile uint32_t gLoopHeartbeat = 0;   // loop tăng mỗi chu kỳ
static volatile uint32_t gNetHeartbeat  = 0;   // netTask tăng mỗi vòng
static volatile int8_t   gBatteryPct    = -1;  // -1 = không rõ

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
float    peakAcc = 0, peakGyro = 0;
bool     hardImpactPath = false;
uint32_t lastLowTime = 0, ffLowMs = 0;
float    rotPeakGyro = 0;
bool     rotationVerified = false;

// điều khiển cục bộ
bool     localAlarmOn = false, buzzerOn = false;
bool     alarmAcked = false;
bool     alarmEscalated = false;               // false = đang pre-alarm (chỉ báo tại chỗ)
uint8_t  alarmTrigger = TRIG_FREEFALL;
uint32_t alarmStartMs = 0, escalateMs = 0;
static FallEvent gAlarmEvent;                  // sự kiện chờ gửi khi leo thang
static uint16_t  gToneNow = 0;

// nút nhấn
uint8_t  btnLowCount = 0;
bool     btnHeldHandled = false;
bool     btnReleasedInAlarm = false;
uint32_t btnPressStart = 0;

// giám sát cảm biến
static uint16_t gBadSamples   = 0;
static uint32_t gFaultSinceMs = 0;

// xác thực Firebase (chỉ netTask dùng sau setup)
String   idToken = "", refreshToken = "";
unsigned long tokenObtainedTime = 0;

// Wi-Fi (chỉ netTask dùng sau setup)
static WiFiManager*          gWm       = nullptr;
static WiFiManagerParameter* gDevParam = nullptr;
static volatile bool         gDevIdDirty = false;
static bool                  gForcePortal = false;

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

float vecNorm(float x, float y, float z) { return sqrtf(x * x + y * y + z * z); }

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
  gSnapshot.acked    = acked;
  gSnapshot.acc      = acc;
  gSnapshot.gyro     = gyro;
  gSnapshot.tilt     = tilt;
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
    case STATE_IDLE:  return PUB_IDLE;
    case STATE_ALARM: return alarmEscalated ? PUB_ALARM : PUB_CHECKING;
    default:          return PUB_CHECKING;
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
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(10);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
  }
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(10);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(10);
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
  mpuWriteRegister(MPU_REG_CONFIG, MPU_DLPF_CFG);   // DLPF 44 Hz - chống chồng phổ
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
  return fabsf(x) < 0.001f && fabsf(y) < 0.001f && fabsf(z) < 0.001f &&
         fabsf(x - y) < 0.00002f && fabsf(y - z) < 0.00002f;
}

// true = mẫu hợp lệ và đã cập nhật curAx/Ay/Az.
bool readSensor() {
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() != 0) return false;     // MPU không phản hồi
  mpu.update();
  float x = mpu.getAccX(), y = mpu.getAccY(), z = mpu.getAccZ();
  if (sampleIsGarbage(x, y, z)) return false;
  curAx = x; curAy = y; curAz = z;
  return true;
}

float computeAccTotal() { return vecNorm(curAx, curAy, curAz) + accOffset; }

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
        sax += curAx; say += curAy; saz += curAz;
        sgx += gx; sgy += gy; sgz += gz;
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

    gyroBiasX = sgx / n; gyroBiasY = sgy / n; gyroBiasZ = sgz / n;
    accOffset = 1.0f - (sumMag / n);
    if (fabsf(accOffset) > 0.3f) {
      logLine("Offset gia toc bat thuong - bo qua bu offset.");
      accOffset = 0;
    }
    gAvgX = sax / n; gAvgY = say / n; gAvgZ = saz / n;
    gRefX = gAvgX;   gRefY = gAvgY;   gRefZ = gAvgZ;
    logf("Hieu chuan xong. accOffset=%.4f | gyroBias=%.2f,%.2f,%.2f",
         accOffset, gyroBiasX, gyroBiasY, gyroBiasZ);
    return true;
  }
  return false;
}

// ======================= CẢNH BÁO TẠI CHỖ =======================
static void setTone(uint16_t f) {                  // chỉ cấu hình lại khi đổi tần số
  if (f == gToneNow) return;
  gToneNow = f;
  if (f) tone(PIN_BUZZER, f);
  else   noTone(PIN_BUZZER);
}

void startLocalAlarm() { localAlarmOn = true; buzzerOn = true; }

void muteBuzzer() { buzzerOn = false; setTone(0); }

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
  else            digitalWrite(PIN_LED, HIGH);
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

bool buttonPressed() { return btnLowCount >= BTN_DEBOUNCE_SAMPLES; }

bool buttonHeldFor(uint32_t now, uint32_t ms) {
  return buttonPressed() && (now - btnPressStart >= ms);
}

bool buttonHeldForSos(uint32_t now) {
  if (!buttonPressed() || btnHeldHandled) return false;
  if (now - btnPressStart >= BTN_SOS_HOLD_MS) { btnHeldHandled = true; return true; }
  return false;
}

// ======================= CHUYỂN TRẠNG THÁI =======================
void setState(FallState s) {
  currentState = s;
  StatusSnapshot cur = readSnapshot();
  publishSnapshot(pubStateNow(), alarmAcked, cur.acc, cur.gyro, cur.tilt, true);
}

void returnToIdle(const char* why) {
  if (buttonPressed()) btnHeldHandled = true;        // nút còn giữ: không kích SOS oan
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
  if (gEventQueue) xQueueSend(gEventQueue, &gAlarmEvent, 0);   // không chặn vòng lấy mẫu
  logLine("[ALARM] Leo thang len ung dung.");
}

void enterAlarm(uint8_t trigger, float tiltChange, uint16_t impactToRestMs) {
  const bool immediate = (trigger == TRIG_SOS || trigger == TRIG_TEST || PRE_ALARM_MS == 0);

  alarmAcked         = false;
  alarmEscalated     = false;
  btnReleasedInAlarm = !buttonPressed();             // đang giữ nút (SOS) => chờ nhả
  alarmTrigger       = trigger;
  alarmStartMs       = millis();
  escalateMs         = alarmStartMs;

  gAlarmEvent.trigger        = trigger;
  gAlarmEvent.peakAcc        = peakAcc;
  gAlarmEvent.peakGyro       = peakGyro;
  gAlarmEvent.tiltChange     = tiltChange;
  gAlarmEvent.freefallMs     = freefallDurationMs;
  gAlarmEvent.impactToRestMs = impactToRestMs;
  gAlarmEvent.atMs           = alarmStartMs;

  setState(STATE_ALARM);
  startLocalAlarm();                                 // cảnh báo tại chỗ TRƯỚC, không chờ mạng
  logf("[ALARM] trigger=%u peakAcc=%.2fg peakGyro=%.1f tilt=%.1f (%s)",
       trigger, peakAcc, peakGyro, tiltChange, immediate ? "gui ngay" : "pre-alarm");

  if (immediate) escalateAlarm();
}

// ======================= MÁY TRẠNG THÁI 4 GIAI ĐOẠN =======================
static float   gGyroHist[PRE_IMPACT_BUF_SAMPLES];
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
  impactTime       = now;
  restAccumMs      = 0;
  motionRunMs      = 0;
  hardImpactPath   = hardPath;
  rotationVerified = false;
  setState(STATE_CHECK);
  logf("[Giai doan 2] Impact! duong=%s", hardPath ? "va cham manh" : "sau roi tu do");
}

void runStateMachine(uint32_t now, uint32_t dt, float acc, float gyro) {
  pushGyroHistory(gyro);

  switch (currentState) {

    // ---------- GIAI ĐOẠN 1: chờ rơi tự do / va chạm mạnh ----------
    case STATE_IDLE: {
      if (buttonHeldForSos(now)) {                 // SOS thủ công
        peakAcc = acc; peakGyro = gyro; freefallDurationMs = 0;
        gSnapX = gAvgX; gSnapY = gAvgY; gSnapZ = gAvgZ;
        enterAlarm(TRIG_SOS, 0.0f, 0);
        break;
      }
      if (acc < FREEFALL_THRESHOLD) {              // đường 1: rơi tự do
        freeFallTime = now;
        lastLowTime  = now;
        ffLowMs      = dt;
        gSnapX = gAvgX; gSnapY = gAvgY; gSnapZ = gAvgZ;   // tư thế TRƯỚC khi ngã
        peakAcc = acc; peakGyro = gyro; rotPeakGyro = gyro;
        setState(STATE_FREE_FALL);
        logLine("[Giai doan 1] Free-fall!");
      } else if (acc > IMPACT_HARD_THRESHOLD) {    // đường 2: va chạm mạnh trực tiếp
        gSnapX = gAvgX; gSnapY = gAvgY; gSnapZ = gAvgZ;
        freefallDurationMs = 0;
        rotPeakGyro = fmaxf(maxGyroHistory(), gyro);
        peakAcc = acc; peakGyro = rotPeakGyro;
        enterCheckPhase(now, true);
      }
      break;
    }

    // ---------- GIAI ĐOẠN 1 -> 2: đang rơi, chờ va chạm ----------
    case STATE_FREE_FALL: {
      peakAcc     = fmaxf(peakAcc, acc);
      peakGyro    = fmaxf(peakGyro, gyro);
      rotPeakGyro = fmaxf(rotPeakGyro, gyro);

      if (buttonHeldFor(now, BTN_CANCEL_HOLD_MS)) { returnToIdle("Nguoi dung huy bang nut nhan."); break; }

      if (acc < FREEFALL_THRESHOLD) {
        ffLowMs += dt;
        lastLowTime = now;
      }

      if (acc > IMPACT_THRESHOLD) {
        const uint32_t span = lastLowTime - freeFallTime;
        const uint32_t gap  = now - lastLowTime;
        freefallDurationMs = (uint16_t)min<uint32_t>(ffLowMs, 65535);
#if ENABLE_DATA_LOG
        logf("[DATA] freefall=%lums (span=%lums gap=%lums) impactAcc=%.2fg peakGyro=%.1fdeg/s",
             (unsigned long)ffLowMs, (unsigned long)span, (unsigned long)gap, acc, peakGyro);
#else
        (void)span;
#endif
        if (ffLowMs < MIN_FREEFALL_DURATION)    returnToIdle("Bo qua - roi tu do qua ngan.");
        else if (gap > IMPACT_GAP_MAX_MS)       returnToIdle("Bo qua - va cham khong lien ngay sau roi.");
        else                                    enterCheckPhase(now, false);
      } else if (now - freeFallTime > TIME_IMPACT_MAX) {
        returnToIdle("Bo qua - het thoi gian cho va cham.");
      }
      break;
    }

    // ---------- GIAI ĐOẠN 2b + 3 + 4: xoay -> nằm yên -> đổi tư thế ----------
    case STATE_CHECK: {
      peakAcc  = fmaxf(peakAcc, acc);
      peakGyro = fmaxf(peakGyro, gyro);

      if (buttonHeldFor(now, BTN_CANCEL_HOLD_MS)) { returnToIdle("Nguoi dung huy bang nut nhan."); break; }

      uint32_t elapsed = now - impactTime;

      if (elapsed < SETTLE_MS) {                   // pha nảy/lăn: chỉ ghi nhận xoay
        rotPeakGyro = fmaxf(rotPeakGyro, gyro);
        break;
      }
      if (elapsed > CHECK_MAX_MS) { returnToIdle("Bo qua - het thoi gian kiem tra."); break; }

#if ENABLE_ROTATION_CHECK
      if (!rotationVerified) {
        rotationVerified = true;
        const float gyroNeed = hardImpactPath ? GYRO_THRESHOLD_HARD : GYRO_THRESHOLD;
        logf("[Giai doan 2] Xoay: dinh=%.1f can>=%.1f deg/s", rotPeakGyro, gyroNeed);
        if (rotPeakGyro < gyroNeed) { returnToIdle("Bo qua - khong xoay du."); break; }
      }
#endif

      const bool resting = (acc >= POST_REST_MIN_ACC && acc <= POST_REST_MAX_ACC &&
                            gyro < REST_GYRO_THRESHOLD);
      if (resting) {
        restAccumMs += dt;
        motionRunMs  = 0;
      } else {
        const uint32_t penalty = dt * MOTION_PENALTY;
        restAccumMs  = (restAccumMs > penalty) ? (restAccumMs - penalty) : 0;
        motionRunMs += dt;
        if (motionRunMs >= MOTION_CANCEL_MS) { returnToIdle("Bo qua - nguoi dung van dang chuyen dong."); break; }
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
        if (tiltChange < orientNeed) { returnToIdle("Bo qua - tu the khong thay doi."); break; }
#endif
        enterAlarm(hardImpactPath ? TRIG_HARD_IMPACT : TRIG_FREEFALL,
                   tiltChange, (uint16_t)min<uint32_t>(elapsed, 65535));
      }
      break;
    }

    // ---------- BÁO ĐỘNG (pre-alarm -> leo thang) ----------
    // Nút phải được NHẢ ra ít nhất một lần rồi mới GIỮ lại để huỷ; nếu không, SOS (giữ nút 2 s)
    // sẽ tự huỷ ngay vì nút vẫn đang bị giữ.
    case STATE_ALARM: {
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
          peakAcc = 0; peakGyro = 0; freefallDurationMs = 0;
          enterAlarm(TRIG_TEST, 0.0f, 0);
        }
        break;
      default: break;
    }
  }
}


// ======================= GIÁM SÁT TÁC VỤ MẠNG (gọi từ loop) =======================
static void checkNetTaskAlive(uint32_t now) {
  static uint32_t lastBeat = 0, lastChange = 0;
  if (lastChange == 0) lastChange = now;
  if (gNetHeartbeat != lastBeat) { lastBeat = gNetHeartbeat; lastChange = now; }
  else if (currentState == STATE_IDLE && (now - lastChange) > NET_STALL_RESTART_MS) {
    logLine("netTask bi treo qua lau (dang IDLE) - khoi dong lai thiet bi.");
    delay(100);
    esp_restart();
  }
}


// ======================= LOOP (100 Hz, không gọi mạng) =======================
void loop() {
  esp_task_wdt_reset();
  gLoopHeartbeat++;

  static uint32_t nextTick = 0, lastNow = 0;
  uint32_t now = millis();
  if (nextTick == 0) { nextTick = now; lastNow = now; }
  uint32_t dt = now - lastNow;
  if (dt == 0 || dt > 200) dt = SAMPLE_PERIOD_MS;
  lastNow = now;

  updateButton(now);
  processCommands();
  checkNetTaskAlive(now);

  if (readSensor()) {
    // ----- mẫu hợp lệ -----
    if (gFaultSinceMs != 0) {                        // vừa phục hồi sau lỗi cảm biến
      gFaultSinceMs = 0;
      if (!localAlarmOn) digitalWrite(PIN_LED, LOW);
      logLine("Cam bien da hoat dong tro lai.");
    }
    gBadSamples = 0;

    float acc  = computeAccTotal();
    float gyro = computeGyroTotal();

    updateGravity(acc);
    runStateMachine(now, dt, acc, gyro);
    updateAlarmOutputs(now);

    // Công bố: IDLE -> giá trị tức thời; có sự cố -> giá trị ĐỈNH của sự kiện
    float pubAcc  = (currentState == STATE_IDLE) ? acc  : peakAcc;
    float pubGyro = (currentState == STATE_IDLE) ? gyro : peakGyro;
    publishSnapshot(pubStateNow(), alarmAcked, pubAcc, pubGyro, computePostureTilt(), false);

#if ENABLE_CSV_STREAM
    Serial.printf("%lu,%.3f,%.1f,%.1f,%u\n", (unsigned long)now, acc, gyro,
                  computePostureTilt(), (unsigned)currentState);
#endif
  } else {
    // ----- mẫu lỗi: bỏ qua, không đưa vào thuật toán -----
    if (gFaultSinceMs == 0) { gFaultSinceMs = now; logLine("Mau cam bien loi (I2C)."); }
    if (++gBadSamples >= I2C_FAIL_REINIT_SAMPLES) { gBadSamples = 0; recoverSensor(); }

    // Báo động đang chạy vẫn phải huỷ/leo thang được dù cảm biến lỗi.
    if (currentState == STATE_ALARM) runStateMachine(now, dt, 1.0f, 0.0f);
    updateAlarmOutputs(now);
    if (!localAlarmOn) digitalWrite(PIN_LED, ((now / 100) % 2) == 0 ? HIGH : LOW);   // nháy nhanh = lỗi cảm biến

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
    if (chirpEnd != 0 && now >= chirpEnd) { if (!localAlarmOn) setTone(0); chirpEnd = 0; }
    if (!localAlarmOn && currentState == STATE_IDLE && gBatteryPct >= 0 && gBatteryPct < BATT_LOW_PCT &&
        (lastChirp == 0 || now - lastChirp > BATT_CHIRP_MS)) {
      lastChirp = now;
      chirpEnd  = now + 120;
      setTone(1500);
    }
  }
#endif

  nextTick += SAMPLE_PERIOD_MS;
  int32_t waitMs = (int32_t)(nextTick - millis());
  if (waitMs > 0) delay((uint32_t)waitMs);
  else nextTick = millis();
}
