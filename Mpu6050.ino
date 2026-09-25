

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
#define FW_VERSION       "2.0.0"
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

// ======================= CHÂN KẾT NỐI =======================
#define PIN_LED      5
#define PIN_BUZZER   18
#define PIN_BUTTON   4

// ======================= CẤU HÌNH CẢM BIẾN =======================
// MPU6050_light: acc_config 3 = ±16 g, gyro_config 3 = ±2000 °/s
#define MPU_ACC_CONFIG    3
#define MPU_GYRO_CONFIG   3
#define MPU_DLPF_CFG      0x03     // thanh ghi CONFIG(0x1A) = 44 Hz
#define MPU_REG_CONFIG    0x1A
#define I2C_CLOCK_HZ      400000

// ======================= NGƯỠNG THUẬT TOÁN =======================
#define SAMPLE_PERIOD_MS            10      // 100 Hz
#define FREEFALL_THRESHOLD          0.60f   // g  : dưới ngưỡng => rơi tự do
#define IMPACT_THRESHOLD            1.50f   // g  : va chạm sau pha rơi tự do
#define IMPACT_HARD_THRESHOLD       2.60f   // g  : va chạm mạnh, vào thẳng pha kiểm tra
#define MIN_REST_ACC                0.70f   // g  : vùng "nằm yên"
#define MAX_REST_ACC                1.30f
#define REST_GYRO_THRESHOLD         30.0f   // °/s: nằm yên thì xoay phải nhỏ hơn mức này
#define GYRO_THRESHOLD              50.0f   // °/s: xoay tối thiểu (đường rơi tự do)
#define GYRO_THRESHOLD_HARD        100.0f   // °/s: xoay tối thiểu (đường va chạm mạnh)
#define ORIENT_CHANGE_THRESHOLD     45.0f   // độ : đổi tư thế (đường rơi tự do)
#define ORIENT_CHANGE_THRESHOLD_HARD 55.0f  // độ : đổi tư thế (đường va chạm mạnh)

#define TIME_IMPACT_MAX             1000    // ms : hạn chờ va chạm sau khi rơi
#define MIN_FREEFALL_DURATION        100    // ms : thời gian rơi tối thiểu
#define SETTLE_MS                    500    // ms : bỏ qua pha nảy/lăn sau va chạm
#define REST_REQUIRED_MS            2000    // ms : nằm yên liên tục (đường rơi tự do)
#define REST_REQUIRED_MS_HARD       2500    // ms : nằm yên liên tục (đường va chạm mạnh)
#define MOTION_CANCEL_MS             300    // ms : chuyển động liên tục => huỷ chuỗi
#define CHECK_MAX_MS                8000    // ms : hạn tối đa của pha kiểm tra

#define GRAV_ALPHA_SLOW            0.01f    // lọc hướng trọng lực khi IDLE
#define GRAV_ALPHA_FAST            0.10f    // lọc nhanh khi đang kiểm tra

#define CALIBRATION_SAMPLES          100
#define CALIB_MAX_ATTEMPTS             3
#define CALIB_MOVE_GYRO_LIMIT      20.0f    // °/s: coi là đang bị rung khi hiệu chuẩn

#define BTN_DEBOUNCE_SAMPLES           3    // 3 mẫu (30 ms) mới tính là nhấn
#define BTN_SOS_HOLD_MS             2000    // giữ nút 2 s ở IDLE => SOS

#define ENABLE_DATA_LOG   true
#define ENABLE_CSV_STREAM false             // true: in CSV mỗi mẫu để phân tích

// ======================= CẤU HÌNH MẠNG =======================
#define WDT_TIMEOUT_SECONDS       20
#define LOOP_STALL_LIMIT_MS     2000        // vòng lấy mẫu treo quá mức này => khởi động lại
#define WIFI_RECONNECT_INTERVAL 5000
#define HTTP_CONNECT_TIMEOUT_MS 5000
#define HTTP_TIMEOUT_MS         6000
#define HEARTBEAT_MS           10000
#define RETRY_BASE_MS           2000        // lùi theo cấp số nhân: 2s,4s,8s... tối đa 30s
#define RETRY_MAX_MS           30000
#define CMD_POLL_ALARM_MS       2000        // chu kỳ đọc lệnh khi đang cảnh báo
#define CMD_POLL_IDLE_MS       15000        // chu kỳ đọc lệnh khi bình thường (nhận lệnh TEST)
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
  uint32_t seq;                  // tăng mỗi lần đổi trạng thái/acked
};

struct FallEvent {               // loop() -> netTask
  uint8_t  trigger;
  float    peakAcc;
  float    peakGyro;
  float    tiltChange;
  uint16_t freefallMs;
  uint16_t impactToRestMs;
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

// hiệu chuẩn
float accOffset = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float gRefX = 0, gRefY = 0, gRefZ = 1;         // hướng trọng lực lúc hiệu chuẩn (tư thế chuẩn)

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

// điều khiển cục bộ
bool    localAlarmOn = false, buzzerOn = false;
bool    alarmAcked = false;
uint8_t alarmTrigger = TRIG_FREEFALL;
uint32_t alarmStartMs = 0;

// nút nhấn
uint8_t  btnLowCount = 0;
bool     btnHeldHandled = false;
uint32_t btnPressStart = 0;

// xác thực Firebase (chỉ netTask dùng sau setup)
String   idToken = "", refreshToken = "";
unsigned long tokenObtainedTime = 0;

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

// ======================= CẢM BIẾN =======================
void mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readSensor() {
  mpu.update();
  curAx = mpu.getAccX();
  curAy = mpu.getAccY();
  curAz = mpu.getAccZ();
}

float computeAccTotal() { return vecNorm(curAx, curAy, curAz) + accOffset; }

float computeGyroTotal() {
  float gx = mpu.getGyroX() - gyroBiasX;
  float gy = mpu.getGyroY() - gyroBiasY;
  float gz = mpu.getGyroZ() - gyroBiasZ;
  return vecNorm(gx, gy, gz);
}

// Góc lệch tư thế so với tư thế lúc hiệu chuẩn (dùng để hiển thị trên app)
float computePostureTilt() {
  return vecAngleDeg(gAvgX, gAvgY, gAvgZ, gRefX, gRefY, gRefZ);
}

void updateGravity(float acc) {
  if (acc < MIN_REST_ACC || acc > MAX_REST_ACC) return;   // loại mẫu có gia tốc chuyển động
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

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      mpu.update();
      float ax = mpu.getAccX(),  ay = mpu.getAccY(),  az = mpu.getAccZ();
      float gx = mpu.getGyroX(), gy = mpu.getGyroY(), gz = mpu.getGyroZ();
      sumMag += vecNorm(ax, ay, az);
      sax += ax; say += ay; saz += az;
      sgx += gx; sgy += gy; sgz += gz;
      sumGyroMag += vecNorm(gx, gy, gz);
      delay(20);
    }

    float n = (float)CALIBRATION_SAMPLES;
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
    gRefX = gAvgX;   gRefY = gAvgY;   gRefZ = gAvgZ;   // tư thế chuẩn ban đầu
    logf("Hieu chuan xong. accOffset=%.4f | gyroBias=%.2f,%.2f,%.2f",
         accOffset, gyroBiasX, gyroBiasY, gyroBiasZ);
    return true;
  }
  return false;
}

// ======================= CẢNH BÁO TẠI CHỖ =======================
void startLocalAlarm() { digitalWrite(PIN_LED, HIGH); localAlarmOn = true; buzzerOn = true; }

void muteBuzzer() {
  if (buzzerOn) { noTone(PIN_BUZZER); buzzerOn = false; }
}

void stopLocalAlarm() {
  if (localAlarmOn) { digitalWrite(PIN_LED, LOW); localAlarmOn = false; }
  muteBuzzer();
  noTone(PIN_BUZZER);
  digitalWrite(PIN_LED, LOW);
}

// Còi kêu theo nhịp 300 ms bật / 200 ms tắt: nghe rõ hơn, đỡ tốn pin.
void updateAlarmOutputs(uint32_t now) {
  if (!localAlarmOn) return;
  if (buzzerOn) {
    uint32_t phase = (now - alarmStartMs) % 500;
    if (phase < 300) tone(PIN_BUZZER, 3000);
    else             noTone(PIN_BUZZER);
  }
  // Đã tiếp nhận: LED nháy 1 Hz để người xung quanh biết cảnh báo còn hiệu lực.
  if (alarmAcked) digitalWrite(PIN_LED, ((now / 500) % 2) == 0 ? HIGH : LOW);
  else            digitalWrite(PIN_LED, HIGH);
}

void setState(FallState s) {
  currentState = s;
  uint8_t pub = (s == STATE_IDLE) ? PUB_IDLE : ((s == STATE_ALARM) ? PUB_ALARM : PUB_CHECKING);
  StatusSnapshot cur = readSnapshot();
  publishSnapshot(pub, alarmAcked, cur.acc, cur.gyro, cur.tilt, true);
}

void returnToIdle(const char* why) {
  stopLocalAlarm();
  alarmAcked = false;
  hardImpactPath = false;
  restAccumMs = motionRunMs = 0;
  setState(STATE_IDLE);
  logLine(why);
}

void enterAlarm(uint8_t trigger, float tiltChange, uint16_t impactToRestMs) {
  alarmAcked = false;
  alarmTrigger = trigger;
  alarmStartMs = millis();
  setState(STATE_ALARM);
  startLocalAlarm();                     // cảnh báo tại chỗ TRƯỚC, không chờ mạng
  logf("[ALARM] trigger=%u peakAcc=%.2fg peakGyro=%.1f tilt=%.1f",
       trigger, peakAcc, peakGyro, tiltChange);

  FallEvent ev;
  ev.trigger        = trigger;
  ev.peakAcc        = peakAcc;
  ev.peakGyro       = peakGyro;
  ev.tiltChange     = tiltChange;
  ev.freefallMs     = freefallDurationMs;
  ev.impactToRestMs = impactToRestMs;
  if (gEventQueue) xQueueSend(gEventQueue, &ev, 0);   // không chặn vòng lấy mẫu
}

// ======================= NÚT NHẤN =======================
// Nhấn ngắn: huỷ cảnh báo / huỷ chuỗi xác nhận.
// Giữ >= 2 s khi đang an toàn: kích hoạt SOS thủ công.
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

bool buttonHeldForSos(uint32_t now) {
  if (!buttonPressed() || btnHeldHandled) return false;
  if (now - btnPressStart >= BTN_SOS_HOLD_MS) { btnHeldHandled = true; return true; }
  return false;
}

// ======================= MÁY TRẠNG THÁI =======================
void enterCheckPhase(uint32_t now, bool hardPath) {
  impactTime     = now;
  restAccumMs    = 0;
  motionRunMs    = 0;
  hardImpactPath = hardPath;
  setState(STATE_CHECK);
  logf("[Giai doan 2] Impact! duong=%s", hardPath ? "va cham manh" : "sau roi tu do");
}

void runStateMachine(uint32_t now, uint32_t dt, float acc, float gyro) {
  switch (currentState) {

    case STATE_IDLE: {
      if (buttonHeldForSos(now)) {                 // SOS thủ công
        peakAcc = acc; peakGyro = gyro; freefallDurationMs = 0;
        gSnapX = gAvgX; gSnapY = gAvgY; gSnapZ = gAvgZ;
        enterAlarm(TRIG_SOS, 0.0f, 0);
        break;
      }
      if (acc < FREEFALL_THRESHOLD) {              // đường 1: rơi tự do
        freeFallTime = now;
        gSnapX = gAvgX; gSnapY = gAvgY; gSnapZ = gAvgZ;
        peakAcc = acc; peakGyro = gyro;
        setState(STATE_FREE_FALL);
        logLine("[Giai doan 1] Free-fall!");
      } else if (acc > IMPACT_HARD_THRESHOLD) {    // đường 2: va chạm mạnh trực tiếp
        gSnapX = gAvgX; gSnapY = gAvgY; gSnapZ = gAvgZ;
        peakAcc = acc; peakGyro = gyro; freefallDurationMs = 0;
        enterCheckPhase(now, true);
      }
      break;
    }

    case STATE_FREE_FALL: {
      peakAcc  = fmaxf(peakAcc, acc);
      peakGyro = fmaxf(peakGyro, gyro);

      if (buttonPressed()) { returnToIdle("Nguoi dung huy bang nut nhan."); break; }

      if (acc > IMPACT_THRESHOLD) {
        uint32_t dur = now - freeFallTime;
        freefallDurationMs = (uint16_t)min<uint32_t>(dur, 65535);
#if ENABLE_DATA_LOG
        logf("[DATA] freefallDuration=%lums impactAcc=%.2fg peakGyro=%.1fdeg/s",
             (unsigned long)dur, acc, peakGyro);
#endif
        if (dur < MIN_FREEFALL_DURATION)        returnToIdle("Bo qua - qua nhanh.");
        else if (peakGyro < GYRO_THRESHOLD)     returnToIdle("Bo qua - khong xoay.");
        else                                    enterCheckPhase(now, false);
      } else if (now - freeFallTime > TIME_IMPACT_MAX) {
        returnToIdle("Bo qua - het thoi gian cho va cham.");
      }
      break;
    }

    case STATE_CHECK: {
      peakAcc  = fmaxf(peakAcc, acc);
      peakGyro = fmaxf(peakGyro, gyro);

      if (buttonPressed()) { returnToIdle("Nguoi dung huy bang nut nhan."); break; }

      uint32_t elapsed = now - impactTime;
      if (elapsed < SETTLE_MS) break;              // bỏ qua pha nảy/lăn

      if (elapsed > CHECK_MAX_MS) {                // hạn tối đa của pha kiểm tra
        returnToIdle("Bo qua - het han pha kiem tra.");
        break;
      }

      bool rest = (acc >= MIN_REST_ACC && acc <= MAX_REST_ACC && gyro < REST_GYRO_THRESHOLD);
      if (rest) {
        restAccumMs += dt;                         // yêu cầu nằm yên LIÊN TỤC
        motionRunMs  = 0;
      } else {
        motionRunMs += dt;
        restAccumMs  = 0;                          // đứt quãng => đếm lại từ đầu
        if (motionRunMs >= MOTION_CANCEL_MS) {
          returnToIdle("Bo qua - con chuyen dong.");
          break;
        }
      }

      uint32_t restNeed   = hardImpactPath ? REST_REQUIRED_MS_HARD : REST_REQUIRED_MS;
      float    orientNeed = hardImpactPath ? ORIENT_CHANGE_THRESHOLD_HARD : ORIENT_CHANGE_THRESHOLD;
      float    gyroNeed   = hardImpactPath ? GYRO_THRESHOLD_HARD : GYRO_THRESHOLD;

      if (restAccumMs >= restNeed) {
        float tiltChange = vecAngleDeg(gAvgX, gAvgY, gAvgZ, gSnapX, gSnapY, gSnapZ);
#if ENABLE_DATA_LOG
        logf("[DATA] tiltChange=%.1fdeg peakGyro=%.1f restAccum=%lums",
             tiltChange, peakGyro, (unsigned long)restAccumMs);
#endif
        if (peakGyro < gyroNeed)            returnToIdle("Bo qua - xoay khong du (duong va cham manh).");
        else if (tiltChange < orientNeed)   returnToIdle("Bo qua - tu the khong thay doi.");
        else enterAlarm(hardImpactPath ? TRIG_HARD_IMPACT : TRIG_FREEFALL,
                        tiltChange, (uint16_t)min<uint32_t>(elapsed, 65535));
      }
      break;
    }

    case STATE_ALARM: {
      if (buttonPressed()) returnToIdle("Da huy bao dong bang nut nhan.");
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
        if (currentState == STATE_ALARM && !alarmAcked) {
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

// ======================= TIỆN ÍCH MẠNG =======================
static WiFiClientSecure gSecure;

String jsonGetString(const String& s, const char* key) {
  String k = String("\"") + key + "\"";
  int i = s.indexOf(k);            if (i < 0) return "";
  int c = s.indexOf(':', i + k.length()); if (c < 0) return "";
  int q1 = s.indexOf('"', c + 1);  if (q1 < 0) return "";
  int q2 = s.indexOf('"', q1 + 1); if (q2 < 0) return "";
  return s.substring(q1 + 1, q2);
}

double jsonGetNumber(const String& s, const char* key, double def) {
  String k = String("\"") + key + "\"";
  int i = s.indexOf(k);            if (i < 0) return def;
  int c = s.indexOf(':', i + k.length()); if (c < 0) return def;
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
  http.setReuse(true);                              // tái sử dụng kết nối TLS
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
  if (code != 200) { logf("Dang nhap Firebase that bai, ma: %d", code); return false; }
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
  String url  = String("https://securetoken.googleapis.com/v1/token?key=") + FIREBASE_API_KEY;
  String body = String("grant_type=refresh_token&refresh_token=") + refreshToken;
  int code = httpsRequest(url, "POST", body.c_str(), "application/x-www-form-urlencoded", resp);
  if (code != 200) {
    logf("Lam moi token that bai, ma: %d", code);
    if (code == 400) { refreshToken = ""; saveRefreshToken(); }
    return false;
  }
  String tok = jsonGetString(resp, "id_token");
  if (tok.length() == 0) return false;
  idToken = tok;
  String newRt = jsonGetString(resp, "refresh_token");
  if (newRt.length() > 0 && newRt != refreshToken) { refreshToken = newRt; saveRefreshToken(); }
  tokenObtainedTime = millis();
  return true;
}

bool ensureValidToken() {
  if (idToken.length() > 0 && (millis() - tokenObtainedTime) < TOKEN_LIFETIME_MS) return true;
  if (refreshIdToken()) return true;
  return signInAnonymously();
}

// Dựng payload bằng bộ đệm tĩnh (không dùng String => không phân mảnh heap)
void buildStatusPayload(const StatusSnapshot& s, char* out, size_t n) {
  static const char* NAMES[3] = {"IDLE", "CHECKING", "ALARM"};
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
}

bool sendStatus(const StatusSnapshot& s) {
  if (!ensureValidToken()) return false;
  char payload[320];
  buildStatusPayload(s, payload, sizeof(payload));
  String resp;
  String url = String(FIREBASE_HOST) + "/devices/" + gDeviceId + "/status.json?auth=" + idToken;
  int code = httpsRequest(url, "PUT", payload, "application/json", resp);
  if (code == 200) return true;
  logf("Gui Firebase LOI, ma: %d", code);
  if (code == 401 || code == 403) idToken = "";
  return false;
}

bool sendEvent(const FallEvent& ev) {
  if (!ensureValidToken()) return false;
  static const char* TRIG[4] = {"FREEFALL", "HARD_IMPACT", "SOS", "TEST"};
  char payload[320];
  time_t nowT = time(nullptr);
  snprintf(payload, sizeof(payload),
           "{\"trigger\":\"%s\",\"peakAcc\":%.2f,\"peakGyro\":%.1f,\"tiltChange\":%.1f,"
           "\"freefallMs\":%u,\"impactToRestMs\":%u,\"timestamp\":%ld,"
           "\"at\":{\".sv\":\"timestamp\"}}",
           TRIG[ev.trigger > 3 ? 0 : ev.trigger], ev.peakAcc, ev.peakGyro, ev.tiltChange,
           (unsigned)ev.freefallMs, (unsigned)ev.impactToRestMs,
           (nowT > 100000) ? (long)nowT : -1);
  String resp;
  String url = String(FIREBASE_HOST) + "/devices/" + gDeviceId + "/events.json?auth=" + idToken;
  int code = httpsRequest(url, "POST", payload, "application/json", resp);
  if (code == 401 || code == 403) idToken = "";
  return code == 200;
}

// Giao thức lệnh idempotent: ứng dụng ghi {cmd, id}; thiết bị chỉ ĐỌC và bỏ
// qua mọi id nhỏ hơn hoặc bằng id đã xử lý => không có ghi ngược, không mất
// lệnh do tranh chấp ghi như phiên bản cũ.
Command pollCommand(double& lastCmdId) {
  if (!ensureValidToken()) return CMD_NONE;
  String resp;
  String url = String(FIREBASE_HOST) + "/devices/" + gDeviceId + "/command.json?auth=" + idToken;
  int code = httpsRequest(url, "GET", nullptr, nullptr, resp);
  if (code == 401 || code == 403) idToken = "";
  if (code != 200 || resp.length() < 5) return CMD_NONE;

  double id = jsonGetNumber(resp, "id", 0);
  if (id <= lastCmdId) return CMD_NONE;
  String cmd = jsonGetString(resp, "cmd");
  lastCmdId = id;

  if (cmd == "ACK")   return CMD_ACK;
  if (cmd == "RESET") return CMD_RESET;
  if (cmd == "TEST")  return CMD_TEST;
  return CMD_NONE;
}

// ======================= TÁC VỤ MẠNG (nhân 0) =======================
void netTask(void* pv) {
  esp_task_wdt_add(NULL);
  unsigned long lastOk = 0, lastAttempt = 0, lastWifiRetry = 0, lastPoll = 0;
  unsigned long backoff = RETRY_BASE_MS;
  bool lastFailed = false, ntpStarted = false;
  uint32_t lastSeq = 0xFFFFFFFF;
  uint32_t lastLoopBeat = 0, lastBeatChange = millis();
  double lastCmdId = 0;
  FallEvent pendingEvent;
  bool hasPendingEvent = false;

  for (;;) {
    esp_task_wdt_reset();
    unsigned long now = millis();

    // 0) Giám sát chéo: vòng lấy mẫu (chức năng an toàn) phải luôn chạy
    uint32_t beat = gLoopHeartbeat;
    if (beat != lastLoopBeat) { lastLoopBeat = beat; lastBeatChange = now; }
    else if (now - lastBeatChange > LOOP_STALL_LIMIT_MS) {
      logLine("Vong lay mau bi treo - khoi dong lai thiet bi.");
      delay(100);
      esp_restart();
    }

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
    if (!ntpStarted) { configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com"); ntpStarted = true; }

    // 3) Gửi trạng thái: khi đổi trạng thái, theo nhịp tim, hoặc gửi lại khi lỗi
    StatusSnapshot s = readSnapshot();
    bool changed = (s.seq != lastSeq);
    bool due     = (lastOk == 0) || (now - lastOk >= HEARTBEAT_MS);
    bool canTry  = !lastFailed || (now - lastAttempt >= backoff);

    if ((changed || due) && canTry) {
      lastAttempt = now;
      if (sendStatus(s)) {
        lastOk = millis(); lastFailed = false; backoff = RETRY_BASE_MS; lastSeq = s.seq;
      } else {
        lastFailed = true;
        backoff = min<unsigned long>(backoff * 2, RETRY_MAX_MS);   // lùi theo cấp số nhân
      }
    }

    // 4) Đẩy nhật ký sự kiện ngã (giữ lại và gửi lại nếu đang mất mạng)
    if (!hasPendingEvent && gEventQueue) {
      hasPendingEvent = (xQueueReceive(gEventQueue, &pendingEvent, 0) == pdTRUE);
    }
    if (hasPendingEvent && canTry) {
      if (sendEvent(pendingEvent)) hasPendingEvent = false;
    }

    // 5) Đọc lệnh từ ứng dụng: dày khi đang cảnh báo, thưa khi bình thường
    unsigned long pollPeriod = (s.pubState == PUB_ALARM) ? CMD_POLL_ALARM_MS : CMD_POLL_IDLE_MS;
    if (now - lastPoll >= pollPeriod) {
      lastPoll = now;
      Command c = pollCommand(lastCmdId);
      if (c != CMD_NONE && gCmdQueue) xQueueSend(gCmdQueue, &c, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ======================= WIFI =======================
void connectWiFi(bool forcePortal) {
  WiFiManager wm;
  WiFiManagerParameter pDev("devid", "Ma thiet bi (device id)", gDeviceId, sizeof(gDeviceId) - 1);
  wm.addParameter(&pDev);
  wm.setConfigPortalTimeout(180);
  logLine("Dang ket noi WiFi...");

  bool connected = forcePortal
      ? wm.startConfigPortal("FallDetector-Setup", "12345678")
      : wm.autoConnect("FallDetector-Setup", "12345678");

  if (strlen(pDev.getValue()) > 0 && strcmp(pDev.getValue(), gDeviceId) != 0) {
    strncpy(gDeviceId, pDev.getValue(), sizeof(gDeviceId) - 1);
    gDeviceId[sizeof(gDeviceId) - 1] = '\0';
    saveDeviceId(gDeviceId);
  }

  if (connected) logf("WiFi da ket noi! IP: %s", WiFi.localIP().toString().c_str());
  else           logLine("Chua co WiFi. Thiet bi van phat hien te nga cuc bo va se tu thu lai.");
}

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  gSerialMutex = xSemaphoreCreateMutex();
  gCmdQueue    = xQueueCreate(4, sizeof(Command));
  gEventQueue  = xQueueCreate(8, sizeof(FallEvent));

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_LED, LOW);
  noTone(PIN_BUZZER);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  delay(200);

  // Dải đo rộng: va chạm khi ngã thường đạt 3-8 g và 300-600 °/s.
  byte status = mpu.begin(MPU_GYRO_CONFIG, MPU_ACC_CONFIG);
  if (status != 0) {
    logf("LOI: Khong ket noi duoc MPU6050! Ma loi: %d", status);
    while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(200); }
  }
  mpuWriteRegister(MPU_REG_CONFIG, MPU_DLPF_CFG);   // DLPF 44 Hz - chống chồng phổ
  logLine("Ket noi MPU6050 thanh cong (+-16g, +-2000 deg/s, DLPF 44Hz).");

  calibrateSensor();

  loadDeviceConfig();
  bool forcePortal = (digitalRead(PIN_BUTTON) == LOW);
  connectWiFi(forcePortal);

#if TLS_VERIFY
  gSecure.setCACert(FIREBASE_ROOT_CA);
#else
  gSecure.setInsecure();     // giới hạn đã nêu trong báo cáo (mục 3.3)
#endif

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_deinit();
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);                            // giám sát loop()

  xTaskCreatePinnedToCore(netTask, "netTask", 12288, NULL, 1, NULL, 0);

  logLine("==================================================");
  logf("HE THONG PHAT HIEN TE NGA v%s - SAN SANG (device=%s)", FW_VERSION, gDeviceId);
  logLine("==================================================");
}

// ======================= LOOP (100 Hz, không gọi mạng) =======================
void loop() {
  esp_task_wdt_reset();
  gLoopHeartbeat++;                                  // netTask giám sát biến này

  static uint32_t nextTick = 0, lastNow = 0;
  uint32_t now = millis();
  if (nextTick == 0) { nextTick = now; lastNow = now; }
  uint32_t dt = now - lastNow;
  if (dt == 0 || dt > 200) dt = SAMPLE_PERIOD_MS;    // chống nhảy bậc thời gian
  lastNow = now;

  updateButton(now);
  processCommands();
  readSensor();

  float acc  = computeAccTotal();
  float gyro = computeGyroTotal();

  updateGravity(acc);
  runStateMachine(now, dt, acc, gyro);
  updateAlarmOutputs(now);

  // Công bố: IDLE -> giá trị tức thời; có sự cố -> giá trị ĐỈNH của sự kiện
  float pubAcc  = (currentState == STATE_IDLE) ? acc  : peakAcc;
  float pubGyro = (currentState == STATE_IDLE) ? gyro : peakGyro;
  uint8_t pub = (currentState == STATE_IDLE) ? PUB_IDLE
              : ((currentState == STATE_ALARM) ? PUB_ALARM : PUB_CHECKING);
  publishSnapshot(pub, alarmAcked, pubAcc, pubGyro, computePostureTilt(), false);

#if ENABLE_CSV_STREAM
  Serial.printf("%lu,%.3f,%.1f,%.1f,%u\n", (unsigned long)now, acc, gyro,
                computePostureTilt(), (unsigned)currentState);
#endif

  nextTick += SAMPLE_PERIOD_MS;
  int32_t waitMs = (int32_t)(nextTick - millis());
  if (waitMs > 0) delay((uint32_t)waitMs);
  else nextTick = millis();
}
