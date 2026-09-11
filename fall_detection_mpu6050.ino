#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

// ================== CẤU HÌNH CHÂN ==================
#define PIN_LED      5
#define PIN_BUZZER   18
#define PIN_BUTTON   4

// ================== NGƯỠNG THUẬT TOÁN ==================
// ⚠️ LƯU Ý: Các giá trị dưới đây là ước lượng ban đầu dựa trên lý thuyết,
// CẦN được tinh chỉnh lại bằng dữ liệu thực nghiệm thu thập từ chính thiết bị này.
// Xem log "📊 [DATA]" để lấy số liệu thật, sau đó điều chỉnh các #define này.

#define FREEFALL_THRESHOLD    0.6   // g   - ngưỡng rơi tự do
#define IMPACT_THRESHOLD      2.5   // g   - ngưỡng va chạm
#define MIN_REST_ACC          0.7   // g   - cận dưới vùng nghỉ
#define MAX_REST_ACC          1.3   // g   - cận trên vùng nghỉ

#define TIME_IMPACT_MAX        600  // ms  - thời gian tối đa từ free-fall đến impact
#define TIME_REST_REQUIRED    2000  // ms  - thời gian phải nằm yên để xác nhận
#define MIN_FREEFALL_DURATION  100  // ms  - free-fall tối thiểu (loại trừ động tác tay)

#define GYRO_THRESHOLD          100.0  // độ/s - tốc độ xoay tối thiểu lúc va chạm
#define ORIENTATION_CHANGE_THRESHOLD 45.0  // độ - thay đổi góc nghiêng tối thiểu

#define MOTION_CANCEL_SAMPLES    6   // số mẫu chuyển động liên tục để hủy (chống nhiễu)
#define CALIBRATION_SAMPLES    100   // số mẫu dùng hiệu chuẩn lúc khởi động

// Bật/tắt log chi tiết phục vụ thu thập dữ liệu tinh chỉnh (Milestone 5)
#define ENABLE_DATA_LOG true

// ================== MÁY TRẠNG THÁI ==================
enum FallState {
  STATE_IDLE,
  STATE_FREE_FALL,
  STATE_CHECK_INACTIVITY,
  STATE_ALARM
};

MPU6050 mpu(Wire);
FallState currentState = STATE_IDLE;

unsigned long freeFallTime = 0;
unsigned long inactivityStartTime = 0;
int motionCancelCounter = 0;
float accOffset = 0;

float preFallTilt = 0;
float freefallTiltSnapshot = 0;

// ================== HÀM TIỆN ÍCH ==================
// MPU6050_light trả accel trực tiếp theo đơn vị "g" (không cần chia 9.81)
// và gyro trực tiếp theo độ/s, nên các hàm dưới đây được rút gọn tương ứng.

float computeAccTotal() {
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float raw = sqrt(ax * ax + ay * ay + az * az);
  return raw + accOffset;
}

float computeGyroTotal() {
  float gx = mpu.getGyroX();
  float gy = mpu.getGyroY();
  float gz = mpu.getGyroZ();
  return sqrt(gx * gx + gy * gy + gz * gz);
}

float computeTiltAngle() {
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float totalAcc = sqrt(ax * ax + ay * ay + az * az);
  if (totalAcc < 0.1) return preFallTilt; // tránh chia 0 khi đang free-fall thật
  float cosAngle = constrain(az / totalAcc, -1.0, 1.0);
  return acos(cosAngle) * 180.0 / PI;
}

void calibrateSensor() {
  Serial.println("🔧 Đang hiệu chuẩn cảm biến... Giữ yên thiết bị!");
  float sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    mpu.update();
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float raw = sqrt(ax * ax + ay * ay + az * az);
    sum += raw;
    delay(20);
  }
  float avgReading = sum / CALIBRATION_SAMPLES;
  accOffset = 1.0 - avgReading;

  Serial.print("✅ Hiệu chuẩn xong. Giá trị đo trung bình: ");
  Serial.println(avgReading, 4);
  Serial.print("   Hệ số bù (offset): ");
  Serial.println(accOffset, 4);
}

// ================== SETUP ==================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  digitalWrite(PIN_LED, LOW);
  noTone(PIN_BUZZER);

  Wire.begin();
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("❌ LỖI: Không kết nối được MPU6050! Mã lỗi: ");
    Serial.println(status);
    Serial.println("Kiểm tra dây SDA(21), SCL(22).");
    while (1) { delay(10); }
  }

  Serial.println("✅ Kết nối MPU6050 thành công (MPU6050_light).");

  calibrateSensor();

  Serial.println("==================================================");
  Serial.println("✅ HỆ THỐNG PHÁT HIỆN TÉ NGÃ - 4 LỚP PHÒNG VỆ SẴN SÀNG");
  Serial.println("==================================================");
}

// ================== LOOP ==================

void loop() {
  mpu.update(); // bắt buộc gọi trước khi đọc bất kỳ giá trị nào từ mpu

  float accTotal = computeAccTotal();
  float currentTilt = computeTiltAngle();
  unsigned long currentTime = millis();

  switch (currentState) {

    case STATE_IDLE:
      preFallTilt = 0.3 * currentTilt + 0.7 * preFallTilt;

      if (accTotal < FREEFALL_THRESHOLD) {
        currentState = STATE_FREE_FALL;
        freeFallTime = currentTime;
        freefallTiltSnapshot = preFallTilt;
        Serial.println("📉 [Giai đoạn 1] Phát hiện Rơi Tự Do (Free-fall)!");
      }
      break;

    case STATE_FREE_FALL:
      if (accTotal > IMPACT_THRESHOLD) {
        unsigned long freefallDuration = currentTime - freeFallTime;
        float gyroTotal = computeGyroTotal();

        #if ENABLE_DATA_LOG
        Serial.print("📊 [DATA] freefallDuration=");
        Serial.print(freefallDuration);
        Serial.print("ms, impactAcc=");
        Serial.print(accTotal, 2);
        Serial.print("g, gyro=");
        Serial.print(gyroTotal, 1);
        Serial.println("deg/s");
        #endif

        if (freefallDuration < MIN_FREEFALL_DURATION) {
          currentState = STATE_IDLE;
          Serial.println("🔄 Va chạm đến quá nhanh -> Bỏ qua (nghi ngờ động tác tay).");
          break;
        }

        if (gyroTotal < GYRO_THRESHOLD) {
          currentState = STATE_IDLE;
          Serial.print("🔄 Va chạm mạnh nhưng KHÔNG xoay người (");
          Serial.print(gyroTotal, 1);
          Serial.println(" độ/s) -> Bỏ qua.");
          break;
        }

        currentState = STATE_CHECK_INACTIVITY;
        inactivityStartTime = currentTime;
        motionCancelCounter = 0;
        Serial.print("💥 [Giai đoạn 2] Va chạm mạnh + Xoay người (");
        Serial.print(gyroTotal, 1);
        Serial.println(" độ/s) - Đang xác nhận...");
      }
      else if (currentTime - freeFallTime > TIME_IMPACT_MAX) {
        currentState = STATE_IDLE;
      }
      break;

    case STATE_CHECK_INACTIVITY:
      if (accTotal >= MIN_REST_ACC && accTotal <= MAX_REST_ACC) {
        motionCancelCounter = 0;

        if (currentTime - inactivityStartTime >= TIME_REST_REQUIRED) {
          float tiltChange = abs(currentTilt - freefallTiltSnapshot);

          #if ENABLE_DATA_LOG
          Serial.print("📊 [DATA] tiltBefore=");
          Serial.print(freefallTiltSnapshot, 1);
          Serial.print("deg, tiltAfter=");
          Serial.print(currentTilt, 1);
          Serial.print("deg, change=");
          Serial.print(tiltChange, 1);
          Serial.println("deg");
          #endif

          if (tiltChange >= ORIENTATION_CHANGE_THRESHOLD) {
            currentState = STATE_ALARM;
            Serial.print("⚠️ [Giai đoạn 3] XÁC NHẬN TÉ NGÃ THẬT! Đổi tư thế ");
            Serial.print(tiltChange, 1);
            Serial.println(" độ. Kích hoạt báo động.");
          } else {
            currentState = STATE_IDLE;
            Serial.print("🔄 Nằm yên đủ lâu nhưng tư thế KHÔNG đổi nhiều (");
            Serial.print(tiltChange, 1);
            Serial.println(" độ) -> Bỏ qua.");
          }
        }
      } else {
        motionCancelCounter++;
        if (motionCancelCounter >= MOTION_CANCEL_SAMPLES) {
          currentState = STATE_IDLE;
          motionCancelCounter = 0;
          Serial.println("🔄 Phát hiện chuyển động thật -> Hủy xác nhận té ngã.");
        }
      }
      break;

    case STATE_ALARM:
      digitalWrite(PIN_LED, HIGH);
      tone(PIN_BUZZER, 1000);

      if (digitalRead(PIN_BUTTON) == LOW) {
        digitalWrite(PIN_LED, LOW);
        noTone(PIN_BUZZER);
        currentState = STATE_IDLE;
        Serial.println("✅ ĐÃ BẤM NÚT: Hủy báo động thành công!");
        delay(300);
      }
      break;
  }

  delay(50);
}
