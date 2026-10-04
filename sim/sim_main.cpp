// Harness mô phỏng FW 3.0.0 (Mpu6050_VIP.ino): biên dịch NGUYÊN VĂN lõi firmware, cấp tín hiệu tổng hợp tất định.
#include "mock.h"

uint32_t g_now_ms = 0;
int g_level[64] = {0};
int g_button_level = HIGH;
uint16_t g_buzzer_hz = 0;
bool g_i2c_ok = true;
TwoWire Wire;
std::function<Sample(uint32_t)> g_gen;
Sample g_last_sample{0, 0, 1, 0, 0, 0};
int g_mpu_begin_status = 0;
int g_wdt_resets = 0;
std::vector<std::string> g_logs;
SerialMock Serial;

#include "fw_core.inc"   // <- trích nguyên văn từ Mpu6050_VIP.ino (không chỉnh sửa)

// ---------------- bộ phát tín hiệu ----------------
static uint32_t rng = 12345;
static float nz(float amp) {                       // nhiễu tất định, đều trong [-amp, amp]
  rng = rng * 1664525u + 1013904223u;
  return (((rng >> 8) & 0xFFFF) / 65535.0f * 2.0f - 1.0f) * amp;
}
typedef std::function<float(float)> F;
static F K(float v) { return [v](float) { return v; }; }
static F LIN(float a, float b, float dur) { return [=](float t) { return a + (b - a) * std::min(1.0f, t / dur); }; }
static F EXPD(float a, float b, float tau) { return [=](float t) { return b + (a - b) * expf(-t / tau); }; }

struct Seg { uint32_t dur; F acc, th, gy; bool garbage; };
static std::vector<Seg> segs;
static void add(uint32_t dur, F acc, F th, F gy, bool garbage = false) { segs.push_back({dur, acc, th, gy, garbage}); }
static void still(uint32_t dur, float th) { add(dur, K(1.0f), K(th), K(0.0f)); }
static void seq10(const std::vector<float>& acc, float th0, float th1, const std::vector<float>& gy) {
  for (size_t i = 0; i < acc.size(); i++) {
    float th = th0 + (th1 - th0) * (float)i / std::max<size_t>(1, acc.size() - 1);
    add(10, K(acc[i]), K(th), K(gy[i]));
  }
}

struct Plan {
  std::vector<std::pair<uint32_t, uint32_t>> btn, i2cFail;      // khoảng thời gian (ms tương đối)
  std::vector<std::pair<uint32_t, Command>> cmdAbs;            // lệnh tại thời điểm tuyệt đối
  std::vector<std::pair<uint32_t, Command>> cmdAfterAlarm;     // lệnh sau thời điểm vào ALARM (pre-alarm)
  std::vector<std::pair<uint32_t, Command>> cmdAfterEscalate;  // lệnh sau thời điểm leo thang
  uint32_t total = 15000;
  bool calibMoving = false;
};
static Plan plan;

// Các "khối" chuyển động
static void blk_stand(uint32_t ms) { still(ms, 0); }
static void blk_freefall(uint32_t ms, float g0, float g1) { add(ms, K(0.12f), LIN(0, 10, ms), LIN(g0, g1, ms)); }
static void blk_impact(float peak, float gpeak, float th0) {       // 40 ms
  seq10({0.5f * (peak + 1.0f), peak, 0.62f * peak, 1.6f}, th0, th0 + 30, {gpeak * 0.8f, gpeak, gpeak * 0.9f, gpeak * 0.7f});
}
static void blk_bounce(uint32_t ms, float gpeak, float th0, float th1) {
  std::vector<float> a, g;
  for (uint32_t t = 0; t < ms; t += 10) {
    a.push_back(1.0f + 0.35f * expf(-(float)t / 150.0f) * sinf(6.2831853f * (float)t / 120.0f));
    g.push_back(40.0f + (gpeak - 40.0f) * expf(-(float)t / 150.0f));
  }
  seq10(a, th0, th1, g);
}
static void blk_lie(uint32_t ms, float th) { still(ms, th); }
static void blk_fall_standard(float peak = 3.42f, float gpeak = 412.6f, float thLie = 85.0f, uint32_t ffMs = 210) {
  blk_stand(1500);
  blk_freefall(ffMs, 60, 250);
  blk_impact(peak, gpeak, 10);
  blk_bounce(460, gpeak, 40, thLie);
}

static bool inAny(const std::vector<std::pair<uint32_t, uint32_t>>& v, uint32_t t) {
  for (auto& p : v) if (t >= p.first && t < p.second) return true;
  return false;
}

static void build(const std::string& n) {
  segs.clear();
  plan = Plan();
  if (n == "calib_still") { plan.total = 500; blk_stand(500); }
  else if (n == "calib_moving") { plan.calibMoving = true; plan.total = 500; blk_stand(500); }
  else if (n == "fall_freefall") { blk_fall_standard(); blk_lie(14000, 85); plan.total = 16000; }
  else if (n == "fall_hard") {
    blk_stand(1500);
    add(250, K(1.0f), LIN(0, 30, 250), K(140.0f));          // nghiêng người trước va chạm (con quay 140 °/s)
    blk_impact(3.1f, 160.0f, 30);
    blk_bounce(460, 160.0f, 50, 75);
    blk_lie(14000, 75); plan.total = 16000;
  }
  else if (n == "sit_lowrot") {        // ngồi mạnh xuống ghế: va chạm 2,8 g, xoay 80 °/s, tư thế gần như không đổi
    blk_stand(1500);
    add(200, K(1.0f), LIN(0, 6, 200), K(60.0f));
    blk_impact(2.8f, 80.0f, 6);
    blk_bounce(460, 80.0f, 6, 8);
    blk_lie(8000, 8); plan.total = 12000;
  }
  else if (n == "sit_highrot") {       // như trên nhưng xoay 150 °/s: bị loại ở điều kiện đổi tư thế
    blk_stand(1500);
    add(200, K(1.0f), LIN(0, 6, 200), K(120.0f));
    blk_impact(2.8f, 150.0f, 6);
    blk_bounce(460, 150.0f, 6, 8);
    blk_lie(8000, 8); plan.total = 12000;
  }
  else if (n == "drop_table") {        // đánh rơi/đặt mạnh thiết bị xuống bàn
    blk_stand(1500);
    blk_freefall(250, 60, 300);
    blk_impact(6.0f, 300.0f, 10);
    blk_bounce(460, 300.0f, 10, 10);
    blk_lie(8000, 10); plan.total = 12000;
  }
  else if (n == "freefall_short" || n == "freefall_short_hard") {    // rơi tự do chỉ 60 ms
    float pk = (n == "freefall_short") ? 2.2f : 3.4f;     // 2,2 g: chỉ xét đường 1; 3,4 g: cú va chạm vượt ngưỡng đường 2
    blk_stand(1500); blk_freefall(60, 60, 250); blk_impact(pk, 412.0f, 10);
    blk_bounce(460, 412.0f, 40, 85); blk_lie(6000, 85); plan.total = 9000;
  }
  else if (n == "impact_norot") {      // va chạm nhưng gần như không xoay (30 °/s)
    blk_stand(1500); blk_freefall(210, 10, 25); blk_impact(3.0f, 30.0f, 10);
    blk_bounce(460, 30.0f, 40, 85); blk_lie(6000, 85); plan.total = 9000;
  }
  else if (n == "freefall_noimpact") { // mất trọng lượng thoáng qua rồi trở lại 1 g, không có va chạm
    blk_stand(1500); blk_freefall(200, 10, 20); still(3000, 0); plan.total = 6000;
  }
  else if (n == "impact_late" || n == "impact_late_hard") {       // va chạm đến muộn 600 ms sau mẫu mất trọng lượng cuối (> 500 ms)
    float pk = (n == "impact_late") ? 2.2f : 3.4f;
    blk_stand(1500); blk_freefall(200, 60, 250); still(600, 0);
    blk_impact(pk, 412.0f, 10); blk_bounce(460, 412.0f, 40, 85); blk_lie(6000, 85); plan.total = 10000;
  }
  else if (n == "motion_after") {      // sau va chạm người vẫn tiếp tục cử động mạnh
    blk_fall_standard(); add(5000, K(1.0f), K(85), K(90.0f)); plan.total = 10000;
  }
  else if (n == "noise_rest") {        // nằm yên có hai nhiễu tức thời (1 mẫu và 2 mẫu)
    blk_fall_standard(); blk_lie(900, 85);
    add(10, K(1.6f), K(85), K(60.0f));
    blk_lie(300, 85);
    add(20, K(1.5f), K(85), K(55.0f));
    blk_lie(12000, 85); plan.total = 16000;
  }
  else if (n == "intermittent_long") { // nằm 1000 ms / cử động 400 ms lặp lại
    blk_fall_standard();
    for (int i = 0; i < 12; i++) { blk_lie(1000, 85); add(400, K(1.0f), K(85), K(70.0f)); }
    plan.total = 15000;
  }
  else if (n == "intermittent_short") {// nằm 1300 ms / cử động 100 ms lặp lại
    blk_fall_standard();
    for (int i = 0; i < 8; i++) { blk_lie(1300, 85); add(100, K(1.0f), K(85), K(70.0f)); }
    blk_lie(12000, 85); plan.total = 15000;
  }
  else if (n == "btn_cancel_check") {  // giữ nút 1,6 s trong giai đoạn kiểm tra
    blk_fall_standard(); blk_lie(14000, 85); plan.btn = {{2000, 3700}}; plan.total = 9000;
  }
  else if (n == "btn_short_check") {   // nút bị đè 1,0 s trong giai đoạn kiểm tra (ngã đè lên nút)
    blk_fall_standard(); blk_lie(14000, 85); plan.btn = {{2000, 3000}}; plan.total = 9000;
  }
  else if (n == "btn_cancel_prealarm") {
    blk_fall_standard(); blk_lie(14000, 85); plan.btn = {{8000, 9700}}; plan.total = 20000;
  }
  else if (n == "btn_short_prealarm") {
    blk_fall_standard(); blk_lie(14000, 85); plan.btn = {{8000, 9000}}; plan.total = 20000;
  }
  else if (n == "btn_cancel_escalated") {
    blk_fall_standard(); blk_lie(20000, 85); plan.btn = {{15000, 16700}}; plan.total = 20000;
  }
  else if (n == "uc04_flow") {         // ACK trong pre-alarm (bị bỏ qua), sau leo thang: ACK rồi RESET
    blk_fall_standard(); blk_lie(30000, 85);
    plan.cmdAfterAlarm = {{3000, CMD_ACK}};
    plan.cmdAfterEscalate = {{3000, CMD_ACK}, {8000, CMD_RESET}};
    plan.total = 24000;
  }
  else if (n == "buzzer_timeout") {    // không ai xử lý: còi tự tắt sau ALARM_BUZZER_MAX_MS
    blk_fall_standard(); blk_lie(400000, 85); plan.total = 330000;
  }
  else if (n == "sos") {               // giữ nút 2,2 s ở IDLE => SOS; nhả; giữ 1,6 s => huỷ
    blk_stand(12000); plan.btn = {{3000, 5200}, {7000, 8700}}; plan.total = 12000;
  }
  else if (n == "test_cmd") {
    blk_stand(10000); plan.cmdAbs = {{2000, CMD_TEST}}; plan.cmdAfterEscalate = {{3000, CMD_RESET}}; plan.total = 8000;
  }
  else if (n == "garbage") {           // 300 ms mẫu rác (0,0,0) khi IDLE
    blk_stand(1500); add(300, K(0.0f), K(0), K(0), true); blk_stand(2000); plan.total = 4000;
  }
  else if (n == "i2c_recover") {       // I2C hỏng 700 ms rồi phục hồi
    blk_stand(4000); plan.i2cFail = {{1500, 2200}}; plan.total = 4000;
  }
  else if (n == "fault_in_check") {    // I2C hỏng 2,6 s ngay trong pha kiểm tra
    blk_fall_standard(); blk_lie(14000, 85); plan.i2cFail = {{2200, 4800}}; plan.total = 8000;
  }
  else if (n == "fault_idle_restart") {// I2C hỏng kéo dài khi IDLE
    blk_stand(1500); plan.i2cFail = {{1500, 40000}}; plan.total = 40000;
  }
  else if (n == "fault_cancel_alarm") {// pre-alarm đang chạy, cảm biến hỏng, vẫn huỷ được bằng nút
    blk_fall_standard(); blk_lie(14000, 85); plan.i2cFail = {{6000, 14000}}; plan.btn = {{8000, 9700}}; plan.total = 14000;
  }
  else { fprintf(stderr, "unknown scenario %s\n", n.c_str()); exit(2); }
}

static uint32_t T0 = 0;
static Sample genAt(uint32_t t) {
  uint32_t rel = (t >= T0) ? t - T0 : 0;
  uint32_t acc = 0;
  for (auto& s : segs) {
    if (rel < acc + s.dur || &s == &segs.back()) {
      float tau = (float)(rel - std::min(rel, acc));
      if (s.garbage) return Sample{0, 0, 0, 0, 0, 0};
      float th = s.th(tau) * 3.14159265f / 180.0f, a = s.acc(tau), g = s.gy(tau);
      return Sample{sinf(th) * a + nz(0.012f), nz(0.012f), cosf(th) * a + nz(0.012f), nz(0.6f), g + nz(1.2f), nz(0.6f)};
    }
    acc += s.dur;
  }
  return Sample{0, 0, 1, 0, 0, 0};
}
static Sample genCalib(uint32_t) {
  float g = plan.calibMoving ? 35.0f : 0.0f;
  return Sample{nz(0.012f), nz(0.012f), 1.0f + nz(0.012f), nz(0.8f) + (plan.calibMoving ? g : 0), nz(0.8f), nz(0.8f)};
}

int main(int argc, char** argv) {
  std::string name = argc > 1 ? argv[1] : "fall_freefall";
  bool csv = argc > 2;
  build(name);

  gSerialMutex = xSemaphoreCreateMutex();
  gCmdQueue = xQueueCreate(4, sizeof(Command));
  gEventQueue = xQueueCreate(8, sizeof(FallEvent));
  digitalWrite(PIN_LED, LOW);
  noTone(PIN_BUZZER);

  bool sensorOk = false;
  for (int i = 0; i < 5 && !sensorOk; i++) sensorOk = initSensor();
  g_gen = genCalib;
  calibrateSensor();
  g_gen = genAt;
  T0 = millis();

  struct Row { uint32_t t; float acc, gyro; int st, pub, led, hz, btn; };
  std::vector<Row> rows;
  int tCheck = -1, tAlarm = -1, tEsc = -1, tIdleAfter = -1;
  bool restarted = false; uint32_t tRestart = 0;
  size_t cmdAbsIdx = 0, ca = 0, ce = 0;
  uint8_t prevPub = 0;
  bool acked = false; int tAck = -1;

  try {
    while (millis() - T0 < plan.total) {
      uint32_t rel = millis() - T0;
      g_button_level = inAny(plan.btn, rel) ? LOW : HIGH;
      g_i2c_ok = !inAny(plan.i2cFail, rel);
      while (cmdAbsIdx < plan.cmdAbs.size() && rel >= plan.cmdAbs[cmdAbsIdx].first) {
        Command c = plan.cmdAbs[cmdAbsIdx++].second; xQueueSend(gCmdQueue, &c, 0);
      }
      if (tAlarm >= 0)
        while (ca < plan.cmdAfterAlarm.size() && (int)rel >= tAlarm + (int)plan.cmdAfterAlarm[ca].first) {
          Command c = plan.cmdAfterAlarm[ca++].second; xQueueSend(gCmdQueue, &c, 0);
        }
      if (tEsc >= 0)
        while (ce < plan.cmdAfterEscalate.size() && (int)rel >= tEsc + (int)plan.cmdAfterEscalate[ce].first) {
          Command c = plan.cmdAfterEscalate[ce++].second; xQueueSend(gCmdQueue, &c, 0);
        }

      loop();

      StatusSnapshot s = readSnapshot();
      if (s.pubState == PUB_CHECKING && tCheck < 0) tCheck = rel;
      if (currentState == STATE_ALARM && tAlarm < 0) tAlarm = rel;
      if (s.pubState == PUB_ALARM && tEsc < 0) { tEsc = rel; printf("SNAP_AT_ESC state=ALARM acc=%.2f gyro=%.1f tilt=%.1f acked=%d\n", s.acc, s.gyro, s.tilt, (int)s.acked); }
      if (s.acked && tAck < 0) tAck = rel;
      if (tAlarm >= 0 && currentState == STATE_IDLE && tIdleAfter < 0) tIdleAfter = rel;
      prevPub = s.pubState;
      float acc = vecNorm(g_last_sample.ax, g_last_sample.ay, g_last_sample.az);
      float gyro = vecNorm(g_last_sample.gx, g_last_sample.gy, g_last_sample.gz);
      rows.push_back({rel, acc, gyro, (int)currentState, (int)s.pubState, g_level[PIN_LED], (int)g_buzzer_hz, g_button_level == LOW});
    }
  } catch (RestartException&) {
    restarted = true; tRestart = millis() - T0;
  }
  (void)prevPub;

  // ---- đầu ra ----
  printf("=== SCENARIO %s ===\n", name.c_str());
  for (auto& l : g_logs) printf("%s\n", l.c_str());
  printf("--- SUMMARY\n");
  printf("T0_ms=%u (sau hieu chuan)\n", T0);
  printf("t_checking=%d t_alarm_local=%d t_escalate=%d t_ack=%d t_idle_after_alarm=%d restart=%s@%u\n",
         tCheck, tAlarm, tEsc, tAck, tIdleAfter, restarted ? "YES" : "no", tRestart);
  printf("final_state=%d pub=%d acked=%d\n", (int)currentState, (int)readSnapshot().pubState, (int)readSnapshot().acked);
  FallEvent ev; int n = 0;
  while (xQueueReceive(gEventQueue, &ev, 0)) {
    n++;
    printf("EVENT trig=%u peakAcc=%.2f peakGyro=%.1f tilt=%.1f freefallMs=%u impactToRestMs=%u at_rel=%d\n",
           ev.trigger, ev.peakAcc, ev.peakGyro, ev.tiltChange, ev.freefallMs, ev.impactToRestMs, (int)ev.atMs - (int)T0);
  }
  printf("events_in_queue=%d\n", n);
  if (csv) {
    FILE* f = fopen(("out/" + name + ".csv").c_str(), "w");
    fprintf(f, "t,acc,gyro,state,pub,led,hz,btn\n");
    for (auto& r : rows) fprintf(f, "%u,%.3f,%.1f,%d,%d,%d,%d,%d\n", r.t, r.acc, r.gyro, r.st, r.pub, r.led, r.hz, r.btn);
    fclose(f);
  }
  return 0;
}
