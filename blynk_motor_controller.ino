#include "BLEConfig.h"
#include <Bounce.h>
#include "Ticker.h"
#include "MotorDriver.h"
#include "PulseIn.h"
#include "Encoder.h"
#include "PID.h"
#include "Ring.h"
#include "Logging.h"

//ピン割り当て
const int m0_pwm = 6;
const int m0_in1 = 5;
const int m0_in2 = 10;
const int enc0_A = 1;
const int enc0_B = 0;

const int m1_pwm = 11;
const int m1_in1 = 20;  // <-21
const int m1_in2 = 21;  // <-20 反対側のモータは逆方向に回転させる
const int enc1_A = 13;
const int enc1_B = 12;

const int stby = 9;
const int button = 19;  // A5

MotorDriver m0(m0_in1, m0_in2, m0_pwm);
MotorDriver m1(m1_in1, m1_in2, m1_pwm);

PulseIn plsin0(enc0_A, enc0_B);
PulseIn plsin1(enc1_A, enc1_B);

Encoder enc0(enc0_A, enc0_B);
Encoder enc1(enc1_A, enc1_B);

PID m0_pid;
PID m1_pid;

Bounce btn = Bounce(button, 10);

// Create ble instance, see pinouts above
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void encoder_polling(void)
{
  enc0.poll();
  enc1.poll();
}

volatile float target0 = 0; // パルス/秒
volatile float target1 = 0; // パルス/秒

volatile float velocity = 0;
volatile float angular_velocity = 0;

const float GEAR_RATIO = (244904.f) / (12000.f);
const float COUNT_PER_REV = 12.f * GEAR_RATIO;
const float WHEEL_RADIUS = 0.027f;
const float TREAD_WIDTH = 0.185f;

void update_target()
{
  // 運動ベースの移動計算
  float vR = (1 / WHEEL_RADIUS) * velocity + ( TREAD_WIDTH / (2 * WHEEL_RADIUS)) * angular_velocity;
  float vL = (1 / WHEEL_RADIUS) * velocity + (-TREAD_WIDTH / (2 * WHEEL_RADIUS)) * angular_velocity;
  vR = (COUNT_PER_REV * vR) / (2 * PI);
  vL = (COUNT_PER_REV * vL) / (2 * PI);
  Serial.println(velocity);
  Serial.println(angular_velocity);
  Serial.println(vR);
  Serial.println(vL);
  target0 = vR;
  target1 = vL;
}

void pid_process(void)
{
  float p0, p1, u0 = 0, u1 = 0;
  int d0, d1;

  // 1秒辺りのパルス数と回転方向を取得
  noInterrupts();
  p0 = plsin0.pulsePerSecond();
  p1 = plsin1.pulsePerSecond();
  d0 = enc0.direction();
  d1 = enc1.direction();
  interrupts();

  p0 *= d0;
  p1 *= d1;

  // 速度フィードバック制御
  if (target0) {
    u0 = m0_pid.process(((float)target0) - p0);
    m0.go(u0);
  } else {
    m0.stop();
  }
  if (target1) {
    u1 = m1_pid.process(((float)target1) - p1);
    m1.go(u1);
  } else {
    m1.stop();
  }
}

void setup() {
  // シリアルポート初期化
  Serial.begin(250000);
  // BLEライブラリ初期化
  ble.begin(BLUEFRUIT_VERBOSE_MODE);
  ble.factoryReset(); // Optional
  ble.setMode(BLUEFRUIT_MODE_DATA);
  // Blynkライブラリ初期化
  Blynk.begin(auth, ble);
  // ボタン入力
  pinMode(button, INPUT_PULLUP);
  // モータドライバ初期化
  m0.begin();
  m1.begin();
  // モータドライバスタンバイ解除
  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH);
  // エンコーダ初期化
  enc0.begin();
  enc1.begin();
  // パルス入力初期化
  plsin0.begin();
  plsin1.begin();
  // PID制御初期化
  m0_pid.setInterval(0.001F);
  m1_pid.setInterval(0.001F);
  //#define KP 5.71824F
  //#define KI 62.83784F
  //#define KD 0.02287F
#define KP 3.33564F
#define KI 30.54617F
#define KD 0.F
  m0_pid.setGain(KP, KI, KD);
  m1_pid.setGain(KP, KI, KD);
  m0_pid.setOutputLimits(-1023, 1023);
  m1_pid.setOutputLimits(-1023, 1023);
  // 割り込み処理開始
  attachTickerInterrupt(0, encoder_polling, 25000);
  attachTickerInterrupt(2, pid_process, 1000);
}

// 前進
BLYNK_WRITE(V0) {
  velocity = (float)param.asInt() / 1000.f;
  update_target();
}
// 後退
BLYNK_WRITE(V1) {
  velocity = (float)param.asInt() / 1000.f;
  update_target();
}
// 右旋回
BLYNK_WRITE(V2) {
  angular_velocity = radians(param.asInt());
  update_target();
}
// 左旋回
BLYNK_WRITE(V3) {
  angular_velocity = radians(param.asInt());
  update_target();
}
// 右モータ正転
BLYNK_WRITE(V5) {
  target0 = param.asInt();
}
// 右モータ逆転
BLYNK_WRITE(V7) {
  target0 = param.asInt();
}
// 左モータ正転
BLYNK_WRITE(V4) {
  target1 = param.asInt();
}
// 左モータ逆転
BLYNK_WRITE(V6) {
  target1 = param.asInt();
}

void loop()
{
  Blynk.run();

  btn.update();
  if ( btn.fallingEdge() ) {
    target0 = target1 = 0;
    m0.stop();
    m1.stop();
  }
}

