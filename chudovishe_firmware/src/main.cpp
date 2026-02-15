#include <Arduino.h>

#define L_PWM_PIN 5
#define R_PWM_PIN 6
#define L_DIR1_PIN 7
#define L_DIR2_PIN 8
#define R_DIR1_PIN 12
#define R_DIR2_PIN 13

#define L_ENC_A 2
#define L_ENC_B 4
#define R_ENC_A 3
#define R_ENC_B 11

volatile long l_count = 0;
volatile long r_count = 0;

long l, r = 0;

void onLeftAChange() {
  bool b = digitalRead(L_ENC_B);
  if (b == HIGH) ++l_count;
  else --l_count;
}

void onRightAChange() {
  bool b = digitalRead(R_ENC_B);
  if (b == HIGH) ++r_count;
  else --r_count;
}

void setMotorPins(uint8_t pwmPin, uint8_t dir1, uint8_t dir2, uint8_t pwmVal, uint8_t dirBit) {
  if (dirBit == 0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  } else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
  }
  analogWrite(pwmPin, pwmVal);
}

unsigned long prevSendMs = 0;

void setup() {
  Serial.begin(115200);

  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(L_DIR1_PIN, OUTPUT);
  pinMode(L_DIR2_PIN, OUTPUT);
  pinMode(R_DIR1_PIN, OUTPUT);
  pinMode(R_DIR2_PIN, OUTPUT);

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), onLeftAChange, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), onRightAChange, RISING);
}


void loop() {
  // Receive commands in 5-byte frames: l_pwm r_pwm l_dir r_dir checksum
  while (Serial.available() >= 5) {
    uint8_t buf[5];
    for (uint8_t i = 0; i < 5; ++i) {
      int v = Serial.read();
      if (v < 0) v = 0;
      buf[i] = (uint8_t)v;
    }
    uint8_t sum = (uint8_t)((buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF);
    if (sum == buf[4]) {
      setMotorPins(L_PWM_PIN, L_DIR1_PIN, L_DIR2_PIN, buf[0], buf[2]);
      setMotorPins(R_PWM_PIN, R_DIR1_PIN, R_DIR2_PIN, buf[1], buf[3]);
    }
  }

  // Send encoder telemetry every 20 ms: l_enc r_enc checksum (all bytes)
  if (millis() - prevSendMs >= 20) {
    prevSendMs = millis();
    noInterrupts();
    l = l_count;
    r = r_count;
    interrupts();
    uint8_t l_byte = (uint8_t)(l & 0xFF);
    uint8_t r_byte = (uint8_t)(r & 0xFF);
    uint8_t ck = (uint8_t)((l_byte + r_byte) & 0xFF);
    uint8_t out[3] = { l_byte, r_byte, ck };
    Serial.write(out, 3);
  }
}