#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Pinos usados na pcb
#define PIN_M1 13
#define PIN_M2 12
#define PIN_M3 14
#define PIN_M4 27

Servo m1, m2, m3, m4;

#define RAD_TO_DEG 57.295779

float alpha = 0.02;
float dt = 0.004;
float pitch = 0, roll = 0;

// PID
float erroRoll = 0, erroPitch = 0;
float oldErroRoll = 0, oldErroPitch = 0;
float integralRoll = 0, integralPitch = 0;

float KpRoll = 1.5, KiRoll = 0.5, KdRoll = 0.02;
float KpPitch = 1.5, KiPitch = 0.5, KdPitch = 0.02;

float outputRoll = 0, outputPitch = 0;

// Setpoints
float setpointRoll = 0;
float setpointPitch = 0;

int Throttle = 1200;

void setup() {
  Serial.begin(115200);

  m1.attach(PIN_M1);
  m2.attach(PIN_M2);
  m3.attach(PIN_M3);
  m4.attach(PIN_M4);

  if (!mpu.begin()) {
    while (1);
  }
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g,&temp);

  filterAngles(g.gyro.x * RAD_TO_DEG,g.gyro.y * RAD_TO_DEG,a.acceleration.y,a.acceleration.x);

  PID();
  mixMotors();
}

void filterAngles(float gx, float gy, float ay, float ax) {
  pitch = (1 - alpha) * (pitch + gx * dt)
        + alpha * atan2(ax, ay) * RAD_TO_DEG;

  roll  = (1 - alpha) * (roll + gy * dt)
        + alpha * atan2(ay, ax) * RAD_TO_DEG;
}

void PID() {
  erroPitch = setpointPitch - pitch;
  erroRoll  = setpointRoll  - roll;

  integralPitch += erroPitch * dt;
  integralRoll  += erroRoll  * dt;

  integralPitch = constrain(integralPitch, -200, 200);
  integralRoll  = constrain(integralRoll,  -200, 200);

  outputPitch =
    KpPitch * erroPitch +
    KiPitch * integralPitch +
    KdPitch * (erroPitch - oldErroPitch) / dt;

  outputRoll =
    KpRoll * erroRoll +
    KiRoll * integralRoll +
    KdRoll * (erroRoll - oldErroRoll) / dt;

  oldErroPitch = erroPitch;
  oldErroRoll  = erroRoll;
}

void mixMotors() {
  int m1v = Throttle + outputPitch + outputRoll;
  int m2v = Throttle + outputPitch - outputRoll;
  int m3v = Throttle - outputPitch + outputRoll;
  int m4v = Throttle - outputPitch - outputRoll;

  m1v = constrain(m1v, 1000, 2000);
  m2v = constrain(m2v, 1000, 2000);
  m3v = constrain(m3v, 1000, 2000);
  m4v = constrain(m4v, 1000, 2000);

  m1.writeMicroseconds(m1v);
  m2.writeMicroseconds(m2v);
  m3.writeMicroseconds(m3v);
  m4.writeMicroseconds(m4v);
}
