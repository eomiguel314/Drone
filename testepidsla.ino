#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

Adafruit_MPU6050 mpu;

float angularVelocity = 0.0;
unsigned long lastTime;

//Pid variables

float Kp = 0;
float Ki = Kd/Ti;
float Kd = Kp*Td;

Tu = 0; 

Td = Tu/8;
Ti = Tu/2; 
float erro = 0;
int setpoint = 0;

float PID = Kp*erro + Ki + Kd;


void setup() {
    Serial.begin(9600);
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("MPU6050 nao encontrado");
        while (1);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    lastTime = micros();
}

void loop() {
    sensors_event_t acc, gyro, temp;
    mpu.getEvent(&acc, &gyro, &temp);

    angularVelocity = gyro.gyro.x; // rad/s

    delay(10);

    //PID
    sensor = angularVelocity;
    erro = setpoint - sensor;

    m1 = trottle + PI

    
}
