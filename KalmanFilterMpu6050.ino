
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
float Yaw  = 0, Pitch  =0, Roll = 0;
float NoiseYaw = 0, NoisePitch  = 0, NoiseRoll = 0 ;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 
  Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  Serial.println("");
  delay(100);
}


void loop() {
  // Inicnando Sensores
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Primeiro com x , aplicar mesma logica para Yaw , Pitch , Roll
  NoiseRoll = g.gyro.x;
  NoisePitch = g.gyro.y;
  NoiseYaw = g.gyro.z;

   float RollFilter = KalmanR(NoiseRoll);
   float PitchFilter = KalmanP(NoisePitch);
   float YawFilter = KalmanY(NoiseYaw);

  Serial.print(1023);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(NoiseRoll);
  Serial.print(",");
  Serial.print(NoisePitch);
  Serial.print(",");
  Serial.print(NoiseYaw);
  Serial.print(",");
  Serial.println(RollFilter);
  Serial.print(",");
  Serial.println(PitchFilter);
  Serial.print(",");
  Serial.println(YawFilter);

  delay(5);
}
float KalmanY(float input){
  static  float Q = 0.01; //Variancia do processo
  static float R = 3; // Variancia da mediçao
  static float X_est = 0.0;
  static float P = 1.0; //Estimatica inicial do erro

  float Z = (float)input; // Armazena a medida atual
  float X_pred = X_est;// Estado de prediçao 
  float P_pred = P + Q; //Atualiza erro de prediçao 
  float K = P_pred / (P_pred + R);

  X_est = X_pred + K * (Z-X_pred); // Atualiza a estimativa com a mediçao
  P = (1-K)* P_pred;// Atualiza o erro de estimativa
  return X_est;
}
float KalmanP(float input){
  static  float Q = 0.01; //Variancia do processo
  static float R = 3; // Variancia da mediçao
  static float X_est = 0.0;
  static float P = 1.0; //Estimatica inicial do erro

  float Z = (float)input; // Armazena a medida atual
  float X_pred = X_est;// Estado de prediçao 
  float P_pred = P + Q; //Atualiza erro de prediçao 
  float K = P_pred / (P_pred + R);

  X_est = X_pred + K * (Z-X_pred); // Atualiza a estimativa com a mediçao
  P = (1-K)* P_pred;// Atualiza o erro de estimativa
  return X_est;
}
float KalmanR(float input){
  static  float Q = 0.01; //Variancia do processo
  static float R = 3; // Variancia da mediçao
  static float X_est = 0.0;
  static float P = 1.0; //Estimatica inicial do erro

  float Z = (float)input; // Armazena a medida atual
  float X_pred = X_est;// Estado de prediçao 
  float P_pred = P + Q; //Atualiza erro de prediçao 
  float K = P_pred / (P_pred + R);

  X_est = X_pred + K * (Z-X_pred); // Atualiza a estimativa com a mediçao
  P = (1-K)* P_pred;// Atualiza o erro de estimativa
  return X_est;
}
