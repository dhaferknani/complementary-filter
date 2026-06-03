#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D

// Variables angles
float roll = 0, pitch = 0, yaw = 0;
float lastRoll = 0, lastPitch = 0, lastYaw = 0;
const float threshold = 0.5; // seuil en degrés

// Temps
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialisation MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Initialisation QMC5883L
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x09);
  Wire.write(0b00011101);
  Wire.endTransmission();

  lastTime = millis();
  Serial.println("Ready");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0) dt = 0.01f; // sécurité
  lastTime = now;

  // Lecture MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Température ignorée
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Lecture QMC5883L
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 6);
  int16_t mx_raw = (Wire.read() | (Wire.read() << 8));
  int16_t my_raw = (Wire.read() | (Wire.read() << 8));
  int16_t mz_raw = (Wire.read() | (Wire.read() << 8));

  // Mapping axes NED selon ta config
  float accelX = ay / 16384.0f;    // Nord (Y MPU)
  float accelY = ax / 16384.0f;    // Est (X MPU)
  float accelZ = -az / 16384.0f;   // Down (-Z MPU)

  float gyroX = gy / 131.0f;       // Nord
  float gyroY = gx / 131.0f;       // Est
  float gyroZ = -gz / 131.0f;      // Down

  float magX = mx_raw * 0.0833f;   // Nord
  float magY = my_raw * 0.0833f;   // Est
  float magZ = mz_raw * 0.0833f;   // Down

  // Calcul angles accel (en degrés)
  float rollAcc = atan2(accelY, accelZ) * 180.0 / M_PI;
  float pitchAcc = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;

  // Calcul yaw magnétomètre (en degrés)
  float magYaw = atan2(magY, magX) * 180.0 / M_PI;

  // Filtre complémentaire coefficients
  const float alpha = 0.98f;

  // Intégration gyro + correction accel pour roll et pitch
  roll = alpha * (roll + gyroX * dt) + (1 - alpha) * rollAcc;
  pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * pitchAcc;

  // Intégration gyro + correction mag pour yaw
  yaw = alpha * (yaw + gyroZ * dt) + (1 - alpha) * magYaw;

  // Normalisation yaw dans [-180,180]
  if (yaw > 180) yaw -= 360;
  else if (yaw < -180) yaw += 360;

  // Affichage uniquement si changement > seuil
  if (fabs(roll - lastRoll) > threshold ||
      fabs(pitch - lastPitch) > threshold ||
      fabs(yaw - lastYaw) > threshold) {

    Serial.print("Roll: "); Serial.print(roll, 2);
    Serial.print(" | Pitch: "); Serial.print(pitch, 2);
    Serial.print(" | Yaw: "); Serial.println(yaw, 2);

    lastRoll = roll;
    lastPitch = pitch;
    lastYaw = yaw;
  }

  delay(10);
}