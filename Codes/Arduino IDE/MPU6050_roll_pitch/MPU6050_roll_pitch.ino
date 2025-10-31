#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Variables para almacenar ángulos
float roll = 0.0;
float pitch = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Inicializar MPU6050
  if (!mpu.begin()) {
    Serial.println("Error: No se encontró el MPU6050!");
    while (1) {
      delay(10);
    }
  }

  // Configurar rangos del sensor
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  // Obtener lecturas del sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calcular Roll y Pitch desde acelerómetro
  calculateRollPitch(a.acceleration.x, a.acceleration.y, a.acceleration.z);

  // Formato para Serial Plotter (con etiquetas)
  Serial.print("Roll:");
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.println(pitch, 2);

  delay(50); // 20 Hz - Frecuencia de actualización
}

void calculateRollPitch(float ax, float ay, float az) {
  // Roll (φ): Rotación alrededor del eje X
  roll = atan2(ay, az) * 180.0 / PI;
  
  // Pitch (θ): Rotación alrededor del eje Y
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}