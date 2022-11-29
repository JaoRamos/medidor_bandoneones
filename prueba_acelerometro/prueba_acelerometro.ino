// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

bool alternar = false;

unsigned long tiempo = 0;
int intervalo = 10;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);      //(MPU6050_BAND_5_HZ);      //    //(MPU6050_BAND_21_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_5_HZ);         //(MPU6050_HIGHPASS_DISABLE);
  //mpu.setCycleRate(MPU6050_CYCLE_1_25_HZ);
  Serial.println("Iniciado");
  delay(100);
}

void loop() {
  unsigned long t = millis();
  if ((t - tiempo) >= intervalo){
    leer();
    tiempo = t;
  }

  //delay(10);
}


void leer(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  // Serial.print("AccelX:");
  // Serial.print(a.acceleration.x);
  // Serial.print(",");
  // Serial.print("AccelY:");
  // Serial.print(a.acceleration.y);
  // Serial.print(",");
  
  Serial.print("Min:");
  Serial.print(-150);
  Serial.print(",");
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  Serial.print(",");
  Serial.print("Suma:");
  Serial.print(a.acceleration.x + a.acceleration.y + a.acceleration.z);
  Serial.print(",");
  Serial.print("Max:");
  
  Serial.print(alternar ? 150 : 160);
  
  Serial.println("");

  // Serial.print(", ");
  // Serial.print("GyroX:");
  // Serial.print(g.gyro.x);
  // Serial.print(",");
  // Serial.print("GyroY:");
  // Serial.print(g.gyro.y);
  // Serial.print(",");
  // Serial.print("GyroZ:");
  // Serial.print(g.gyro.z);
  // Serial.println("");

  alternar = !alternar;
  
}