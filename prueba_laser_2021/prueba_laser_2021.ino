// sensor de distancia local
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X laser;
float distancia = 0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  laser.init();
  //sensor.setTimeout(50);
  laser.startContinuous();
  laser.setMeasurementTimingBudget(20000);
}

void loop() {
  // put your main code here, to run repeatedly:
  distancia = min(laser.readReg16Bit(laser.RESULT_RANGE_STATUS + 10), 400) * 0.1;
  Serial.println(int(distancia));
  delay(10);
}
