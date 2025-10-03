#include "MS5611.h"
#include <Wire.h>

MS5611 MS5611(0x77);

#ifndef LED_BUILTIN
#define LED_BUILTIN    13
#endif

// ฟังก์ชันบันทึกข้อมูล pressure+temp เฉลี่ยต่อชั้น
void recordPressure(int floor) {
  float pressure, temperature;
  float sumP = 0, sumT = 0;
  float minP = 999999, maxP = -999999;
  int count = 0;

  unsigned long startTime = millis();

  Serial.print("# Start recording floor ");
  Serial.println(floor);

  while (millis() - startTime < 15000) {   // เก็บข้อมูล 15 วิ
    if (MS5611.read() == MS5611_READ_OK) {
      pressure = MS5611.getPressure() * 100.0;   // hPa -> Pa (ชดเชยแล้ว)
      temperature = MS5611.getTemperature();     // °C (ชดเชยแล้ว)

      sumP += pressure;
      sumT += temperature;
      if (pressure < minP) minP = pressure;
      if (pressure > maxP) maxP = pressure;
      count++;

      // CSV format: floor,time(ms),pressure(Pa),temperature(C)
      Serial.print(floor);
      Serial.print(",");
      Serial.print(millis() - startTime);
      Serial.print(",");
      Serial.print(pressure, 2);
      Serial.print(",");
      Serial.println(temperature, 2);
    }
    delay(500); // ทุก 0.5 วินาที
  }

  if (count > 0) {
    float avgP = sumP / count;
    float avgT = sumT / count;

    Serial.println("# ------ Summary ------");
    Serial.print("# Floor: "); Serial.println(floor);
    Serial.print("# Samples: "); Serial.println(count);
    Serial.print("# Avg Pressure (Pa): "); Serial.println(avgP, 2);
    Serial.print("# Avg Temp (C): "); Serial.println(avgT, 2);
    Serial.print("# Min Pressure (Pa): "); Serial.println(minP, 2);
    Serial.print("# Max Pressure (Pa): "); Serial.println(maxP, 2);
    Serial.println("# ---------------------");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  if (!MS5611.begin()) {
    Serial.println("MS5611 not found. halt.");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }

  // ✅ ปรับตาม MS5611_adjust_pressure_math.ino
  MS5611.reset(1);  

  // ✅ (optional) เลือก oversampling เพื่อเพิ่มความเสถียร
  MS5611.setOversampling(OSR_HIGH);

  Serial.println("MS5611 ready (adjusted math).");
  Serial.println("กรุณาพิมพ์เลข 1-9 เพื่อเลือกชั้น");
  Serial.println("floor,time(ms),pressure(Pa),temperature(C)");
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch >= '1' && ch <= '9') {
      int floor = ch - '0';
      recordPressure(floor);
      Serial.println("# Done. พิมพ์เลข 1-9 เพื่อบันทึกใหม่...");
    }
  }
}
