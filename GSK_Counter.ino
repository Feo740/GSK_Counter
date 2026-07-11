#include <Arduino.h>
#include <Wire.h>      // Для I2C (DS3231)
#include <SPI.h>       // Обязательно оставить, даже если не используешь SPI напрямую
#include <RTClib.h>

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();

  if (!rtc.begin()) {
    Serial.println(F("Ошибка: DS3231 не найден. Проверь подключение I2C."));
    while (1) delay(10);
  }

  // Если питание пропадало — ставим время компиляции
  if (rtc.lostPower()) {
    Serial.println(F("RTC потерял питание. Устанавливаем время по компиляции..."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime now = rtc.now();
  Serial.print(F("RTC OK. Время: "));
  Serial.print(now.year());
  Serial.print('/');
  Serial.print(now.month());
  Serial.print('/');
  Serial.println(now.day());
}

void loop() {
  delay(1000);
}
