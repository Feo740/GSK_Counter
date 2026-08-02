#include <Arduino.h>
#include "mqtt_handler.h"
#include <Wire.h>
#include <SPI.h>
#include <RTClib.h> // библиотека для часов реального времени
//#include <SoftwareSerial.h> //библиотека для работы с RS485
//#include <ETH.h>
#include <WiFi.h>
#include <HTTPClient.h> // для работы с гугл таблицами
#include <AsyncMqttClient.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
//-------- порты для rs 485
#define SSerialRx        19  // Serial Receive pin RO
#define SSerialTx        17   // Serial Transmit pin DI
//-------- инициализация
//SoftwareSerial RS485Serial(SSerialRx, SSerialTx); // Rx, Tx
//// линия управления передачи приема
#define SerialControl 16  // RS485 Direction control 5 or 18
/////// флаг приема передачи
#define RS485Transmit    HIGH
#define RS485Receive     LOW

// концевик заслонки вентиляции
//#define LIMSW_X 16
//-------- Инициализация аппаратного UART (UART1)
HardwareSerial RS485Serial(1);  // Используем UART1

TimerHandle_t wifiReconnectTimer;

unsigned int crc16MODBUS(byte *s, int count) {
  unsigned int crcTable[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };
    unsigned int crc = 0xFFFF;
    for(int i = 0; i < count; i++) {
        crc = ((crc >> 8) ^ crcTable[(crc ^ s[i]) & 0xFF]);
    }
    return crc;
}
/////// команды
//byte testConnect[] = { 0x00, 0x00 };
byte testConnect[] = { 0x16, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}; // пакет подключения к счетчику
byte Access[]      = { 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
//byte Sn[]          = { 0x00, 0x08, 0x00 }; // серийный номер
//byte Freq[]        = { 0x00, 0x08, 0x16, 0x40 }; // частота
byte Current[]     = { 0x00, 0x08, 0x16, 0x21 };//  ток фаза 1
//byte Suply[]       = { 0x00, 0x08, 0x16, 0x11 }; // напряжение
//byte Power[]       = { 0x00, 0x08, 0x16, 0x00 };// мощность
//byte Angle[]       = { 0x00, 0x08, 0x16, 0x51 }; // углы
//byte activPower[]  = { 0x00, 0x05, 0x00, 0x00 };//  суммарная энергия прямая + обратная + активная + реактивная
byte sumPower[]    = { 0x1, 0x08, 0x16, 0x08 };// команда запроса потребляемой мощности
byte odometr[]     = { 0x1, 0x05, 0x00, 0x00 }; // команда запроса общего пробега
byte p_v[]         = { 0x1, 0x08, 0x11, 0x11 }; // команда запроса напряжения по фазе
byte response[19];
int byteReceived;
int byteSend;
int netAdr;
//Массив для данных с терминала
char incomingBytes[15];

// глобальная переменная для хранения результата пробега из счетчика.
float r1;

String odometr_data; //строка пробега считанного со счетчика функцией GetOdo
String voltage_data; // строка значения напряжения считанного функцией GetVoltage по фазе
String current_data; // строка значения тока считанного функцией GetCurrent по фазе
String sumpower_data; // строка значения суммарной мощности по всем фазам
String power_data1; // строка значения  мощности по фазе 1
String power_data2; // строка значения  мощности по фазе 2
String power_data3; // строка значения  мощности по фазе 3

//String voltage_data2; // строка значения напряжения считанного функцией GetVoltage по фазе 2
//String voltage_data3; // строка значения напряжения считанного функцией GetVoltage по фазе 3
// дней*(24 часов в сутках)*(60 минут в часе)*(60 секунд в минуте)*(1000 миллисекунд в секунде)
unsigned long period_counter = 43200000;//86400000;  ///< таймер для проверки счетчика, раз в сутки
unsigned long p_counter = 0; ///< Техническая переменная счетчика таймера
unsigned long period_voltage = 5000; // таймер для снятия показаний напряжения и тока
unsigned long p_clapan = 0; ///< Техническая переменная счетчика таймера
unsigned int period_18b20_read = 500; ///< таймер ожидания преобразования в датчике 18b20
unsigned long dht22 = 0; ///< Техническая переменная счетчика таймера
unsigned long dht22_power = 0; ///< Техническая переменная счетчика таймера
unsigned long T18b20_1 = 0; ///< Техническая переменная счетчика таймера
unsigned long read_18b20 = 0; ///< Техническая переменная счетчика таймера
unsigned long voltageP = 0; ///< Техническая переменная счетчика таймера для снятия напр и тока

// логин и пароль сети WiFi
const char* ssid = "MikroTik-1EA2D2";
const char* password = "ferrari220";
//const char* ssid = "US_WIFI";
//const char* password = "beeline2022";
//const char* ssid = "4G-UFI-3a43";
//const char* password = "1234567890";
String GOOGLE_SCRIPT_ID = "AKfycbwHpzQ8bA0wraHN7WdZJJ7oMgI4xG_gV070WzbCrgiyIDQEgr1O2D42vUslEp1gkkKL"; //ID Google таблички
//String GOOGLE_SCRIPT_ID = "1Flzse1pfy-nzjjS-P8k3HPZ7YkLm4w85BFGZREPJXeo"; //ID Google таблички
IPAddress ip;

RTC_DS3231 rtc; // объект для часов реального времени

String inputBuffer = "";      // буфер для накопления строки
bool newCommand = false;     // флаг: пришла ли полная команда
//Создаем структуру для хранения связной информации гараж-счетчик-одометр
struct GarageData {
  uint16_t garageNumber;      // трёхзначный номер гаража (0–999)
  uint32_t meterNumber;       // номер счётчика 
  float    odometerReading;   // показание (кВт·ч)
  bool     isValid;           // флаг: удалось ли успешно опросить счётчик
};

// Таблица соответствия: номер бокса -> буква столбца в Google Sheets
struct BoxMapping {
  uint16_t boxNumber;
  const char* column;
};

const BoxMapping boxMap[] = {
  {70, "B"},
  //{85, "C"},
  {43, "D"},
  {138, "E"},
  {64, "F"},
  {17, "G"},
  {14, "H"},
  {183, "I"},
  {83, "J"},
  {15, "K"},
  {18, "L"},
  {94, "M"},
  {67, "N"},
  {182, "O"},
  {23, "P"},
  {24, "Q"},
  {74, "R"},
  {201, "S"},
  {96, "T"},
  {84, "U"},
  {22, "V"},
  {39, "W"},
  {181, "X"},
  {16, "Y"},
  {89, "Z"},
  {63, "AA"},
  {85, "AB"},
  {61, "AC"},
  {68, "AD"},
  {40, "AE"},
  {36, "AF"},
  {69, "AG"},
  {77, "AH"},
  {213, "AI"},
  {158, "AJ"},
  {55, "AK"},
  {51, "AL"},
  {2, "AM"},
  {33, "AN"},
  {52, "AO"},
  {60, "AP"},
  {205, "AQ"},
  {78, "AR"},
  {93, "AS"},
  {86, "AT"},
  {46, "AU"},
  {169, "AV"},
  {206, "AW"},
  {0, nullptr} // маркер конца массива (ноль не может быть номером бокса)
};

// Вот здесь объявляем boxMapCount
const int boxMapCount = sizeof(boxMap) / sizeof(boxMap[0]);

//Заполняем массив струтур тестовыми значениями
GarageData garages[] = {
  //тестовый счетчик
  {2, 70, 0.0f, true},

  //головной счетчик 743
  {1, 43, 1.0f, true},

  // первый бокс
  {100, 138, 2.0f, true},
  {102, 64, 3.0f, true},
  {103, 17, 4.0f, true},
  {107, 14, 5.0f, true},
  {108, 183, 6.0f, true},
  {109, 83, 7.0f, true},
  {110, 15, 8.0f, true},
  {111, 18, 9.0f, true},
  {113, 94, 10.0f, true},
  // второй бокс
  {200, 67, 11.0f, true},
  {201, 182, 12.0f, true},
  {206, 23, 13.0f, true},
  {207, 24, 14.0f, true},
  {210, 74, 15.0f, true},
  {213, 201, 16.0f, true},
  {218, 96, 17.0f, true},
  // третий бокс
  {300, 84, 18.0f, true},
  {307, 22, 19.0f, true},
  {311, 39, 20.0f, true},
  {312, 181, 21.0f, true},
  {313, 16, 22.0f, true},
  {314, 89, 23.0f, true},
  {315, 63, 24.0f, true},
  // четветый бокс
  {400, 85, 25.0f, true},
  {402, 61, 26.0f, true},
  {403, 68, 27.0f, true},
  {404, 40, 28.0f, true},
  {406, 36, 29.0f, true},
  {407, 69, 30.0f, true},
  {408, 77, 31.0f, true},
  {410, 213, 32.0f, true},
  {411, 158, 33.0f, true},
  {413, 55, 34.0f, true},
  {414, 51, 35.0f, true},
  {420, 2, 36.0f, true},
  {421, 33, 37.0f, true},
  {422, 52, 38.0f, true},
  {423, 60, 39.0f, true},
  {424, 205, 40.0f, true},
  {425, 78, 41.0f, true},
  {426, 93, 42.0f, true},
  {427, 86, 43.0f, true},
  {428, 46, 44.0f, true},
  {429, 169, 45.0f, true},
  {430, 206, 46.0f, true},
};
// определяем размер массива
const int GARAGES_COUNT = sizeof(garages) / sizeof(garages[0]); // определяем размер массива
// функция вывода в формате CVS

//функция получения из структуры данных одо по номеру гаража
float getOdometerForGarage(uint16_t targetNumber) {
  for (int i = 0; i < GARAGES_COUNT; i++) {
    if (garages[i].garageNumber == targetNumber) {
      return garages[i].odometerReading;
    }
  }
  // Если гараж не найден — вернём -1 как признак ошибки
  return -1.0f;
}

//прототип
void write_to_google_sheet(String params);


void printGarageListCSV() {
  Serial.println("garage,meter,reading,status");
  for (int i = 0; i < GARAGES_COUNT; i++) {
    char buf[16];
    dtostrf(garages[i].odometerReading, 8, 2, buf);
    Serial.print(garages[i].garageNumber);
    Serial.print(',');
    Serial.print(garages[i].meterNumber);
    Serial.print(',');
    Serial.print(buf);
    Serial.print(',');
    Serial.println(garages[i].isValid ? "OK" : "NO_DATA");
  }
}
//функция вывода даты и времени
void printDateTime(const DateTime& dt) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%04u/%02u/%02u %02u:%02u:%02u",
           dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());
  Serial.println(buf);
}

// функция  вывода в удобоваримом формате
void printGarageList() {
  // Заголовок таблицы
  Serial.println("Garage | Meter      | Reading   | Status");
  Serial.println("-------|------------|-----------|---------");

  for (int i = 0; i < GARAGES_COUNT; i++) {
    Serial.print(garages[i].garageNumber);
    Serial.print("      | ");
    Serial.print(garages[i].meterNumber);
    Serial.print(" | ");

    // Форматируем показание до 2 знаков после запятой
    char buffer[16];
    dtostrf(garages[i].odometerReading, 8, 2, buffer);
    Serial.print(buffer);
    Serial.print(" | ");

    if (garages[i].isValid) {
      Serial.println("OK");
    } else {
      Serial.println("NO DATA");
    }
  }
}

void send(byte *cmd, int s, byte *response) {
 // Serial.print("sending...");
  unsigned int crc = crc16MODBUS(cmd, s);
  unsigned int crc1 = crc & 0xFF;
  unsigned int crc2 = (crc>>8) & 0xFF;
  delay(10);
  // Переключаем в режим передачи
  digitalWrite(SerialControl, RS485Transmit);  // Init Transceiver
  delay(1);  // Небольшая задержка для стабилизации
       for(int i=0; i<s; i++)
       {
              RS485Serial.write(cmd[i]);
       }
  RS485Serial.write(crc1);
  RS485Serial.write(crc2);
  RS485Serial.flush();  // Ждём окончания передачи

  // Переключаем в режим приёма
  digitalWrite(SerialControl, RS485Receive);  // Init Transceiver
  delay(2);// Даем время счётчику на ответ
  int i = 0;
  unsigned long timeout = millis() + 1000;  // Таймаут 1 сек
  
  while (millis() < timeout && i < 19) {
    if (RS485Serial.available()) {
      response[i++] = RS485Serial.read();
      timeout = millis() + 100;  // Обновляем таймаут при получении байта
    }
  }

  // Обнуляем остатки, если пришло меньше
  while (i < 19) {
    response[i++] = 0;
  }

}

// Функция считывает пробег со счетчика, формирует MQTT сообщение с пробегом, текстовую переменную odometr_data для формирования строки гугл-табли
void GetOdo(byte number){
  testConnect[0] = number;
  odometr[0] = number;
  // Опрос счетчика 22
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  delay(1000);
  send(odometr, sizeof(odometr), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  long result_odo=0;
  result_odo=response[2];
  result_odo=result_odo<<8;
  result_odo=result_odo+response[1];
  result_odo=result_odo<<8;
  result_odo=result_odo+response[4];
  result_odo=result_odo<<8;
  result_odo=result_odo+response[3];
  float r = result_odo;
  r1 = r/1000.0f;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.3f", r1);   // всегда будет точка, например "0.018"
  odometr_data = String(buf);
  Serial.println(odometr_data);
  /*String var2 = "ESP32Counter/Counter"+String(number);
  char var1[23];
  var2.toCharArray(var1,23);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, odometr_data.c_str());
  odometr_data.replace(".",",");
  for (int i=0; i<19; i++){
    response[i]=0;
  }*/
  
}


// на основе данных функции GetOdo, формирует полный пакет опроса счетчика
void odo(int target) {
  GetOdo(target);
    /*String param = "box";
  param += target;
  param += "=";
  param += odometr_data;

  // ГАРАНТИРОВАННО заменяем запятую на точку в итоговой строке
  param.replace(',', '.');

  Serial.print("FINAL PARAM: ");
  Serial.println(param);  
  delay(500);
   Serial.println(param);
  write_to_google_sheet(param);
  */
}

//функция передачи собранных данных в гугл табличку
void writeAllGarages(GarageData* garages, size_t count) {
  String queryString = "";

  for (size_t i = 0; i < count; ++i) {
    if (!garages[i].isValid) continue;

    // Ищем соответствие по номеру бокса, чтобы получить букву столбца
    for (int j = 0; j < boxMapCount; ++j) {
      if (garages[i].meterNumber == boxMap[j].boxNumber) {
        const char* col = boxMap[j].column; // например "D" или "AA"

        char buf[20];
        snprintf(buf, sizeof(buf), "%.3f", garages[i].odometerReading);

        if (queryString.length() > 0) {
          queryString += "&";
        }
        queryString += col;          // параметр = буква столбца (D, AA и т.д.)
        queryString += "=";
        queryString += buf;
        break;
      }
    }
  }

  if (queryString.length() == 0) {
    Serial.println("No valid data to send");
    return;
  }

  Serial.print("Sending all in one request: ");
  Serial.println(queryString);

  //String baseUrl = "https://script.google.com/macros/s/AKfycbxwurwRRddUcZicLEqtov0QGkh9jDIjnCa8uorSOR40XKirSNvfyvXQqiIgGy0tZUTZ/exec?";
  //String fullUrl = baseUrl + queryString;

  //write_to_google_sheet(fullUrl);
  write_to_google_sheet(queryString);
}


//обработка команд терминала
void handleCommand(const String& cmdRaw) {
  String cmd = cmdRaw;
  cmd.trim();

  if (cmd.startsWith("settime ")) {
    // Ожидаем формат: settime 2024 10 25 14 30 00
    String args = cmd.substring(8);
    int y, m, d, h, mi, s;
    char buf[64];
    args.toCharArray(buf, sizeof(buf));

    if (sscanf(buf, "%d %d %d %d %d %d", &y, &m, &d, &h, &mi, &s) == 6) {
      DateTime newTime(y, m, d, h, mi, s);
      rtc.adjust(newTime);
      Serial.print(F("Время установлено: "));
      printDateTime(newTime);
    } else {
      Serial.println(F("Неверный формат. Используйте: settime YYYY MM DD HH MM SS"));
    }
    return;
  }

  if (cmd == "time") {
     DateTime now = rtc.now();
    Serial.print(F("Текущее время: "));
    Serial.println(now.timestamp());
  }
  if (cmd == "pull") {
    //получаем данные пробега для гаража 404
   float reading = getOdometerForGarage(404);  
   String inputBuffer = "";      // буфер для накопления строки
   char buffer[16];
   dtostrf(reading, 0, 2, buffer);  // 0 = автоширина, 2 знака после запятой
   String s(buffer);    
   Serial.print(F("отправляем данные в гугл "));
   Serial.print(s);
   s  = "box40="+s; 
   write_to_google_sheet(s);
  } 



  if (cmd == "spisok") {
    printGarageList();
    return;
  }

  if (cmd.startsWith("find ")) {
    String numStr = cmd.substring(5);
    uint16_t target = numStr.toInt();
    bool found = false;

    for (int i = 0; i < GARAGES_COUNT; i++) {
      if (garages[i].garageNumber == target) {
        Serial.print("Гараж: "); Serial.print(garages[i].garageNumber);
        Serial.print(", счётчик: "); Serial.print(garages[i].meterNumber);
        Serial.print(", показание: ");
        char buf[16];
        dtostrf(garages[i].odometerReading, 8, 2, buf);
        Serial.println(buf);
        found = true;
        break;
      }
    }
    if (!found) {
      Serial.println("Гараж не найден.");
    }
    return;
  }

  // функция записи в структуру
  if (cmd.startsWith("read ")) {
    String numStr = cmd.substring(5);
    uint16_t target = numStr.toInt(); 
    bool found = false;

    for (int i = 0; i < GARAGES_COUNT; i++) {
      if (garages[i].garageNumber == target) {
        uint32_t meterNumber_var;       // номер счётчика 
        meterNumber_var=garages[i].meterNumber;
        GetOdo(meterNumber_var);
        garages[i].odometerReading=r1; 
        found = true;
        break;
      }
    }
    if (!found) {
      Serial.println("Гараж не найден.");
    }
    return;
  }

  //функция записи всех значений в структуру
  if (cmd.startsWith("read all")) {
        bool found = false; 
      for (int i = 0; i < GARAGES_COUNT; i++) {
        uint32_t meterNumber_var;       // номер счётчика 
        meterNumber_var=garages[i].meterNumber;
        GetOdo(meterNumber_var);
        garages[i].odometerReading=r1; 
        found = true;
        break;
      } 
    if (!found) {
      Serial.println("Гараж не найден.");
    }
    return;
}

if (cmd.startsWith("send all")) {
    writeAllGarages(garages, GARAGES_COUNT);
    return;
}

  Serial.println("Доступные: spisok, find <номер>, read <номер>, read all, send all, time, settime YYYY MM DD HH MM SS");
}


/*!
 \brief функция подключения к сети wifi
  осуществляет подключение к сети.
 */
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
             //  "Подключаемся к WiFi..."
  WiFi.begin(ssid, password);
IPAddress ip = WiFi.localIP();
}





//Функция переподключения к Wifi и MQTT  при обрыве связи
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");  //  "Подключились к WiFi"
      Serial.println("IP address: ");  //  "IP-адрес: "
      Serial.println(WiFi.localIP());
      //connectToMqtt();  // FIX: ВКЛЮЧИТЬ КОГДА БУДЕМ РАБОТАТЬ С mqtt
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
                 //  "WiFi-связь потеряна"
      // делаем так, чтобы ESP32
      // не переподключалась к MQTT
      // во время переподключения к WiFi:
      Serial.printf("SSID=");
      Serial.println(ssid);
      Serial.printf("PASS=");
      Serial.println(password);
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}


void setup() {
Serial.begin(9600);  // Отладочный вывод через USB (UART0)
  // настраиваем сеть
RS485Serial.begin(9600, SERIAL_8N1, SSerialRx, SSerialTx);  // UART1, пины RX=19, TX=17

// 5 пин в режим выхода
pinMode(SerialControl, OUTPUT);

// ставим на прием
digitalWrite(SerialControl, RS485Receive);// По умолчанию — приём
//delay(300);

//настройка сети mqtt и wifi пока не используем
//mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
connectToWifi();
WiFi.onEvent(WiFiEvent);

//mqttClient.onConnect(onMqttConnect);
//mqttClient.onDisconnect(onMqttDisconnect);
//mqttClient.onSubscribe(onMqttSubscribe);
//mqttClient.onUnsubscribe(onMqttUnsubscribe);
//mqttClient.onMessage(onMqttMessage);
//mqttClient.onPublish(onMqttPublish); и так был в комментарии
//mqttClient.setServer(MQTT_HOST, MQTT_PORT);
//mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

while (!Serial) {}
  Serial.println("Система готова.");
 
  Wire.begin();

 // if (!rtc.begin()) {
  //  Serial.println(F("Ошибка: модуль DS3231 не найден. Проверь I2C."));
   // while (1) delay(10);
  //}

  // Если питание пропадало — ставим время компиляции
 // if (rtc.lostPower()) {
    //Serial.println(F("RTC потерял питание. Устанавливаем время по компиляции..."));
   // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //}

 // printDateTime(rtc.now());

}


//Функция отправки данных в гугл таблицу
void write_to_google_sheet(String params) {
   HTTPClient http;
   String url="https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+params;
   //Serial.print(url);
    Serial.println("Posting data to Google Sheet");
    Serial.println(url);
    //---------------------------------------------------------------------
    //starts posting data to google sheet
    http.begin(url.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    //---------------------------------------------------------------------
    //getting response from google sheet
    String payload;
    if (httpCode > 0) {
        //payload = http.getString();
        Serial.println("Payload: ");//+payload);
    }
    //---------------------------------------------------------------------
    http.end();
}


//Функция считывает параметр "потребляемая мощность"
void GetPower(byte number){
  testConnect[0] = number; //определяем адрес счетчика к которому подключаемся для запрос
  sumPower[0] = number; // определеяем адрес четчика с которого считываем мощность
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  delay(1000);
  send(sumPower, sizeof(sumPower), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
  }
  //Формируем результат суммарной мощности
  Serial.println("");
  long result_power=0; // переменная для общей мощности
  long result_power1=0; // переменная для мощности по первой фазе
  long result_power2=0; // переменная для мощности по второй фазе
  long result_power3=0; // переменная для мощности по третьей фазе
  result_power = response[3];
  result_power=result_power<<8;
  result_power=result_power+response[2];
  //result_power=result_power<<8;
  //result_power=result_power+response[2];
  // Формируем результат мощности по фазе 1
  result_power1 = response[6];
  result_power1=result_power1<<8;
  result_power1=result_power1+response[5];
  //result_power1=result_power1<<8;
  //result_power1=result_power1+response[5];
  // Формируем результат мощности по фазе 2
  result_power2 = response[9];
  result_power2=result_power2<<8;
  result_power2=result_power2+response[8];
  //result_power2=result_power2<<8;
  //result_power2=result_power2+response[8];
  // Формируем результат мощности по фазе 3
  result_power3 = response[12];
  result_power3=result_power3<<8;
  result_power3=result_power3+response[11];
  //result_power3=result_power3<<8;
  //result_power3=result_power3+response[11];

  //Обработка результата суммарной мощности
  float r = result_power;
  float r1 = r/100.0f;
  Serial.println(r1,3);
  sumpower_data = String(r1);
  // формируем топик ESP32Counter/Counter40/SumPower
  String var2 = "ESP32Counter/Counter"+String(number)+"/SumPower";
  Serial.println("string");
  Serial.println(var2);
  char var1[32];
  var2.toCharArray(var1,32);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, sumpower_data.c_str());
  sumpower_data.replace(".",",");

  //Обработка результата мощности по фазе 1
  r = result_power1;
  r1 = r/100.0f;
  Serial.println(r1,3);
  power_data1 = String(r1);
  // формируем топик ESP32Counter/Counter40/PowerPhase1
  var2 = "ESP32Counter/Counter"+String(number)+"/PowerPhase1";
  Serial.println("string");
  Serial.println(var2);
  char var3[35];
  var2.toCharArray(var3,35);
  packetIdPub = mqttClient.publish(var3, 1, true, power_data1.c_str());
  power_data1.replace(".",",");

  //Обработка результата мощности по фазе 2
  r = result_power2;
  r1 = r/100.0f;
  Serial.println(r1,3);
  power_data2 = String(r1);
  // формируем топик ESP32Counter/Counter40/PowerPhase2
  var2 = "ESP32Counter/Counter"+String(number)+"/PowerPhase2";
  Serial.println("string");
  Serial.println(var2);
  var2.toCharArray(var3,35);
  packetIdPub = mqttClient.publish(var3, 1, true, power_data2.c_str());
  power_data2.replace(".",",");

  //Обработка результата мощности по фазе 3
  r = result_power3;
  r1 = r/100.0f;
  Serial.println(r1,3);
  power_data3 = String(r1);
  // формируем топик ESP32Counter/Counter40/PowerPhase1
  var2 = "ESP32Counter/Counter"+String(number)+"/PowerPhase3";
  Serial.println("string");
  Serial.println(var2);
  var2.toCharArray(var3,35);
  packetIdPub = mqttClient.publish(var3, 1, true, power_data3.c_str());
  power_data3.replace(".",",");

  //обнуляем массив принятого ответа
  for (int i=0; i<19; i++){
    response[i]=0;
   }
}

void power(){
  GetPower(40);
  String param;
  param  = "box40SumPower="+sumpower_data;
  param += "&box40Power1="+power_data1;
  param += "&box40Power2="+power_data2;
  param += "&box40Power3="+power_data3;
  write_to_google_sheet(param);
}

////Функция считывает параметр ток по определенной фазе
void GetCurrent (byte number){
  testConnect[0] = number; //определяем адрес счетчика к которому подключаемся для запрос
  Current[0] = number; // определеяем адрес четчика с которого считываем ток
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  delay(1000);
  send(Current, sizeof(Current), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  //Выделяем значение тока по фазе 1
  long result_current=0;
  result_current=response[3];
  result_current=result_current<<8;
  result_current=result_current+response[2];
  float r = result_current;
  float r1 = r/1000.0f;
  Serial.println(r1,3);
  current_data = String(r1);
  // формируем топик ESP32Counter/Counter40/CurrentPhase1
  String var2 = "ESP32Counter/Counter"+String(number)+"/CurrentPhase1";
  Serial.println("string");
  Serial.println(var2);
  char var1[37];
  var2.toCharArray(var1,37);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, current_data.c_str());

  //Выделяем значение тока по фазе 2
  result_current=0;
  result_current=response[6];
  result_current=result_current<<8;
  result_current=result_current+response[5];
   r = result_current;
   r1 = r/1000.0f;
  Serial.println(r1,3);
  current_data = String(r1);
  // формируем топик ESP32Counter/Counter40/CurrentPhase1
  var2 = "ESP32Counter/Counter"+String(number)+"/CurrentPhase2";
  Serial.println("string");
  Serial.println(var2);
  var1[37];
  var2.toCharArray(var1,37);
  packetIdPub = mqttClient.publish(var1, 1, true, current_data.c_str());

  //Выделяем значение тока по фазе 3
  result_current=0;
  result_current=response[9];
  result_current=result_current<<8;
  result_current=result_current+response[8];
   r = result_current;
   r1 = r/1000.0f;
  Serial.println(r1,3);
  current_data = String(r1);
  // формируем топик ESP32Counter/Counter40/CurrentPhase1
  var2 = "ESP32Counter/Counter"+String(number)+"/CurrentPhase3";
  Serial.println("string");
  Serial.println(var2);
  var1[37];
  var2.toCharArray(var1,37);
  packetIdPub = mqttClient.publish(var1, 1, true, current_data.c_str());

  for (int i=0; i<19; i++){
    response[i]=0;
   }
}

//Функция считывает параметр "напряжение по фазе"
void GetVoltage (byte number, byte phase_voltage){
  testConnect[0] = number; //определяем адрес счетчика к которому подключаемся для запрос
  p_v[0] = number; // определеяем адрес четчика с которого считываем напряжение по фазе
  p_v[3] = phase_voltage; // определеям фазу, по которой считываем значение
  // Опрос счетчика
  send(testConnect, sizeof(testConnect), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  delay(1000);
  send(p_v, sizeof(p_v), response);
  for (int i=0; i<19; i++){
    Serial.print(response[i]);
    Serial.print(", ");
}
  Serial.println("");
  long result_voltage=0;
  result_voltage=response[3];
  result_voltage=result_voltage<<8;
  result_voltage=result_voltage+response[2];
  float r = result_voltage;
  float r1 = r/100.0f;
  Serial.println(r1,3);
  byte phase;
  if(phase_voltage == 0x11) {phase = 1;}
  if(phase_voltage == 0x12) {phase = 2;}
  if(phase_voltage == 0x13) {phase = 3;}
  voltage_data = String(r1);
  // формируем топик ESP32Counter/Counter40/VoltagePhase1
  String var2 = "ESP32Counter/Counter"+String(number)+"/VoltagePhase"+String(phase);
  Serial.println("string");
  Serial.println(var2);
  char var1[37];
  var2.toCharArray(var1,37);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, voltage_data.c_str());
  voltage_data.replace(".",",");
  for (int i=0; i<19; i++){
    response[i]=0;
   }
}

// на основе данных функции GetCurrent, формирует полный пакет опроса счетчика
void current(){
  GetCurrent(40);
  delay(500);
  GetCurrent(85);
}

// на основе данных функции GetVoltage, формирует полный пакет опроса счетчика
void voltage(){
  GetVoltage(40, 0x11);
  //String param;
  //param  = "box40Voltage1="+voltage_data;
  GetVoltage(40, 0x12);
  //param += "&box40Voltage2="+voltage_data;
  GetVoltage(40, 0x13);
  //param += "&box40Voltage3="+voltage_data;
  //write_to_google_sheet(param);
}







void loop() {

/* пока закомментируем, т.к. отрабатываем меню
// Снятие данных счетчика раз в сутки
if ((millis() - p_counter) >= period_counter) {
  p_counter = millis();
  odo();
  voltage();
  current();
}

// снятие данных напряжения и тока по таймеру
if ((millis() - voltageP) >= period_voltage) {
  voltageP = millis();
  voltage();
  delay(500);
  current();
}

*/

if (Serial.available()) {
    char c = Serial.read();

    // ЭХО: отправляем символ обратно, чтобы видеть ввод
    Serial.write(c);

    if (c == '\n' || c == '\r') {
      newCommand = true;
      inputBuffer.trim();
    } else {
      inputBuffer += c;
    }
  }

  if (newCommand && !inputBuffer.isEmpty()) {
    handleCommand(inputBuffer);
    inputBuffer = "";
    newCommand = false;
    Serial.print("\nВведите команду: ");
  }
  }



/*
String getSerialNumber(int netAdr)
{
  String s1,s2,s3,s4;
  response[0]=0;
  Sn[0] = netAdr;
  send(Sn, sizeof(Sn),response);
  if((int)response[1] < 10) { s1="0" + String((int)response[1]); } else {s1=String((int)response[1]);}
  if((int)response[2] < 10) { s2="0" + String((int)response[2]); } else {s2=String((int)response[2]);}
  if((int)response[3] < 10) { s3="0" + String((int)response[3]); } else {s3=String((int)response[3]);}
  if((int)response[4] < 10) { s4="0" + String((int)response[4]); } else {s4=String((int)response[4]);}
  //String n = String((int)response[1]) + String((int)response[2]) +String((int)response[3])+ String((int)response[4]);
  String n = s1+s2+s3+s4;
  return String(response[0])+";"+n;
}

String getPowerNow(int netAdr)
{
  response[0]=0;
  Power[0] = netAdr;
  send(Power, sizeof(Power),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U0= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U1= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U2= String(r);
  r = 0;
  r |= (long)response[10]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U0+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
}

String getAngle(int netAdr)
{
  response[0]=0;
  Angle[0] = netAdr;
  send(Angle, sizeof(Angle),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
}

String getCurrent(int netAdr)
{
  response[0]=0;
  Current[0] = netAdr;
  send(Current, sizeof(Current),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
}

String getSuply(int netAdr)
{
  response[0]=0;
  Suply[0] = netAdr;
  send(Suply, sizeof(Suply),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
}


String getFreq(int netAdr)
{
  response[0]=0;
  Freq[0] = netAdr;
  send(Freq, sizeof(Freq),response);
  //String n = String((int)response[1]) + String((int)response[2]) +String((int)response[3])+ String((int)response[4]);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String fr= String(r);
  //return fr;
  if(response[0] == netAdr)   return String(response[0])+";"+fr;
  else   return String("Error");
}

String getARPower(int netAdr)
{
  response[0]=0;
  activPower[0] = netAdr;
  send(activPower, sizeof(activPower),response);
  if(response[0] == netAdr)
  {
  long r = 0;
  r |= (long)response[2]<<24;
  r |= (long)response[1]<<16;
  r |= (long)response[4]<<8;
  r |= (long)response[3];
  String A_plus= String(r);
  r=0;
  r |= (long)response[6]<<24;
  r |= (long)response[5]<<16;
  r |= (long)response[8]<<8;
  r |= (long)response[7];
  String A_minus= String(r);
  r = 0;
  r |= (long)response[10]<<24;
  r |= (long)response[9]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String R_plus= String(r);
  r = 0;
  r |= (long)response[14]<<24;
  r |= (long)response[13]<<16;
  r |= (long)response[16]<<8;
  r |= (long)response[15];
  String R_minus= String(r);
  return String(String(response[0])+";"+A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  }
  //if(response[0] == netAdr)   String(A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  else   return String("Error");
}*/
//////////////////////////////////////////////////////////////////////////////////



