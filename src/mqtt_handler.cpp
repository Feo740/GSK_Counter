#include "mqtt_handler.h"
#include <AsyncMqttClient.h>
#include <WiFi.h>


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer = NULL;
//Функция подключения к MQTT
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
             //  "Подключаемся к MQTT... "
  mqttClient.connect();
}

// в этом фрагменте добавляем топики,
// на которые будет подписываться ESP32:
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  // подписываем ESP32 на топики:
 uint16_t packetIdSub2 = mqttClient.subscribe("phone/Counter", 0); //топик счетчика электроэнергии
 uint16_t packetIdSub3 = mqttClient.subscribe("phone/Went", 0);//топик мотора вытяжки
 uint16_t packetIdSub4 = mqttClient.subscribe("phone/Went_valve", 0);//топик клапана вытяжки
 uint16_t packetIdSub5 = mqttClient.subscribe("phone/Zopen", 0);//топик клапана вытяжки
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
             //  "Отключились от MQTT."
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
             //  "Подписка подтверждена."
  Serial.print("  packetId: ");  //  "  ID пакета: "
  Serial.println(packetId);
  Serial.print("  qos: ");  //  "  Уровень качества обслуживания: "
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
            //  "Отписка подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

/*void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
            //  "Публикация подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

// этой функцией управляется то, что происходит
// при получении того или иного сообщения в топиках

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  }

  void send_mqtt(String value, String addr, int leght){
  char var1[leght];
  addr.toCharArray(var1,leght);
  uint16_t packetIdPub = mqttClient.publish(var1, 1, true, value.c_str());
}