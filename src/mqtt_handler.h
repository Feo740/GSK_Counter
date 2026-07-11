#include <Arduino.h>
#include <AsyncMqttClient.h>
// Определения для сервера MQTT
//#define MQTT_HOST IPAddress(212, 92, 170, 246) ///< адрес сервера MQTT
#define MQTT_HOST IPAddress(192, 168, 191, 141) ///< адрес сервера MQTT
#define MQTT_PORT 1883 ///< порт сервера MQTT
#define MQTT_USERNAME "feo"
#define MQTT_PASSWORD "ferrari220"
/// создаем объекты для управления MQTT-клиентом:
///Создаем объект для управления MQTT-клиентом и таймеры, которые понадобятся для повторного подключения к MQTT-брокеру или WiFi-роутеру, если связь вдруг оборвется.
extern AsyncMqttClient mqttClient;
extern TimerHandle_t mqttReconnectTimer;

void connectToMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void send_mqtt(String value, String addr, int leght);