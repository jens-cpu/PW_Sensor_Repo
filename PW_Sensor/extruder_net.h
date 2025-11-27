#ifndef EXTRUDER_NET_H
#define EXTRUDER_NET_H

#include <ESP8266WiFi.h>
#include <MQTT.h>

// -- WIFI and MQTT --
#define WIFI_SSID "Projektwerkstatt"
#define WIFI_PWD "VoronFTW"

#define MQTT_BROKER "192.168.0.127"
#define MQTT_PORT 1883
#define MQTT_TOPIC "/diameter"

extern WiFiClient net;
extern MQTTClient mqtt;

void connect_wifi_mqtt();
void update_mqtt(float actMM);

#endif