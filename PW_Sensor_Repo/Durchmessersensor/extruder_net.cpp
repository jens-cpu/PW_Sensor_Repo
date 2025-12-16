#include "extruder_net.h"

void handleMessage(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

}

void connect_wifi_mqtt() {
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  Serial.print("looking for wifi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000); // 1s
  }

  Serial.print("connected to " + WiFi.SSID() + " with IP: ");
  Serial.println(WiFi.localIP());

  mqtt.begin(MQTT_BROKER, MQTT_PORT, net);
  mqtt.onMessage(handleMessage);
}

void update_mqtt(float actMM) {
    char diameter[12];

    // TODO convert to string
    dtostrf(actMM, 4, 4, diameter);  // (float, min width, precision, buffer)

    mqtt.publish(MQTT_TOPIC, diameter);
    //Serial.print("mqtt diameter: ");
    //Serial.println(diameter);

  }