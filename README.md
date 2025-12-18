🚀 Filament Durchmesser Sensor (IoT)
Dieses Projekt ermöglicht die präzise Überwachung des Filament-Durchmessers für 3D-Drucker in Echtzeit. Die Daten werden von einem ESP8266 erfasst, über ein lokales Web-Interface angezeigt und via MQTT an einen Docker-Stack zur Langzeit-Protokollierung und Visualisierung gesendet.

📋 Features
Echtzeit-Messung: Erfassung des Durchmessers über einen Hall-Effekt-basierten Sensor (via ADS1115 ADC).

Lokale Anzeige: Integriertes 8-Segment-Display zur direkten Kontrolle am Gerät.

Web-Interface: Mobile-optimierte Webseite auf dem ESP8266 zur schnellen Überprüfung im Browser.

Daten-Logging: Vollständiger Docker-Stack (InfluxDB) zur Speicherung der Messwerte.

Professionelle Visualisierung: Live-Charts und Historie über Grafana Dashboards.




💻 Software Installation
1. ESP8266 Firmware
Öffne die Datei PW_Sensor/Durchmessersensor/Durchmessersensor.ino in der Arduino IDE.

Passe in der Datei extruder_net.h deine WLAN-Zugangsdaten und die IP deines Laptops/Servers an:

C++

#define WIFI_SSID "Deine_SSID"
#define WIFI_PWD  "Dein_Passwort"
#define MQTT_BROKER "192.168.0.xxx" // Deine Laptop IP
Installiere benötigte Bibliotheken: Adafruit ADS1X15, MQTT (Joel Gaehwiler), LedController, EasyButton.



2. Server-Infrastruktur (Docker)
Der Server nutzt Docker Compose, um Mosquitto (MQTT), InfluxDB und Grafana zu starten.

Navigiere in den Ordner sensor_server/.

Starte die Container:

Bash

sudo docker compose up -d
Die Dienste sind nun erreichbar unter:

Grafana: http://localhost:3000 (Login: admin / admin)

MQTT Broker: Port 1883

📊 Grafana Konfiguration
Öffne Grafana im Browser.

Füge eine Data Source vom Typ InfluxDB hinzu:

URL: http://influxdb:8086

Database: sensors

Erstelle ein Dashboard und füge ein Panel hinzu:

Query: FROM mqtt_consumer SELECT field(value)

Visualisierung: "Time series" oder "Gauge".

🔧 Kalibrierung
Der Sensor wird über zwei Referenzpunkte kalibriert (Standard: 1.6mm und 1.9mm).

Halte den Taster am Gerät gedrückt, um in den Kalibrierungsmodus zu wechseln.

Führe die Referenzstücke ein und bestätige die Messung durch einen Tastendruck.

Die Werte werden dauerhaft im EEPROM des ESP8266 gespeichert.

📂 Projektstruktur
Plaintext

├── PW_Sensor/
│   └── Durchmessersensor/
│       ├── Durchmessersensor.ino  # Hauptprogramm (Messlogik)
│       ├── extruder_net.cpp       # WLAN & MQTT Logik
│       └── extruder_net.h         # Netzwerk-Konfiguration
└── sensor_server/
    ├── docker-compose.yml         # Server-Stack Definition
    └── telegraf.conf              # MQTT-zu-Datenbank Vermittler