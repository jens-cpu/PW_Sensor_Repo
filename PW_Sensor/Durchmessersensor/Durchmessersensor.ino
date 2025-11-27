/*
  Firmware for the hal-sensor based diameter sensor.
  Written with Arduino IDE 2.3.4.
  
  Board Framework: esp8266 by ESP8266 Community 3.1.2

  Board: LOLIN(WEMOS) D1 mini Pro

	Libraries:
  - Adafruit ADS1X15 by Adafruit 2.5.0
  - MQTT by Joel Gaehwiler 2.5.2
  - LedController by Noa Sakurajin 2.0.2
  - EasyButton by Evert Arias 2.0.3
*/

#include <Arduino.h>

#include "EasyButton.h"       // version 2.0.3
#include "LedController.hpp"  // version 2.0.2
#include <Wire.h>             // is needed by Adafruit
#include <Adafruit_ADS1X15.h> // version 2.5.0
#include "EEPROM.h"

#include "extruder_net.h"

// -- Button --
#define BUTTON_PIN D3
#define DURATION 2000  // time for long press [ms]
#define DEBOUNCE 40

// -- LCD --
#define DIN_PIN D7
#define CLK_PIN D5
#define CS_PIN D4 // although D8 is the CS PIN in pinout diagrams, it does not work
#define SEGMENTS 8  
#define delayTime 200

// -- Sensor --
#define SENSOR_PIN_1 D1
#define SENSOR_PIN_2 D2
#define MEASUREMENTS 50 
#define NUM_DIAMETERS 2


enum States {act_mm, act_sens, calib0, calib0_change, calib1, calib1_change};

// -- WIFI and MQTT --
WiFiClient net;
MQTTClient mqtt;

unsigned long lastMqttUpdate = 0;


// -- Controll --
EasyButton button(BUTTON_PIN, DEBOUNCE);
States state;

// -- LCD --
LedController<SEGMENTS,1> lc = LedController<SEGMENTS, 1>();

// -- Measurement --
const float calibDiameters[NUM_DIAMETERS] = {1.6, 1.9};
uint16_t calibValues[NUM_DIAMETERS];

uint32_t values[MEASUREMENTS];
uint8_t  values_idx = 0;
uint16_t actValue = 0;
float actMM = -1;
Adafruit_ADS1115 ads1115;


uint16_t current_reading()
{
    uint32_t sum_values = 0;
  for (int i = 0; i < MEASUREMENTS; i++)
  {
    sum_values += values[i];
  }

  return sum_values / (MEASUREMENTS*5*2);
}

void takeMeasurement() {
  uint32_t sum = 0;
  uint16_t tmp = 0;
  for(int i = 0; i < 5; i++) {
    tmp = ads1115.readADC_SingleEnded(0);
    //Serial.println("analog read sensor 1 " + String(tmp));
    sum += tmp;
    tmp = ads1115.readADC_SingleEnded(1);
    //Serial.println("analog read sensor 2 " + String(tmp));
    sum += tmp;
  }
  values[values_idx] = sum;
  values_idx = (values_idx + 1) % MEASUREMENTS;
  // Serial.println("take_measurement " + String(actValue));
}

void calibrate(int i) {
  takeMeasurement();
  calibValues[i] = current_reading();
  EEPROM.begin(sizeof(uint16_t) * 2);
  EEPROM.put(sizeof(uint16_t)*i, calibValues[i]);
  EEPROM.commit();
  EEPROM.end();
  Serial.printf("Saving EEPROM calibration value %d: %d", i, calibValues[i]);
}

void clicked() {
  //Serial.println("clicked");
  switch(state) {
    case act_mm:
      state = act_sens;
      break;
    case act_sens:
      state = calib0;
      break;
        case calib0:
        state = calib1;
      break;
    case calib0_change:
      calibrate(0);
      state = calib0;
      break;
    case calib1:
      state = act_mm;
      break;
    case calib1_change:
      calibrate(1);
      state = calib1;
      break;
    default:
      state = act_mm;
      break;
  }
}

void pressed() {
  switch(state) {
    case calib0:
      state = calib0_change;
      break;
    case calib1:
      state = calib1_change;
      break;
    default:
      break;
    }
}

void displayText(String txt) {
  bool dot = false;
  int pos = SEGMENTS - 1; // starts at SEGMENTS ends at 0
  unsigned int len = SEGMENTS;
  for (unsigned int i = 0; i < len; i++) 
  {
    if (i < txt.length())
    {
      if (dot == true)
      { // skip this character in the source string, we already set it in the last iteration
        dot = false;
        continue;
      }
      if (i+1 < txt.length() && i+1 < len && txt[i+1] == '.')
      {
        dot = true; // activate the dot for the current character and make the loop skip it in the next iteration
        len++; // go one character further, because the dot takes no character on the display
      }
    

      lc.setChar(0, pos, txt[i], dot);
    }
    else // pad with space
    {
      lc.setChar(0, pos, ' ', false);
    }
    pos--; // go one position further on the display
  }

  // fill the rest with blanks
  for (int i = pos; i < SEGMENTS; i++)
    lc.setChar(0, pos, ' ', false);
}

void printLcd() {
  switch(state) {
    case act_mm:
      displayText("D  " + String(actMM, 3));
      break;
    case act_sens:
      displayText("R " + String(current_reading()));
      break;
    case calib0:
      displayText(String(calibDiameters[0], 1) + " " + String(calibValues[0]));
      break;
    case calib0_change:
      displayText(String(calibDiameters[0], 1) + " ----");
      break;
    case calib1:
      displayText(String(calibDiameters[1], 1) + " " + String(calibValues[1]));
      break;
    case calib1_change:
      displayText(String(calibDiameters[1], 1) + " ----");
      break;
    default:
      break;
  }      
}



void convert() {
  float A = ((float) calibValues[1] - (float) calibValues[0]) / ((float) calibDiameters[1] - (float) calibDiameters[0]); // steigung

  float B = (float) calibValues[0] - A * (float) calibDiameters[0];
  actMM = ((float) current_reading() - B) / A;
  if (actMM < 1.0 || actMM > 2.5) actMM = -1;
}

void setup() {
  Serial.begin(115200);
  state  = act_mm;

  Serial.println();Serial.println();

  Serial.println("Initializing ADS1115 sensor.");

  // -- Sensor --
  if (!ads1115.begin()) {Serial.println("Failed to initialize ADS.");}

  // ads1115.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV (default)
  // ads1115.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
  ads1115.setGain(GAIN_TWO);     // 2x gain   +/- 2.048V  1 bit = 1mV
  // ads1115.setGain(GAIN_FOUR);    // 4x gain   +/- 1.024V  1 bit = 0.5mV
  // ads1115.setGain(GAIN_EIGHT);   // 8x gain   +/- 0.512V  1 bit = 0.25mV
  // ads1115.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV

  ads1115.setDataRate(RATE_ADS1115_860SPS);

  Wire.setClock(400000); // fast mode I2C

  //pinMode(CS_PIN, );
  //elay(100);
digitalWrite(CS_PIN, 0);
pinMode(CS_PIN, OUTPUT);
delay(100);
pinMode(CS_PIN, INPUT);
  // -- LCD --
  Serial.println("Initializing LCD.");
  auto conf = sakurajin::controller_configuration<SEGMENTS, 1>();
  conf.useHardwareSpi = true;
  conf.SPI_CS = CS_PIN;
  conf.SPI_MOSI = DIN_PIN;
  conf.SPI_CLK = CLK_PIN;
  //conf.onlySendOnChange = true;
  //conf.spiTransferSpeed = 10000000;
  conf.debug_output = false;
  if (!conf.isValid()) {Serial.println("  -> MAX7219 configuration is not valid");}
  lc.init(conf);

  lc.setIntensity(2); 
  displayText("01234567");
  delay(500);

  Serial.println("Initializing Button.");

  // -- Button --
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.begin();
  button.onPressed(clicked);
  button.onPressedFor(DURATION, pressed);

  EEPROM.begin(sizeof(uint16_t) * 2);
  EEPROM.get(sizeof(uint16_t)*0, calibValues[0]);
  EEPROM.get(sizeof(uint16_t)*1, calibValues[1]);
  EEPROM.end();

  Serial.printf("reading EEPROM calibration values: %d %d\n", calibValues[0], calibValues[1]);

  Serial.println("Connection WiFi for MQTT.");

  displayText("UIFI....");

  connect_wifi_mqtt();

  Serial.println("Setup done.");
}

void loop()
  {
  static unsigned long update_display = millis();
  //unsigned long start;
  //unsigned long end;

  //start = micros();
  button.read();
  //end = micros();
  //Serial.println("button.read(): " + String(end-start));

  //start = micros();
  takeMeasurement();
  //end = micros();
  //Serial.println("takeMeasurement(): " + String(end-start));

  //start = micros();

  //end = micros();
  //Serial.println("convert() " + String(end-start));

  //start = micros();
  if (millis() - update_display > 200)
  {
    convert();
    printLcd();
    update_display = millis();

    /* 
    Log to serial console in CSV-Format.
    You can record the stream with something likt
    "picocom --imap lfcrlf -b 115200 -g logfile /dev/ttyUSB0"
    */
    Serial.printf("%1.1f,%1.3f\n", millis() / 1000.0f, actMM);
  }
  //end = micros();
  //Serial.println("printLcd(): " + String(end-start));
  
  //Serial.println("---");

  if (millis() - lastMqttUpdate > 1000) {
    update_mqtt(actMM);
    lastMqttUpdate = millis();
  }
}

