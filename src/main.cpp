#include <Arduino.h>
#include <BLEDevice.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>

// provides the PRIx64 macro
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "config.h"

WiFiClient client;
PubSubClient mqtt(client);
BLEScan* scanner;
String station;
char clientid[20];

const char mqtt_host[] = MQTT_HOST;
int mqtt_port = MQTT_PORT;
const char mqtt_topic[] = MQTT_TOPIC;

bool scanning = false;
volatile int publish_ok = 0;
volatile int publish_fail = 0;
time_t last_ok = 0;

String hexlify(const uint8_t bytes[], int len)
{
  String output;
  output.reserve(len*2);
  for (int i=0; i<len; i++) {
    char hex[3];
    sprintf(hex, "%02x", bytes[i]);
    output.concat(hex);
  }
  return output;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getPayloadLength() == 0) {
      return;
    }
    StaticJsonDocument<512> doc;
    doc["millis"] = millis();
    doc["station"] = station;
    std::string address = advertisedDevice.getAddress().toString().c_str();
    doc["address"] = address;
    doc["payload"] = hexlify(advertisedDevice.getPayload(), advertisedDevice.getPayloadLength());
    if (advertisedDevice.haveRSSI()) {
      doc["rssi"] = advertisedDevice.getRSSI();
    }
    String json;
    serializeJson(doc, json);
    if (mqtt.connected()) {
      if (mqtt.publish(mqtt_topic, json.c_str())) {
        publish_ok++;
        last_ok = millis();
      } else {
        publish_fail++;
      }
    }
  }
};

void scan_complete(BLEScanResults results) {
  Serial.print("ok=");
  Serial.print(publish_ok, DEC);
  Serial.print(" fail=");
  Serial.println(publish_fail, DEC);

  scanner->clearResults();

  scanning = false;
  publish_ok = 0;
  publish_fail = 0;
}

void setup() {
  snprintf(clientid, sizeof(clientid), "esp%" PRIx64, ESP.getEfuseMac());

  Serial.begin(115200);
  Serial.println();
  Serial.print(ESP.getSketchMD5());
  Serial.print(" ");
  Serial.println(clientid);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.waitForConnectResult();

  mqtt.setServer(mqtt_host, mqtt_port);

  BLEDevice::init("");
  station = BLEDevice::getAddress().toString().c_str();

  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  scanner->setActiveScan(false);
  scanner->setInterval(100);
  scanner->setWindow(99);
}

void loop() {
  static time_t last_reconnect_check = 0;

  if (millis() - last_ok > WATCHDOG_TIMEOUT) {
    Serial.println("application watchdog triggered");
    ESP.restart();
  }

  if (millis() - last_reconnect_check > 1000) {
    if (!mqtt.connected()) {
      Serial.println("reconnecting mqtt");
      mqtt.connect(clientid);
    }
    last_reconnect_check = millis();
  }

  mqtt.loop();

  if (!scanning) {
    scanner->start(31, scan_complete, false);
    scanning = true;
  }
}
