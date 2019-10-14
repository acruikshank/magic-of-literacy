#include <ESP8266WiFi.h>
#include "network.h"

/* Set these to your desired AP credentials. */
const char* ssid     = "anthazoa";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "chaartdev";     // The password of the Wi-Fi network

uint32_t computeMacAddress() {
  uint32_t mac = 0;

  // determine macAddress as a 32 bit int
  uint8_t macAddress[6] = {0};

  WiFi.macAddress(&macAddress[0]);
  for (int i=0; i<6; i++) {
    Serial.printf("%d:", macAddress[i]);
  }
  Serial.println();

  for(int i=3; i < 6; i++) {
    mac |= (macAddress[i] << (8*(5-i)));
  }
  Serial.printf("mac: %x\n", mac);
  return mac;
}

bool connectToWifi() {
  WiFi.begin(ssid, password);             // Connect to the network

  Serial.print("Connecting to ");
  Serial.print(ssid); 
  Serial.println(" ...");

  long waitStart = millis();

  int i = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - waitStart < WIFI_TIMEOUT) { // Wait for the Wi-Fi to connect
    delay(500);
    Serial.print(++i); Serial.print(' ');
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Timeout connecting to WiFi");
    return true;
  }

  Serial.println('\n');
  Serial.println("Connection established!");
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  return false;
}

