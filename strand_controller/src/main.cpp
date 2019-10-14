#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "FastLED.h"
#include "structures.h"

#define FASTLED_ESP8266_RAW_PIN_ORDER
#define NUM_LEDS 50
#define LED_SIGNAL_PIN 15
#define COLOR_ORDER RGB
#define INFOPort 50050
#define UPDATE_PORT 50060
#define BUCKETS 100
#define STRANDS 16
#define WIFI_TIMEOUT 30000
#define INFO_TIMEOUT 10000
#define DISRUPTIONS 15

CRGB leds[NUM_LEDS];
MPU6050 accelgyro;

/* Set these to your desired AP credentials. */
const char* ssid     = "anthazoa";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "chaartdev";     // The password of the Wi-Fi network

#define BUCKET_WEIGHT 0.01
#define DAMPNING .95
#define INERTIA .10
#define SCALE 20
#define GAIN .008
#define MIN_HUE 140.0
#define MAX_HUE 0.0
#define SCINT_AMP 10.0
#define SCINT_T_FREQ .008
#define SCINT_S_FREQ 1.2
#define MAX_DISTURBANCE 1200.0
#define MIN_BRIGHTNESS 50.0
#define MAX_BRIGHTNESS 255.0
#define MIN_DISRUPTION 600.0
#define DISRUPTION_DELAY 5000
#define DISRUPT_VELOCITY 0.003
#define DISRUPT_AMP 1.3
#define DISRUPT_DAMPING 0.4
#define DISRUPT_FREQ 2.0
#define DISRUPT_DONE 10000


typedef struct Location {
  float x;
  float y;
} Location;

typedef struct Strand {
  uint32_t macAddress;
  Location location;
} Strand;

typedef struct Disruption {
  uint8_t level;
  uint32_t from;
  long time;
  float distance;
} Disruption;

typedef struct Info {
  Strand distanceTable[STRANDS];
  float bucketWeight;
  float dampning;
  float inertia;
  float scale;
  float gain;
  float minHue;
  float maxHue;
  float scintAmp;
  float scintTFreq;
  float scintSFreq;
  double maxDisturbance;
  float minBrightness;
  float maxBrightness;
  float minDisruption;
  uint32_t disruptionDelay;
  float disruptVelocity;
  float disruptAmp;
  float disruptDamping;
  float disruptFreq;
  uint32_t disruptDone;
} Info;

WandUpdate update;

WiFiUDP Udp;

uint32_t myMac = 0;
Info systemInfo;
Location myLocation;

float samples[BUCKETS];
int avgPtr = 0;
float avg = 500.0;
double instDisturbance = 0.0;
double disturbance = 0.0;
Disruption disruptions[DISRUPTIONS];
int nextDisruption = 0;
long lastDisruption = 0;

float distance(Location loc1, Location loc2) {
  float dx = loc1.x - loc2.x;
  float dy = loc1.y - loc2.y;
  return sqrt(dx*dx + dy*dy);
}

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
  Serial.print(ssid); Serial.println(" ...");

  long waitStart = millis();

  int i = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - waitStart < WIFI_TIMEOUT) { // Wait for the Wi-Fi to connect
    delay(1000);
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

  delay(4000); // ap delay

  return false;
}

void readUpdate() {
  int packetSize = Udp.parsePacket();
  if (packetSize == 0) {
    return;
  }

  Udp.read((char *) &update, sizeof(WandUpdate));

  Serial.printf("Update: %d %d\n", update.theta, update.amplitude);
  // use our own clock time
  // disruption.time = millis();

}

void setup() {
  FastLED.addLeds<WS2811, LED_SIGNAL_PIN>(leds, NUM_LEDS);

  update.theta = 0;
  update.amplitude = 0;

  // default info
  systemInfo.bucketWeight = BUCKET_WEIGHT;
  systemInfo.dampning = DAMPNING;
  systemInfo.inertia = INERTIA;
  systemInfo.scale = SCALE;
  systemInfo.gain = GAIN;
  systemInfo.minHue = MIN_HUE;
  systemInfo.maxHue = MAX_HUE;
  systemInfo.scintAmp = SCINT_AMP;
  systemInfo.scintTFreq = SCINT_T_FREQ;
  systemInfo.scintSFreq = SCINT_S_FREQ;
  systemInfo.maxDisturbance = MAX_DISTURBANCE;
  systemInfo.minBrightness = MIN_BRIGHTNESS;
  systemInfo.maxBrightness = MAX_BRIGHTNESS;
  systemInfo.minDisruption = MIN_DISRUPTION;
  systemInfo.disruptionDelay = DISRUPTION_DELAY;
  systemInfo.disruptVelocity = DISRUPT_VELOCITY;
  systemInfo.disruptAmp = DISRUPT_AMP;
  systemInfo.disruptDamping = DISRUPT_DAMPING;
  systemInfo.disruptFreq = DISRUPT_FREQ;
  systemInfo.disruptDone = DISRUPT_DONE;

  Serial.begin(115200);
  delay(1000); // serial delay

  myMac = computeMacAddress();

  // connect to wifi
  if (connectToWifi()) {
    return;
  }

  // start listening to updates
  Udp.begin(UPDATE_PORT);
}

void loop() {
  readUpdate();

  int thetaIndex = map(update.theta, 0, 255, 0, NUM_LEDS-1);
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = i==thetaIndex ? CHSV(0, 0, update.amplitude) : leds[i].nscale8(250);
  }
  FastLED.show();

  delay(10); // to fast might crash terminals
}
