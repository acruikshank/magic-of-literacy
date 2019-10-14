#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "FastLED.h"

#define FASTLED_ESP8266_RAW_PIN_ORDER
#define NUM_LEDS 50
#define PIN D8
#define COLOR_ORDER RGB
#define INFOPort 50050
#define DISRUPTPort 50060
#define BUCKETS 100
#define STRANDS 16
#define WIFI_TIMEOUT 30000
#define INFO_TIMEOUT 10000
#define DISRUPTIONS 15
#define ACC_RESET D5

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

void setupAccelerometer() {
  digitalWrite(ACC_RESET, LOW);
  delay(10);
  digitalWrite(ACC_RESET, HIGH);
  delay(100);

  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  delay(100);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
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

bool retrieveInfoPacket() {
  Udp.begin(INFOPort);
  long waitStart = millis();
  int packetSize = 0;
  for (packetSize = Udp.parsePacket(); packetSize == 0 && millis() - waitStart < INFO_TIMEOUT; packetSize = Udp.parsePacket()) {
    delay(10);
  }

  if (packetSize == 0) {
    Serial.println("Info Timeout");
    return true;
  }

  Udp.read((char *) &systemInfo, sizeof(Info));
  Udp.stop();

  for (int i=0; i<STRANDS; i++) {
    if (myMac == systemInfo.distanceTable[i].macAddress) {
      myLocation = systemInfo.distanceTable[i].location;
      Serial.printf("my location: %f, %f\n", systemInfo.distanceTable[i].location.x, systemInfo.distanceTable[i].location.y);
    }
  }
  for (int i=0; i<STRANDS; i++) {
    Serial.printf("%6x: %f, %f d: %f\n", systemInfo.distanceTable[i].macAddress, systemInfo.distanceTable[i].location.x,
                  systemInfo.distanceTable[i].location.y, distance(myLocation, systemInfo.distanceTable[i].location));
  }
  Serial.println();

  Serial.printf("bucketWeight: %f\n", systemInfo.bucketWeight);
  Serial.printf("dampning: %f\n", systemInfo.dampning);
  Serial.printf("inertia: %f\n", systemInfo.inertia);
  Serial.printf("scale: %f\n", systemInfo.scale);
  Serial.printf("gain: %f\n", systemInfo.gain);
  Serial.printf("minHue: %f\n", systemInfo.minHue);
  Serial.printf("maxHue: %f\n", systemInfo.maxHue);
  Serial.printf("scintAmp: %f\n", systemInfo.scintAmp);
  Serial.printf("scintTFreq: %f\n", systemInfo.scintTFreq);
  Serial.printf("scintSFreq: %f\n", systemInfo.scintSFreq);
  Serial.printf("maxDisturbance: %f\n", systemInfo.maxDisturbance);
  Serial.printf("minBrightness: %f\n", systemInfo.minBrightness);
  Serial.printf("maxBrightness: %f\n", systemInfo.maxBrightness);
  Serial.printf("minDisruption: %f\n", systemInfo.minDisruption);
  Serial.printf("disruptionDelay: %d\n", systemInfo.disruptionDelay);
  Serial.printf("disruptVelocity: %f\n", systemInfo.disruptVelocity);
  Serial.printf("disruptAmp: %f\n", systemInfo.disruptAmp);
  Serial.printf("disruptDamping: %f\n", systemInfo.disruptDamping);
  Serial.printf("disruptFreq: %f\n", systemInfo.disruptFreq);
  Serial.printf("disruptDone: %d\n", systemInfo.disruptDone);


  return false;
}

void readDisruption() {
  int packetSize = Udp.parsePacket();
  if (packetSize == 0) {
    return;
  }

  Disruption disruption;
  Udp.read((char *) &disruption, sizeof(disruption));

  // ignore our own disruptions
  if (disruption.from == myMac) return;

  // use our own clock time
  disruption.time = millis();

  // identify sender and compute distance
  for (int i=0; i<STRANDS; i++) {
    if (disruption.from == systemInfo.distanceTable[i].macAddress) {
      disruption.distance = distance(myLocation, systemInfo.distanceTable[i].location);
      break;
    }
  }

  // add to ring buffer
  disruptions[(nextDisruption++) % DISRUPTIONS] = disruption;

  // for (int i=0; i<DISRUPTIONS; i++) {
  //   Serial.printf("disrupt[%d]: t: %d, l: %d, f: %6x\n", i, disruptions[i].time, disruptions[i].level, disruptions[i].from);
  // }
}

void maybeSendDisruption() {
  if (disturbance < systemInfo.minDisruption) return;
  if (millis() - lastDisruption < systemInfo.disruptionDelay) return;
  lastDisruption = millis();

  Disruption disruption;
  disruption.from = myMac;
  disruption.time = 0;
  disruption.level = (uint8_t) ((255*min(systemInfo.maxDisturbance, disturbance)) / systemInfo.maxDisturbance);
  Udp.beginPacket("192.168.0.255", DISRUPTPort);
  delay(5);
  Udp.write((char *) &disruption, sizeof(Disruption));
  Udp.endPacket();
  Udp.flush();
}

void setup() {
  FastLED.addLeds<WS2811, PIN>(leds, NUM_LEDS);

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

  pinMode(0, INPUT);
  pinMode(ACC_RESET, OUTPUT);
  digitalWrite(ACC_RESET, HIGH);
  for (int i=0; i<BUCKETS; i++) samples[i] = 500;
  for (int i=0; i<DISRUPTIONS; i++) {
    disruptions[i].time = 0;
    disruptions[i].level = 0;
  }

  setupAccelerometer();
  myMac = computeMacAddress();

  // connect to wifi
  if (connectToWifi()) {
    return;
  }

  if (retrieveInfoPacket()) {
    return;
  }

  // start listening to disruptions
  Udp.begin(DISRUPTPort);
}

float dampedSin(long time, float distance) {
  float x = systemInfo.disruptVelocity*time - distance;
  return x > 0 ? -systemInfo.disruptAmp*exp(-systemInfo.disruptDamping*x)*sin(systemInfo.disruptFreq*x) : 0;
}

void loop() {
  readDisruption();

  if (!accelgyro.testConnection()) {
    setupAccelerometer();
  }

  int16_t ax, ay, az, gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float nextSample = systemInfo.gain * max(az,ay);
  instDisturbance *= systemInfo.dampning;
  instDisturbance += abs(nextSample - avg);
  disturbance += systemInfo.inertia*(instDisturbance - disturbance);

  maybeSendDisruption();

  avg += (nextSample - samples[avgPtr])*systemInfo.bucketWeight;
  samples[avgPtr] = nextSample;
  avgPtr = (avgPtr + 1) % BUCKETS;

  double cDisturbance = min(systemInfo.maxDisturbance, max(0.0, disturbance)) / systemInfo.maxDisturbance;
  uint8_t brightness = (uint8_t) (systemInfo.minBrightness + sqrt(cDisturbance) * (systemInfo.maxBrightness - systemInfo.minBrightness));

  float disruptEffect = 0.0;
  long now = millis();
  for (int i=0; i<DISRUPTIONS; i++) {
    if (now - disruptions[i].time < systemInfo.disruptDone) {
      disruptEffect += dampedSin(now - disruptions[i].time, disruptions[i].distance);
    }
  }

  if (disruptEffect > 0.0) {
    brightness += (uint8_t) (disruptEffect*(255-brightness));
  } else {
    brightness = (uint8_t) ((1.0-disruptEffect)*brightness);
  }

  for(int i = 0; i < NUM_LEDS; i++) {
    double scintHue = systemInfo.minHue + systemInfo.scintAmp * sin(systemInfo.scintTFreq*millis() + systemInfo.scintSFreq*i);
    uint8_t hue = (uint8_t) (scintHue + cDisturbance * (systemInfo.maxHue - systemInfo.minHue));
    leds[i] = CHSV(hue, 255, brightness);
  }
  FastLED.show();

  delay(10); // to fast might crash terminals
}
