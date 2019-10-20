#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Arduino.h>
#include "Wire.h"
#include "FastLED.h"
#include "structures.h"

#define FASTLED_ESP8266_RAW_PIN_ORDER
#define NUM_LEDS 50
#define LED_SIGNAL_PIN 15
#define COLOR_ORDER RGB
#define INFOPort 50050
#define UPDATE_PORT 50060
#define STRANDS 8
#define WIFI_TIMEOUT 30000
#define INFO_TIMEOUT 10000

/*
TODO:
- Rainbow effect
- Info
- channel selection
- fluid motion effect
*/

#define MOTION_FADE 500000
#define MOTION_BRIGHT 120

CRGB leds[NUM_LEDS];
uint8_t motionBright[NUM_LEDS];
int8_t flow[NUM_LEDS];

#define NUM_STREAKS 100
#define STREAK_DELAY 1500000
#define STREAK_VELOCITY 0.003
typedef struct Streak {
  uint8_t color;
  uint8_t position;
  unsigned long added;
} Streak;

Streak streaks[NUM_STREAKS];
uint16_t streakStart = 0;
uint16_t streakCount = 0;

/* Set these to your desired AP credentials. */
const char* ssid     = "anthazoa";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "chaartdev";     // The password of the Wi-Fi network

typedef struct Location {
  float x;
  float y;
} Location;

typedef struct Strand {
  uint32_t macAddress;
  Location location;
} Strand;

typedef struct Info {
  Strand distanceTable[STRANDS];
  float streakVelocity;
} Info;

WandUpdate update;

WiFiUDP Udp;

#define FLASH_DURATION 250000
uint32_t myMac = 0;
Info systemInfo;
Location myLocation;

uint8_t currentTheta = 0;
long unsigned thetaUpdate = micros64();
uint8_t lastTheta = 0;
long unsigned previousUpdate = micros64();

uint8_t state;
unsigned long triggerStart = micros64();
uint32_t theta = 0;

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

  delay(2000); // ap delay

  return false;
}

void readUpdate() {
  int packetSize = Udp.parsePacket();
  if (packetSize == 0) {
    return;
  }

  Udp.read((char *) &update, sizeof(WandUpdate));
  unsigned long now = micros64();
  if (update.hasStreak && streakCount < NUM_STREAKS) {
    Serial.println("has streak");
    Streak *streak = &streaks[(streakStart+streakCount)%NUM_STREAKS];
    streak->color = update.streakColor;
    streak->position = map(update.theta,0,255,0,NUM_LEDS-1);
    streak->added = now;
    streakCount = (streakCount+1)%NUM_STREAKS;
  }

  lastTheta = currentTheta;
  previousUpdate = thetaUpdate;
  currentTheta = update.theta;
  thetaUpdate = now;

  if (state != update.state) {
    Serial.printf("New state %d\n", update.state);
    state = update.state;  
    triggerStart = micros64();
  }
}

void setup() {
  FastLED.addLeds<WS2811, LED_SIGNAL_PIN>(leds, NUM_LEDS);

  update.theta = 0;
  update.amplitude = 0;

  // default info
  systemInfo.streakVelocity = STREAK_VELOCITY;

  Serial.begin(115200);

  myMac = computeMacAddress();

  // connect to wifi
  if (connectToWifi()) {
    return;
  }

  // start listening to updates
  Udp.begin(UPDATE_PORT);
}

void loopWaiting() {
  Serial.println("WAITING");
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = leds[i].lerp8(CRGB(0,0,0), 254);
  }

  FastLED.show();
}

void loopCollecting() {
  unsigned long now = micros64();
  // if (streakCount < NUM_STREAKS && random(10000) < 100) {
  //   Streak *streak = &streaks[(streakStart+streakCount)%NUM_STREAKS];
  //   streak->color = random(256);
  //   streak->position = random(NUM_LEDS);
  //   streak->added = now;
  //   streakCount = (streakCount+1)%NUM_STREAKS;
  // }

  // base layer is darkish white noise
  for(int i = 0; i < NUM_LEDS; i++) {
    uint8_t noise = inoise8((i+1)*73, millis() >> 2);
    uint32_t n2 = noise*noise;
    uint8_t amp = (n2*n2)>>28;
    leds[i] = CRGB(amp, amp, amp);
    // leds[i] = i==thetaIndex ? CHSV(0, 0, update.amplitude) : leds[i].nscale8(250);
  }

  // layer white motion on top
  uint16_t startTheta, endTheta;
  unsigned long startTime, endTime;
  if (currentTheta > lastTheta) {
    if (currentTheta > 128 && lastTheta < 128) {
      startTheta = currentTheta; endTheta = lastTheta + 256; startTime = thetaUpdate; endTime = previousUpdate;
    } else {
      startTheta = lastTheta; endTheta = currentTheta; startTime = previousUpdate; endTime = thetaUpdate;
    }
  } else {
    if (lastTheta > 128 && currentTheta < 128) {
      startTheta = lastTheta; endTheta = currentTheta + 256; startTime = previousUpdate; endTime = thetaUpdate;
    } else {
      startTheta = currentTheta; endTheta = lastTheta; startTime = thetaUpdate; endTime = previousUpdate;
    }
  }
  uint8_t startLight = map(startTheta, 0, 255, 0, NUM_LEDS-1);
  uint8_t endLight = map(endTheta, 0, 255, 0, NUM_LEDS-1);
  uint8_t startAmp = map(now-startTime, 0, MOTION_FADE, 255, 0);
  uint8_t endAmp = map(now-endTime, 0, MOTION_FADE, 255, 0);
  for (int i=startLight; i<=endLight; i++) {
    uint8_t amp = constrain(map(i, startLight, endLight, startAmp, endAmp), 0, 255);
    if (endLight > startLight)
      amp /= endLight-startLight;
    leds[i%NUM_LEDS] = leds[i%NUM_LEDS].lerp8(CRGB(MOTION_BRIGHT, MOTION_BRIGHT, MOTION_BRIGHT), amp);
  }

  // add streaks
  int deleted = 0;
  for (int i=0; i<streakCount; i++) {
    Streak *streak = &streaks[(streakStart+i)%NUM_STREAKS];
    if (streak->added + STREAK_DELAY < now) {
      deleted++;
      continue;
    }
    uint16_t amp = map(now - streak->added, 0, STREAK_DELAY, 255, 0);
    amp = (amp*amp) >> 8;
    leds[streak->position] = leds[streak->position].lerp8(CHSV(streak->color, 255, 255), amp);
  }
  streakCount -= deleted;  
  streakStart = (streakStart+deleted) % NUM_STREAKS;

  FastLED.show();
}

void loopTriggered() {
  uint16_t flash = map(constrain(micros64()-triggerStart, 0, FLASH_DURATION), 0, FLASH_DURATION, 255, 0);
  CRGB flashColor = CRGB(200,200,200);
  flash = (flash*flash) >> 8;
  uint32_t m = 40;
  uint32_t cycleSpace = m * NUM_LEDS;

  for (int i=0; i<NUM_LEDS; i++) {
    CRGB color = CHSV(map((i*m+theta)%cycleSpace, 0, cycleSpace, 0, 255), 255, 200);
    leds[i] = color.lerp8(flashColor, flash);
  }
  theta += 1;
  FastLED.show();
}

void loop() {
  readUpdate();

  switch (state) {
    case STATE_COLLECT: 
      return loopCollecting();
    case STATE_READY: 
    case STATE_POISED:
      return loopWaiting();
    case STATE_TRIGGERED:
      return loopTriggered();
  }

  delay(10);
}
