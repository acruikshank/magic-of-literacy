#include "main.h"
#include "network.h"

CRGB leds[NUM_LEDS];
uint8_t adjRainbow[256];
unsigned long updateInterval = 100000;
unsigned long lastUpdate = micros64();

const float PI2 = 2.0*PI;

const float COLLECT_ANGLE = 30.0;
const float STREAK_CONSTANT = 4.0;
int nextStreak = 0;
int streakCount = 0;
float lastTheta = 0.0;
float collectedTheta = 0.0;
float lastCollectedTheta = 0.0;
bool aboveThreshold = false;

uint8_t colorCycle = 0;

unsigned long chSelectStreakStart = micros64();
int streakDirection = 0;
int channel = 0;

int state = STATE_INIT_CHANNEL;

WiFiUDP Udp;
uint32_t myMac = 0;

void setupLights() {
    // create ranbow
  for (int i=0; i<10; i++) adjRainbow[i] = map(i,0,10,237,255);
  for (int i=10; i<43; i++) adjRainbow[i] = map(i,10,43,0,60);
  for (int i=43; i<85; i++) adjRainbow[i] = map(i,43,85,61,78);
  for (int i=85; i<127; i++) adjRainbow[i] = map(i,85,127,79,88);
  for (int i=127; i<169; i++) adjRainbow[i] = map(i,127,169,89,100);
  for (int i=169; i<211; i++) adjRainbow[i] = map(i,169,211,101,138);
  for (int i=211; i<256; i++) adjRainbow[i] = map(i,211,255,139,255);

  FastLED.addLeds<WS2811, LED_SIGNAL_PIN>(leds, NUM_LEDS);
}

void sendUpdate(WandUpdate update) {
  Udp.beginPacket("192.168.0.255", DISRUPTPort);
  Udp.write((char *) &update, sizeof(WandUpdate));
  Udp.endPacket();
  Udp.flush();
}

float angle(float x, float y) {
  if (x==0.0)
    return y >= 0.0 ? PI2 : -PI2;
  return atan(y/x);
}

float dist(float x, float y) {
  return sqrt(x*x + y*y);
}

void blink(CRGB color, int times) {
  for (int i=0; i<times; i++) {
    for (int j=0; j<NUM_LEDS; j++) {
      leds[j] = color;
    }
    FastLED.show();
    delay(250);
    for (int j=0; j<NUM_LEDS; j++) {
      leds[j] = CRGB(0,0,0);
    }
    FastLED.show();
    delay(250);
  }
}

void switchToCollect() {  
  state = STATE_COLLECT;
  lastCollectedTheta = 0.0;
  collectedTheta = 0.0;
  aboveThreshold = false;
}

void switchToReady() {
  Serial.println("READY!");
  state = STATE_READY;
  colorCycle = 0;
  streakDirection = 0; 

  WandUpdate update;
  update.state = state;
  update.channel = channel;
  sendUpdate(update);
}

void switchToPoised(Motion *motion) {
  Serial.println("POISED");
  motion->angle_pitch = 0.0;
  motion->angle_roll = 0.0;
  motion->angle_yaw = 0.0;
  motion->angle_roll_output = 0.0;
  motion->angle_yaw_output = 0.0;
  motion->angle_yaw_output = 0.0;
  state = STATE_POISED;

  WandUpdate update;
  update.state = state;
  update.channel = channel;
  sendUpdate(update);
}

void switchToTriggered() {
  Serial.println("TRIGGERED");
  state = STATE_TRIGGERED;
  lastUpdate = micros64();

  for (int i=0; i<5; i++) {
    WandUpdate update;
    update.state = state;
    update.channel = channel;
    sendUpdate(update);
    delay(100);
  }
}

void loopTriggered(Motion *motion) {
  if (micros64() - lastUpdate > TRIGGER_DURATION) {
    blink(CHSV(0,0, 40), 2);
    switchToCollect();
    return;
  }

  for (int i=0; i<NUM_LEDS; i++) {
    leds[i] = leds[i].lerp8(CHSV(0,0,0), 200);
  }
  FastLED.show();  
  delay(20);
}

void loopPoised(Motion *motion) {
  float amp = dist(motion->angle_roll_output, motion->angle_yaw_output);
 
  if (amp > TRIGGER_AMP_THRESHOLD) {
    switchToTriggered();
    return;
  }

  for (int i=0; i<NUM_LEDS; i++) {
    leds[i] = leds[i].lerp8(CHSV(0,0,240), 40);
  }
  FastLED.show();
  delay(20);
}

void loopReady(Motion *motion) {
  if (motion->acc_x < -READY_TO_POISED_THRESHOLD) {
    if (streakDirection == -1) {
      if (micros64() - chSelectStreakStart > READY_TO_POISED_DURATION) {
        switchToPoised(motion);
        return;
      }
    } else {
      chSelectStreakStart = micros64();
      streakDirection = -1;
    }
  } else {
    streakDirection = 0;
  }

  leds[0] = CHSV(colorCycle, 200, 220);
  leds[1] = CHSV((colorCycle+64)%256, 200, 220);
  leds[2] = CHSV((colorCycle+128)%256, 200, 220);
  leds[3] = CHSV((colorCycle+192)%256, 200, 220);
  FastLED.show();

  colorCycle += 5;

  delay(20);
}

void loopInitChannel(Motion *motion) {
  if (motion->acc_x > CH_SELECTION_THRESHOLD) {
    if (streakDirection == 1) {
      if (micros64() - chSelectStreakStart > CH_SELECTION_DURATION) {
        channel = 0;
        blink(CRGB(0,0,200), 3);
        switchToCollect();
        return;
      }
    } else {
      chSelectStreakStart = micros64();
      streakDirection = 1;
    }
  } else if (motion->acc_x < -CH_SELECTION_THRESHOLD) {
    if (streakDirection == -1) {
      if (micros64() - chSelectStreakStart > CH_SELECTION_DURATION) {
        channel = 1;
        blink(CRGB(200,0,0), 3);
        switchToCollect();
        return;
      }
    } else {
      chSelectStreakStart = micros64();
      streakDirection = -1;
    }
  } else {
    streakDirection = 0;
  }
}

void loopCollect(Motion *motion) {
  float theta = atan2(motion->angle_yaw_output, motion->angle_roll_output);
  float amp = dist(motion->angle_roll_output, motion->angle_yaw_output);

  if (amp > AMP_THRESHOLD) {
    if (aboveThreshold) {
      collectedTheta += theta - lastTheta;
      if (abs(theta-lastTheta) > 4.0) {
        collectedTheta += (theta > lastTheta) ? -2.0*PI : 2.0*PI;
      }
    } else {
      aboveThreshold = true;
    }
    lastTheta = theta;
  } else {
    aboveThreshold = false;
  }

  if (abs(collectedTheta) >= COLLECT_ANGLE) {
    switchToReady();
    return;
  }

  int hue = adjRainbow[constrain(map(fmod(abs(collectedTheta),PI2)*10000l, 0, 62831, 0, 255), 0, 255)];
  int v = map((long) (10000.0*abs(collectedTheta)/COLLECT_ANGLE), 0, 10000, 0, 200);
  CRGB color = CHSV(hue, 100, v);

  leds[0] = color;
  leds[1] = color;
  leds[2] = color;
  leds[3] = color;
  FastLED.show();

  if (lastUpdate < micros64() - updateInterval) {
    uint8_t translatedTheta = constrain(map((theta+PI)*10000l, 0, 62831, 0, 255), 0, 255);

    WandUpdate update;
    update.state = state;
    update.channel = channel;
    update.theta = translatedTheta;
    update.amplitude = constrain(map((long) amp, 0, 20, 0, 200), 0, 255);

    if (random(1000)*COLLECT_ANGLE <= 1000.0*STREAK_CONSTANT*abs((collectedTheta-lastCollectedTheta)*collectedTheta)) {
      update.hasStreak = true;
      update.streakColor = random(0, 256);
    } else {
      update.hasStreak = false;
    }
    lastCollectedTheta = collectedTheta;

    sendUpdate(update);
    lastUpdate = micros64();
    Serial.printf("%2.3f\n", collectedTheta);
  }

  delay(4);
}


void loop() {
  Motion *motion = computeMotion();

  switch(state) {
    case STATE_INIT_CHANNEL:
      loopInitChannel(motion); break;
    case STATE_COLLECT:
      loopCollect(motion); break;
    case STATE_READY:
      loopReady(motion); break;
    case STATE_POISED:
      loopPoised(motion); break;
    case STATE_TRIGGERED:
      loopTriggered(motion); break;
  }
}

void setup() {
  setupLights();

  for (int i=0; i<NUM_LEDS; i++)
    leds[i] = CRGB(0,20,0);
  FastLED.show();

  Serial.begin(115200);

  setupAccelerometer();
  myMac = computeMacAddress();

  // connect to wifi
  if (connectToWifi()) {
      for (int j=0; j<5; j++) {
        for (int i=0; i<NUM_LEDS; i++)
          leds[i] = CRGB(0,200,0);
        FastLED.show();
        delay(500);
        for (int i=0; i<NUM_LEDS; i++)
          leds[i] = CRGB(0,0,0);
        FastLED.show();
        delay(500);
      }

    return;
  }

  for (int i=0; i<NUM_LEDS; i++)
    leds[i] = CRGB(20,0,20);
  FastLED.show();
}
