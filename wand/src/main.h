#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Arduino.h>
#include "ets_sys.h"
#include "structures.h"
#include "FastLED.h"
#include "accelerometer.h"

#define FASTLED_ESP8266_RAW_PIN_ORDER
#define NUM_LEDS 4
#define LED_SIGNAL_PIN 15
#define COLOR_ORDER RGB
#define CH_SELECTION_THRESHOLD 3800
#define CH_SELECTION_DURATION 1000000
#define AMP_THRESHOLD 5.0
#define READY_TO_POISED_THRESHOLD 3000
#define READY_TO_POISED_DURATION 300000
#define TRIGGER_AMP_THRESHOLD 4.0
#define TRIGGER_DURATION 10000000

#define DISRUPTPort 50060

void setupLights();
void sendUpdate(WandUpdate update);
float angle(float x, float y);
float dist(float x, float y);
void blink(CRGB color, int times);
void switchToCollect();
void loopInitChannel(Motion *motion);
void loopCollect(Motion *motion);
bool connectToWifi();