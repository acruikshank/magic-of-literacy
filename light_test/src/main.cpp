#include <Arduino.h>
#include "FastLED.h"

#define FASTLED_ESP8266_RAW_PIN_ORDER
#define NUM_LEDS 4
#define LED_SIGNAL_PIN 15
#define COLOR_ORDER RGB

/*
Green: -18-60
Yellow: 60-78
Orange: 78-88
Red: 88-100
Purple: 100-138
Blue: 138-237
*/

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
uint8_t adjRainbow[256];

int theta = 0;
int sensorValue = 0;  // value read from the pot
int outputValue = 0;  // value to output to a PWM pin
CRGB leds[NUM_LEDS];

void setup() {
  // create ranbow
  for (int i=0; i<10; i++) adjRainbow[i] = map(i,0,10,237,255);
  for (int i=10; i<43; i++) adjRainbow[i] = map(i,10,43,0,60);
  for (int i=43; i<85; i++) adjRainbow[i] = map(i,43,85,61,78);
  for (int i=85; i<127; i++) adjRainbow[i] = map(i,85,127,79,88);
  for (int i=127; i<169; i++) adjRainbow[i] = map(i,127,169,89,100);
  for (int i=169; i<211; i++) adjRainbow[i] = map(i,169,211,101,138);
  for (int i=211; i<256; i++) adjRainbow[i] = map(i,211,255,139,255);

  FastLED.addLeds<WS2811, LED_SIGNAL_PIN>(leds, NUM_LEDS);

  // initialize serial communication at 115200
  Serial.begin(115200);
  pinMode(5, OUTPUT);
  analogWrite(5, 0);
  pinMode(4, OUTPUT);
  analogWrite(4, 0);
  pinMode(0, OUTPUT);
  analogWrite(0, 0);
  pinMode(2, OUTPUT);
  analogWrite(2, 0);
  pinMode(14, OUTPUT);
  analogWrite(14, 0);
}

void loop() {
  // read the analog in value
  // sensorValue = analogRead(analogInPin);
  
  // map it to the range of the PWM out
  // outputValue = map(sensorValue, 0, 1024, 0, 255);
  // analogWrite(5, outputValue);
  // analogWrite(4, outputValue);
  // analogWrite(0, outputValue);
  // analogWrite(2, outputValue);
  // analogWrite(14, outputValue);  

  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(adjRainbow[(theta+i*64)%256], 200, 200);
  }
  Serial.println(outputValue);
  FastLED.show();
  theta+=3;
  if (theta > 255) theta = 0;

  delay(20);
}
