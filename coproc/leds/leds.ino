#include "FastLED.h"

// How many leds in your strip?
#define NUM_LEDS 144

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806, define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 51
#define CLOCK_PIN 52

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() {
    LEDS.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, 144);
    LEDS.setBrightness(255);
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
      FastLED.show();
  }
  FastLED.show();
  delay(50);

  leds[NUM_LEDS/2] = CHSV(23,255,255);
  for (int i = 0; i < NUM_LEDS/2; i++){
    leds[NUM_LEDS/2 + i] = CHSV(23,255,255);
    leds[NUM_LEDS/2 - i] = CHSV(23,255,255);
    FastLED.show();
    delay(25);
  }
  delay(1000);
  int hue = 23;
  while (hue != 180){
    for(int i=0;i<NUM_LEDS;i++)
      leds[i] = CHSV(hue,255,255);
    hue++;
    FastLED.show();
    delay(25);
  }
}

void fadeall() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].nscale8(250);
  }
}

int repeat = 60;

void loop() {
  // First slide the led in one direction
  for (int i = 0; i < NUM_LEDS; i++) {
    // Set the i'th led to red
    leds[i] = CHSV(179, 255, 255);
    fadeall();
    FastLED.show();
    delay(10);
  }
}
