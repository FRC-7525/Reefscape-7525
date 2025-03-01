#include <Adafruit_NeoPixel.h>
#include <FastLED.h>

#define DATA_PIN 6            // Data pin to first LED
#define NUM_LEDS 50      // Number of LEDs in the chain (fake)

unsigned long pulseWidth = 0;  // Store the pulse width from A0

// Defines LED state structure (ahh structs are so nice)
struct LedState {
  unsigned long minPulseWidth; // Minimum pulse width for this state
  unsigned long maxPulseWidth; // Maximum pulse width for this state
  void (*pattern)();
  const char* name;           // Name of the state for debugging
};

CRGB leds[NUM_LEDS];

int spots[] = {0, 16, 32};
const int numSpots = sizeof(spots) / sizeof(spots[0]);

// Different states
void disabled() {

}

void idle() {

}

void intaking() {

}

void scoring() {

}

void autoalign() {

}

void algae() {

}

void climbing() {
  if (spots[0] >= NUM_LEDS) spots[0] = 0;

  //Need to replace with actual values of blue and purple
  fill_gradient(leds, spots[0] - 5, CHSV(160, 255, 10), spots[0], CHSV(192, 255, 255));
  fill_gradient(leds, spots[0], CHSV(192, 255, 255), spots[0] + 5, CHSV(160, 255, 10));

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < spots[0] - 5 || i > spots[0] + 5) {
      leds[i].setHSV(160, 255, 10); //Replace with actual blue
    }
  }

  spots[0]++;
}

void l1() {
  //TODO: Figure out cool-looking gradient colors
  fill_gradient(leds, 0, CHSV(160, 255, 10), 12, CHSV(192, 255, 255));
  fill_gradient(leds, 12, CHSV(192, 255, 255), NUM_LEDS - 1, CHSV(160, 255, 10));
}

void l2() {
  //TODO: Figure out cool-looking gradient colors
  fill_gradient(leds, 0, CHSV(160, 255, 10), 24, CHSV(192, 255, 255));
  fill_gradient(leds, 24, CHSV(192, 255, 255), NUM_LEDS - 1, CHSV(160, 255, 10));
}

void l3() {
  //TODO: Figure out cool-looking gradient colors
  fill_gradient(leds, 0, CHSV(160, 255, 10), 37, CHSV(192, 255, 255));
  fill_gradient(leds, 37, CHSV(192, 255, 255), NUM_LEDS - 1, CHSV(160, 255, 10));
}

void l4() {
  //TODO: Figure out cool-looking gradient colors
  fill_gradient(leds, 0, CHSV(160, 255, 10), 49, CHSV(192, 255, 255));
  fill_gradient(leds, 49, CHSV(192, 255, 255), NUM_LEDS - 1, CHSV(160, 255, 10));
}

const LedState LED_STATES[] = {
  {0, 372, &disabled, "Disabled"},
  {373, 745, &idle, "Idle"},
  {746, 1117, &intaking, "Intaking"},
  {1118, 1489, &scoring, "Scoring"},
  {1490, 1862, &autoalign, "Autoalign"},
  {1863, 2234, &algae, "Algae"},
  {2235, 2607, &climbing, "Climbing"},
  {2608, 2979, &l1, "L1"},
  {2980, 3351, &l2, "L2"},
  {3352, 3724, &l3, "L3"},
  {3725, 4096, &l4, "L4"}
};

const int NUM_STATES = sizeof(LED_STATES) / sizeof(LED_STATES[0]);

void setup() {     
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  
  pinMode(A5, INPUT);  // Set A5 as input for PWM signal (input from rio)
  
  Serial.begin(9600);
  Serial.println("RGB LED Controller Started");
}

void loop() {
  // Read the PWM pulse width from A5
  pulseWidth = pulseIn(A5, HIGH);
  // Serial.println(pulseWidth);

  // Find the corresponding color state based on the pulse width
  for (int i = 0; i < NUM_STATES; i++) {
    if (pulseWidth >= LED_STATES[i].minPulseWidth && pulseWidth <= LED_STATES[i].maxPulseWidth) {
      // Set all LEDs to the corresponding color for this state
      LED_STATES[i].pattern();
      break;
    }
  }
  
  FastLED.show();
  delay(30); 
}