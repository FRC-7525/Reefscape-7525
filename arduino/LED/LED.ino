#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include "Pattern.cpp"

#define DATA_PIN 6            // Data pin to first LED
#define NUM_LEDS 50      // Number of LEDs in the chain (fake)

unsigned long pulseWidth = 0;  // Store the pulse width from A0

// Defines LED state structure (ahh structs are so nice)
struct LedState {
  unsigned long minPulseWidth; // Minimum pulse width for this state
  unsigned long maxPulseWidth; // Maximum pulse width for this state
  Pattern pattern;
  const char* name;           // Name of the state for debugging
};

// Different states

const LedState LED_STATES[] = {
  {0, 372, new Disabled(), "Disabled"},
  {373, 745, new Idle(), "Idle"},
  {746, 1117, new Intaking(), "Intaking"},
  {1118, 1489, new Scoring(), "Scoring"},
  {1490, 1862, new Autoalign(), "Autoalign"},
  {1863, 2234, new Algae(), "Algae"},
  {2235, 2607, new Climbing(), "Climbing"},
  {2608, 2979, new L1(), "L1"},
  {2980, 3351, new L2(), "L2"},
  {3352, 3724, new L3(), "L3"},
  {3725, 4096, new L4(), "L4"}
};

const int NUM_STATES = sizeof(LED_STATES) / sizeof(LED_STATES[0]);

CRGB leds[NUM_LEDS];

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
      LED_STATES[i].pattern.run(leds);
      break;
    }
  }

  delay(30); 
}