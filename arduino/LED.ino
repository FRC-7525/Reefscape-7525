#include <Adafruit_NeoPixel.h>

#define PIN 6            // Data pin to first LED
#define NUM_LEDS 50      // Number of LEDs in the chain (fake)

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

unsigned long pulseWidth = 0;  // Store the pulse width from A0
unsigned long previousPulseWidth = 0; // Store the previous pulse width from A0 (lil optimization)

// Defines LED state structure (ahh structs are so nice)
struct LedState {
  unsigned long minPulseWidth; // Minimum pulse width for this state
  unsigned long maxPulseWidth; // Maximum pulse width for this state
  const char* name;           // Name of the state for debugging
};

// Different color states, 500 micro second thresholds bc PWM is NOT trustworthy (I think, idrk)

const LedState LED_STATES[] = {
  {0, 372, "Disabled"},
  {373, 745, "Idle"},
  {746, 1117, "Intaking"},
  {1118, 1489, "Scoring"},
  {1490, 1862, "Autoalign"},
  {1863, 2234, "Algae"},
  {2235, 2607, "Climbing"},
  {2608, 2979, "L1"},
  {2980, 3351, "L2"},
  {3352, 3724, "L3"},
  {3725, 4096, "L4"}
};

const int NUM_STATES = sizeof(LED_STATES) / sizeof(LED_STATES[0]);

void setup() {
  strip.begin(); // Initialize the LED strip
  strip.show();  // Initialize all LEDs to 'off'
  
  pinMode(A5, INPUT);  // Set A5 as input for PWM signal (input from rio)
  
  Serial.begin(9600);
  Serial.println("RGB LED Controller Started");
}

void loop() {
  // Read the PWM pulse width from A5
  pulseWidth = pulseIn(A5, HIGH);
  // Serial.println(pulseWidth);
  
  if (pulseWidth == previousPulseWidth) {
    // W optimization, if the pulse width hasn't changed, no need to update the LEDs
    delay(20);
    return;
  }

  // Find the corresponding color state based on the pulse width
  for (int i = 0; i < NUM_STATES; i++) {
    if (pulseWidth >= LED_STATES[i].minPulseWidth && pulseWidth <= LED_STATES[i].maxPulseWidth) {
      // Set all LEDs to the corresponding color for this state
      setLedColor(LED_STATES[i].color);
      break;
    }
  }
  previousPulseWidth = pulseWidth;
  delay(20); 
}

void setLedColor(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);  // Set the color for each LED
  }
  strip.show();  // Update the LEDs with the new color data
}