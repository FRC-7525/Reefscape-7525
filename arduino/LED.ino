#include <Adafruit_NeoPixel.h>

#define PIN 6            // Data pin to first LED
#define NUM_LEDS 300      // Number of LEDs in the chain (fake)

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

unsigned long pulseWidth = 0;  // Store the pulse width from A0
unsigned long previousPulseWidth = 0; // Store the previous pulse width from A0 (lil optimization)

// Defines LED state structure (ahh structs are so nice)
struct LedState {
  unsigned long minPulseWidth; // Minimum pulse width for this state
  unsigned long maxPulseWidth; // Maximum pulse width for this state
  uint32_t color;             // RGB color value (use Adafruit's RGB format)
  const char* name;           // Name of the state for debugging
};

// Different color states, 500 micro second thresholds bc PWM is NOT trustworthy (I think, idrk)

const LedState LED_STATES[] = {
  {0, 750, strip.Color(255, 0, 0), "Red"},       // Pulse width 0-750 => Red
  {751, 1250, strip.Color(255, 165, 0), "Orange"}, // Pulse width 751-1250 => Orange
  {1251, 1750, strip.Color(0, 255, 0), "Green"}, // Pulse width 1251-1750 => Green
  {1751, 2250, strip.Color(128, 0, 128), "Purple"}, // Pulse width 1751-2250 => Purple
  {2251, 2750, strip.Color(0, 0, 255), "Blue"},  // Pulse width 2251-2750 => Blue
  {2751, 3250, strip.Color(255, 255, 0), "Yellow"}, // Pulse width 2751-3250 => Yellow
  {3251, 3750, strip.Color(255, 255, 255), "White"}, // Pulse width 3251-3750 => White
  {3751, 4096, strip.Color(0, 0, 0), "Off"}     // Pulse width 3751-4096 => Off (black)
};

const int NUM_STATES = sizeof(LED_STATES) / sizeof(LED_STATES[0]);

void setup() {
  strip.begin(); // Initialize the LED strip
  strip.show();  // Initialize all LEDs to 'off'
  
  pinMode(A5, INPUT);  // Set A0 as input for PWM signal (input from rio)
  
  Serial.begin(9600);
  Serial.println("RGB LED Controller Started");
}

void loop() {
  // Read the PWM pulse width from A0
  pulseWidth = pulseIn(A5, HIGH);
  // Serial.println(pulseWidth);
        setLedColor(LED_STATES[4].color);
        Serial.print("lala");
  
  if (pulseWidth == previousPulseWidth) {
    // W optimization, if the pulse width hasn't changed, no need to update the LEDs
    delay(20);
    return;
  }

  // Find the corresponding color state based on the pulse width
  for (int i = 0; i < NUM_STATES; i++) {
    if (pulseWidth >= LED_STATES[i].minPulseWidth && pulseWidth <= LED_STATES[i].maxPulseWidth) {
      // Set all LEDs to the corresponding color for this state
      // setLedColor(LED_STATES[i].color);
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