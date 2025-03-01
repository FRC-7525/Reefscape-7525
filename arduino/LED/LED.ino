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

bool increases[] = {true, true, true};

bool increase = false;

int darkness = 255;

int elevatorHeight = 0;

// Different states
void disabled() {
  if (darkness >= 255) {
    darkness = 255;
    increase = false;
  } else if (darkness <= 0) {
    darkness = 0;
    increase = true;
  }
  
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setHSV(0, 255, darkness);
  }

  if (increase) darkness += 5;
  else darkness -= 5;
}

void idle() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setHSV(18.42, 204.26, 231.03);
    FastLED.show();
    delay(30);
  }

  delay(100);

  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i].setHSV(151, 140, 200);
    FastLED.show();
    delay(30);
  }

  delay(100);
}

void intaking() {
  for (int i = 0; i < numSpots; i++) {
    if (spots[i] >= NUM_LEDS) spots[i] = 0;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    bool lit = false;
    for (int j = 0; j < numSpots; j++) {
      int dimPercent = ((1 - abs(spots[j] - i)) / 5.0) * 255;

      if (spots[j] - 5 <= i && i < spots[j]) {
        leds[i].setHSV(160, 255, dimPercent);
        lit = true;
      } else if (i == spots[j]) {
        leds[i].setHSV(160, 255, 255);
        lit = true;
      } else if (spots[j] < i && i <= spots[j] + 5) {
        leds[i].setHSV(160, 255, dimPercent);
        lit = true;
      }
    }

    if (lit) continue;
    else leds[i].setHSV(160, 255, 0);
  }

  for (int i = 0; i < numSpots; i++) spots[i]++;
}

void scoring() {
  if (spots[0] >= elevatorHeight) {
    spots[0] = 0;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    if (spots[0] - 2 <= i && i < spots[0] + 2) {
      leds[i].setRGB(255, 0, 0);
    } else if (elevatorHeight - 2 <= i && i <= elevatorHeight + 2) {
      leds[i].setRGB(255, 0, 0);
    } else {
      leds[i].setRGB(255, 255, 255);
    }
  }

  spots[0]++;
}

void autoalign() {
  if (spots[0] >= NUM_LEDS) {
    spots[0] = NUM_LEDS;
    increase = false;
  } else if (spots[0] < 0) {
    spots[0] = 0;
    increase = true;
  }

  fill_gradient(leds, 0, CHSV(32.58, 255, 249.9), spots[0], CHSV(5.67, 255, 128.01));
  fill_gradient(leds, spots[0], CHSV(5.67, 255, 128.01), NUM_LEDS - 1, CHSV(32.58, 255, 249.9));

  if (increase) spots[0]++;
  else spots[0]--;
}

void algae() {

  for (int i = 0; i < numSpots; i++) {
    if (spots[i] >= NUM_LEDS) {
      spots[i] = NUM_LEDS - 1;
      increases[i] = false;
    } else if (spots[i] < 0) {
      spots[i] = 0;
      increases[i] = true;
    }
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    bool lit = false;
    for (int j = 0; j < numSpots; j++) {
      int dimPercent = ((1 - abs(spots[j] - i)) / 5.0) * 255;

      if (spots[j] - 5 <= i && i < spots[j]) {
        leds[i].setHSV(110, 255, dimPercent);
        lit = true;
      } else if (i == spots[j]) {
        leds[i].setHSV(110, 255, 255);
        lit = true;
      } else if (spots[j] < i && i <= spots[j] + 5) {
        leds[i].setHSV(110, 255, dimPercent);
        lit = true;
      }
    }

    if (lit) continue;
    else leds[i].setHSV(160, 255, 0);
  }

  for (int i = 0; i < numSpots; i++) {
    if (increases[i]) spots[i]++;
    else spots[i]--;
  }
}

void climbing() {
  if (spots[0] >= NUM_LEDS) spots[0] = 0;

  //Need to replace with actual values of blue and purple
  fill_gradient(leds, spots[0] - 5, CHSV(160, 255, 10), spots[0], CHSV(210.375, 176.97, 121.125));
  fill_gradient(leds, spots[0], CHSV(210.375, 176.97, 121.125), spots[0] + 5, CHSV(160, 255, 10));

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < spots[0] - 5 || i > spots[0] + 5) {
      leds[i].setHSV(160, 255, 10); //Replace with actual blue
    }
  }

  spots[0]++;
}

void l1() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(255, 255, 255);

    if (10 <= i && i <= 14) {
      leds[i].setHSV(0, 255, 255);
    }
  }

  elevatorHeight = 12;
}

void l2() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(255, 255, 255);

    if (23 <= i && i <= 27) {
      leds[i].setHSV(0, 255, 255);
    }
  }

  elevatorHeight = 25;
}

void l3() {  
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(255, 255, 255);

    if (35 <= i && i <= 39) {
      leds[i].setHSV(0, 255, 255);
    }
  }

  elevatorHeight = 37;
}

void l4() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(255, 255, 255);

    if (46 <= i && i <= 50) {
      leds[i].setHSV(0, 255, 255);
    }
  }

  elevatorHeight = 48;
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
  pulseWidth = 2300;
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