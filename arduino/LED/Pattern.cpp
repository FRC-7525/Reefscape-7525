#include <FastLED.h>

//Dumb way to do this but im not gonna think that hard
class Pattern {
    public:
        void run(CRGB leds[]) {}
};

class Disabled: public Pattern {
    public:
        void run(CRGB leds[]) {
        
        }
};

class Idle: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class Intaking: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class Scoring: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class Autoalign : public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class Algae: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class Climbing: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class L1: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class L2: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class L3: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};

class L4: public Pattern {
    public:
        void run(CRGB leds[]) {
            
        }
};