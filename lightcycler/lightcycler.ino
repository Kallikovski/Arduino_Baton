
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


#define PIXEL_PIN    5  // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 10  // Number of NeoPixels

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void loop() {
  for(int i = 0; i < 4; ++i){
    switch(i) {           // Start the new animation...
      case 0:
        colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
        break;
      case 1:
        colorWipe(strip.Color(255,   0,   0), 50);    // Red
        break;
      case 2:
        colorWipe(strip.Color(  0, 255,   0), 50);    // Green
        break;
      case 3:
        colorWipe(strip.Color(  0,   0, 255), 50);    // Blue
        break;
    }
  }
}
