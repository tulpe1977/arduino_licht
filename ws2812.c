#include <Adafruit_NeoPixel.h>
#include <math.h>

#define N_PIXELS  107  // Number of pixels in strand
#define MIC_PIN   A0  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define SAMPLE_WINDOW   10  // Sample window for average level
#define PEAK_HANG 24 //Time of pause before peak dot falls
#define PEAK_FALL 4 //Rate of falling peak dot
#define INPUT_FLOOR 10 //Lower range of analogRead input
#define INPUT_CEILING 300 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)

byte peak = 16;      // Peak level of column; used for falling dots
unsigned int sample;

byte dotCount = 0;  //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot

// #include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 6

int x;
int y;
int z;
const int tasterPin = 2;     // Taster an Pin 2 angeschlossen
volatile int lichtmodus = 0;          // Variable für die verschiedenen festgelegten Farben
int tasterStatus = HIGH;      // Variable zu speichern des Tasterstatus

// (Variablendeklarationen)
const int debounce = 150; // Taster entprellen mit 150ms
volatile unsigned long nowMillis;
volatile unsigned long prevMillis;

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(107, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup()
{
  pinMode(tasterPin, INPUT_PULLUP);      // Setzt den TasterPin als Eingang
  strip.begin();
  strip.setBrightness(200);
  strip.show(); // Initialize all pixels to 'off'
  attachInterrupt(digitalPinToInterrupt(tasterPin), ChangeMode, RISING);

  
}

void ChangeMode() {
    nowMillis = millis();
    if (nowMillis - prevMillis > debounce) {
        lichtmodus ++;
        prevMillis = nowMillis;
    }
}
void loop() {
  
  // Abfrage ob der Taster gedrückt ist
  tasterStatus = digitalRead(tasterPin);
  
  // Wenn Taster gedrückt ist...
  if (tasterStatus == LOW)
  {
    lichtmodus++;     // Lichtmodus +1
    delay(300);       // 300ms warten
  }
  
  
//+++++++++++++++ LEUCHTPROGRAMME +++++++++++++++++
  
// Modus 0 = Licht aus
  if (lichtmodus == 0)
    {
     for (int t=1;t<6;t++)
     {
       x=random(1,255);
       y=random(1,255);
       z=random(1,255);
       colorWipe(strip.Color(x, y, z), 10);
      
       x=random(1,255);
       y=random(1,255);
       z=random(1,255);
       colorWipe2(strip.Color(x, y, z), 10);
     }
    rainbow(15);
    rainbowCycle(15);
    theaterChaseRainbow(30);    
    }
    
  // Modus 1 = Blau
  else if (lichtmodus == 1)
    {
        unsigned long startMillis= millis();  // Start of sample window
        float peakToPeak = 0;   // peak-to-peak level

        unsigned int signalMax = 0;
        unsigned int signalMin = 1023;
        unsigned int c, y;


        // collect data for length of sample window (in mS)
        while (millis() - startMillis < SAMPLE_WINDOW)
        {
          sample = analogRead(MIC_PIN);
          if (sample < 1024)  // toss out spurious readings
          {
           if (sample > signalMax)
          {
            signalMax = sample;  // save just the max levels
          }
          else if (sample < signalMin)
          {
            signalMin = sample;  // save just the min levels
          }
        }  
      }
  
      peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

  // Serial.println(peakToPeak);


  //Fill the strip with rainbow gradient
      for (int i=0;i<=strip.numPixels()-1;i++){
       strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
      }


  //Scale the input logarithmically instead of linearly
      c = fscale(INPUT_FLOOR, INPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);

  


      if(c < peak) {
         peak = c;        // Keep dot on top
         dotHangCount = 0;    // make the dot hang before falling
      }
      if (c <= strip.numPixels()) { // Fill partial column with off pixels
        drawLine(strip.numPixels(), strip.numPixels()-c, strip.Color(0, 0, 0));
      }

// Set the peak dot to match the rainbow gradient
      y = strip.numPixels() - peak;
  
      strip.setPixelColor(y-1,Wheel(map(y,0,strip.numPixels()-1,30,150)));

      strip.show();

// Frame based peak dot animation
      if(dotHangCount > PEAK_HANG) { //Peak pause length
        if(++dotCount >= PEAK_FALL) { //Fall rate
          peak++;
          dotCount = 0;
        }
      }    
      else {
        dotHangCount++;
      }
     }    
  


// Anzahl der Leutmodi auf 2 begrenzen. (0 bis 1)
      else
{
      lichtmodus = 0;
}
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

// Fill the dots one after the other with a color
void colorWipe2(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(107-i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}





// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  for(int i=from; i<=to; i++){
    strip.setPixelColor(i, c);
  }
}


float fscale( float originalMin, float originalMax, float newBegin, float
newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println();
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {  
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;

}