#include <TimerOne.h>
#include "LPD6803.h"

// Use EEPROM to track state across reboots
#include <EEPROM.h>

//Example to control LPD6803-based RGB LED Modules in a strand
// Original code by Bliptronics.com Ben Moyes 2009
//Use this as you wish, but please give credit, or at least buy some of my LEDs!

// Code cleaned up and Object-ified by ladyada, should be a bit easier to use

/*****************************************************************************/

// Choose which 2 pins you will use for output.
// Can be any valid output pins.
int dataPin = 2;       // 'yellow' wire
int clockPin = 3;      // 'green' wire
// Don't forget to connect 'blue' to ground and 'red' to +5V

// Timer 1 is also used by the strip to send pixel clocks

// Set the first variable to the NUMBER of pixels. 20 = 20 pixels in a row
LPD6803 strip = LPD6803(10, dataPin, clockPin);

#include <Button.h>        //https://github.com/JChristensen/Button

#define BUTTON_PIN 10       //Connect a tactile button switch (or something similar)
                           //from Arduino pin 2 to ground.
#define PULLUP true        //To keep things simple, we use the Arduino's internal pullup resistor.
#define INVERT true        //Since the pullup resistor will keep the pin high unless the
                           //switch is closed, this is negative logic, i.e. a high state
                           //means the button is NOT pressed. (Assuming a normally open switch.)
#define DEBOUNCE_MS 20     //A debounce time of 20 milliseconds usually works well for tactile button switches.
#define LED_PIN 9         //The standard Arduino "Pin 13" LED
long ledOn = 0;

Button myBtn(BUTTON_PIN, PULLUP, INVERT, DEBOUNCE_MS);    //Declare the button

int state = 0;
int stateChanged = 0;
#define NSTATES 9

void setup() {
  
  pinMode(LED_PIN, OUTPUT);    //Set the LED pin as an output
 
  // The Arduino needs to clock out the data to the pixels
  // this happens in interrupt timer 1, we can change how often
  // to call the interrupt. setting CPUmax to 100 will take nearly all all the
  // time to do the pixel updates and a nicer/faster display, 
  // especially with strands of over 100 dots.
  // (Note that the max is 'pessimistic', its probably 10% or 20% less in reality)
  
  strip.setCPUmax(90);  // start with 50% CPU usage. up this if the strand flickers or is slow
  
  // Start up the LED counter
  strip.begin();

  // Update the strip, to start they are all 'off'
  strip.show();

  // Read last state from EEPROM
  Serial.begin(115200);
  state = EEPROM.read(0);
  if ( state == 255 ) {
    state = 0;
  }
  Serial.print("Initial state=");
  Serial.println(state);
}

void checkButton() {
  if ( myBtn.wasReleased() ) {
    digitalWrite(LED_PIN,HIGH);
    ledOn = millis();
    state = (state+1) % NSTATES;
    EEPROM.write(0,state);
    stateChanged = 1;
    Serial.print("New state=");
    Serial.println(state);
  }
}

void loop() {
  
    if ( ledOn && (millis() > ledOn+500) ) {
      ledOn = 0;
      digitalWrite(LED_PIN,LOW);
    }
    
    stateChanged = 0;

    myBtn.read();                    //Read the button
    
    switch (state) {
      case 0:
        colorWipe(Color(63,63,63), 50);
        checkButton();
        break;
      case 1:
        colorWipe(Color(63,0,0), 50);
        checkButton();
        break;
      case 2:
        colorWipe(Color(0,0,63), 50);
        checkButton();
        break;
      case 3:
        colorWipe(Color(0,63,0), 50);
        checkButton();
        break;
      case 4:
        colorWipe(Color(53,63,0), 50);
        checkButton();
        break;
      case 5:
        rainbow(100);
        break;
      case 6:
        rainbow(500);
        break;
      case 7:
        rainbowCycle(100);
        break;
      case 8:
        rainbowCycle(500);
        break;
    }
}

void rainbow(uint8_t wait) {
  int i, j;

  for (j=0; j < 96 * 3; j++) {     // 3 cycles of all 96 colors in the wheel
    if ( ledOn && (millis() > ledOn+500) ) {
      ledOn = 0;
      digitalWrite(LED_PIN,LOW);
    }
    
    for (i=0; i < strip.numPixels(); i++) {
      myBtn.read();
      strip.setPixelColor(i, Wheel( (i + j) % 96));
     checkButton();
     if ( stateChanged ) return;
    }  
    strip.show();   // write all the pixels out
//    delay(wait);
   for (int d=0; d < wait; d++) {
     myBtn.read();
     delay(1);
     checkButton();
     if ( stateChanged ) return;
   }
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed 
// along the chain
void rainbowCycle(uint8_t wait) {
  int i, j;
  
  for (j=0; j < 96 * 5; j++) {     // 5 cycles of all 96 colors in the wheel
    if ( ledOn && (millis() > ledOn+500) ) {
      ledOn = 0;
      digitalWrite(LED_PIN,LOW);
    }
    
    for (i=0; i < strip.numPixels(); i++) {
      myBtn.read();                    //Read the button
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      strip.setPixelColor(i, Wheel( ((i * 96 / strip.numPixels()) + j) % 96) );
      checkButton();
      if ( stateChanged ) return;
    }  
    strip.show();   // write all the pixels out
//    delay(wait);
   for (int d=0; d < wait; d++) {
     myBtn.read();
     delay(1);
     checkButton();
     if ( stateChanged ) return;
   }
  }
}

// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint16_t c, uint8_t wait) {
  int i;
  
  for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

/* Helper functions */

// Create a 15 bit color value from R,G,B
unsigned int Color(byte r, byte g, byte b)
{
  //Take the lowest 5 bits of each value and append them end to end
  return( ((unsigned int)g & 0x1F )<<10 | ((unsigned int)b & 0x1F)<<5 | (unsigned int)r & 0x1F);
}

//Input a value 0 to 127 to get a color value.
//The colours are a transition r - g -b - back to r
unsigned int Wheel(byte WheelPos)
{
  byte r,g,b;
  switch(WheelPos >> 5)
  {
    case 0:
      r=31- WheelPos % 32;   //Red down
      g=WheelPos % 32;      // Green up
      b=0;                  //blue off
      break; 
    case 1:
      g=31- WheelPos % 32;  //green down
      b=WheelPos % 32;      //blue up
      r=0;                  //red off
      break; 
    case 2:
      b=31- WheelPos % 32;  //blue down 
      r=WheelPos % 32;      //red up
      g=0;                  //green off
      break; 
  }
  return(Color(r,g,b));
}

    
    
