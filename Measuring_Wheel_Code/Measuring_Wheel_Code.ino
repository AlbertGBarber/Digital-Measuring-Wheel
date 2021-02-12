//This code is placed under the MIT license
//Copyright (c) 2021 Albert Barber
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

// Encoder.h must be installed as a library, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <VoltageReference.h>

//=================================================================
#define DISPLAY_ADDR 0x3C //I2C address for the SSD1306 display
//=================================================================

// ================================================================
// ===                         MODES                            ===
// ================================================================
//Defines the order of modes, you can re-order modes as you wish
//you can remove modes by setting the mode number to greater than the NUM_MODES ex: 999
//if you do this don't forget to decrease NUM_MODES to match the number of modes remaining
//and also change the order of the remaining modes (going from 0 to however remain)
#define MEASURING_WHEEL  0  
#define TACHOMETER       1  

uint8_t NUM_MODES =      2; //total number of active modes

volatile uint8_t mode =  0; //inital mode
// ================================================================
// ===                         PIN SETUP                        ===
// ================================================================

//Arduino Pro-Mini Pins
#define ENCODER_PIN_1     2
#define ENCODER_PIN_2     3

#define MODE_BUTTON_PIN   4
#define ZERO_BUTTON_PIN   5

// ================================================================
// ===                     BAT SENSE SETUP                      ===
// ================================================================
//our range for lipo voltage is 4.2-3.4V, 
//after 3.4 the LiPo is 95% empty, so we should recharge
const unsigned long batteryUpdateTime = 5000; //how often we update the battery level in ms
unsigned long prevBatReadTime = 0; //the last time we read the battery in ms
uint8_t batteryLvl; //the battery percentage
#define MAX_BAT_VOLTAGE 4200 //max battery voltage
#define MIN_BAT_VOLTAGE 3400 //min battery voltage

//initialize the voltage reference library to read the Arduino's internal reference voltage
VoltageReference vRef;

// ================================================================
// ===                     SCREEN SETUP                         ===
// ================================================================
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64
#define YELLOW_BAR_HEIGHT 16
#define BASE_FONT_LENGTH  6
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1// Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================================================================
// ===                  PROGRAM GLOBAL VARS                     ===
// ================================================================
boolean zeroButtonToggle = false;
boolean zeroButtonRun = true;
boolean previousZeroButtonState = HIGH;
boolean previousModeButtonState = HIGH;
unsigned long currentTime = millis();
unsigned long lastZeroPress = millis();

const float cmToIn = 0.393701; //conversion from cm to in
const int32_t usInMin = 60 * 1000000; //amount of microseconds in a min

//default units for length
String units = "cm";
boolean unitSwitch = false;

// ================================================================
// ===                   TACHOMETER SETUP                       ===
// ================================================================

boolean tachOn = false; //if a tachometer mode is active
boolean tactReadingStarted = false; //if we're reading from rpm
volatile long irTacRevs = 0; //number of IR Tachometer revolutions (encoder revs are measured using the encoder's vars)
unsigned long rpm;
unsigned long tacStartTime = 0; //start time of tachometer reading
boolean tachStartTimeSet = false;  //starting time is only set once the tach detects motion, flag for it that has happened
const unsigned long measureIntervalMaxSec = 10; //maxium measurement period in seconds
const unsigned long measureIntervalMax = measureIntervalMaxSec * 1000000; //maxium measurement period in us //10 sec
const String measureIntervalMaxString = String(measureIntervalMaxSec); //maxium measurement period as a string in sec

// ================================================================
// ===                      ENCODER SETUP                       ===
// ================================================================

//both work as interrupts so we should get good performance
Encoder myEnc(ENCODER_PIN_1, ENCODER_PIN_2);

#define WHEEL_DIAM 2.5 //encoder wheel diameter in cm
#define GEAR_RATIO 56/8 //number of outer gear teeth / number of inner gear teeth
#define ENC_STEPS_PER_REV 48 //number of encoder steps per revolution
//distance covered per encoder step in cm
const float encStepLength = (M_PI * WHEEL_DIAM ) / (ENC_STEPS_PER_REV * GEAR_RATIO);

// ================================================================
// ================================================================
// ================================================================
// ===                   SETUP FUNCTION                         ===
// ================================================================
// ================================================================
// ================================================================

void setup() {
  //Serial.begin(115200);
  
  // ================================================================
  // ===                  DISPLAY CONFIG                          ===
  // ================================================================
  //note debug messages after this are displayed on the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR)) { // Address 0x3C for 128x64
    //Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // Clear the buffer, set the default text size, and color
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // ================================================================
  // ===                  BATTERY CONFIG                          ===
  // ================================================================
  //start reading the internal voltage reference, and take an inital reading
  vRef.begin();
  batteryLvl = getBatteryLevel(vRef.readVcc(), MIN_BAT_VOLTAGE, MAX_BAT_VOLTAGE);

  // ================================================================
  // ===                  BUTTON PINS CONFIG                      ===
  // ================================================================

  pinMode(MODE_BUTTON_PIN, INPUT);
  pinMode(ZERO_BUTTON_PIN, INPUT);

  //=================================================================
  //  //start up splash screen
  display.clearDisplay();
  drawHeader("By AGB");
  display.setCursor(28, 16);
  display.setTextSize(2);
  centerString("Digital", 16, 2);
  display.setCursor(35, 33);
  centerString("Measuring Wheel", 33, 2);
  display.display();
  delay(3000);
}

// ================================================================
// ================================================================
// ================================================================
// ===                     MAIN PROGRAM                         ===
// ================================================================
// ================================================================
// ================================================================

//Basic Function Outline
//each mode is contained in its own while loop
//until the mode is changed we continually loop through the current mode's loop
//for every mode a few things are common:
//we read the active sensor (or let interrupts for the sensor happen)
//we act on any button flags (zeroing, etc)
//we clear the display and redraw it (including the header)
//(we could overwrite only the changed parts of the display, but that is far more complicated, and for small display's it's quick to redraw the whole thing)
//we check any for any button presses at the end of the loop
//(this must come at the end of the loop, because if the mode changes we want to catch it at the while mode check, without doing anything in the loop)
//The zero button has two state boolean flags, a "run" and a "toggle"
//the "toggle" changes whenever the button is pressed
//the "run" is set false whenever the button is pressed and must be cleared by the current mode
//Both flags are reset to defaults (true for "run" and false for "toggle") when a mode changes
void loop() {

  //displays the distance covered by the wheel and encoder (ie a ruler)
  //we always show a positive distance (although the reading can increment and decerement)
  while (mode == MEASURING_WHEEL) {
    yield();
    display.clearDisplay();
    drawHeader("Measuring Wheel");

    long newPosition = myEnc.read();
    //we always show a positive distance
    //the user can zero it if needed
    double displayPos = abs(newPosition) * encStepLength;
    //zeros the encoder count
    if (!zeroButtonRun) {
      myEnc.write(0);
      zeroButtonRun = true;
    }
    display.setTextSize(3);
    display.setCursor(10, 20);
    if (unitSwitch) {
      displayPos = displayPos * cmToIn;  //convert from cm to in
      units = "in";
    } else {
      units = "cm";
    }
    centerString( doubleToString((double)displayPos, 2), 20, 3);
    display.setCursor(46, 43);
    display.print(units);
    display.display();
    readButtons();
  }
  
  //a tachometer that uses the encoder wheel to measure rpm
  while (mode == TACHOMETER) {
    runTachometer();
  }

}

//resets button variables and turns off/resets all sensors
//sets a clean state for the next mode and tries to save some power by turning off unused sensors
void resetSystemState() {
  display.clearDisplay();
  zeroButtonToggle = false;
  zeroButtonRun = true;
  myEnc.write(0);
}


//read the mode and zero buttons
//if the buttons have been pressed set flags
//The zero button has two state boolean flags, a "run" and a "toggle"
//the "toggle" changes whenever the button is pressed
//the "run" is set false whenever the button is pressed and must be cleared by the current mode
//Both flags are reset to defaults (true for "run" and false for "toggle") when a mode changes
void readButtons(void) {

  currentTime = millis();
  
  //if the mode pin is low (pressed) and it was not previously low (ie it's not being held down)
  //advance the mode counter
  //otherwise, the button is not pushed, set the previous state to high (not pressed)
  if (digitalRead(MODE_BUTTON_PIN) == LOW && previousModeButtonState != LOW) {
    previousModeButtonState = LOW; //set the previous state to low while the button is being held
    //if the mode pin is high, we need change units if in measuring wheel mode
    //or switch out of tachometer mode if we're in it
    if(mode == MEASURING_WHEEL){
      unitSwitch = !unitSwitch;
    } else {
      resetSystemState();
      mode = (mode + 1) % NUM_MODES;
    }
  } else if ( digitalRead(MODE_BUTTON_PIN) == HIGH) {
    previousModeButtonState = HIGH;
  }

  //if the zero pin is low (pressed) and it was not previously low (ie it's not being held down)
  //set zero toggle and run flags
  //otherwise, the button is not pushed, set the previous state to high (not pressed)
  if (digitalRead(ZERO_BUTTON_PIN) == LOW && previousZeroButtonState != LOW) {
    //double tap to switch to tachometer
    //checks if the zero button has been pressed in the last x milliseconds
    if (currentTime - lastZeroPress < 400) {
      resetSystemState();
      mode = (mode + 1) % NUM_MODES;
    }
    lastZeroPress = millis();
    previousZeroButtonState = LOW;
    zeroButtonRun = false;
    zeroButtonToggle = !zeroButtonToggle;
  } else if ( digitalRead(ZERO_BUTTON_PIN) == HIGH) {
    previousZeroButtonState = HIGH;
  }
}

//code for running the tachometer
//accessed by double pressing the zero button
//esentially the code will wait for the user to hit zero and then start recording revolutions
//when the user hits zero again or when measureIntervalMax time has passed, we stop reading,
//divide the revolutions by the time passed and convert to minutes
void runTachometer() {
  yield();
  //if the tachometer isn't on we're on the first pass through the function
  //so we display a prompt for the user to start the first reading
  if (!tachOn) {
    tactReadingStarted = false;
    display.clearDisplay();
    drawHeader("Tachometer");
    display.setTextSize(2);
    display.setCursor(15, 20);
    display.print("Hit Zero to start");
    display.display();
  }

  //if the zero button is pressed and we havn't started reading we start a new reading
  //we turn on the ir tach (if needed), reset the tach revolutions, and record the start time
  if (zeroButtonToggle && !tactReadingStarted) {
    myEnc.write(0);
    tactReadingStarted = true;
    tachStartTimeSet = false;
    display.clearDisplay();
    drawHeader("Tachometer");
    display.setTextSize(2);
    display.setCursor(5, 20);
    display.print("Reading...");
    display.setTextSize(1);
    display.setCursor(15, 45);
    display.print("Hit Zero to stop");
    display.setCursor(0, 55);
    display.print(String("Auto stop after " + measureIntervalMaxString + "sec"));
    display.display();
  }

  //the tach reading has started but the zero button has not been pressed again to stop
  //we only start the timer if we detect motion from either the IR photodiode or the encoder
  //we want to auto stop after measureIntervalMax, so we check how much time has passed
  //if enough time has passed, we set the zero button as pressed
  if (zeroButtonToggle && tactReadingStarted) {
    //if we have not detected motion yet, check for some
    //if we have, check if measureIntervalMax time has passed
    if (!tachStartTimeSet) {
      //if we detect motion ie either the encoder or tachRevs change
      //set the start time and flag that the time has been set
      if ( myEnc.read() != 0 ) {
        tachStartTimeSet = true;
        tacStartTime = micros();
      }
    } else {
      unsigned long currentTime = micros();
      if ( (currentTime - tacStartTime) > measureIntervalMax) {
        zeroButtonToggle = false;
      }
    }
  }

  //if we were reading, but the zero button has been pressed, it's time to stop and caculate rpm
  if (!zeroButtonToggle && tactReadingStarted) {
    tactReadingStarted = false;
    unsigned long currentTime = micros();
    //get the total elapsed time for the measurment
    unsigned long elapsedTime = currentTime - tacStartTime;
    //get the encoder's new position
    long newPosition = myEnc.read();
    //convert total revs to rpm
    rpm = ( ( abs(newPosition) / ENC_STEPS_PER_REV ) / (double)elapsedTime) * usInMin;
    display.clearDisplay();
    drawHeader("Tachometer");
    display.setTextSize(2);
    display.setCursor(10, 20);
    centerString( doubleToString( (double)rpm, 0 ), 20, 2);
    //display.print(rpm);
    display.setCursor(46, 40);
    display.print("RPM");
    display.setTextSize(1);
    display.setCursor(6, 55);
    display.print("Hit Zero to restart");
    display.display();
  }
  readButtons();
}

//ir tach interrupt counting routine
void countIRTactPulse() {
  irTacRevs++;
}

//copied from Roberto Lo Giacco Battery Sense library
//the LiPo's remaining power is not linearly related to its voltage
//so we use a best fit line to approximate the remaining power percentage
uint8_t getBatteryLevel(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
  // slow
  // uint8_t result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

  // steep
  // uint8_t result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

  // normal
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage) / (maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}

//fills in the header on the screen (the yellow bar) with the current mode name and the battery charge level
//because the battery level can fluctuate with current draw / noise, we only measure it at fixed intervals
//this prevents it from changing too often on the display, confusing the user
void drawHeader(String modeName) {
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(modeName);
  currentTime = millis();
  //update the battery level after batteryUpdateTime ms
  if (currentTime - prevBatReadTime > batteryUpdateTime) {
    batteryLvl = getBatteryLevel(vRef.readVcc(), MIN_BAT_VOLTAGE, MAX_BAT_VOLTAGE);
    prevBatReadTime = millis();
  }
  display.setCursor(100, 0);
  display.print(batteryLvl);
  display.print(char(37)); //prints the % symbol
}


//centers a string on the display at the specified y coordinate and text size
void centerString(String text, uint16_t yCoord, int textSize) {
  //the number of pixels needed to the left of the string
  //found by subtracting the screen width from the total length of the string in pixels,
  //then dividing whats left by two (because we want the padding to be equal on left and right)
  int16_t padding = (SCREEN_WIDTH - (text.length() * BASE_FONT_LENGTH * textSize) ) / 2;
  //if the text string is too long, the padding will be negative
  //we set it to zero to avoid this
  if (padding < 0) {
    padding = 0;
  }
  //draw the text at the input y coord
  display.setTextSize(textSize);
  display.setCursor(padding, yCoord);
  display.print(text);
}

//takes a double and returns a string of the double to the input decimal places
//uses the standard dtostrf() function and then trims the result to remove any leading spaces
String doubleToString(double num, int decimal) {
  const int bufferSize = 10;
  char buffer[bufferSize];
  String tmpStr = dtostrf(num, bufferSize, decimal, buffer);
  tmpStr.trim();
  return tmpStr;
}


//returns the sign of the input number (either -1 or 1)
int getSign(double num) {
  if (num >= 0) {
    return 1;
  } else {
    return -1;
  }
}