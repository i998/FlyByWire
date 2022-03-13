/*
  v1.3 - Fly By Wire

  Status: Works OK

  Change list:
  v1.3:
  - added Median filter which shall reduce effect of potential jitter/outlier values for RC channels. 5-point median filtering is used  
  - minor bugfixes
  v1.2:
  - added FlyByWire_STM32_FreqTest example to measure internal oscillator frequency for PCA 9685 servo driver board
  - updated PCA9685ServoDriver library to align with Adafruit library v.2.3.1, mostly to cover issues with Prescale
  calculation (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/pull/61)
  - updated Fly By Wire code to use setServoAll function

  v1.0:
  - RC model updated, tested and configured with a test model.

  Notes:
  - Compiled with Fastest (-O3) settings

  =================================================================
  (C)2022, 2021, 2018 ifh
  This file is part of Fly By Wire.

  Fly By Wire is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Fly By Wire is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

*/


//use local copies of the libraries
#include "src\PCA9685ServoDriver.h"
#include "src\SBUS.h"
#include "src\PPMReader.h"
#include "src\Model_Test1.h"
#include "src\MedianFilter.h"

// uncomment to print some debug messages
//#define ENABLE_DEBUG_OUTPUT_LOOP_IN
//#define ENABLE_DEBUG_OUTPUT_LOOP_OUT

//define input signal type - SBUS or PPM
#define _SBUS_
//#define _PPM_

// Pin number of a LED pin that will signal the data flow
const int ledPin =  LED_BUILTIN;

//timestamp variables
uint16_t timestampOld = 0;
uint16_t timestampNew = 0;

//======Setup I2C interface===================================
//TwoWire I2C_FAST (1,I2C_FAST_MODE); //I2C1
//TwoWire I2C_FAST (2,I2C_FAST_MODE); //I2C2
TwoWire I2C_FAST = TwoWire(1, I2C_FAST_MODE); //I2C1
//TwoWire I2C_FAST=TwoWire(2,I2C_FAST_MODE); //I2C2

//============================================================


//=======Define Servo Driver and number of output signals  ===
// called this way, it uses the default address 0x40
//PCA9685ServoDriver pwm = PCA9685ServoDriver();
// you can also call it with a different address you want
//PCA9685ServoDriver pwm = PCA9685ServoDriver(0x41);
// you can also call it with a different address and I2C interface
//PCA9685ServoDriver pwm = PCA9685ServoDriver(&Wire, 0x40);

PCA9685ServoDriver pwm = PCA9685ServoDriver(&I2C_FAST, 0x40);
int channelAmountOut = 12;
uint16_t channelsOUT[17];  // for write 16 channels, 1..16

//============================================================


#if defined(_SBUS_)
//=================Define SBUS receiver ======================
// a SBUS object, which is on STM32 hardware serial port 1
SBUS sbus(Serial1);

bool failSafeSBUS = false;
bool failSafeLocal = false;
uint16_t lostFrames = 0;

int channelAmountIn = 16;
//17 values, indexed {0..16}
uint16_t channelsIN[17];  // for readRaw and  readNormalisedInteger
//float channelsIN[17];   // for readNormalisedFloat

uint16_t channelsIN_MF[17];  // for Median Filter  - contains input channels after filter is applied 

//===========================================================
#endif

#if defined(_PPM_)
//=================Define PPM receiver ======================
//set a pin number for PPM input
int PPMinputPin = 2;
// Initialize a PPMReader on digital pin 3 with 8 expected channels.
//Note interrupt will be attached separately in Setup()
int channelAmountIn = 8;
PPMReader ppm(channelAmountIn);

//number of values to cover amount of channels, indexed {1..channelAmount}
uint16_t channelsIN[9];  // for PPM, 8 channels, 1..8, 9 values indexed {0..8}
//float channelsIN[9];  // for PPM, 8 channels, 1..8,9 values indexed {0..8}

uint16_t channelsIN_MF[9];  // for Median Filter  - contains input channels after filter is applied 

//===========================================================
#endif


//========Define  Median Filter =====================
MedianFilter Filter;
//===========================================================



//========Define model calculation rules=====================
Model Model_Test1;
//===========================================================



//=================SETUP()===================================
void setup() {

  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);

  //==============================
  Serial.begin(9600);
  //The program will wait for serial to be ready up to 10 sec then it will contunue anyway
  for (int i = 1; i <= 10; i++) {
    delay(1000);
    if (Serial) {
      break;
    }
  }
  Serial.println("Setup() started ");
  //===============================


  //==set default output values for servo driver, microseconds=

  for (uint8_t i = 0; i <= 16; i++) {
    channelsOUT[i] = 1500;
  }
  //==========================================================

#if defined(_SBUS_)
  //================Setup SBUS================================

  //Calibration multipliers to apply to raw channel data values before
  //they are returned as a normalised data (rawValues[i] * multiplierScale + multiplierBias;)
  //Walkera DEVO 12E values:
  //to uS (1100..1900):
  //  Scale=0.6250f
  //  Bias=880.0f
  //to percentage (-100..+100):
  //  Scale=0.156250f
  //  Bias=-155.0f
  //to float (-1..+1):
  //  Scale=0.00156250f
  //  Bias=-1.550f
  sbus.multiplierScale = 0.6250f;
  sbus.multiplierBias = 880.0f;

  // begin the SBUS communication
  sbus.begin();
  //==================================================================
  Serial.println("SBUS setup completed");
#endif


#if defined(_PPM_)
  //================Setup PPM=========================

  //set the PPMinputPin as input,  pulled up for inverted polarity , pulled down for hormal
  pinMode(PPMinputPin, INPUT_PULLUP);
  //attach interrupt  to the input pin.  The function is in the PPMReader class and it sets the interrupt pin and signal polarity
  ppm.setupInterrupt(PPMinputPin, INVERTED);

  //correct the default values
  // The range of a channel's possible values, microseconds
  ppm.minChannelValue = 700;
  ppm.maxChannelValue = 2200;

  //The minimum value (time) after which the signal frame is considered to
  //be finished and we can start to expect a new signal frame.
  //Minimal blank time for 8 channels is:
  // PPM signal period - (150% max servo duration * 8 + 400us trailing pulse)
  // 22000 us - (2100*8 + 400) = 4800us  */
  ppm.blankTime = 5000;

  //Calibration multipliers to apply to raw channel data values before
  //it is returned as a normalised data (rawValues[i] * multiplierScale + multiplierBias;)
  //Walkera DEVO 12E values:
  //to uS (1100..1900):
  //  Scale=1.0f
  //  Bias=-9.0f
  //to percentage (-100..+100):
  //  Scale=0.250f
  //  Bias=-377.250f
  //to float (-1..+1):
  //  Scale=0.00250f
  //  Bias=-3.77250f

  ppm.multiplierScale = 1; //   1.0f;
  ppm.multiplierBias = -9; //  -9.0f;
  //====================================
  Serial.println("PPM setup completed");
#endif



  //=====setup Median Filter ===============
  Filter.channelAmountIn = channelAmountIn;
  Filter.channelAmountOut = channelAmountIn;  //use same number of channels for both input and output
  //===============================================
  Serial.println("Median Filter setup completed");
  
  //===========setup model=========================
  Model_Test1.channelAmountIn = channelAmountIn;
  Model_Test1.channelAmountOut = channelAmountOut;
  //===============================================
  Serial.println("Model setup completed");


  //================Setup Servo Driver============
  //begin the Servo Driver communication
  pwm.begin();
  pwm.freqCalibration = 1.0504; // adjustment for the real frequency of PCA9685 chip, see FlyByWire_STM32_FreqTest example
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  //==============================================
  Serial.println("Servo Driver setup completed");

  Serial.println("Setup() completed");
}


//==================LOOP()==============================================
void loop() {

#if defined(_SBUS_)

  // look for a good SBUS packet from the receiver received and parced sucessfully
  timestampNew = sbus.readNormalisedInteger(&channelsIN[0]);
#endif


#if defined(_PPM_)
  //acquire the data into a local array
  timestampNew = ppm.readNormalisedInteger(&channelsIN[0]);
#endif


  if (timestampNew != 0 && timestampNew != timestampOld) { //data is ready and it is a new data
    //update timestamp
    timestampOld = timestampNew;


    //optional - blinking /serial debug
    // set the LED with the ledState of the variable:
    //http://wiki.stm32duino.com/index.php?title=API
    //This is a function you can call, that doesn't take the pinnumber
    //but the GPIOPort, and which pin of the port you want to access.
    // It's a lot faster than the digitalWrite function
    //example: gpio_write_bit(GPIOB, 0, LOW);
    // builtin led:  gpio_write_bit(GPIOB,1,ledState);
    gpio_write_bit(GPIOB, 1, HIGH);


    //=======Debug Input values==============================================
#ifdef ENABLE_DEBUG_OUTPUT_LOOP_IN
    // Print latest valid values from all channels
    for (int i = 1; i <= channelAmountIn; ++i) {
      Serial.print("Ch" + String(i) + ":" + String(channelsIN[i]) + " ");
    }
    Serial.print(" Ch0:"); Serial.print(channelsIN[0]); //"Byte 23 of SBUS protocol or PPM failsafe value"
#if defined(_SBUS_)
    //print specific values  for SBUS
    Serial.print(" FailSafe:");
    Serial.print(failSafeSBUS);
    Serial.print(" LostFrames:");
    Serial.print(lostFrames);
    Serial.print(" micros:");
    Serial.print(sbus.GetDataInputTimeStamp());
    Serial.println();
#endif
#if defined(_PPM_)
    //print specific values  for PPM
    Serial.print(" micros:");
    Serial.print(ppm.GetDataInputTimeStamp());
    Serial.println();
#endif
#endif
    //==========================================================================


    //===Check for Faisafe, apply MEDIAN FILTER  and MODEL - CALCULATE==========
    //Check for failsafe: using Channel0 which contains a faisafe code.
    //Apparently Walkera DEVO 12E returns 3 when the receiver is binded and signal is ok,  otherwise 0 is returned for failsafe
    if (channelsIN[0] != 0) {
      //Normal signal
          //Apply Median Filter   
          Filter.ApplyFilter(channelsIN, channelsIN_MF);
          //Filter.Passthrough(channelsIN, channelsIN_MF);
          
          //Apply Model
          Model_Test1.Calculate(channelsIN_MF, channelsOUT);
         //Model_Test1.Passthrough(channelsIN, channelsOUT);
         //Serial.println("Normal CalcTime,us:"+String(Model_Test1.CalculationTime) + " ");

    }
    else
    {
      //Failsafe detected, Apply Model Failsafe
      Model_Test1.Failsafe(channelsIN, channelsOUT, false);
      //Serial.println("Failsafe");
    }

    //====Debug Output values=============================================
#ifdef ENABLE_DEBUG_OUTPUT_LOOP_OUT
    // Print values from all output channels
    for (int i = 1; i <= channelAmountOut; ++i) {
      Serial.print("Ch" + String(i) + ":" + String(channelsOUT[i]) + " ");
    }
    Serial.println();
#endif


    //===========Write 16 chammels to servo driver ======================
    //use the setServo function:
    for (uint8_t i = 1; i <= 16; i++) {
      if (i <= channelAmountOut)  {
        //set output signal value
        pwm.setServo(i, channelsOUT[i] );
      }
      else
      {
        // set middle position of a servo
        pwm.setServo(i, 1500);
      }
    }

    //use the setServoAll function:
    //  pwm.setServoAll(channelsOUT);



    //===================================================================


  }
  else
  {
    // data not ready yet, do something else in this loop

    if (timestampNew == 0) { //data is being received
      // do something
    }
    if (timestampNew == timestampOld) { // looping too fast, the same old data is available
      // do something
    }


    //optional - blinking /serial debug
    gpio_write_bit(GPIOB, 1, LOW);

  }

}
