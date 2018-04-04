/*
Original library is from https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
Updated by IF 
2018-02-02
- tested support hardware I2C for Maple Mini board / STM32 (https://github.com/rogerclarkmelbourne/Arduino_STM32/)
- added setServo function
- added getServoResolution function
- changed set frequency - added configurable multiplier
- channels renumbered to start from 1 
- added comments/descriptions 
TODO - see the .h file 
=================================================================
/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "PCA9685ServoDriver.h"

#if defined(_BOARD_MAPLE_MINI_H_)
 #include <Wire.h>
#else
 #include <Wire.h>
#endif

// Set to true to print some debug messages, or false to disable them.
//#define ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver

/**************************************************************************/
/*! 
    @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on the Wire interface. 
	On Due we use Wire1 since its the interface on the 'default' I2C pins.
    @param  addr The 7-bit I2C address to locate this chip, default is 0x40
*/
/**************************************************************************/
PCA9685ServoDriver::PCA9685ServoDriver(uint8_t addr) {
  _i2caddr = addr;

#if defined(ARDUINO_SAM_DUE)
  _i2c = &Wire1;
#else
   #if defined(_BOARD_MAPLE_MINI_H_)
    // Maple Mini I2C hardware support 
//	HardWire HWIRE(1,I2C_FAST_MODE); // I2c1
    //HardWire HWIRE(2,I2C_FAST_MODE); // I2c
	//_i2c = &Wire;
	_i2c = &Wire; // _i2c = &HWIRE;
   #else
   //catch all 
     _i2c = &Wire;
   #endif
#endif
}


/**************************************************************************/
/*! 
    @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a TwoWire interface
    @param  i2c  A pointer to a 'Wire' compatible object that we'll use to communicate with
    @param  addr The 7-bit I2C address to locate this chip, default is 0x40
*/
/**************************************************************************/
PCA9685ServoDriver::PCA9685ServoDriver(TwoWire *i2c, uint8_t addr) {
  _i2c = i2c;
  _i2caddr = addr;
}

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/
void PCA9685ServoDriver::begin(void) {
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.println("PCA9685ServoDriver::begin stated"); 
#endif

  _i2c->begin();
  reset();
  
  // set a default frequency
  setPWMFreq(_freq);
  //switch off all the PCA9685 PWM driver chip output
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.println("Switching off all the PCA9685 PWM driver chip output ..."); 
#endif
   for (uint8_t i = 1; i<=16; i++) {
    setPWM(i, 0, 4096);
   }

   #ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.println("PCA9685ServoDriver::begin completed"); 
#endif
}


/**************************************************************************/
/*! 
    @brief  Sends a reset command to the PCA9685 chip over I2C
*/
/**************************************************************************/
void PCA9685ServoDriver::reset(void) {
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.println("PCA9685ServoDriver::reset started"); 
#endif

  write8(PCA9685_MODE1, 0x80);
  delay(10);

  }

/**************************************************************************/
/*! 
    @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
    @param  freq Floating point frequency that we will attempt to match
*/
/**************************************************************************/
void PCA9685ServoDriver::setPWMFreq(float freq) {
// valid range in Hz, constrained between 40 and 1000. As per datasheet frequency from a typical of 24 Hz to 1526 Hz
 _freq = constrain(freq,40,1000); 

#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("PCA9685ServoDriver::setPWMFreq - Attempting to set frequency as ");
  Serial.println(_freq);
#endif

//calculate PCA9685 tick value in  microseconds 
//  microseconds per period: 1,000,000 / frequency
//  microseconds per tick: 1,000,000 / frequency /4096
_tickPCA9685inMicros = 1000000 / _freq / 4096;
  
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("  Frequency:"); Serial.println(_freq);
  Serial.print("  tickPCA9685inMicros:"); Serial.println(_tickPCA9685inMicros);
#endif 

// set PCA9685 chip
  _freq = _freq * freqCalibration; // Correct for overshoot in the frequency setting (see https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/issues/11 ).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= _freq;
  prescaleval -= 1;

#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("  Estimated pre-scale: "); Serial.println(prescaleval);
#endif

  uint8_t prescale = floor(prescaleval + 0.5);
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("  Final pre-scale: "); Serial.println(prescale);
#endif
  
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa0);  //  This sets the MODE1 register to turn on auto increment.

#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("  Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
  Serial.println("PCA9685ServoDriver::setPWMFreq completed"); 
#endif
}

/**************************************************************************/
/*! 
    @brief  Sets the PWM output of one of the PCA9685 pins
    @param  num One of the PWM output pins, from 1 to 16
    @param  on At what point in the 4096-part cycle to turn the PWM output ON
    @param  off At what point in the 4096-part cycle to turn the PWM output OFF
*/
/**************************************************************************/
void PCA9685ServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("PCA9685ServoDriver::setPWM - Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);
#endif
   num=num-1; // convert {1..16} to {0..15}
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(LED0_ON_L+4*num);
  _i2c->write(on);
  _i2c->write(on>>8);
  _i2c->write(off);
  _i2c->write(off>>8);
  _i2c->endTransmission();
  
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver  
  Serial.println("PCA9685ServoDriver::setPWM completed"); 
#endif  
}

/**************************************************************************/
/*! 
    //@brief  Sets the Servo at of one of the PCA9685 pins. 
	// Note the function does not check parameters for validity (i.e. n<=15, pulseLength is reasonable etc.)  
    //@param  num One of the PWM output pins, from 1 to 16
    //@param  pulseLength in microseconds
*/
/**************************************************************************/

void PCA9685ServoDriver::setServo(uint8_t num, uint16_t pulseLength) {
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver  
  Serial.println("PCA9685ServoDriver::setServo started"); 
#endif 

uint16_t ticksPCA9685=4096; // variable range: 0-4095,  4096 is a default value for fully off
ticksPCA9685=(uint16_t) pulseLength / _tickPCA9685inMicros;  //calculate number of ticks 

#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("  Num:"); Serial.println(num);
  Serial.print("  pulseLength:"); Serial.println(pulseLength);
  Serial.print("  ticksPCA9685:"); Serial.println(ticksPCA9685);
#endif

//pass the command to PCA9685 
setPWM(num, 0, ticksPCA9685);

}

/**************************************************************************/
/*! 
    @brief  Helper to set pin PWM output. Sets pin without having to deal with on/off tick placement and properly handles a zero value as completely off and 4095 as completely on.  Optional invert parameter supports inverting the pulse for sinking to ground.
    @param  num One of the PWM output pins, from 1 to 16
    @param  val The number of ticks out of 4096 to be active, should be a value from 0 to 4095 inclusive.
    @param  invert If true, inverts the output, defaults to 'false'
*/
/**************************************************************************/
void PCA9685ServoDriver::setPin(uint8_t num, uint16_t val, bool invert)
{
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver  
  Serial.println("PCA9685ServoDriver::setPin started"); 
#endif 

  // Clamp value between 0 and 4095 inclusive.
  val = min(val, (uint16_t)4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, val);
    }
  }

#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver  
  Serial.println("PCA9685ServoDriver::setPin completed"); 
#endif 
}


/**************************************************************************/
/*! 
    //@brief  Returns the Servo resolution (i.e. minimal change) in microseconds
   
*/
/**************************************************************************/
 float PCA9685ServoDriver::getServoResolution(void){

return _tickPCA9685inMicros;

}

/******************************************************************************/
// actual read/write to I2C connected device 
/******************************************************************************/


uint8_t PCA9685ServoDriver::read8(uint8_t addr) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return _i2c->read();
}

void PCA9685ServoDriver::write8(uint8_t addr, uint8_t d) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->write(d);
  _i2c->endTransmission();
}
