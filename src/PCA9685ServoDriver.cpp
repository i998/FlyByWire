/*
Original library is from https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
Updated by IF 
2021-01-28
- aligned with Adafruit library v.2.3.1, mostly to cover issues with Prescale calculation (https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/pull/61)
- added setServoAll function which sets the Servo at all of the PCA9685 pins at once 
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

// uncomment to print some debug messages
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
  
  // set a default frequency and populate internal variables with the latest values
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
// Valid range in Hz, constrained between 24 and 3100. 
// As per datasheet frequency is typically from 24 Hz to 1526 Hz
// if internal oscillator is used. For external oscillators it is 3052=50MHz/(4*4096) 
// as  external clock input pin that will accept user-supplied clock (50 MHz max.) "
 _freq = constrain(freq,24,3100); 

#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("PCA9685ServoDriver::setPWMFreq - Attempting to set frequency as ");
  Serial.println(_freq);
#endif

//calculate oscillator frequency value that will be used in the calculations 
  _oscillator_freq=oscillator_freq * freqCalibration;


//calculate PCA9685 prescale value
         /*
		 * Formula for the prescale as  Equation 1 from the datasheet section 7.3.5:
		 *
		 *           (     clk_freq      )
		 *      round( ----------------- ) - 1
		 *           (  4096 * pwm_freq  )
		 *
		 * pwm_freq == 1000000000 / period.
		 */
  
  float prescaleval = _oscillator_freq;
  prescaleval /= 4096;
  prescaleval /= _freq;
  prescaleval -= 1;

#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("  Estimated pre-scale: "); Serial.println(prescaleval);
#endif
  // round equals adding 0.5 then floor 
  prescaleval = floor(prescaleval + 0.5);
  
  // limit the calculated prescale value by the range accepted by the PCA9685 chip 
  // and store it in the variable  
  _prescale = (uint8_t)constrain(prescaleval,PCA9685_PRESCALE_MIN,PCA9685_PRESCALE_MAX);
  
  
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("  Final pre-scale: "); Serial.println(_prescale);
#endif
  
  
  
//calculate PCA9685 tick value in  microseconds 
	//  Option 1 (that would be the target tick time based on frequency we would like to set):
	//  microseconds per period: 1,000,000 / frequency
	//  microseconds per tick: 1,000,000 / frequency /4096
	//_tickPCA9685inMicros = 1000000 / _freq / 4096;
	
	//  Option 2 (that would be actual tick time based on what PCA9685 chip delivers with its oscillator and prescale): 
	//  using Equation 1 from the datasheet section 7.3.5:  
      _tickPCA9685inMicros = (float)(_prescale + 1 ) * 1000000 / _oscillator_freq;
  
  
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver
  Serial.print("  Frequency:"); Serial.println(_freq);
  Serial.print("  tickPCA9685inMicros:"); Serial.println(_tickPCA9685inMicros);
#endif 
  
   
// set PCA9685 chip 
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode);                             // go to sleep
  write8(PCA9685_PRESCALE, _prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  // This sets the MODE1 register to turn on auto increment.
  write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
 

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
   
   //validate the values 
   on=(uint16_t)constrain(on,0,4096);
   off=(uint16_t)constrain(off,0,4096);
   
   //write to chip
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(PCA9685_LED0_ON_L+4*num);  // send the register address   
  _i2c->write(on);     // send low byte   ( i.e. LEDx_ON_L),  chip's auto increment will switch to the next register  
  _i2c->write(on>>8);  // send high byte  ( i.e. LEDx_ON_H),  chip's auto increment will switch to the next register
  _i2c->write(off);    // send low byte   ( i.e. LEDx_OFF_L), chip's auto increment will switch to the next register 
  _i2c->write(off>>8); // send high byte  ( i.e. LEDx_OFF_H), chip's auto increment will switch to the next register 
  _i2c->endTransmission();
    
#ifdef ENABLE_DEBUG_OUTPUT_PCA9685ServoDriver  
  Serial.println("PCA9685ServoDriver::setPWM completed"); 
#endif  
}

/**************************************************************************/
/*! 
    @brief  Sets the Servo at of one of the PCA9685 pins. 
	 Note the function does not check parameters for validity (i.e. num<=16, pulseLength is reasonable etc.)  
    @param  num One of the PWM output pins, from 1 to 16
    @param  pulseLength in microseconds
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
//Note that PCA9685 counter starts from 0,  ticksPCA9685 would represent a value when the signal will go OFF, 
//so ticks while the signal shall be ON  will be from 0 to ticksPCA9685-1, and signal duration will be the 
//number of ticks from 0 to ticksPCA9685-1 , i,e  as per the ticksPCA968 value. 
//Also see the data sheet section 7.3.3 , example 1.  
setPWM(num, 0, ticksPCA9685);

}


/**************************************************************************/
/*! 
    @brief  Sets the Servo at all of the PCA9685 pins at once 
	 Note the function does not check parameters for validity (i.e. pulseLength is reasonable etc.)  
    @param  pulseLengthArray: array of pulse length in microseconds, from 1 to 16, i.e shall be defined as pulseLengthArray[17] 
 
*/
/**************************************************************************/
void PCA9685ServoDriver::setServoAll(uint16_t pulseLengthArray[]) {


   for (uint8_t i=1; i<=16; i++) {
   
                 //set output signal value
                 setServo(i,pulseLengthArray[i] );
           }



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
    @brief  Returns the Servo resolution (i.e. minimal change) in microseconds
   
*/
/**************************************************************************/
 float PCA9685ServoDriver::getServoResolution(void){

return _tickPCA9685inMicros;

}

/**************************************************************************/
/*!
    @brief  Reads set Prescale from PCA9685 chip 
    @return prescale value
 */
 /**************************************************************************/
uint8_t PCA9685ServoDriver::readPrescale(void) {
  return read8(PCA9685_PRESCALE);
}



  //getOscillatorFrequency and  setOscillatorFrequency
  //Returns or sets Oscillator Frequency value that PCA9685 chip is expected to run on.
  //This value can also be corrected by a multiplier freqCalibration. 
  //If setting the Oscillator Frequency value is required that need to be set BEFORE calling setPWMFreq(freq) function!
  //Technically these functions doesn't set the frequency of the chip or read it from the chip itself  
  //but sets the value of a variable the library uses to calculate other things like the PWM frequency.
/**************************************************************************/
/*!
   @brief  Getter for the internally tracked oscillator used for freq
   calculations. Gets the value of a variable the library uses for calculations, 
   not from the chip. 
   @returns The frequency the library thinks PCA9685 is running at (it cannot
  introspect)
 */
 /**************************************************************************/
uint32_t PCA9685ServoDriver::getOscillatorFrequency(void) {
  return _oscillator_freq;
}


/**************************************************************************/
/*!
   @brief Setter for the internally tracked oscillator used for freq
   calculations. Sets the value of a variable the library uses for calculations, 
   not the chip itself. The variable will be refreshed by setPWMFreq function anyway. 
   @param freq The frequency the library should use for frequency calculations
 */
 /**************************************************************************/
  
void PCA9685ServoDriver::setOscillatorFrequency(uint32_t freq) {
  _oscillator_freq = freq;
}



/**************************************************************************/
/*!
 *  @brief  Puts board into sleep mode
 */
 /**************************************************************************/
void PCA9685ServoDriver::sleep() {
  uint8_t awake = read8(PCA9685_MODE1);
  uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
  write8(PCA9685_MODE1, sleep);
  delay(5); // wait until cycle ends for sleep to be active
}

/**************************************************************************/
/*!
 *  @brief  Wakes board from sleep
 */
 /**************************************************************************/
void PCA9685ServoDriver::wakeup() {
  uint8_t sleep = read8(PCA9685_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
  write8(PCA9685_MODE1, wakeup);
}

/**************************************************************************/
/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
 /**************************************************************************/
void PCA9685ServoDriver::setExtClk(uint8_t prescale) {
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

  write8(PCA9685_PRESCALE, prescale); // set the prescaler

  delay(5);
  // clear the SLEEP bit to start
  write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  Serial.println(read8(PCA9685_MODE1), HEX);
#endif

 //store the new prescale in the variable 
  _prescale=prescale; 
}

/**************************************************************************/
/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
 /**************************************************************************/
void PCA9685ServoDriver::setOutputMode(bool totempole) {
  uint8_t oldmode = read8(PCA9685_MODE2);
  uint8_t newmode;
  if (totempole) {
    newmode = oldmode | MODE2_OUTDRV;
  } else {
    newmode = oldmode & ~MODE2_OUTDRV;
  }
  write8(PCA9685_MODE2, newmode);
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting output mode: ");
  Serial.print(totempole ? "totempole" : "open drain");
  Serial.print(" by setting MODE2 to ");
  Serial.println(newmode);
#endif
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
void PCA9685ServoDriver::writeMicroseconds(uint8_t num,
                                                uint16_t Microseconds) {

// writeMicroseconds function is Adafruit's alias for setServo function 
//so it is added here for compatibility purposes
num=num+1; // convert  {0..15} to {1..16} 

setServo(num, Microseconds); 
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
