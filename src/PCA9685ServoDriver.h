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
=================================================================*/
/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef PCA9685ServoDriver_H
#define PCA9685ServoDriver_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#if defined(_BOARD_MAPLE_MINI_H_)
 #include "Wire.h"   
#else
 #include "Wire.h"
#endif


//Define PCA9685 registers 

#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1                                                          \
  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */


/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with PCA9685 PWM chip
*/
/**************************************************************************/
class PCA9685ServoDriver {
 public:
  PCA9685ServoDriver(uint8_t addr = 0x40);
  PCA9685ServoDriver(TwoWire *I2C, uint8_t addr = 0x40);
  
  //Setups the I2C interface and hardware
  void begin(void);
  //Sends a reset command to the PCA9685 chip over I2C
  void reset(void);
  
  
  //This function can be used to adjust the PWM frequency, which determines how many full 'pulses' per second are generated by the IC.
  //freq: A number representing the frequency in Hz, between 24 and 3100
  void setPWMFreq(float freq);
  
  //Oscillator ferquency, can be set itself or corrected by freqCalibration variable. Default datasheet value  
  //for PCA9658 internal oscillator is 25MHz (however chips often run in a range 26-27 MHz).   
  //If required, need to be set BEFORE calling setPWMFreq(freq) function!
  //Default value 25000000.
  uint32_t oscillator_freq = 25000000;  
  
    
  //Oscillator Frequency calibration 
  //Multiplier to the oscillator frequency to compensate deviations of the internal oscillator of PCA9685 chip  
  //If required, need to be set BEFORE calling setPWMFreq(freq) function!
  //Default value 1.07
  float freqCalibration = 1.07;
  
  //This function sets the start (on) and end (off) of the high segment of the PWM pulse on a specific channel. 
  //  You specify the 'tick' value between 0..4095 when the signal will turn on, and when it will turn off. 
  //Channel indicates which of the 16 PWM outputs should be updated with the new values.
  //Arguments
  //• num: The channel that should be updated with the new values (1..16)
  //• on: The tick (between 0..4095) when the signal should transition from low to high
  //• off:the tick (between 0..4095) when the signal should transition from high to low
  //
  //Note:
  //You can set the pin to be fully on with
  //pwm.setPWM(pin, 4096, 0);
  //You can set the pin to be fully off with
  //pwm.setPWM(pin, 0, 4096);
  void setPWM(uint8_t num, uint16_t on, uint16_t off);


  //Helper to set pin PWM output. Sets pin without having to deal with on/off tick placement 
  //and properly handles a zero value as completely off and 4095 as completely on. 
  //Optional invert parameter supports inverting the pulse for sinking to ground.
  //Arguments
  //• num: One of the PWM output pins, from 1 to 16
  //• val: The number of ticks out of 4096 to be active, should be a value from 0 to 4095 inclusive.
  //• invert: If true, inverts the output, defaults to 'false'
  void setPin(uint8_t num, uint16_t val, bool invert=false);

   
  //Sets the Servo at of one of the PCA9685 pins
  //Arguments
  //• num: One of the PWM output pins, from 1 to 16
  //• pulseLength: pulse length in microseconds
  void setServo(uint8_t num, uint16_t pulseLength);
  
  //Sets the Servo at all of the PCA9685 pins at once 
  //Arguments
  //• pulseLengthArray: array of pulse length in microseconds, from 1 to 16, i.e shall be defined as pulseLengthArray[17] 
  void setServoAll(uint16_t pulseLengthArray[]);
  
  //Returns the Servo resolution 
  //(i.e. minimal input change that will be recognised byPCA9685 chip) in microseconds. 
  //Depends in the frequency the chip is set on by setPWMFreq function.
  float getServoResolution(void);
  
  
  //Retrns Prescale value from PCA9685 chip
  uint8_t readPrescale(void);   
  
  
  //Returns or sets Oscillator Frequency value that PCA9685 chip is expected to run on.
  //This value can also be corrected by a multiplier freqCalibration. 
  //If setting the Oscillator Frequency value is required that need to be set BEFORE calling setPWMFreq(freq) function!
  //Technically these functions doesn't set the frequency of the chip or read it from the chipp itself  
  //but sets the value of a variable the library uses to calculate other things like the PWM frequency.
  uint32_t getOscillatorFrequency(void);
  void setOscillatorFrequency(uint32_t freq);
  
  
  //Sets EXTCLK pin to use the external clock
  //Arguments
  //• prescale: Configures the prescale value to be used by the external clock
  void setExtClk(uint8_t prescale);

  
  //Wakes board from sleep
  void wakeup();

  
  //Puts board into sleep mode
  void sleep();

  //Sets the output mode of the PCA9685 to either open drain or push pull / totempole.
  //Warning: LEDs with integrated zener diodes should only be driven in open drain mode!
  //Arguments
  //• totempole: Totempole if true, Open Drain if false.
  void setOutputMode(bool totempole);
    


  // writeMicroseconds function is Adafruit's alias for setServo function 
  //so it is added here for compatibility purposes
  //Arguments
  //• num: One of the PWM output pins, from 0 to 15
  //• Microseconds: pulse length in microseconds
  void writeMicroseconds(uint8_t num, uint16_t Microseconds);
  
  
 private:
  uint8_t _i2caddr;
  
 // TwoWire *_i2c;
  #if defined(_BOARD_MAPLE_MINI_H_)
  TwoWire *_i2c;  // Wire *_i2c;
  #else
  TwoWire *_i2c;
  #endif
  
  
  float _freq=50;  // default value 50Hz, valid range in Hz, between 24 and 3100, PCA9685 chip internal oscillator handles 24-1526 Hz.  
  float _tickPCA9685inMicros = 4.9;  // duration of one PCA9685 tick in microseconds. Default is 4.9 us for 50Hz servo rate. 
  uint8_t _prescale=121;  // value for the PCA9685 prescaler. Default is 121 for 50Hz servo rate. 
  uint32_t _oscillator_freq = 25000000; // value for the PCA9685 oscillator frequency. Default is 25000000 for internal oscillator.
  uint8_t read8(uint8_t addr);
  void write8(uint8_t addr, uint8_t d);
};

#endif
