/*
Original library is from https://github.com/bolderflight/SBUS and  https://github.com/simondlevy/SBUS
Updated by IF 
2018-01-13 
- includes support for Maple Mini board / STM32 (https://github.com/rogerclarkmelbourne/Arduino_STM32/)
- added a timestamp that indicates when the latest data was received
- channels renumbered to start from 1  
- return 23rd byte of SBUS protocol as Channel 0 to handle custom fail safe logic 
- debug functionality added 
- comments added
=============================================================================================
SBUS.h
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-13

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
================================================================================================
*/

#ifndef SBUS_H
#define SBUS_H

#include "Arduino.h"


class SBUS{
	public:
	    //Defines the SBUS object and the serial port to use 
		//for example, SBUS sbus(Serial1);
    	SBUS(HardwareSerial& bus);
		
		//Starts the SBUS communication
    	void begin();
		
		//Read the last available raw data, 
		//Returns a timestamp in microseconds to indicate when the data was received.
		//channels is an array from 0 to 17 to cover the number  of channels from 1 to 16
        //plus a SBUS flags values (23rd byte of SBUS protocol) as channel 0  */
		//channels is an array from 1 to 17. 16 channels from 1 to 16; channel 0 is a 23rd byte 
		//of SBUS protocol: 
        //* Bit 7: digital channel 17 (0x80)
        //* Bit 6: digital channel 18 (0x40)
        //* Bit 5: frame lost (0x20)
        //* Bit 4: failsafe activated (0x10)
        //* Bit 0 - 3: n/a
		//https://os.mbed.com/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
		//apparently Walkera returns 3 when the receiver is binded and signal is ok,  otherwise 0 is returned for failsafe 		
    	uint32_t readRaw(uint16_t* channels, bool* failsafe, uint16_t* lostFrames);
	    uint32_t readRaw(uint16_t* channels);
		
		//Functions to read the last available normalised data into an array.
		//Returns a timestamp in microseconds to indicate when the data was received.
        //channels is an array of 17 values (indexed as 0..16) to cover the number  of channels from 1 to 16
        //plus a SBUS flags values (23rd byte of SBUS protocol) as channel 0 */
		//failsafe and lostframes are calculated out of SBUS data when data packets are received 
		uint32_t readNormalisedInteger(uint16_t* channels, bool* failsafe, uint16_t* lostFrames);
		uint32_t readNormalisedInteger(uint16_t* channels);
    	uint32_t readNormalisedFloat(float* channels, bool* failsafe, uint16_t* lostFrames);
		uint32_t readNormalisedFloat(float* channels);

		
		//Generate a SBUS signal as per the channels array, Channels array shall have 17 elements;
        //channels are from 1 to 16; sbus flags (packet 23) are set to 0
     	void write(uint16_t* channels);

		
		//Calibration multipliers to apply to raw channel data values before 
	    //it is returned as a normalised data (rawValues[i] * multiplierScale + multiplierBias;)
		//Default values for Teensy and FrSky.	
		//They are for mapping the minimum and maximum values to +/- 1. When using 
		//an FrSky receiver and transmitter set to 100% range, these minimum value 
		//was 172 and the maximum was 1811. It may be different with a Futaba or other 
		//receivers and transmitters or if the transmitter range is set to other than 100%.
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

  		float multiplierScale = 0.00122025625f;
  		float multiplierBias = -1.2098840f;

		
		//Returns status of current data packet 
	    bool IsDataReady();
	
	    //Returns time in microseconds when the last data packet was received 
	    //or 0 if the current data packet is being received  
	    uint32_t GetDataInputTimeStamp();
		
		
		
		
  	private:
  		uint8_t _fpos;
        const uint16_t SBUS_TIMEOUT = 10000;
	
  		const uint8_t _sbusHeader = 0x0F;
  		const uint8_t _sbusFooter = 0x00;
        const uint8_t _sbus2Footer = 0x04;
  		const uint8_t _sbusLostFrame = 0x20; //0x04;
  		const uint8_t _sbusFailSafe = 0x10; //0x08;
  	
	static const uint8_t _payloadSize = 24;
  		uint8_t _payload[_payloadSize];
  		HardwareSerial* _bus;
  		
  		uint32_t parse();
		
		//Indicates that SBUS packet received and says when (in microseconds)
	    uint8_t isDataReady = 0;
	    uint32_t dataInputTimeStamp = 0;
		
		
};

#endif
