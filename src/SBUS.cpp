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
SBUS.cpp
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
*/

// Set to true to print some debug messages, or false to disable them.
//#define ENABLE_DEBUG_OUTPUT_SBUS


// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC  || STM32L4 || Maple Mini
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
	defined(__MK66FX1M0__) || defined(__MKL26Z64__) || defined(__arm__) || (_BOARD_MAPLE_MINI_H_)

#include "Arduino.h"
#include "SBUS.h"

#if defined(__MK20DX128__) || defined(__MK20DX256__)
	// globals needed for emulating two stop bytes on Teensy 3.0 and 3.1/3.2
	IntervalTimer serialTimer;
	HardwareSerial* SERIALPORT;
	uint8_t PACKET[25];
	volatile int SENDINDEX;
	void sendByte();
#endif

/* SBUS object, input the serial bus */
SBUS::SBUS(HardwareSerial& bus){
    _bus = &bus;
}

/* Function to start the serial communication */
void SBUS::begin(){
#ifdef ENABLE_DEBUG_OUTPUT_SBUS
  Serial.println("SBUS::begin started"); 
#endif

	// initialize parsing state
	_fpos = 0;
	#if defined(__MK20DX128__) || defined(__MK20DX256__)  // Teensy 3.0 || Teensy 3.1/3.2
		// begin the serial port for SBUS
		_bus->begin(100000,SERIAL_8E1_RXINV_TXINV);
		SERIALPORT = _bus;
	#endif

	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)  // Teensy 3.5 || Teensy 3.6 || Teensy LC 
		// begin the serial port for SBUS
		_bus->begin(100000,SERIAL_8E2_RXINV_TXINV);
	#endif

	#if defined(__arm__) && !defined(_BOARD_MAPLE_MINI_H_)  // STM32L4
		// begin the serial port for SBUS
		_bus->begin(100000,SERIAL_SBUS);
    #endif
	
// Added Maple Mini
// definitions for serial frames are in
//  \Arduino\hardware\Arduino_STM32\STM32F1\cores\maple\HardwareSerial.h
	#if defined(_BOARD_MAPLE_MINI_H_) // Maple Mini

		// begin the serial port for SBUS
		_bus->begin(100000,SERIAL_8E2);
	#endif
	
	
#ifdef ENABLE_DEBUG_OUTPUT_SBUS
  Serial.println("SBUS::begin completed"); 
#endif	
}

/* Function to read the last available normalised SBUS data into an array. 
Returns a timestamp in microseconds to indicate when the data was received.
channels is an array of 17 values (indexed as 0..16) to cover the number  of channels from 1 to 16
plus a SBUS flags values as channel 0 */
uint32_t SBUS::readNormalisedInteger(uint16_t* channels){
bool _failsafe=false; 
uint16_t _lostFrames=0;
uint32_t _timestamp = readNormalisedInteger(channels,&_failsafe,&_lostFrames);
return _timestamp;
}

/* Function to read the last available normalised SBUS data into an array. 
Returns a timestamp in microseconds to indicate when the data was received.
channels is an array of 17 values (indexed as 0..16) to cover the number  of channels from 1 to 16
plus a SBUS flags values as channel 0 */
uint32_t SBUS::readNormalisedInteger(uint16_t* channels, bool* failsafe, uint16_t* lostFrames){
#ifdef ENABLE_DEBUG_OUTPUT_SBUS
  Serial.println("SBUS::readNormalisedFloat started"); 
#endif	
    //Set an array to get the Raw data 
	uint16_t _channels[17];
	
	//Get status of the data and the data itself into the channels local array
    uint32_t _timestamp = readRaw(&_channels[0],failsafe,lostFrames);
	
	//Calibrate the SBUS data if the data is ready (read() returned time stamp <> 0)
	if(_timestamp != 0){

		// linear calibration for the 16 channels 
    	for(uint8_t i = 1; i <= 16; i++){
      		channels[i] = (uint16_t) _channels[i] * multiplierScale + multiplierBias;
    	}
        // and return the 0 channel as is:
		channels[0] = _channels[0];
		
    	// return return timestamp in microseconds to indicate when the data was received 
		#ifdef ENABLE_DEBUG_OUTPUT_SBUS
         Serial.println("SBUS::readNormalisedFloat - completed, data ready"); 
        #endif 
    	return _timestamp;
  	}
  	else{

    	// return 0 if a full packet is not received
    	#ifdef ENABLE_DEBUG_OUTPUT_SBUS
         Serial.println("SBUS::readNormalisedFloat - completed, data not ready"); 
        #endif 
		return 0;
  }

}

/* Function to read the last available normalised SBUS data into an array. 
Returns a timestamp in microseconds to indicate when the data was received.
channels is an array of 17 values (indexed as 0..16) to cover the number  of channels from 1 to 16
plus a SBUS flags values as channel 0 */
uint32_t SBUS::readNormalisedFloat(float* channels){
bool _failsafe=false; 
uint16_t _lostFrames=0;
uint32_t _timestamp = readNormalisedFloat(channels,&_failsafe,&_lostFrames);
return _timestamp;
}

/* Function to read the last available normalised SBUS data into an array. 
Returns a timestamp in microseconds to indicate when the data was received.
channels is an array of 17 values (indexed as 0..16) to cover the number  of channels from 1 to 16
plus a SBUS flags values as channel 0 */
uint32_t SBUS::readNormalisedFloat(float* channels, bool* failsafe, uint16_t* lostFrames){
#ifdef ENABLE_DEBUG_OUTPUT_SBUS
  Serial.println("SBUS::readNormalisedFloat started"); 
#endif	
    //Set an array of 17 values (indexed as 0..16) to get the Raw data 
	uint16_t _channels[17];
	
	//Get status of the data and the data itself into the channels local array
    uint32_t _timestamp = readRaw(&_channels[0],failsafe,lostFrames);
	
	//Calibrate the SBUS data if the data is ready (read() returned time stamp <> 0)
	if(_timestamp != 0){

		// linear calibration for the 16 channels 
    	for(uint8_t i = 1; i <= 16; i++){
      		channels[i] = (float) _channels[i] * multiplierScale + multiplierBias;
    	}
        // and return the 0 channel as is:
		channels[0] = _channels[0];
		
    	// return return timestamp in microseconds to indicate when the data was received 
		#ifdef ENABLE_DEBUG_OUTPUT_SBUS
         Serial.println("SBUS::readNormalisedFloat - completed, data ready"); 
        #endif 
    	return _timestamp;
  	}
  	else{

    	// return 0 if a full packet is not received
    	#ifdef ENABLE_DEBUG_OUTPUT_SBUS
         Serial.println("SBUS::readNormalisedFloat - completed, data not ready"); 
        #endif 
		return 0;
  }

}

/* Function to read the last available raw SBUS data into an array. 
Returns a timestamp in microseconds to indicate when the data was received.
channels is an array of 17 values (indexed as 0..16) to cover the number  of channels from 1 to 16
plus a SBUS flags values as channel 0 */
uint32_t SBUS::readRaw(uint16_t* channels){
bool _failsafe=false; 
uint16_t _lostFrames=0;
uint32_t _timestamp = readRaw(channels,&_failsafe,&_lostFrames);
return _timestamp;
}

/* Function to read the last available raw SBUS data into an array. 
Returns a timestamp in microseconds to indicate when the data was received.
channels is an array of 17 values (indexed as 0..16) to cover the number  of channels from 1 to 16
plus a SBUS flags values as channel 0 */
uint32_t SBUS::readRaw(uint16_t* channels, bool* failsafe, uint16_t* lostFrames){
 #ifdef ENABLE_DEBUG_OUTPUT_SBUS
   Serial.println("SBUS::readRaw started"); 
 #endif 
    //Get status of the data 
    uint32_t _timestamp= parse();
 	//Parse the SBUS packet if the data is ready ( returned time stamp <> 0) 
  	if(_timestamp != 0){

    	// 16 channels of 11 bit data
    	channels[1]  = (int16_t) ((_payload[0]    |_payload[1]<<8)                          & 0x07FF);
    	channels[2]  = (int16_t) ((_payload[1]>>3 |_payload[2]<<5)                          & 0x07FF);
    	channels[3]  = (int16_t) ((_payload[2]>>6 |_payload[3]<<2 |_payload[4]<<10)  		& 0x07FF);
    	channels[4]  = (int16_t) ((_payload[4]>>1 |_payload[5]<<7)                          & 0x07FF);
    	channels[5]  = (int16_t) ((_payload[5]>>4 |_payload[6]<<4)                          & 0x07FF);
    	channels[6]  = (int16_t) ((_payload[6]>>7 |_payload[7]<<1 |_payload[8]<<9)   		& 0x07FF);
    	channels[7]  = (int16_t) ((_payload[8]>>2 |_payload[9]<<6)                          & 0x07FF);
    	channels[8]  = (int16_t) ((_payload[9]>>5 |_payload[10]<<3)                         & 0x07FF);
    	channels[9]  = (int16_t) ((_payload[11]   |_payload[12]<<8)                         & 0x07FF);
    	channels[10]  = (int16_t) ((_payload[12]>>3|_payload[13]<<5)                         & 0x07FF);
    	channels[11] = (int16_t) ((_payload[13]>>6|_payload[14]<<2|_payload[15]<<10) 		& 0x07FF);
    	channels[12] = (int16_t) ((_payload[15]>>1|_payload[16]<<7)                         & 0x07FF);
    	channels[13] = (int16_t) ((_payload[16]>>4|_payload[17]<<4)                         & 0x07FF);
    	channels[14] = (int16_t) ((_payload[17]>>7|_payload[18]<<1|_payload[19]<<9)  		& 0x07FF);
    	channels[15] = (int16_t) ((_payload[19]>>2|_payload[20]<<6)                         & 0x07FF);
    	channels[16] = (int16_t) ((_payload[20]>>5|_payload[21]<<3)                         & 0x07FF);
    	
		//return 23rd byte of protocol (22nd of payload) as a Channel 0
		//* Byte[23]:
        //* Bit 7: digital channel 17 (0x80)
        //* Bit 6: digital channel 18 (0x40)
        //* Bit 5: frame lost (0x20)
        //* Bit 4: failsafe activated (0x10)
        //* Bit 0 - 3: n/a
		//https://os.mbed.com/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
		//apparently Walkera DEVO 12E returns 3 when the receiver is binded and signal is ok,  otherwise 0 is returned for failsafe 
		channels[0] = (int16_t) (_payload[22]);
		
		// count lost frames
    	if (_payload[22] & _sbusLostFrame) {
      		*lostFrames = *lostFrames + 1;
    	}

    	// failsafe state
    	if (_payload[22] & _sbusFailSafe) {
      		*failsafe = true;
    	} 
    	else{
      		*failsafe = false;
    	}

    	// return timestamp in microseconds on receiving a full packet
		#ifdef ENABLE_DEBUG_OUTPUT_SBUS
         Serial.println("SBUS::readRaw - completed, data ready"); 
        #endif
    	return _timestamp;
  	}
  	else{

    	// return 0 if a full packet is not received
		#ifdef ENABLE_DEBUG_OUTPUT_SBUS
         Serial.println("SBUS::readRaw - completed, data ready"); 
        #endif
    	return 0;
  	}
}

/* Function to parse the SBUS data */
uint32_t SBUS::parse(){
#ifdef ENABLE_DEBUG_OUTPUT_SBUS
 Serial.println("SBUS::parse started"); 
#endif

//===========================
//Simon Levy's fork (https://github.com/simondlevy/SBUS) of Bolderflight SBUS library which was done for STM32:
// A workaround to emulate Teensy's elapsedTime support
    static uint32_t startTime;
    static uint32_t sbusTime;
    uint32_t currTime = micros();
    sbusTime = currTime - startTime;
    startTime = currTime;
//============================
    if(sbusTime > SBUS_TIMEOUT){_fpos = 0;}

  	// see if serial data is available
  	while(_bus->available() > 0){
      sbusTime = 0;
    	static uint8_t c;
      static uint8_t b;
      c = _bus->read();

    	// find the header
    	if(_fpos == 0){
      		if((c == _sbusHeader)&&((b == _sbusFooter)||((b & 0x0F) == _sbus2Footer))){
        		_fpos++;
      		}
      		else{
        		_fpos = 0;
      		}
    	}
    	else{

    		// strip off the data
    		if((_fpos-1) < _payloadSize){
      			_payload[_fpos-1] = c;
      			_fpos++;
    		}

    		// check the end byte
    		if((_fpos-1) == _payloadSize){
      			if((c == _sbusFooter)||((c & 0x0F) == _sbus2Footer)) {
        			_fpos = 0;
					//return a time stamp in microseconds to indicate when the data received 
					isDataReady=true;
			        dataInputTimeStamp=micros();
        			return dataInputTimeStamp;
      			}
      			else{
        			_fpos = 0;
					isDataReady=false;
			        dataInputTimeStamp=0;
        			return 0;
      			}
    		}
    	}
      b = c;
  	}
  	// return 0 if a partial packet
	isDataReady=false;
	dataInputTimeStamp=0;
  	return 0;
}

/* Function to return an indicator that PPM packet received */
bool SBUS::IsDataReady() {
return this->isDataReady;
}

/* Function to return a timestamp when PPM packet received 
or 0 if the current data packet is being received  */
uint32_t SBUS::GetDataInputTimeStamp() {
return this->dataInputTimeStamp;
}


/*Function to write SBUS packets, 
Channels is an array of 17 values (indexed as 0..16) to cover the number  of channels from 1 to 16
sbus flags (packet 23) are set to 0*/
void SBUS::write(uint16_t* channels){
#ifdef ENABLE_DEBUG_OUTPUT_SBUS
 Serial.println("SBUS::write started"); 
#endif


	uint8_t packet[25];

	/* assemble the SBUS packet */

	// SBUS header
	packet[0] = _sbusHeader; 

	// 16 channels of 11 bit data
  	packet[1] = (uint8_t) ((channels[1] & 0x07FF));
  	packet[2] = (uint8_t) ((channels[1] & 0x07FF)>>8 | (channels[2] & 0x07FF)<<3);
  	packet[3] = (uint8_t) ((channels[2] & 0x07FF)>>5 | (channels[3] & 0x07FF)<<6);
  	packet[4] = (uint8_t) ((channels[3] & 0x07FF)>>2);
  	packet[5] = (uint8_t) ((channels[3] & 0x07FF)>>10 | (channels[4] & 0x07FF)<<1);
  	packet[6] = (uint8_t) ((channels[4] & 0x07FF)>>7 | (channels[5] & 0x07FF)<<4);
  	packet[7] = (uint8_t) ((channels[5] & 0x07FF)>>4 | (channels[6] & 0x07FF)<<7);
  	packet[8] = (uint8_t) ((channels[6] & 0x07FF)>>1);
  	packet[9] = (uint8_t) ((channels[6] & 0x07FF)>>9 | (channels[7] & 0x07FF)<<2);
  	packet[10] = (uint8_t) ((channels[7] & 0x07FF)>>6 | (channels[8] & 0x07FF)<<5);
  	packet[11] = (uint8_t) ((channels[8] & 0x07FF)>>3);
  	packet[12] = (uint8_t) ((channels[9] & 0x07FF));
  	packet[13] = (uint8_t) ((channels[9] & 0x07FF)>>8 | (channels[10] & 0x07FF)<<3);
  	packet[14] = (uint8_t) ((channels[10] & 0x07FF)>>5 | (channels[11] & 0x07FF)<<6);  
  	packet[15] = (uint8_t) ((channels[11] & 0x07FF)>>2);
  	packet[16] = (uint8_t) ((channels[11] & 0x07FF)>>10 | (channels[12] & 0x07FF)<<1);
  	packet[17] = (uint8_t) ((channels[12] & 0x07FF)>>7 | (channels[13] & 0x07FF)<<4);
  	packet[18] = (uint8_t) ((channels[13] & 0x07FF)>>4 | (channels[14] & 0x07FF)<<7);
  	packet[19] = (uint8_t) ((channels[14] & 0x07FF)>>1);
  	packet[20] = (uint8_t) ((channels[14] & 0x07FF)>>9 | (channels[15] & 0x07FF)<<2);
  	packet[21] = (uint8_t) ((channels[15] & 0x07FF)>>6 | (channels[16] & 0x07FF)<<5);
  	packet[22] = (uint8_t) ((channels[16] & 0x07FF)>>3);

  	// flags
	packet[23] = 0x00;

	// footer
	packet[24] = _sbusFooter;

	#if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 || Teensy 3.1/3.2
		// use ISR to send byte at a time, 
		// 130 us between bytes to emulate 2 stop bits
		noInterrupts();
		memcpy(&PACKET,&packet,sizeof(packet));
		interrupts();
		serialTimer.priority(255);
		serialTimer.begin(sendByte,130);
	#endif

	#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)  // Teensy 3.5 || Teensy 3.6 || Teensy LC
		// write packet
		_bus->write(packet,25);
	#endif
	
	#if defined(_BOARD_MAPLE_MINI_H_) || defined(__arm__)// Maple Mini and also arm
		// write packet
		_bus->write(packet,25);
	#endif
}

// function to send byte at a time with
// ISR to emulate 2 stop bits on Teensy 3.0 and 3.1/3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 || Teensy 3.1/3.2
	void sendByte(){
		if(SENDINDEX < 25) {
			SERIALPORT->write(PACKET[SENDINDEX]);
			SENDINDEX++;
		}
		else{
			serialTimer.end();
			SENDINDEX = 0;
		}
	}
#endif



//end of class
#endif
