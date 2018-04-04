/*
RC Model logic 

This is an example of a class that implements a custom logic  
to drive servos. 

Note1: Before use with a real model, update values for 
SubTrim, TravelAdjustMin and TravelAdjustMax for each of 
your output channels in the .cpp file

Note2: As a reference - calculation time of this example 
of Calculate() function is around 50-60 microseconds. 

TODO:
-alternative failsafe settings - to stall the plane
-some data validation (if required?) 
-data archiving and input averaging  
- support servo hold for failsafe
=================================================================
(C)2018 ifh  
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

#ifndef MODEL_TEST1_H
#define MODEL_TEST1_H

#include "Arduino.h"


class Model {
	public:
		//Set Model_Test1 object
		Model();
	
		//Delete Model_Test1 object
		~Model();
	
		//The code is written with an assumption that  
		//channelAmountIn <= 16 and channelAmountOut <= 16. 
		//The amount of channels to be expected from the input signal.
		uint8_t channelAmountIn = 16;
	
		//The amount of channels to be expected for the output signal.
		uint8_t channelAmountOut = 16;
	
		//This function calculates outcome for servos in normal conditions
		// parameter chIN[] - an array of input values from receiver, pulse length in us 
		// parameter chOUT[] - an array of output to servo driver, pulse length in us
		// function output - chOUT[] array updated  		 
		void Calculate(uint16_t chIN[0], uint16_t chOUT[0]);
	
		//This function calculates outcome for servos in case a transmitter signal is lost
		// parameter chIN[] - an array of input values from receiver, pulse length in us 
		// parameter chOUT[] - an array of output to servo driver, pulse length in us
		// parameter isServoHoldMode - whether or not just hold the latest valid input data (optional)
		// function output - chOUT[] array updated
		void Failsafe(uint16_t chIN[0], uint16_t chOUT[0], bool isServoHoldMode = true);
	
		//This function passes the input to servos without changes
		// parameter chIN[] - an array of input values from receiver, pulse length in us 
		// parameter chOUT[] - an array of output to servo driver, pulse length in us
		// function output - chOUT[] array updated 
		void Passthrough(uint16_t chIN[0], uint16_t chOUT[0]);
		
		//CalcTime, micros
		uint32_t CalculationTime = 0;

	private:
		uint32_t _timestamp = 0;

		//Arrays for model adjustments in microseconds
		int16_t SubTrim[17];   // can hold  negative values
		uint16_t TravelAdjustMin[17];
		uint16_t TravelAdjustMax[17];
		
		//Arrays for pre-calculated values 
		//servo midpoints with and subtrim applied   
		uint16_t ServoMidPoint[17];
		//the range of a channel's normal values, microseconds with default values are for +/-100% and subtrim applied 
		uint16_t ServoLow[17];
	    uint16_t ServoHigh[17];
		
		//Default servo midpoint, microseconds
		static const  uint16_t ServoMidPointBase = 1500;
		// 1% of a servo range is  4 microseconds
		static const uint16_t pct2us = 4; 
		// 1 microsecond is 0.25% of a servo range     
		const float us2pct =  1/pct2us;  
		//The range of a channel's normal values, microseconds
		//default values are for +/-100% 
		static const uint16_t minNormalChannelValue = ServoMidPointBase - pct2us*100; //1100;
		static const uint16_t maxNormalChannelValue = ServoMidPointBase + pct2us*100; //1900;
		//The range of a channel's min/max possible values, microseconds
		//default values are for +/-150% plus/minus 100 us for contingency
		static const uint16_t minChannelValue = ServoMidPointBase - pct2us*150-100; //800;
		static const uint16_t maxChannelValue = ServoMidPointBase + pct2us*150+100; //2200;

	
		//Servo and logic helpers
		//This function reverses a servo around a midpoint
		// parameter in - pulse length in us 
		// parameter midpoint - midpoint position, us (optional) 
		// function output - pulse length in us
		uint16_t reverseServo(uint16_t in, uint16_t midpoint = ServoMidPointBase);
    
		//This function decodes 2 way switch
		// parameter in - pulse length in us 
		// parameter midpoint - midpoint position, us (optional) 
		// function output - switch position 0 and 1
		uint8_t twoWaySwitchDecoder(uint16_t in, uint16_t midpoint = ServoMidPointBase);
    
		//This function decodes 3 way switch
		// parameter in - pulse length in us 
		// parameter midpoint - midpoint position, us (optional) 
		// function output - switch position 0,1,2
		uint8_t threeWaySwitchDecoder(uint16_t in, uint16_t midpoint = ServoMidPointBase);
    
		//This function tells if the normal RC channel(-150% to +150% ) is less than specified percentage range
		// parameter in - pulse length in us 
		// parameter pct - percentage of the signal range of a normal RC channel(-150% to +150% ) 
		// parameter midpoint - midpoint position, us (optional) 
		// function output - pulse length in us
		bool isSignalLow(uint16_t in, int16_t pct, uint16_t midpoint = ServoMidPointBase);

		//This function mixes two channels as per specified weighting percentage
		// parameter in1 - pulse length in us 
		// parameter in2 - pulse length in us 
		// parameter pct1 - weighting percentage of the first signal (0% to +100% ) 
		// parameter midpoint1 - midpoint position for channel1, us (optional) 
		// parameter midpoint2 - midpoint position for channel2, us (optional)
		// parameter midpointOut - midpoint position for the resulted mix, us (optional)
		// function output - pulse length in us
		uint16_t twoChannelsMix(uint16_t in1, uint16_t in2, uint8_t pct1 = 50, uint16_t midpoint1 = ServoMidPointBase, uint16_t midpoint2 = ServoMidPointBase, uint16_t midpointOut = ServoMidPointBase);
	
		//These two functions  mixes two channels as per Vtail Mix rules, 
		//with an assumption that midpoint is common for all input and output channels
		//  Out1=0.5*(in1+in2)
		//  Out2=0.5*(in1-in2) + midpoint
		//
		// parameter in1 - pulse length in us 
		// parameter in2 - pulse length in us 
		// parameter midpoint - midpoint position for all channels (optional) 
		// function output - pulse length in us
		uint16_t twoChannelsVtailMixOut1(uint16_t in1, uint16_t in2, uint16_t midpoint = ServoMidPointBase);
		uint16_t twoChannelsVtailMixOut2(uint16_t in1, uint16_t in2, uint16_t midpoint = ServoMidPointBase);
	

		//This function  applies a servo travel limits (travel range asymmetrical mapping)
		// parameter in - pulse length in us 
		// parameter travelLimitLow - min target pulse length in us 
		// parameter travelLimitHigh - max target pulse length in us 
		// parameter midpoint - target midpoint position for servo, us (optional)
		// parameter defaultLow - min pulse length in us for normal(+/-100%) servo range (optional)
		// parameter defaultHigh - max pulse length in us for normal(+/-100%) servo range (optional) 
		// function output - pulse length in us mapped to the travel limit range
	    uint16_t applyTravelRangeLimits(uint16_t in, uint16_t travelLimitLow, uint16_t travelLimitHigh, uint16_t midpoint = ServoMidPointBase, uint16_t defaultLow = minNormalChannelValue, uint16_t defaultHigh = maxNormalChannelValue);
		/*
		//TODO - if a data archive is needed
		// arrays for data archives
		  uint16_t ArchiveIN0[17];
		  uint16_t ArchiveIN1[17];
		  uint16_t ArchiveIN2[17];
		  uint16_t ArchiveOUT0[17];
		  uint16_t ArchiveOUT1[17];
		  uint16_t ArchiveOUT2[17];
		
		//TODO - Data helpers
		// archive last input values
		// archive last output values
		// average last 3 input values
		// average last 3 output values
		*/
 
	};

#endif