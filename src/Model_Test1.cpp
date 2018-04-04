/*
RC Model logic 

Note1: Update values for SubTrim, TravelAdjustMin and TravelAdjustMin for each of your 
output channels below.

Note1: The code is written with an assumption that  channelAmountIn <= 16 and channelAmountOut <= 16. 

Note2: In PPM there is only 8 channels and chIN[] can be less than 16. SBUS will return 16 channels. 
Make sure this file is updated accordingly to your receiver type and number of channels used - some
condition checking and calculations may not be done if not all the input channels are supplied. 

TODO - see the .h file 
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
  
 // Set to true to print some debug messages, or false to disable them.
//#define ENABLE_DEBUG_OUTPUT_MODEL 

#include "Arduino.h"
#include "Model_Test1.h"
 
   
  // Set Model object 
Model::Model() {

	//Set the values for output channels adjustments - Sub Trim. 
    //This is to set an offset for a servo position at midpoint.	
	SubTrim[0]=0; 	//Not Used
	SubTrim[1]=0; 	//ch01 SubTrim
	SubTrim[2]=0; 	//ch02 SubTrim
	SubTrim[3]=0; 	//ch03 SubTrim
	SubTrim[4]=-50; 	//ch04 SubTrim
	SubTrim[5]=0; 	//ch05 SubTrim
	SubTrim[6]=0; 	//ch06 SubTrim
	SubTrim[7]=0; 	//ch07 SubTrim
	SubTrim[8]=0; 	//ch08 SubTrim
	SubTrim[9]=0; 	//ch09 SubTrim
	SubTrim[10]=0; 	//ch10 SubTrim
	SubTrim[11]=0; 	//ch11 SubTrim
	SubTrim[12]=0; 	//ch12 SubTrim
	SubTrim[13]=0; 	//ch13 SubTrim
	SubTrim[14]=0; 	//ch14 SubTrim
	SubTrim[15]=0; 	//ch15 SubTrim
	SubTrim[16]=0; 	//ch16 SubTrim
 	
	//Set the values for output channels adjustments - Travel Adjust. 
    //This is to set minimum and maximum limits for movement of a servo. 	
	TravelAdjustMin[0]= 900; 	//Not Used
	TravelAdjustMin[1]= 900; 	//ch01 TravelAdjustMin
	TravelAdjustMin[2]= 900; 	//ch02 TravelAdjustMin
	TravelAdjustMin[3]= 900; 	//ch03 TravelAdjustMin
	TravelAdjustMin[4]= 930; 	//ch04 TravelAdjustMin
	TravelAdjustMin[5]= 900; 	//ch05 TravelAdjustMin
	TravelAdjustMin[6]= 900; 	//ch06 TravelAdjustMin
	TravelAdjustMin[7]= 900; 	//ch07 TravelAdjustMin
	TravelAdjustMin[8]= 900; 	//ch08 TravelAdjustMin
	TravelAdjustMin[9]= 900; 	//ch09 TravelAdjustMin
	TravelAdjustMin[10]= 900; 	//ch10 TravelAdjustMin
	TravelAdjustMin[11]= 900; 	//ch11 TravelAdjustMin
	TravelAdjustMin[12]= 900; 	//ch12 TravelAdjustMin
	TravelAdjustMin[13]= 900; 	//ch13 TravelAdjustMin
	TravelAdjustMin[14]= 900; 	//ch14 TravelAdjustMin
	TravelAdjustMin[15]= 900; 	//ch15 TravelAdjustMin
	TravelAdjustMin[16]= 900; 	//ch16 TravelAdjustMin
	

	TravelAdjustMax[0]= 2100, 	//Not Used
	TravelAdjustMax[1]= 2100; 	//ch01 TravelAdjustMax
	TravelAdjustMax[2]= 2100; 	//ch02 TravelAdjustMax
	TravelAdjustMax[3]= 2100; 	//ch03 TravelAdjustMax
	TravelAdjustMax[4]= 2000; 	//ch04 TravelAdjustMax
	TravelAdjustMax[5]= 2100; 	//ch05 TravelAdjustMax
	TravelAdjustMax[6]= 2100; 	//ch06 TravelAdjustMax
	TravelAdjustMax[7]= 2100; 	//ch07 TravelAdjustMax
	TravelAdjustMax[8]= 2100; 	//ch08 TravelAdjustMax
	TravelAdjustMax[9]= 2100; 	//ch09 TravelAdjustMax
	TravelAdjustMax[10]= 2100; 	//ch10 TravelAdjustMax
	TravelAdjustMax[11]= 2100; 	//ch11 TravelAdjustMax
	TravelAdjustMax[12]= 2100; 	//ch12 TravelAdjustMax
	TravelAdjustMax[13]= 2100; 	//ch13 TravelAdjustMax
	TravelAdjustMax[14]= 2100; 	//ch14 TravelAdjustMax
	TravelAdjustMax[15]= 2100; 	//ch15 TravelAdjustMax
	TravelAdjustMax[16]= 2100; 	//ch16 TravelAdjustMax
 	
	//Arrays for pre-calculated values: 
	//apply Subtrim values to default servo midpoints, 
	//low and high pulse lengths for normal (-100%/+100%) servo signal  
	for (uint8_t i=1; i<=channelAmountOut; i++) {
		ServoMidPoint[i]=ServoMidPointBase+SubTrim[i];
		ServoLow[i]=minNormalChannelValue+SubTrim[i];
		ServoHigh[i]=maxNormalChannelValue+SubTrim[i];
    }
		
}

// Delete Model object 
Model::~Model() {
    //delete [] chIN;
    //delete [] chOUT;
}
  
   
 //This function calculates outcome for servos in normal conditions 
// parameter chIN[] - an array of input values from receiver, pulse length in us 
// parameter chOUT[] - an array of output to servo driver, pulse length in us
// function output - chOUT[] array updated  
void Model::Calculate(uint16_t chIN[0], uint16_t chOUT[0]){ 
#ifdef ENABLE_DEBUG_OUTPUT_MODEL 
  Serial.println("Model::Calculate started"); 
#endif

 _timestamp=micros();

//=======CALCULATIONS STARTED=========================================================
 
/* Channels Usage Legend:
==Channels IN map:
1 Ail	Ail (Right)             
2 Elev 	Elev    
3 Tro	Throttle
4 Rudd	Rudder
5 Gear	Gear switch
6 Aux1	Ail (Left)        
7 Aux2	Fmod switch       Flight mode
8 Aux3	Aile DR switch    TV on/off 
9 Aux4	Elev DR switch    Snap Flaps on/off
10 Aux5	Mix switch        Flaps

==Channels OUT map:
1 Elev1 	Elev          =Elev IN;		if brake then Elev is full down
2 Ail	Ail (Right)       =Ail IN;		if brake then Ail is up,  if SnapFlaps then mix to Elev 
3 Tro	Throttle          =Tro IN;
4 Rudd	Rudder1           =Rudd IN;		if brake then Rudder1 is left 
5 Gear	Gear switch       =Gear IN;
6 Aux1	Ail (Left)        =Aux1 IN;		if brake then Ail is up,  if SnapFlaps then mix to Elev 
7 Rudder2	              =Rudd IN;		if brake then Rudder2 is left  
8 FrontWheelTurn          =Rudd IN;		allowed if gear is down - same as Rudd	
9 TV_Rudder               =Rudd IN;		allowed if TV is on -  same as Rudd
10 TV_Elev                =Elev IN;		allowed if TV is on -  same as Elev
11 Elev2                  =Elev IN;		if brake then Elev is full down


==Functional logic:
Brake: Fmod switch = 0  AND Gear switch = 0 AND Throttle <15% 
 Elev is full down, Ail is full up, Rudder1 and 2 are full left/right if Brake in ON

TV on: Aile DR switch = 1 

FrontWheelTurn: allowed if Gear switch = 1 

Snap Flap On: Elev DR switch = 1 
  Typical snap flap setup has both the ailerons moving together a few degrees 
  in the opposite direction of the elevator. In other words, with full up elevator 
  the ailerons should both move down a few degrees and with full down elevator the 
  ailerons should move up a few degrees. It sounds counterintuitive, but this 
  action allows the plane to make more aggressive  turns and loops when it's flying
  both upright and inverted. It also reduces stall speed and makes the model more stable.
  Mix:  30% of elevator mixed with Ail (Right) and Ail (Left) if  SnapFlap is ON  AND Brake is OFF 


*/  
 
 //==========Evaluate Conditions==============================================
 
 //TV is On
 bool _isTVOn = false;
 if(twoWaySwitchDecoder(chIN[8])==1)  {_isTVOn = true;}
 //Gear is Down
 bool _isGearDown = false;
  if(twoWaySwitchDecoder(chIN[5])==1)  {_isGearDown = true;}
 //Throttle is less than 15%
 bool _isThrottleLow = isSignalLow(chIN[3], 15, (ServoMidPointBase - pct2us*100));
 //Brake Mode is on
 bool _isBrakeOn = false;
 if (threeWaySwitchDecoder(chIN[7])==0 && _isGearDown && _isThrottleLow) {_isBrakeOn = true;}
 //Snap Flaps is on (and ensure the input channel is provided, as PPM reader supports only 8 input channels) 
 bool _isSnapFlapsOn = false;
   if(twoWaySwitchDecoder(chIN[9])==1 && channelAmountIn >=9)  {_isSnapFlapsOn = true;}
 
 
    //===Print debug values====================================================== 
#ifdef ENABLE_DEBUG_OUTPUT_MODEL 
  Serial.print("Model::Calculate _isTVOn       :"); Serial.println(String(_isTVOn)); 
  Serial.print("Model::Calculate _isGearDown   :"); Serial.println(String(_isGearDown)); 
  Serial.print("Model::Calculate _isThrottleLow:"); Serial.println(String(_isThrottleLow)); 
  Serial.print("Model::Calculate _isBrakeOn    :"); Serial.println(String(_isBrakeOn)); 
  Serial.print("Model::Calculate _isSnapFlapsOn:"); Serial.println(String(_isSnapFlapsOn));
#endif
 
    //===Print debug values======================================================
#ifdef ENABLE_DEBUG_OUTPUT_MODEL 
        // Print values from all input channels
        for (int i = 1; i <= channelAmountIn; ++i) {
        Serial.print("Ch"+String(i)+":"+String(chIN[i]) + " ");
        }
        Serial.println();
#endif
 
 
 
 
 //==========Set Output =========================================================

 
 //Set path-through channels: 
 chOUT[3]=chIN[3]; // 3 Tro  -> 3 Tro
 chOUT[5]=chIN[5]; // 5 Gear -> 5 Gear 
 
 //Set Elev L and R
	if (_isBrakeOn) {
	    // set Brake
		chOUT[1]= minNormalChannelValue;  //Elev1 is full down
		chOUT[11]= minNormalChannelValue;  //Elev2 is full down 
	} 
	else
	{
	    // set normal value
		chOUT[1] = chIN[2];
		chOUT[11] = chIN[2];
	} 
 
 //Set Ail L and R
 	if (_isBrakeOn) {
	    // set Brake
		chOUT[2]= maxNormalChannelValue;  //Ail is full up
		chOUT[6]= maxNormalChannelValue; //Ail is full up
	} 
	else
	{
	    // attempt to set normal value
		if (_isSnapFlapsOn) {
			// set SnapFlaps: 30% of elevator mixed with Ail (Right) and Ail (Left)
			chOUT[2]= twoChannelsMix(chIN[2], chIN[1], 30);
			chOUT[6]= twoChannelsMix(chIN[2], chIN[6], 30);
		}
		else
		{
			// set normal value
			chOUT[2] = chIN[1];
			chOUT[6] = chIN[6];
		}	
	}
  
 //Set Rudd1 and 2
  	if (_isBrakeOn) {
	    // set Brake
		chOUT[4]= minNormalChannelValue;  //Runner1 is full left
		chOUT[7]= maxNormalChannelValue; //rudder2 is full right
	} 
	else
	{
	    // set normal value
		chOUT[4] = chIN[4];
		chOUT[7] = chIN[4];
	}
  
 //Set FrontWheelTurn
   	if (_isGearDown) {
	    // set normal value
		chOUT[8] = chIN[4];  // same as Rudder
	} 
	else
	{
	    // set to midpoint
		chOUT[8] = ServoMidPointBase;
	}	
	
 //Set TV
    if (_isTVOn) {
	    // set normal value
		chOUT[9] = chIN[4];  // same as Rudder
		chOUT[10] = chIN[2];  // same as Elev
	} 
	else
	{
	    // set to midpoint
		chOUT[9] = ServoMidPointBase;
		chOUT[10] = ServoMidPointBase;
	}
 

    //===Print debug values===================================================================
#ifdef ENABLE_DEBUG_OUTPUT_MODEL 
        // Print values from all output channels
        for (int i = 1; i <= channelAmountOut; ++i) {
        Serial.print("Ch"+String(i)+":"+String(chOUT[i]) + " ");
        }
        Serial.println();
#endif

  //========Reverse if required (around default midpoint - subtrim will be applied in later) === 

	chOUT[2]=reverseServo(chOUT[2]); 
	chOUT[6]=reverseServo(chOUT[6]);


  //===========Apply Travel Range and Sub Trim ===================================================
 	for (uint8_t i=1; i<=channelAmountOut; i++) {
		chOUT[i]= applyTravelRangeLimits(chOUT[i] + SubTrim[i], TravelAdjustMin[i], TravelAdjustMax[i], ServoMidPoint[i],ServoLow[i],ServoHigh[i]); 
	}


  //===========Apply Constrain (if required) or set some specific values ========================== 
  // Note: flexible trim settings on receiver may cause output signal to go out of the travel range limits  
  	
	for (uint8_t i=1; i<=channelAmountOut; i++) {
	 chOUT[i]=constrain(chOUT[i], TravelAdjustMin[i], TravelAdjustMax[i]); 
	}
	
	
	
 //=======CALCULATIONS COMPLETED=====================================================
 
  CalculationTime = micros() - _timestamp;
  
#ifdef ENABLE_DEBUG_OUTPUT_MODEL 
  Serial.println("Model::Calculate completed"); 
#endif
} 


 //This function calculates outcome for servos in case a transmitter signal is lost
// parameter chIN[] - an array of input values from receiver, pulse length in us 
// parameter chOUT[] - an array of output to servo driver, pulse length in us
// parameter isServoHoldMode - whether or not just hold the latest valid input data (optional)
// function output - chOUT[] array updated
void Model::Failsafe(uint16_t chIN[0], uint16_t chOUT[0], bool isServoHoldMode){ 
  #ifdef ENABLE_DEBUG_OUTPUT_MODEL 
  Serial.println("Model::Failsafe started"); 
#endif

// TODO - to support servo hold an archive of latest available output values to be stored somewhere
//if (isServoHoldMode) {
// restore these values into chOUT[0] and return
//return;
// } 

  // otherwise set all servos to the middle 
  for (uint8_t i=1; i<=channelAmountOut; i++) {
  chOUT[i]=ServoMidPointBase;
  }

  //TODO - alternative settings - stall the plane
  //elev - UP
  //Rudder - Right
  //Ail - 0 
  
  
#ifdef ENABLE_DEBUG_OUTPUT_MODEL 
  Serial.println("Model::Failsafe completed"); 
#endif
}


//This function passes the input to servos without changes
// parameter chIN[] - an array of input values from receiver, pulse length in us 
// parameter chOUT[] - an array of output to servo driver, pulse length in us
// function output - chOUT[] array updated 
void Model::Passthrough(uint16_t chIN[0], uint16_t chOUT[0]){ 
#ifdef ENABLE_DEBUG_OUTPUT_MODEL 
  Serial.println("Model::Passthrough started"); 
#endif
  for (uint8_t i=1; i<=channelAmountOut; i++) {
    if (i<=channelAmountIn) { 
       chOUT[i]=chIN[i];
	}
    else
    {
     chOUT[i]=ServoMidPointBase;
    }	
  }

#ifdef ENABLE_DEBUG_OUTPUT_MODEL 
  Serial.println("Model::Passthrough completed"); 
#endif
} 



//Servo and logic helpers
//This function decodes 3 way switch
// parameter in - pulse length in us 
// parameter midpoint - midpoint position, us (optional) 
// function output - switch position 0,1,2
 uint8_t Model::threeWaySwitchDecoder(uint16_t in, uint16_t midpoint){
	if (in >= minChannelValue && in <= maxChannelValue){ //valid data  
     
      //input value is less than -30% from midpoint 
	  if (in < (uint16_t)(midpoint - pct2us*30)) {return 0;}
	  //input value is more than +30% from midpoint 
      if (in > (uint16_t)(midpoint + pct2us*30)) {return 2;}
	  else {return 1;}//input value is between +/- 30% from midpoint 
	    
    } 
	else   
	{ // invalid input, defaulted to 0
	 return 0; 
	} 
}  


//Servo and logic helpers  
//This function decodes 2 way switch
// parameter in - pulse length in us 
// parameter midpoint - midpoint position, us (optional) 
// function output - switch position 0 and 1
 uint8_t Model::twoWaySwitchDecoder(uint16_t in, uint16_t midpoint){
	if (in >= minChannelValue && in <= maxChannelValue){ //valid data  
     
      //input value is less than -5% from midpoint 
	  if (in < (uint16_t)(midpoint - pct2us*5)) {return 0;}
	  //input value is more than +5% from midpoint 
      if (in > (uint16_t)(midpoint + pct2us*5)) {return 1;}
	  else {return 0;}//input value is between +/- 5% from midpoint - defaulted to 0  
	    
    } 
	else   
	{ // invalid input, defaulted to 0 
	 return 0; 
	} 
}  

//Servo and logic helpers  
//This function reverses a servo around a midpoint
// parameter in - pulse length in us 
// parameter midpoint - midpoint position, us (optional) 
// function output - pulse length in us
 uint16_t Model::reverseServo(uint16_t in, uint16_t midpoint){

	if (in >= minChannelValue && in <= maxChannelValue){ //valid data  
		
		if (in>=midpoint){
		  return (midpoint-(in-midpoint));
		}
		else
        {
		  return (midpoint+(midpoint-in));
		}
  
	} 
	else   
	{ // invalid input, defaulted to midpoint
	 return midpoint; 
	} 

}


//Servo and logic helpers  
//This function tells if the normal RC channel(-150% to +150% ) is less than specified percentage range
// parameter in - pulse length in us 
// parameter pct - percentage of the signal range of a normal RC channel(-150% to +150% ) 
// parameter midpoint - midpoint position, us (optional) 
// function output - pulse length in us
 bool Model::isSignalLow(uint16_t in, int16_t pct, uint16_t midpoint){
	if (in < (midpoint + pct2us*pct)) {return true;} else {return false;}
}


//Servo and logic helpers  
//This function mixes two channels as per specified weighting percentage
// parameter in1 - pulse length in us 
// parameter in2 - pulse length in us 
// parameter pct1 - weighting percentage of the first signal (0% to +100% ) 
// parameter midpoint1 - midpoint position for channel1, us (optional) 
// parameter midpoint2 - midpoint position for channel2, us (optional)
// parameter midpointOut - midpoint position for the resulted mix, us (optional)
// function output - pulse length in us
uint16_t Model::twoChannelsMix(uint16_t in1, uint16_t in2, uint8_t pct1, uint16_t midpoint1, uint16_t midpoint2, uint16_t midpointOut){
//TODO - validate input. Now assume that input data is valid

  // weighting percentage for the second channel
  uint8_t pct2 = 100 - pct1;

  //relative shift of the input channels from their midpoints, weighed
  int16_t ch1DeltaValue = (uint16_t) ((in1-midpoint1)*pct1/100);
  int16_t ch2DeltaValue = (uint16_t) ((in2-midpoint2)*pct2/100);

   //return of sum of the relative shifts  and the midpointOut as base value  
  return (uint16_t)(midpointOut + ch1DeltaValue + ch2DeltaValue);
 
}

//Servo and logic helpers  
//These two functions  mixes two channels as per Vtail Mix rules, 
//with an assumption that midpoint is common for all input and output channels
//  Out1=0.5*(in1+in2)
//  Out2=0.5*(in1-in2) + midpoint
//
// parameter in1 - pulse length in us 
// parameter in2 - pulse length in us 
// parameter midpoint - midpoint position for all channels (optional) 
// function output - pulse length in us
uint16_t Model::twoChannelsVtailMixOut1(uint16_t in1, uint16_t in2, uint16_t midpoint){
//TODO - validate input. Now assume that input data is valid
  return (uint16_t)((in1+in2)/2);
}
uint16_t Model::twoChannelsVtailMixOut2(uint16_t in1, uint16_t in2, uint16_t midpoint){
//TODO - validate input. Now assume that input data is valid
  return (uint16_t)((in1-in2)/2 + midpoint);
}


//Servo and logic helpers  
//This function  applies a servo travel limits 
// parameter in - pulse length in us 
// parameter travelLimitLow - min target pulse length in us 
// parameter travelLimitHigh - max target pulse length in us 
// parameter midpoint - target midpoint position for servo, us (optional)
// parameter defaultLow - min pulse length in us for normal(+/-100%) servo range (optional)
// parameter defaultHigh - max pulse length in us for normal(+/-100%) servo range (optional) 
// function output - pulse length in us mapped to the travel limit range
 uint16_t Model::applyTravelRangeLimits(uint16_t in, uint16_t travelLimitLow, uint16_t travelLimitHigh, uint16_t midpoint, uint16_t defaultLow, uint16_t defaultHigh){
	//do the asymmetric  mapping around midpoint - apply mapping twice 
	//reference: map(value, fromLow, fromHigh, toLow, toHigh)
	if (in>=midpoint) {
	    // input value is above the midpoint, mapping from midpoint to the Travel Adjust high range 
		return map(in, midpoint, defaultHigh, midpoint, travelLimitHigh);
	}
	else
	{
	    // input value is below the midpoint, mapping from midpoint to the Travel Adjust low range 
		return map(in, defaultLow, midpoint, travelLimitLow, midpoint);
	}

}	

