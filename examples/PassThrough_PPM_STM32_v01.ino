//============================================================================================
// This example reads an PPM packet from a
// PPM receiver and then takes that
// packet and writes it to a servo driver 
//=============================================================================================


//use local copies of the libraries 
#include "src\PCA9685ServoDriver.h"
#include "src\SBUS.h"
#include "src\PPMReader.h"

// uncomment to print some debug messages
//#define ENABLE_DEBUG_OUTPUT
// example of use:
//#ifdef ENABLE_DEBUG_OUTPUT
//  Serial.println("Reset Completed, setting frequency..."); 
//#endif


//======setup I2C interface==============
TwoWire I2C_FAST (1,I2C_FAST_MODE); //I2C1 
//TwoWire I2C_FAST (2,I2C_FAST_MODE); //I2C2 
//========================================

//=======Set Up Servo Driver ==================================
// called this way, it uses the default address 0x40
//PCA9685ServoDriver pwm = PCA9685ServoDriver();
// you can also call it with a different address you want
//PCA9685ServoDriver pwm = PCA9685ServoDriver(0x41);
// you can also call it with a different address and I2C interface
//PCA9685ServoDriver pwm = PCA9685ServoDriver(&Wire, 0x40);

PCA9685ServoDriver pwm = PCA9685ServoDriver(&I2C_FAST, 0x40);
//==============================================================

//====Constants and global Variables==========================
// constants won't change. Used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin
// Variables will change:
int ledState = LOW;             // ledState used to set the LED
//=============================================================



//=================Set Up PPM receiver ======================
//set a pin number for PPM input 
int PPMinputPin=2;

// Initialize a PPMReader on digital pin 3 with 8 expected channels. 
//Note interrupt will be attached separately in Setup()
int channelAmount = 8;
PPMReader PPM(channelAmount);

//===========================================================

//=================SETUP()++++++++++++++++++++++++++++++
void setup() {
  
   // set the digital pin as output:
  pinMode(ledPin, OUTPUT);

//========================
   Serial.begin(9600);
//This way you won't loose any information output on USB serial, but the program will not go on without an established serial connection...
// while ( !Serial.isConnected() ) ; // wait till serial connection is setup, or serial monitor started
//or
//delay(6000); // on Win machine it takes 5 second to re-enumerate the USB COM port

for (int i=1; i<=15; i++){
  delay(1000);

  if (Serial){
  break;
  }

}
Serial.println("Setup() started ");
//========================

//begin the PPM communication
  //=======PPM setup=========
  //set the PPMinputPin as input,  pulled up for inverted polarity , pulled down for normal  
  pinMode(PPMinputPin, INPUT_PULLUP); 
  //attach interrupt  to the input pin.  The function is in the PPMReader class and it sets the interrupt pin and signal polarity  
  PPM.setupInterrupt(PPMinputPin, INVERTED);
  
  // correct the default values  
  // The range of a channel's possible values, microseconds
    PPM.minChannelValue = 700;
    PPM.maxChannelValue = 2200;

 // The minimum value (time) after which the signal frame is considered to
 // be finished and we can start to expect a new signal frame.
 // Walkera has 10000 us as blank frame    
    PPM.blankTime = 5000;

  //Calibration multipliers to apply to raw channel data values before 
  //they are returned as a normalised data (rawValues[i] * multiplierScale + multiplierBias;)
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
    PPM.multiplierScale = 1.0f;
    PPM.multiplierBias = 0.0f;
//====================================
Serial.println("PPM setup completed");


// begin the Servo Driver communication
  pwm.begin(); 
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates 
  Serial.println("Servo Driver setup completed");


Serial.println("Setup() completed");  
}

//==================LOOP()==============================================
void loop() {


    //for PPM
    //number of values to cover amount of channels, indexed {1..channelAmount}
	// for PPM, 8 channels, 1..8
    uint16_t channelsIN[channelAmount+1]; // for readRaw and  readNormalisedInteger  
    // float channelsIN[channelAmount+1];  // for readNormalisedFloat
    
    //for Servo Driver
    uint16_t channelsOUT[17];  // for write 16 channels, 1..16
    

 
    //for PPM
    //acquire the data into a local array
    if(PPM.readRaw(&channelsIN[0])!=0) {

	// or use readNormalised to get the calibrated values
	//   if(PPM.readNormalisedInteger(&channelsIN[0])!=0) {
    //   if(PPM.readNormalisedFloat(&channelsIN[0])!=0) {


	
  //blinking /serial debug
       ledState = HIGH;
       // set the LED with the ledState of the variable:
       //digitalWrite(ledPin, ledState);
       gpio_write_bit(GPIOB,1,ledState); 


  //print values  for PPM 
        // Print latest valid values from all channels
        for (int i = 1; i <= channelAmount; ++i) {
        Serial.print("Ch"+String(i)+":"+String(channelsIN[i]) + " ");
        }
        Serial.println();


    
  //write 16 channels to servo driver  for PPM
        for (uint8_t i=1; i<=16; i++) {
        //Map the values and write them to servo driver
           if (i<=channelAmount)  //channels supported by PPM, currently 8
           {
           //PPM returns microseconds, so no need for mapping 
           channelsOUT[i]=channelsIN[i];  
           pwm.setServo(i,channelsOUT[i] );
           }
           else {
                 channelsOUT[i]=1500; // set servo to a middle position 
                 pwm.setServo(i,channelsOUT[i] );
           }
        }
  }
  else{
       // set the LED to LOW  
		ledState = LOW;
       // set the LED with the ledState of the variable:
       // digitalWrite(ledPin, ledState);
       gpio_write_bit(GPIOB,1,ledState);
      
  }
 
  
}
