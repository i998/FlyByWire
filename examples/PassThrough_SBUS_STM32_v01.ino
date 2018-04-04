//============================================================================================
// This example reads an SBUS packet from a
// SBUS receiver and then takes that
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
//============================================================

//====Constants and global Variables==========================
// constants won't change. Used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin
// Variables will change:
int ledState = LOW;             // ledState used to set the LED
//=============================================================



//=================Set Up SBUS receiver ======================
// a SBUS object, which is on STM32 hardware
// serial port 1
SBUS sbus(Serial1);
//===================================================

//=================SETUP()++++++++++++++++++++++++++++++
void setup() {
  
// set the digital pin as output:
  pinMode(ledPin, OUTPUT);

//==Serial Debug ==========
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


  
// begin the SBUS communication
  sbus.begin();
	
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
	sbus.multiplierScale = 1.0f;
	sbus.multiplierBias = 0.0f;
	
  Serial.println("SBUS setup completed");



// begin the Servo Driver communication
  pwm.begin(); 
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates 
  Serial.println("Servo Driver setup completed");


Serial.println("Setup() completed");  
}

//==================LOOP()==============================================
void loop() {

    //for SBUS
    
    bool failSafeSBUS = false;
    bool failSafeLocal = false;
    uint16_t lostFrames = 0;
    //17 values, indexed {0..16}
    uint16_t channelsIN[17];  // for readRaw and  readNormalisedInteger
    //float channelsIN[17];   // for readNormalisedFloat

       
    //for Servo Driver
    uint16_t channelsOUT[17];  // for write 16 channels, 1..16
    

  // for SBUS
  // look for a good SBUS packet from the receiver received and parced successfully 
   if(sbus.readRaw(&channelsIN[0], &failSafeSBUS, &lostFrames)!=0){

  // or use readNormalised to get the calibrated values
  //  if(sbus.readNormalisedInteger(&channelsIN[0], &failSafeSBUS, &lostFrames)!=0){
  //  if(sbus.readNormalisedFloat(&channelsIN[0], &failSafeSBUS, &lostFrames)!=0){


  //blinking /serial debug
       ledState = HIGH;
       // set the LED with the ledState of the variable:
       //digitalWrite(ledPin, ledState);
       gpio_write_bit(GPIOB,1,ledState); 


  //print values  for SBUS    
        // Print latest valid values from all channels
        for (int i = 1; i <= 16; ++i) {
        Serial.print("Ch"+String(i)+":"+String(channelsIN[i]) + " ");
        }
        Serial.print(" Ch0:");Serial.print(channelsIN[0]);  //"Byte 23 of SBUS protocol"                                              
        Serial.print(" FailSafe:");
        Serial.print(failSafeSBUS);
        Serial.print(" LostFrames:");
        Serial.print(lostFrames);
        Serial.print(" micros:");
        Serial.print(sbus.GetDataInputTimeStamp());
        Serial.println();


  //write 16 chammels to servo driver for SBUS  
       for (uint8_t i=1; i<=16; i++) {
       //Map the values (Walkera  DEVO 12E transmitter) 
       channelsOUT[i]= map (channelsIN[i], 32,1952, 900,2100); // map SBUS values to Servo microseconds (up to 150% range in transmitter)  
       pwm.setServo(i,channelsOUT[i] );
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
