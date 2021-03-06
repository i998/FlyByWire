
## Overview:
The Fly By Wire (FBW) device has been designed to control RC models with multiple control surfaces and servos - by allowing a custom control logic to get an input signal from a RC receiver and calculate required output for servos.  
That would help with the following: 
   - Control models such as EDF jets with two rudders,   trust vectoring etc 
   - Implement complex mixes: spoilerons,  tailerons,  snap  flaps etc.    
   - Implement  several control modes and  switch  between them easily  
   - Allow to use RX transmitters/receivers with small number of channels and/or when 
     channel mixes available in transmitter are not sufficient.  

It is an on-board controller which able to receive an SBUS or PPM input signals and generate PWM or SBUS output signals to drive the servos. 


    
## Hardware:
The FBW consists of three key components:
   -  Inverter  - to  invert the input signal (SBUS is an inverted RS232 with 8E2 frame) and convert it to 3.3 v
   -  Maple  Mini CPU module (https://stm32duinoforum.com/forum/wiki_subdomain/index_title_Maple_Mini.html)
   -  PCA 9685 servo driver  (https://www.adafruit.com/product/815)

Maple Mini was selected as a CPU module over Arduino ones as it has more processing power, has hardware I2C interface and a real serial port. Other board to consider is Teensy (https://www.pjrc.com/teensy/) however it would require some pin allocation changes and code updates.

For the PCA 9685 servo driver board the following changes were made:
   - I2C pull up resistors (10K to +5v) removed and replaced with 3.2k resistors pulled up to 3.3v at MapleMini side 
   - Pins to supply power to servos removed – 16 active servos could potentially take 16A of current, so it would be good to have a separate servo power supply which connected directly to the servos.  The FBW provides  servo control signals only. 
  
The PCA 9685 servo driver boards are chainable so you could control more than 16 servos by adding more servo driver boards.  


## Software:
The FBW software is based on the following:
  - Arduino for STM32 (https://github.com/rogerclarkmelbourne/Arduino_STM32)
  - SBUS library 
  - PPM library 
  - RC Model library
  - Servo Driver library      

The original SBUS, PPM and Servo Driver libraries were customised to suit the FBW needs and therefore they are included into this repository as local versions. 

The FBW software reads and analyses an input signal using SBUS or PPM libraries, once the signal is valid it then calls a Calculate method of an RC Model library. The RC Model class is where the input/output servo channel definitions and all your custom logic are placed. The Servo Driver library then writes the output values (servo pulses lengths in microseconds) to the PCA 9685 chip which generates the output signals to your servos.  Alternatively, you can use SBUS library and generate an SBUS output at TX pin of a Maple Mini serial port.     

      
## Known Limitations:
  - Max input voltage is 5.5v,   limited by the PCA 9685 chip input voltage specifications.  
  - A PWM servo signal resolution depends on the servo signal frequency (the higher frequency the better resolution). 
    At 50Hz servo signal frequency the resolution is about 5 microseconds, increasing the frequency improves the resolution 
  - PCA 9685 servo driver board may have its own internal oscillator running on a frequency that is not exactly 25 Mhz. You may need to adjust configuration settings. Please see the FlyByWire_STM32_FreqTest example provided. 	

	
## Repository Contents:
   - /documentation - A circuit diagram, Data sheets, additional information
   - /src – a set of libraries 
   - /examples - examples on how the libraries can be used 


## Further improvements:
 The following has been thought but not implemented: 
   - Logging input/output data to a hardware logging  device, 
     for example https://github.com/sparkfun/OpenLog or https://github.com/sparkfun/Qwiic_OpenLog
   - GPS data logging  
   - On-board current measurement, for example with  a Hall Effect-Based Current Sensor Allegro ACS770xCB
   - On-board voltage measurement 

   
## License:
Fly By Wire is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Fly By Wire is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.



## P.S.
If someone is interested to design and produce a single PCB with SMD components to reduce size and 
weight of the device please feel free to contact. 

## Donation:
If you like this project or it helps you to reduce your development effort, you can buy me a cup of coffee :) 

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/donate/?business=ifhone777-hub%40yahoo.com&currency_code=USD)
