# BNO055 Gyro Compass
## Marine Rate Gyro and Heading sensor
## NMEA2000 style CAN Bus

## PGN's sent 
- PGN 127257 Attitude
- PGN 127250 Vessel Heading
- PGN 127251 Rate Of Turn
- PGN 127258 Magnetic Variation (calculated from GPS position reports by WMM2015)

## PGN's received
- PGN 127258 Magnetic Variation
- PGN 129029 GPS position
- PGN 126992 System Time

## Calibration status
Blink rate will indicate current IMU internal calibation status 
1 short blink every second indicates a good internal calibation

### Initial setup
Connect USB and open in serial terminal. The Teensy LED will blink one long flash every second while the calibration is missing.
Hold the BNO055 still for a few seconds in each orientation until the led blinking becomes a very short flash.
Send "s" character to device to save the current Accelerometer and Gyro calibration to EEPROM.
	
## MFD Compass calibration
As the BNO055 is strongly affected by the magnetic environment you will need to install the device in it's final location before calibrating the Magnatometer.
At an MFD set NMEA device instance to 200 to start magnatometer calibration.
Turn vessel in circles, Change NMEA device instance any other number to save magnatometer calibration to EEPROM


## MFD Sensor Orientation
At MFD locate device and set NMEA device instance to nx where n is mounting orientation and x is desired NMEA device number 
Based on Adafruit BNO055 module orientation

CODE|PCB	|Mounting|PCB Bottom 
---|---	|---	|---
0x | Horizontal	|-	|Port       
1x | Horizontal	|-	|Stbd       
2x | Horizontal	|-	|Fwd        
3x | Vertical	|Fwd	|Down       
4x | Vertical	|Aft	|Down       
5x | Vertical	|Stbd	|Down       
6x | Vertical	|Port	|Down       
7x | Vertical	|Aft	|Port       
8x | Vertical	|Aft	|Stbd      
9x | Vertical	|Stbd	|Aft	   
10x | Vertical	|Port	|Aft	   
11x | Vertical	|Fwd	|Port      
12x | Vertical	|Fwd	|Stbd      

### Tested Hardware
Teensy 3.2  
Bosch BNO055 Attitude Sensor connected via I2C  
MCP2562 CAN Transceiver  
LM7805 5V Linear regulator  
0.25A Poly fuse  


