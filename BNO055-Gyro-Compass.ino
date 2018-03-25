#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>
#include <NMEA2000_CompilerDefns.h>
#include <NMEA2000.h>
#include <N2kStream.h>
#include <N2kGroupFunctionDefaultHandlers.h>
#include <N2kGroupFunction.h>
#include <N2kDeviceList.h>
#include <N2kDef.h>
#include <N2kCANMsg.h>
#include <N2kMessagesEnumToStr.h>
#include <i2c_t3.h> 
#include "BNO055.h"
#include <EEPROM.h>
#include <TimeLib.h>

// Pin definitions
#define rstPin 15 // BNO055 Reset pin
#define intPin 12 // BNO055 Interrupt pin 
#define stbyPin 14 // CAN tranciver standby pin

#define MSS_TO_G 0.10197162129779 // acceleration
#define EEPROM_ADR_CONFIG 100 // eeprom address for config params

time_t NetTime = 0;  //NMEA network time is invalid

volatile boolean flagDataReady;  // interrupt flag
volatile boolean flagCalibrateMag; // calibration running

// NMEA Atitude X Axis: roll=port up+, Y Axis: pitch=bow up+,  Z Axis: heading
enum mountingOrientation {
	// Case				Bulkhead	Connector	yaw axis
	HL, // Horizontal 	-			Left		GZ
	HR, // Horizontal 	-			Right		GZ
	HF, // Horizontal 	-			Fwd			GZ
	VFD, // Vertical	Fwd			Down		GX
	VAD, // Vertical	Aft			Down		GX
	VSD, // Vertical	Stbd		Down		GX
	VPD, // Vertical	Port		Down		GX
	VAL, // Vertical	Aft			Left		GY
	VAR, // Vertical	Aft			Right		GY
	VSA, // Vertical	Stbd		Aft			GY
	VPA, // Vertical	Port		Aft			GY
	VFL, // Vertical	Fwd			Left		GY
	VFR,  // Vertical	Fwd			Right		GY
	X  // TEST
};

// 9dof IMU Bosch BNO055
BNO055 IMU;
BNO055::accOffset AccOffset;
BNO055::magOffset MagOffset;
BNO055::gyroOffset GyroOffset;
uint8_t systemCalStatus, gyroCalStatus, accelCalStatus, magCalStatus;
int8_t beginStatus; // stores result of IMU.begin() operation
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t accelCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t EulCount[3];    // Stores the 16-bit signed Euler angle output

// EEPROM configuration structure
#define MAGIC 10106 // EPROM struct version check, change this whenever tConfig structure changes
struct tConfig {
	uint16_t Magic; //test if eeprom initialized
	BNO055::accOffset accOffset;
	BNO055::magOffset magOffset;
	BNO055::gyroOffset gyroOffset;
	time_t maglastcaltime;
	time_t gyrolastcaltime;
	time_t acclastcaltime;
	time_t variationAge; // mag var age
	double variation; // magnetic variation
	bool gyroCalComplete;
	bool magCalComplete;
	bool accCalComplete;
	uint8_t deviceInstance;
	uint8_t orientation;
};

// EEPROM contents
tConfig config;

// Default EEPROM contents
const tConfig defConfig PROGMEM = {
	MAGIC,
	0,0,0,1000,	// acc XYZ radius
	0,0,0,1000,	// mag XYZ radius
	0,0,0,	// gyro XYZ
	0,  //mag cal datetime
	0,	//gyro cal datetime
	0,  //acc cal datetime
	0,  //variationAge datetime
	-10.59,  //variation degrees Annapolis 2017
	0,  //mag cal flag
	0,	//gyro cal flag
	0,	//acc cal flag
	0,  //NMEA device instance
	mountingOrientation::VPD // mountingOrientation
};

// output values
float ax, ay, az, gx, gy, gz;
float pitch, yaw, yawrate, roll, heading;
double magneticvariation; // magnetic declination

// serial stream
Stream *OutputStream;
template<typename T> void PrintLabelValWithConversionCheckUnDef(const char* label, T val, double(*ConvFunc)(double val) = 0, bool AddLf = false) {
	OutputStream->print(label);
	if (!N2kIsNA(val)) {
		if (ConvFunc) { OutputStream->print(ConvFunc(val)); }
		else { OutputStream->print(val); }
	}
	else OutputStream->print("not available");
	if (AddLf) OutputStream->println();
}

//*****************************************************************************
tN2kMsg N2kMsg; // N2K message constructor
int SID = 1; // N2K data packet squence id
// configure one device 
#define DEV_COMPASS 0 // NMEA 60-140
// Attidue sensor
const unsigned long TransmitMessagesCompass[] PROGMEM = { 127258L, 127250L,127251L, 127257L , 0 }; //Vessel Heading, Rate of Turn, Attitude
// 	Received Messages			
const unsigned long ReceiveMessage[] PROGMEM = { 127258L, 126992L,129029L,0 }; // magnetic variation, systemtime, GNSS Position
// N2K message handler
typedef struct {
	unsigned long PGN;
	void(*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;
// register the handlers
tNMEA2000Handler NMEA2000Handlers[] = { { 127258L,&MagneticVariationMsg },{ 126992L,&SystemTimeMsg },{ 129029L,&PositionMsg },{ 0,0 } };
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
	int iHandler;
	for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++);
	if (NMEA2000Handlers[iHandler].PGN != 0) {
		NMEA2000Handlers[iHandler].Handler(N2kMsg);
	}
}
// convert N2K days since1970 to unix time seconds since 1970
time_t N2KDate(uint16_t DaysSince1970) {
	return DaysSince1970 * 24 * 3600;
}

//*****************************************************************************
void setup() {
	Serial.begin(115200);
	OutputStream = &Serial;
	Blink(10, 100); //flash LED 10x in 1s
	pinMode(LED_BUILTIN, OUTPUT); //enable LED
	digitalWrite(LED_BUILTIN, HIGH);
	pinMode(stbyPin, OUTPUT);  // take CAN chip out of standby
	digitalWrite(stbyPin, LOW);
	// Get device configuration from EEPROM
	bool validConfig;
	ReadConfig();
	if (config.Magic != MAGIC || (config.magCalComplete == 0 && config.accCalComplete == 0 && config.gyroCalComplete == 0)) {
		InitializeEEPROM();
		Serial.println(F("\nNo stored calibration\r\n Press 'c' to run mag cal"));
	}
	else
	{
		Serial.println(F("Loading stored calibration"));
		validConfig = true;
	}

	// Setup IMU
	OutputStream->println("\nStarting IMU...");
	ResetIMU(); //hard reset IMU
	InitializeIMU();
	if (validConfig) { 
		LoadConfig();
		delay(2000);	
		SetOrientation(config.orientation);
		PrintConfig();
	}
	// Setup NMEA2000
	NMEA2000.SetInstallationDescription1("Mr Bubble Compass");
	NMEA2000.SetProductInformation("072462", // Manufacturer's Model serial code
		666, // Manufacturer's product code
		"Gyro Compass",  // Manufacturer's Model ID
		"1.0.0.0",  // Manufacturer's Software version code
		"1.0.0.0", // Manufacturer's Model version
		2,	// load equivalency *50ma
		0xffff, // NMEA 2000 version - use default
		0xff, // Sertification level - use default
		DEV_COMPASS
	);
	// Set device information
	NMEA2000.SetDeviceInformation(072462, // Unique number. Use e.g. Serial number.
		140, // Device function=Environment See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
		60, // Device class=Sensor Communication Interface. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
		2040, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
		4,
		DEV_COMPASS
	);
	NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, config.deviceInstance);
	NMEA2000.EnableForward(false);
	NMEA2000.ExtendTransmitMessages(TransmitMessagesCompass, DEV_COMPASS);
	NMEA2000.SetN2kCANMsgBufSize(5);
	NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
	NMEA2000.Open();
	PrintHelp(); 
}

// loop status flags
bool SDEBUG = false; // toggle debug print
bool SPRINT = false; // toggle send test output
bool SDATA = false;   // toggle print data values
bool SMAGVAR = false;  // toggle print variation 
bool STIME = false;   // toggle print nettime
bool SGNSS = false;  // GNSS/GPS position reports
// loop timers
long slowloop = 0; // 1s loop
volatile int samplerate = 0;
float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval
uint32_t delt_n = 0; uint32_t N; uint16_t flashDuration = 0;//led flash
time_t ElapsedTime; // time since magcal start

// main loop
void loop() {
	// process serial input from USB debug port
	char command = getCommand();
	switch (command){
		case '0': SetOrientation(mountingOrientation::HL); break;
		case '1': SetOrientation(mountingOrientation::HR); break;
		case '2': SetOrientation(mountingOrientation::HF); break;
		case '3': SetOrientation(mountingOrientation::VFD); break;
		case '4': SetOrientation(mountingOrientation::VAD); break;
		case '5': SetOrientation(mountingOrientation::VSD); break;
		case '6': SetOrientation(mountingOrientation::VPD); break;
		case '7': SetOrientation(mountingOrientation::VAL); break;
		case '8': SetOrientation(mountingOrientation::VAR); break;
		case '9': SetOrientation(mountingOrientation::VSA); break;
		case 'a': SetOrientation(mountingOrientation::VPA); break;
		case 'b': SetOrientation(mountingOrientation::VFL); break;
		case 'c': SetOrientation(mountingOrientation::VFR); break;
	
		case 'm': // start mag calibration, non-blocking	
			if (!flagCalibrateMag){
				StartMagCal();
			}
			else{
				Serial.println(F("Calibration already running!"));
			}
			break;
		case 'd':  // print data
			SDATA = !SDATA;
			break;
		case 'f': // finish mag calibration
			if(flagCalibrateMag)
				FinishMagCal();
			break;
		case 'h':  
			PrintHelp();
			break;
		case 'p':
			PrintConfig();
			PrintIMUConfig();
			break;
		case 'r':  //
			Serial.println("HARD RESET");
			ResetIMU(); //hard reset IMU
			delay(5000);
			LoadConfig(); //load offsets
			SetOrientation(config.orientation);

			break;
		case 'R':  // soft reset
			Serial.println("SOFT RESET");
			IMU.reset(); //soft reset IMU
			Serial.print("Boot time:"); Serial.print(IMU.GetSampleCount()); Serial.println("ms");
			LoadConfig();
			SetOrientation(config.orientation);
			delay(1000);
			break;
		case 's':  // save current IMU settings to EEPROM
			if (systemCalStatus == 3) {
				SaveCurrentSettings();
			}
			else { Serial.print("System CalStatus<>3 not saving");}
			break;
		case 't': //toggle printing received system time 
			STIME = !STIME;
			break;
		case 'v': // print recived magnetic variation
			SMAGVAR = !SMAGVAR;
			break;
		case 'w':  // Write saved config to IMU
			LoadConfig();
			break;	
		case 'x': // toggle debug
			SDEBUG = !SDEBUG;
			SDATA = false;
			break;
		case 'z': // toggle debug
			SPRINT = !SPRINT;
			break;
		default:
			break;
	}

	//blink length
	delt_n = millis() - N;
	if (delt_n > flashDuration) {
		N = millis(); 
		digitalWrite(LED_BUILTIN,false);
	}

	// 50ms
	if (flagCalibrateMag && delt_t % 50 == 0) { magCalLoop(); }

	// 250ms 
	delt_t = millis() - count;
	if (delt_t > 250) { // fast update once per 250ms independent of read rate
		Now = micros();
		deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
		lastUpdate = Now;
		count = millis();

		if (!flagCalibrateMag)
		{		
			IMU.readGyroData(gyroCount);
			// convert gyro counts into degrees per second
			gz = (float)gyroCount[2] / 16.;  
			yawrate = -gz;

			IMU.readEulData(EulCount);  
		    // convert Euler counts to degrees
			yaw = (float)EulCount[0] / 16.;
			roll = -(float)EulCount[1] / 16.;
			pitch = -(float)EulCount[2] / 16.;
			//heading = yaw +magneticvariation;
			heading = yaw;

			// send magnetic heading 
			SetN2kMagneticHeading(N2kMsg, SID, heading*DEG_TO_RAD, N2kDoubleNA, magneticvariation*DEG_TO_RAD);
			NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);

			// Send Rate of Turn 
			double RoT = yawrate; //  deg/s 
			//RoT = RoT*RAD_TO_DEG;
			if (RoT > -0.1 && RoT < 0) RoT = 0; // Remove minus sign flipping on display for small negative values
			RoT = RoT*DEG_TO_RAD; // radians per second
			RoT /= 10; // NOTE MFD's are showing wrong value, this looks correct 
			SetN2kRateOfTurn(N2kMsg, SID, RoT); 
			NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
			
			// stream messages over serial
			if (SDATA)	printData();
			if (SDEBUG) PrintIMUConfig();
			
			// increment N2K SID
			SID++; if (SID > 252) { SID = 1; } 
			samplerate = 0;
		}
		slowloop++;
	}

	// 1000ms
	if (slowloop > 3) { slowloop = 0; SlowLoop(); }
	NMEA2000.ParseMessages();

}

// slow message loop
void SlowLoop() {
	// Slow loop N2K message processing
	// send magnetic variation
	if (config.variationAge > 0) {
		SetN2kMagneticVariation(N2kMsg, SID, N2kmagvar_WMM2015, elapsedDays(config.variationAge), magneticvariation*DEG_TO_RAD);
		NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);
	}

	// Attitude 
	SetN2kAttitude(N2kMsg, SID, yaw*DEG_TO_RAD, pitch*DEG_TO_RAD, roll*DEG_TO_RAD);
	NMEA2000.SendMsg(N2kMsg, DEV_COMPASS);

	// announce calibration status with LED flash length
	digitalWrite(LED_BUILTIN, true);
	//get system calibration status
	IMU.getCalibration(&systemCalStatus, &gyroCalStatus, &accelCalStatus, &magCalStatus);
	// change LED flash duration
	flashDuration = 500 - systemCalStatus * 150;  //3=50ms 2=200ms 1=350ms 0=500ms

	//show mag cal loop state
	if (flagCalibrateMag) {
		PrintCalStatus();
		Serial.print(F("Mag samples: "));	Serial.print(IMU.GetSampleCount());
		Serial.print("Elapsed: ");	Serial.print(NetTime - ElapsedTime); Serial.println(" s");
	}

	// check device instance for instructions from MFD
	tNMEA2000::tDeviceInformation DeviceInformation = NMEA2000.GetDeviceInformation(DEV_COMPASS);
	uint8_t DeviceInstance = DeviceInformation.GetDeviceInstance();
	// set instance to 200 to start magcalibration and swing compass.
	if (DeviceInstance == 200) {
		flagCalibrateMag = true;
		StartMagCal();
	}
	//Calibration finished
	else if (flagCalibrateMag && DeviceInstance != 200) {  // if running exit cal by changing instance from 62.
		FinishMagCal();
	}

	if (DeviceInstance != 200 && DeviceInstance != config.deviceInstance) {
		config.deviceInstance = DeviceInstance;
		UpdateConfig();
	}
}

// read serial input
char getCommand()
{
	char c = '\0';
	if (Serial.available())
	{
		c = Serial.read();
	}
	return c;
}

// NMEA Messages *****************************************************************
// process SystemTime recieved message
void SystemTimeMsg(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	uint16_t SystemDate;
	double SystemTime;
	unsigned long time = 0L;
	const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

	tN2kTimeSource TimeSource;
	if (ParseN2kSystemTime(N2kMsg, SID, SystemDate, SystemTime, TimeSource)) {
		time = SystemDate * 3600 * 24 + SystemTime;
		if (time < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
			time = 0L; // return 0 to indicate that the time is not valid
		}
		if (time) {
			NetTime = time;
			setTime(NetTime);
			//digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		}
		else { 
			// time is off by 60s so invalidate nettime
			if(abs(NetTime-now()) > 60) NetTime = 0;
		}
		// serial print time
		if (STIME) { 
			Serial.print("Last Network time:"); PrintDateTime(NetTime); PrintN2kEnumType(TimeSource, OutputStream);
			Serial.print("\nCurrent local time:"); PrintDateTime(now());
		}
	}
	else {
		OutputStream->print(F("Failed to parse SystemTime PGN: ")); OutputStream->println(N2kMsg.PGN);
	}
}

// process position recieved message and use this to calculate magnetic variation
void PositionMsg(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	uint16_t DaysSince1970;
	double SecondsSinceMidnight;
	double Latitude;
	double Longitude;
	double alt;
	tN2kGNSStype GNSStype;
	tN2kGNSSmethod GNSSmethod;
	unsigned char nSatellites;
	double HDOP;
	double PDOP;
	double GeoidalSeparation;
	unsigned char nReferenceStations;
	tN2kGNSStype ReferenceStationType;
	uint16_t ReferenceSationID;
	double AgeOfCorrection;
	if (ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
		Latitude, Longitude, alt,
		GNSStype, GNSSmethod,
		nSatellites, HDOP, PDOP, GeoidalSeparation,
		nReferenceStations, ReferenceStationType, ReferenceSationID,
		AgeOfCorrection)) {
		// valid fix and time
		if((uint8_t)GNSSmethod > 0 && NetTime>0)
		{
			double h = alt / 1000.0;  //altitude offset, km from geoid?
			// calculate current variation from WMM2015
			double magvar = RAD_TO_DEG*SGMagVar(
				DEG_TO_RAD*Latitude,
				DEG_TO_RAD*Longitude,
				h,
				yymmdd_to_julian_days(year(NetTime) - 2000, month(NetTime), day(NetTime))
			);
			// try to protect the EEPROM  from excessive writes
			// every hour check if the stored variation is upto date if not write config
			// WMM2015 is good until 2020 after that dont update the variationAge, hopefully a better source will come along.
			if (minute(now()) == 0 && round(config.variation *100)/100 != round(magvar * 100)/100) {
				config.variation = magvar;
				if(year(NetTime) < 2021) config.variationAge = NetTime; 
				Serial.print("Writing new Mag var: "); Serial.println(magvar);
				UpdateConfig();
			}
			magneticvariation = magvar;
		}
		if (SGNSS) {
			PrintLabelValWithConversionCheckUnDef("GNSS info: ", SID, 0, true);
			PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ", DaysSince1970, 0, true);
			PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ", SecondsSinceMidnight, 0, true);
			PrintLabelValWithConversionCheckUnDef("  latitude: ", Latitude, 0, true);
			PrintLabelValWithConversionCheckUnDef("  longitude: ", Longitude, 0, true);
			PrintLabelValWithConversionCheckUnDef("  altitude: (m): ", alt, 0, true);
			OutputStream->print("  GNSS type: "); PrintN2kEnumType(GNSStype, OutputStream);
			OutputStream->print("  GNSS method: "); PrintN2kEnumType(GNSSmethod, OutputStream);
			PrintLabelValWithConversionCheckUnDef("  satellite count: ", nSatellites, 0, true);
			PrintLabelValWithConversionCheckUnDef("  HDOP: ", HDOP, 0, true);
			PrintLabelValWithConversionCheckUnDef("  PDOP: ", PDOP, 0, true);
			PrintLabelValWithConversionCheckUnDef("  declination: ", magneticvariation, 0, true);
		}
	}
	else {
		OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
	}

}

// process Magnetic Variation recieved message
void MagneticVariationMsg(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	double magvar;
	tN2kMagneticVariation source;
	uint16_t Age; // in DaysSince1970
	if (ParseN2kMagneticVariation(N2kMsg, SID, source, Age, magvar))
	{
		if (magvar) {
			if (SMAGVAR) {
				OutputStream->print("MagneticVariation: "); OutputStream->print(magneticvariation*RAD_TO_DEG);
				OutputStream->print(" Rad: ");  OutputStream->print(magneticvariation);
				OutputStream->print(" Source: "); PrintN2kEnumType(source, OutputStream);
				OutputStream->print(" Age: ");  PrintDateTime(N2KDate(Age));
				OutputStream->println();
			}
		}
	}
	else {
		OutputStream->print("Failed to parse MagneticVariation PGN: "); OutputStream->println(N2kMsg.PGN);
	}
}

//IMU *****************************************************************************
void StartMagCal(){
	if (!flagCalibrateMag) // not run yet run once
	{
		Blink(7, 150);
		Serial.println(F("Calibrating Magnatometer.."));
		flagCalibrateMag = true;
		ElapsedTime = now();
		IMU.startMagCal();
	}
}

void magCalLoop(){
	IMU.updateMagCal();
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void FinishMagCal(){
	if (flagCalibrateMag) {
		Serial.println("\nCal complete");	
		BNO055::calResult calResult = IMU.finishMagCal();
		IMU.readCalibration(&AccOffset, &GyroOffset, &MagOffset); // get mag radius
		Serial.print("Post cal stored offsets");
		MagOffset.bias_x = calResult.bias[0];
		MagOffset.bias_y = calResult.bias[1];
		MagOffset.bias_z = calResult.bias[2];
		PrintIMUConfig();
		config.magOffset = MagOffset;
		Serial.print("\nWriting stored offsets");
		IMU.setMagOffsets(MagOffset);
		Serial.print("\nRead back stored offsets");
		PrintIMUConfig();

		Serial.println("mag x min/max:"); Serial.print(calResult.max[0]); Serial.print("\t"); Serial.println(calResult.min[0]);
		Serial.println("mag y min/max:"); Serial.print(calResult.max[1]); Serial.print("\t"); Serial.println(calResult.min[1]);
		Serial.println("mag z min/max:"); Serial.print(calResult.max[2]); Serial.print("\t"); Serial.println(calResult.min[2]);
		Serial.print("Bias\t");
		Serial.print(calResult.bias[0]); Serial.print("\t");
		Serial.print(calResult.bias[1]); Serial.print("\t");
		Serial.print(calResult.bias[2]); Serial.print("\t");
		Serial.print("Scale\t");
		Serial.print(config.magOffset.radius); Serial.print("\t");
		Serial.print("Computed new values from: "); Serial.print(IMU.GetSampleCount());	Serial.println(" samples");
		config.magCalComplete = true;
		config.maglastcaltime = now();
		Serial.println("\nWriting EEPROM");
		UpdateConfig();
		PrintConfig();
		flagCalibrateMag = false;
	}
}

float YawtoHeading(float yaw)
{
	float hdm;
	hdm = yaw - 90;               // converts yaw to heading magnetic
	if (yaw < 90 && yaw >= -179.99) {
		hdm = yaw + 270;
	}
	return hdm;
}

// Hard reset
void ResetIMU() {
	Serial.println("Reset IMU");
	pinMode(rstPin, OUTPUT);
	digitalWrite(rstPin, LOW);
	delay(1); // reset IMU
	digitalWrite(rstPin, HIGH);
	delay(2000); // wait for IMU boot, 900 is too short
}

void InitializeIMU() {
	Serial.println("IMU initialization");
	beginStatus = 255;
	while (beginStatus<0) {
		beginStatus = IMU.begin(0x28, BNO055::NDOF, BNO055::external);
		if (beginStatus != 0) {
			Serial.println("IMU initialization unsuccessful ");
			Serial.print(beginStatus);
			ResetIMU(); //try a hard reset
			Blink(5, 500); //flash
			delay(2500);
			ShowStatus();
		}
	}
	ShowStatus();
	Serial.println("IMU init success!");
}

void EstimateOrientation() {
	//set default 
	IMU.axisRemap(0x24, 0x00);
	delay(1000);
	//estimate orientation
	int16_t acc[3];
	IMU.readAccelData(acc);
	Serial.print("Orientation X:");	Serial.print(acc[0]);	Serial.print(" Y:"); Serial.print(acc[1]);	Serial.print(" Z:"); Serial.println(acc[2]);
	//if (acc[2] >  800) { IMU.axisRemap(0x24, 0x00); } // horizonal default
	SetOrientation(config.orientation);
}

void SetOrientation(uint8_t mountorientation) {
	//remap axis 
	//XYZ 000110 0x06
	//XZY 001001 0x09
	//YXZ 010010 0x12
	//YZX 011000 0x18
	//ZXY 100001 0x21
	//ZYX 100100 0x24
	switch (mountorientation) {
	case mountingOrientation::HL: IMU.axisRemap(0x24, 0x00); break; // OK horizontal cable left 
	case mountingOrientation::HR: IMU.axisRemap(0x24, 0x06); break; // OK horizontal cable right
	case mountingOrientation::HF: IMU.axisRemap(0x21, 0x02); break; // OK horizontal cable fwd
	case mountingOrientation::VFD: IMU.axisRemap(0x09, 0x00); break; // OK vertical fwd cable down 
	case mountingOrientation::VAD: IMU.axisRemap(0x09, 0x06); break; // OK vertical aft cable down 
	case mountingOrientation::VSD: IMU.axisRemap(0x06, 0x02); break; // OK vertical stbd cable down 
	case mountingOrientation::VPD: IMU.axisRemap(0x06, 0x04); break; // OK vertical port cable down 
	case mountingOrientation::VAL: IMU.axisRemap(0x18, 0x04); break; // OK vertical fwd cable left 
	case mountingOrientation::VAR: IMU.axisRemap(0x18, 0x06); break; // OK vertical fwd cable right ???
	case mountingOrientation::VSA: IMU.axisRemap(0x18, 0x07); break; // OK vertical aft cable right
	case mountingOrientation::VPA: IMU.axisRemap(0x18, 0x02); break; // OK vertical aft cable left 
	case mountingOrientation::VFL: IMU.axisRemap(0x12, 0x00); break; // OK vertical stbd cable aft
	case mountingOrientation::VFR: IMU.axisRemap(0x12, 0x05); break; // OK vertical port cable aft
	}
	if (config.orientation != mountorientation) {
		config.orientation = mountorientation;
		UpdateConfig();
	}
	PrintOrientation();
	delay(1000);
	// 0x21, 0x04 P0 horizontal cable right roll & pitch reversed 		
	// 0x24, 0x05 P7 horizontal inverted cable left
	// 0x09, 0x03 vertical aft cable up
	// 0x06, 0x01 vertical fwd cable up
	// 0x06, 0x07 vertical port cable up
}

//*****************************************************************************
// LED blinker count flashes in duration ms
void Blink(int count, unsigned long duration){
	unsigned long d = duration / count;
	for (int counter = 0; counter < count; counter++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(d / 2);
		digitalWrite(LED_BUILTIN, LOW);
		delay(d / 2);
	}
}

//non blocking blink
IntervalTimer myTimer;
const int ledPin = LED_BUILTIN;
int ledState = LOW;
volatile unsigned long blinkCount = 0;
void blinkLED() {
	if (ledState == LOW) {
		ledState = HIGH;
		blinkCount = blinkCount + 1;  // increase when LED turns on
	}
	else {
		ledState = LOW;
	}
	digitalWrite(ledPin, ledState);
}

// EEPROM *****************************************************************************
//Load From EEPROM 
void ReadConfig(){
	EEPROM.get(EEPROM_ADR_CONFIG, config);
}

//Write to EEPROM - Teensy non-volatile area size is 2048 bytes  100,000 cycles
void UpdateConfig() {
	Blink(5, 2000);
	Serial.println("Updating config");
	config.Magic = MAGIC;
	EEPROM.put(EEPROM_ADR_CONFIG, config);
}

// Load default config into EEPROM
void InitializeEEPROM() {
	Serial.println("Initialize EEPROM");
	config = defConfig;
	UpdateConfig();
}

// Load settings from EEPROM
void LoadConfig() {
	if (config.accCalComplete) IMU.setAccOffsets(config.accOffset);
	if (config.gyroCalComplete) IMU.setGyroOffsets(config.gyroOffset);
	if (config.magCalComplete) IMU.setMagOffsets(config.magOffset);
	if (config.variationAge) magneticvariation = config.variation;
}

// Remove MAG calibration from EEPROM
void removeConfig() {
	Serial.println(F("Remove MAG calibration from EEPROM"));
	config.magCalComplete = false;
	config.accCalComplete = false;
	config.gyroCalComplete = false;
	config.maglastcaltime = 0;
	config.acclastcaltime = 0;
	config.gyrolastcaltime = 0;
	UpdateConfig();
}

//save all current configuration settings
void SaveCurrentSettings() {
	IMU.readCalibration(&AccOffset, &GyroOffset, &MagOffset);
	config.accOffset = AccOffset;
	config.accCalComplete = true;
	config.acclastcaltime = now();
	config.gyroOffset = GyroOffset;
	config.gyroCalComplete = true;
	config.gyrolastcaltime = now();
	config.magOffset = MagOffset;
	config.magCalComplete = true;
	config.maglastcaltime = now();
	config.variation = magneticvariation;
	Serial.println("\nWriting EEPROM");
	UpdateConfig();
	PrintConfig();
}

// Serial Print *****************************************************************************

// Check calibration status of the sensors
void PrintCalStatus() {
	/* The data should be ignored until the system calibration is > 0 */
	Serial.print("STATUS ");
	if (!systemCalStatus) {
		Serial.print("!");
	}
	Serial.print(" Sys:");
	Serial.print(systemCalStatus, DEC);
	Serial.print(" G:");
	Serial.print(gyroCalStatus, DEC);
	Serial.print(" A:");
	Serial.print(accelCalStatus, DEC);
	Serial.print(" M:");
	Serial.print(magCalStatus, DEC);
}

void ShowIMUErrors() {
	if (IMU.getSystemStatus() == 1) {
		switch (IMU.getSystemError()) {
		case 1: OutputStream->println(F("Peripheral initialization error")); break;
		case 2: OutputStream->println(F("System initialization error")); break;
		case 3: OutputStream->println(F("Self test result failed")); break;
		case 4: OutputStream->println(F("Register map value out of range")); break;
		case 5: OutputStream->println(F("Register map address out of range")); break;
		case 6: OutputStream->println(F("Register map write error")); break;
		case 7: OutputStream->println(F("BNO low power mode not available for selected operation mode")); break;
		case 8: OutputStream->println(F("Accelerometer power mode not available")); break;
		case 9: OutputStream->println(F("Fusion algorithm configuration error")); break;
		case 0xA:OutputStream->println(F("Sensor configuration error")); break;
		}
	}
}

// pad zeros for time format
void PrintDigits(int digits) {
	if (digits < 10)
		OutputStream->print('0');
	OutputStream->print(digits);
}

// print datetime
void PrintDateTime(time_t t) {
	// display the given time
	OutputStream->print(" ");
	OutputStream->print(year(t));
	OutputStream->print("-");
	OutputStream->print(month(t));
	OutputStream->print("-");
	OutputStream->print(day(t));
	OutputStream->print(" ");
	PrintDigits(hour(t));
	OutputStream->print(":");
	PrintDigits(minute(t));
	OutputStream->print(":");
	PrintDigits(second(t));
	OutputStream->print(" GMT ");
}

void printHeading() {
	Serial.print(heading, 2);
	Serial.print("deg\tgx "); Serial.print(gx, 2);
	Serial.print(" gy "); Serial.print(gy, 2);
	Serial.print(" gz "); Serial.print(gz, 2); Serial.print(" deg/s");
	Serial.println();
}

void ShowStatus() {
	switch (IMU.getSystemStatus()) {
	case 0: Serial.println("SYSTEM IDLE"); break;
	case 1: Serial.println("SYSTEM ERROR"); ShowIMUErrors(); break;
	case 2: Serial.println("Initializing peripherals"); break;
	case 3: Serial.println("System Initialization"); break;
	case 4: Serial.println("Executing Selftest"); break;
	case 5: Serial.println("Sensor Fusion Running"); break;
	case 6: Serial.println("System Running - No Fusion"); break;
	}
}

void printData() {
	Serial.println("");
	ShowStatus();
	PrintCalStatus(); Serial.println();
	if (NetTime) { PrintDateTime(NetTime); Serial.println();}
	if (SPRINT) {//debug
		IMU.readAccelData(accelCount);  // Read the accel values in mg
		ax = (float)accelCount[0];
		ay = (float)accelCount[1];
		az = (float)accelCount[2];
		Serial.print("gx "); Serial.print(gx, 2);
		Serial.print(" gy "); Serial.print(gy, 2); 
		Serial.print(" gz "); Serial.print(gz, 2); Serial.println(" deg/s");
		Serial.print("ax "); Serial.print((ax), 2);
		Serial.print(" ay "); Serial.print((ay), 2);
		Serial.print(" az "); Serial.print((az), 2); Serial.println(" mss");
	}
	Serial.print("Roll:\t"); Serial.print(roll, 2);	Serial.print("\tPitch:\t"); Serial.println(pitch, 2);
	Serial.print("Heading\t"); Serial.print(heading); Serial.println(" deg");
	Serial.print("Orientation "); PrintOrientation();
	Serial.print("RateOfTurn "); Serial.print(yawrate, 2); Serial.println(" deg/s");
	Serial.print("deltat = "); Serial.print(deltat,4); Serial.println(" seconds");
//	Serial.print("rate = "); Serial.print((float)sumCount / sum, 2); Serial.println(" Hz");
}

void PrintOrientation() {	
	Serial.print(config.orientation); Serial.print(" ");
	switch (config.orientation) {
		
		case mountingOrientation::HL: IMU.axisRemap(0x24, 0x00); break; // OK horizontal cable left 
		case mountingOrientation::HR: IMU.axisRemap(0x24, 0x06); break; // OK horizontal cable right
		case mountingOrientation::HF: IMU.axisRemap(0x21, 0x02); break; // OK horizontal cable fwd
		case mountingOrientation::VFD: IMU.axisRemap(0x09, 0x00); break; // OK vertical fwd cable down 
		case mountingOrientation::VAD: IMU.axisRemap(0x09, 0x06); break; // OK vertical aft cable down 
		case mountingOrientation::VSD: IMU.axisRemap(0x06, 0x02); break; // OK vertical stbd cable down 
		case mountingOrientation::VPD: IMU.axisRemap(0x06, 0x04); break; // OK vertical port cable down 
		case mountingOrientation::VAL: IMU.axisRemap(0x18, 0x04); break; // OK vertical fwd cable left 
		case mountingOrientation::VAR: IMU.axisRemap(0x18, 0x06); break; // OK vertical fwd cable right ???
		case mountingOrientation::VSA: IMU.axisRemap(0x18, 0x07); break; // OK vertical aft cable right
		case mountingOrientation::VPA: IMU.axisRemap(0x18, 0x02); break; // OK vertical aft cable left 
		case mountingOrientation::VFL: IMU.axisRemap(0x12, 0x00); break; // OK vertical stbd cable aft
		case mountingOrientation::VFR: IMU.axisRemap(0x12, 0x05); break; // OK vertical port cable aft
	}
}

void PrintConfig(){
	Serial.println("\nStored EEPROM contents:"); 
	Serial.print("Accel\tBias\tX:");
	Serial.print(config.accOffset.bias_x); Serial.print("\tY:");
	Serial.print(config.accOffset.bias_y); Serial.print("\tZ:");
	Serial.print(config.accOffset.bias_z); 
	Serial.print("\tScale:");Serial.print(config.accOffset.radius);
	Serial.print("\t@");PrintDateTime(config.acclastcaltime); Serial.println();
	Serial.print("Gyro\tBias\tX:");
	Serial.print(config.gyroOffset.bias_x); Serial.print("\tY:");
	Serial.print(config.gyroOffset.bias_y); Serial.print("\tZ:");
	Serial.print(config.gyroOffset.bias_z); Serial.print("\t\t\t@");
	PrintDateTime(config.gyrolastcaltime); Serial.println();
	Serial.print("Mag\tBias\tX:");
	Serial.print(config.magOffset.bias_x); Serial.print("\tY:");
	Serial.print(config.magOffset.bias_y); Serial.print("\tZ:");
	Serial.print(config.magOffset.bias_z);
	Serial.print("\tScale:");Serial.print(config.magOffset.radius); 
	Serial.print("\t@");PrintDateTime(config.maglastcaltime); Serial.println();
	Serial.print("Magnetic Variation WMM2015\t"); PrintLabelValWithConversionCheckUnDef("Declination: ", config.variation, 0, false);
	Serial.print("\t@"); PrintDateTime(config.variationAge); Serial.println();
	Serial.print("Orientation: "); PrintOrientation();

}

void PrintIMUConfig()
{
	PrintCalStatus();
	IMU.readCalibration(&AccOffset, &GyroOffset, &MagOffset);
	Serial.print("  Acc X:");	Serial.print(AccOffset.bias_x);
	Serial.print(" Y:"); Serial.print(AccOffset.bias_y);
	Serial.print(" Z:"); Serial.print(AccOffset.bias_z); 
	Serial.print(" R:"); Serial.print(AccOffset.radius);
	Serial.print("  Gyro X:");	Serial.print(GyroOffset.bias_x);
	Serial.print(" Y:"); Serial.print(GyroOffset.bias_y);
	Serial.print(" Z:"); Serial.print(GyroOffset.bias_z);
	Serial.print("  Mag X:"); Serial.print(MagOffset.bias_x);
	Serial.print(" Y:"); Serial.print(MagOffset.bias_y); 
	Serial.print(" Z:"); Serial.print(MagOffset.bias_z);
	Serial.print(" R:"); Serial.println(MagOffset.radius); 
}

unsigned long DaysToNMEADate(unsigned long val) {
	if (val != N2kUInt32NA) {
		tmElements_t tm;
		time_t t = val*SECS_PER_DAY; //daysToTime_t((val));
		breakTime(t, tm);
		val = tm.Day * 10000 + (tm.Month) * 100 + (tm.Year + 1970 - 2000);
	}
	return val;
}

void PrintHelp()
{
	Serial.println(F("\nCommands:\n m=Start Mag cal.\n f=Finish Mag cal.\n d=toggle output mag x,y,x."));
	Serial.println(F(" s=save current settings.\tt=toggle system time.\n v=toggle mag var.\n r=remove calibrations\n p=print config.\n\n"));
}

/***********************************************************************************
* WMM2015 calculation
* 2017/09/21 modified for LK8000 by paolo to adopt only WMM2015 model, and no field output.
* The source of the original C code is at www.edwilliams.org
* We are all grateful to Ed for his work!
*/
#define	max(a,b)	(((a) > (b)) ? (a) : (b))

static const double a = 6378.16;	/* major radius (km) IAU66 ellipsoid */
static const double b = 6378.16 * (1.0 - 1.0 / 298.25);
static const double r_0 = 6371.2;	/* "mean radius" for spherical harmonic expansion */

static const double gnm_wmm2015[13][13] = {
	{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
	{ -29438.5,-1501.1,0,0,0,0,0,0,0,0,0,0,0 },
	{ -2445.3,3012.5,1676.6,0,0,0,0,0,0,0,0,0,0 },
	{ 1351.1,-2352.3,1225.6,581.9,0,0,0,0,0,0,0,0,0 },
	{ 907.2,813.7,120.3,-335,70.3,0,0,0,0,0,0,0,0 },
	{ -232.6,360.1,192.4,-141,-157.4,4.3,0,0,0,0,0,0,0 },
	{ 69.5,67.4,72.8,-129.8,-29,13.2,-70.9,0,0,0,0,0,0 },
	{ 81.6,-76.1,-6.8,51.9,15,9.3,-2.8,6.7,0,0,0,0,0 },
	{ 24,8.6,-16.9,-3.2,-20.6,13.3,11.7,-16,-2,0,0,0,0 },
	{ 5.4,8.8,3.1,-3.1,0.6,-13.3,-0.1,8.7,-9.1,-10.5,0,0,0 },
	{ -1.9,-6.5,0.2,0.6,-0.6,1.7,-0.7,2.1,2.3,-1.8,-3.6,0,0 },
	{ 3.1,-1.5,-2.3,2.1,-0.9,0.6,-0.7,0.2,1.7,-0.2,0.4,3.5,0 },
	{ -2,-0.3,0.4,1.3,-0.9,0.9,0.1,0.5,-0.4,-0.4,0.2,-0.9,0 }
};
static const double hnm_wmm2015[13][13] = {
	{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
	{ 0,4796.2,0,0,0,0,0,0,0,0,0,0,0 },
	{ 0,-2845.6,-642,0,0,0,0,0,0,0,0,0,0 },
	{ 0,-115.3,245,-538.3,0,0,0,0,0,0,0,0,0 },
	{ 0,283.4,-188.6,180.9,-329.5,0,0,0,0,0,0,0,0 },
	{ 0,47.4,196.9,-119.4,16.1,100.1,0,0,0,0,0,0,0 },
	{ 0,-20.7,33.2,58.8,-66.5,7.3,62.5,0,0,0,0,0,0 },
	{ 0,-54.1,-19.4,5.6,24.4,3.3,-27.5,-2.3,0,0,0,0,0 },
	{ 0,10.2,-18.1,13.2,-14.6,16.2,5.7,-9.1,2.2,0,0,0,0 },
	{ 0,-21.6,10.8,11.7,-6.8,-6.9,7.8,1,-3.9,8.5,0,0,0 },
	{ 0,3.3,-0.3,4.6,4.4,-7.9,-0.6,-4.1,-2.8,-1.1,-8.7,0,0 },
	{ 0,-0.1,2.1,-0.7,-1.1,0.7,-0.2,-2.1,-1.5,-2.5,-2,-2.3,0 },
	{ 0,-1,0.5,1.8,-2.2,0.3,0.7,-0.1,0.3,0.2,-0.9,-0.2,0.7 }
};
static const double gtnm_wmm2015[13][13] = {
	{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
	{ 10.7,17.9,0,0,0,0,0,0,0,0,0,0,0 },
	{ -8.6,-3.3,2.4,0,0,0,0,0,0,0,0,0,0 },
	{ 3.1,-6.2,-0.4,-10.4,0,0,0,0,0,0,0,0,0 },
	{ -0.4,0.8,-9.2,4,-4.2,0,0,0,0,0,0,0,0 },
	{ -0.2,0.1,-1.4,0,1.3,3.8,0,0,0,0,0,0,0 },
	{ -0.5,-0.2,-0.6,2.4,-1.1,0.3,1.5,0,0,0,0,0,0 },
	{ 0.2,-0.2,-0.4,1.3,0.2,-0.4,-0.9,0.3,0,0,0,0,0 },
	{ 0,0.1,-0.5,0.5,-0.2,0.4,0.2,-0.4,0.3,0,0,0,0 },
	{ 0,-0.1,-0.1,0.4,-0.5,-0.2,0.1,0,-0.2,-0.1,0,0,0 },
	{ 0,0,-0.1,0.3,-0.1,-0.1,-0.1,0,-0.2,-0.1,-0.2,0,0 },
	{ 0,0,-0.1,0.1,0,0,0,0,0,0,-0.1,-0.1,0 },
	{ 0.1,0,0,0.1,-0.1,0,0.1,0,0,0,0,0,0 }
};
static const double htnm_wmm2015[13][13] = {
	{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
	{ 0,-26.8,0,0,0,0,0,0,0,0,0,0,0 },
	{ 0,-27.1,-13.3,0,0,0,0,0,0,0,0,0,0 },
	{ 0,8.4,-0.4,2.3,0,0,0,0,0,0,0,0,0 },
	{ 0,-0.6,5.3,3,-5.3,0,0,0,0,0,0,0,0 },
	{ 0,0.4,1.6,-1.1,3.3,0.1,0,0,0,0,0,0,0 },
	{ 0,0,-2.2,-0.7,0.1,1,1.3,0,0,0,0,0,0 },
	{ 0,0.7,0.5,-0.2,-0.1,-0.7,0.1,0.1,0,0,0,0,0 },
	{ 0,-0.3,0.3,0.3,0.6,-0.1,-0.2,0.3,0,0,0,0,0 },
	{ 0,-0.2,-0.1,-0.2,0.1,0.1,0,-0.2,0.4,0.3,0,0,0 },
	{ 0,0.1,-0.1,0,0,-0.2,0.1,-0.1,-0.2,0.1,-0.1,0,0 },
	{ 0,0,0.1,0,0.1,0,0,0.1,0,-0.1,0,-0.1,0 },
	{ 0,0,0,-0.1,0,0,0,0,0,0,0,0,0 }
};

static const int nmax = 13;

double WMMP[14][14];
double DP[14][14];
double gnm[14][14];
double hnm[14][14];
double sm[14];
double cm[14];

static double root[14];
static double roots[14][14][2];

/* Convert date to Julian day    1950-2049 */
unsigned long int yymmdd_to_julian_days(int yy, int mm, int dd)
{
	unsigned long jd;

	yy = (yy < 50) ? (2000 + yy) : (1900 + yy);
	jd = dd - 32075L + 1461L * (yy + 4800L + (mm - 14) / 12) / 4;
	jd = jd + 367L * (mm - 2 - (mm - 14) / 12 * 12) / 12;
	jd = jd - 3 * ((yy + 4900L + (mm - 14) / 12) / 100) / 4;

	return(jd);
}

/*
* Return variation (in radians) given geodetic latitude (radians), longitude
* (radians) ,height (km) and (Julian) date
* N and E lat and long are positive, S and W negative
*/
double SGMagVar(double lat, double lon, double h, long dat)
{
	/* output field B_r,B_th,B_phi,B_x,B_y,B_z */
	int n, m, nmaxl;
	double yearfrac, sr, r, theta, c, s, psi, fn, fn_0, B_r, B_theta, B_phi, X, Y;
	double sinpsi, cospsi, inv_s;

	static int been_here = 0;

	double sinlat = sin(lat);
	double coslat = cos(lat);

	/* convert to geocentric */
	sr = sqrt(a*a*coslat*coslat + b*b*sinlat*sinlat);
	/* sr is effective radius */
	theta = atan2(coslat * (h*sr + a*a), sinlat * (h*sr + b*b));

	/* theta is geocentric co-latitude */
	//LKASSERT((a*a - (a*a - b*b) * sinlat*sinlat) != 0);

	r = h*h + 2.0*h * sr +
		(a*a*a*a - (a*a*a*a - b*b*b*b) * sinlat*sinlat) /
		(a*a - (a*a - b*b) * sinlat*sinlat);

	r = sqrt(r);

	/* r is geocentric radial distance */
	c = cos(theta);
	s = sin(theta);
	/* protect against zero divide at geographic poles */
	inv_s = 1.0 / (s + (s == 0.)*1.0e-8);

	/*zero out arrays */
	for (n = 0; n <= nmax; n++) {
		for (m = 0; m <= n; m++) {
			WMMP[n][m] = 0;
			DP[n][m] = 0;
		}
	}

	/* diagonal elements */
	WMMP[0][0] = 1;
	WMMP[1][1] = s;
	DP[0][0] = 0;
	DP[1][1] = c;
	WMMP[1][0] = c;
	DP[1][0] = -s;

	/* these values will not change for subsequent function calls */
	if (!been_here) {
		for (n = 2; n <= nmax; n++) {
			root[n] = sqrt((2.0*n - 1) / (2.0*n));
		}

		for (m = 0; m <= nmax; m++) {
			double mm = m*m;
			for (n = max(m + 1, 2); n <= nmax; n++) {
				roots[m][n][0] = sqrt((n - 1)*(n - 1) - mm);
				roots[m][n][1] = 1.0 / sqrt(n*n - mm);
			}
		}
		been_here = 1;
	}

	for (n = 2; n <= nmax; n++) {
		/*  double root = sqrt((2.0*n-1) / (2.0*n)); */
		WMMP[n][n] = WMMP[n - 1][n - 1] * s * root[n];
		DP[n][n] = (DP[n - 1][n - 1] * s + WMMP[n - 1][n - 1] * c) * root[n];
	}

	/* lower triangle */
	for (m = 0; m <= nmax; m++) {
		/*  double mm = m*m;  */
		for (n = max(m + 1, 2); n <= nmax; n++) {
			/* double root1 = sqrt((n-1)*(n-1) - mm); */
			/* double root2 = 1.0 / sqrt( n*n - mm);  */
			WMMP[n][m] = (WMMP[n - 1][m] * c * (2.0*n - 1) -
				WMMP[n - 2][m] * roots[m][n][0]) * roots[m][n][1];
			DP[n][m] = ((DP[n - 1][m] * c - WMMP[n - 1][m] * s) *
				(2.0*n - 1) - DP[n - 2][m] * roots[m][n][0]) * roots[m][n][1];
		}
	}

	/* compute gnm, hnm at dat */
	nmaxl = 12;  /* models except IGRF2005 */

	yearfrac = (dat - yymmdd_to_julian_days(15, 1, 1)) / 365.25;
	for (n = 1; n <= nmaxl; n++)
		for (m = 0; m <= nmaxl; m++) {
			gnm[n][m] = gnm_wmm2015[n][m] + yearfrac * gtnm_wmm2015[n][m];
			hnm[n][m] = hnm_wmm2015[n][m] + yearfrac * htnm_wmm2015[n][m];
		}

	/* compute sm (sin(m lon) and cm (cos(m lon)) */
	for (m = 0; m <= nmaxl; m++) {
		sm[m] = sin(m * lon);
		cm[m] = cos(m * lon);
	}

	/* compute B fields */
	B_r = 0.0;
	B_theta = 0.0;
	B_phi = 0.0;
	fn_0 = r_0 / r;
	fn = fn_0 * fn_0;

	for (n = 1; n <= nmaxl; n++) {
		double c1_n = 0;
		double c2_n = 0;
		double c3_n = 0;
		for (m = 0; m <= n; m++) {
			double tmp = (gnm[n][m] * cm[m] + hnm[n][m] * sm[m]);
			c1_n += tmp * WMMP[n][m];
			c2_n += tmp * DP[n][m];
			c3_n += m * (gnm[n][m] * sm[m] - hnm[n][m] * cm[m]) * WMMP[n][m];
		}
		/* fn=pow(r_0/r,n+2.0);   */
		fn *= fn_0;
		B_r += (n + 1) * c1_n * fn;
		B_theta -= c2_n * fn;
		B_phi += c3_n * fn * inv_s;
	}



	/* Find geodetic field components: */
	psi = theta - (PI / 2.0 - lat);
	sinpsi = sin(psi);
	cospsi = cos(psi);
	X = -B_theta * cospsi - B_r * sinpsi;
	Y = B_phi;

	/* find variation in radians */
	/* return zero variation at magnetic pole X=Y=0. */
	/* E is positive */
	return (X != 0. || Y != 0.) ? atan2(Y, X) : (double) 0.;
}


