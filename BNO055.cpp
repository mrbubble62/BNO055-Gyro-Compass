#include "i2c_t3.h"
#include "BNO055.h"

int BNO055::begin(uint8_t address = 0x28, OPRMode OPRMode = NDOF, Xtal xtal = Xtal::external)
{
	//BNO055_ADDRESS 0x29   P0
	//BNO055_ADDRESS 0x28   P0
	// configurable in fusion modes
	_Ascale = AFS_2G;      // Accel full scale
	_address = address;
	_OPRMode = OPRMode;
	_Xtal = xtal;  // internal or external crystal

	// the following are auto controlled in fusion modes
	_GPwrMode = NormalG;   // Gyro power mode
	_APwrMode = NormalA;   // Accel power mode
	_MPwrMode = Normal;    // Select magnetometer power mode
	_Gscale = GFS_250DPS;  // Gyro full scale
	_Gbw = GBW_23Hz;       // Gyro bandwidth
	_Abw = ABW_31_25Hz;    // Accel bandwidth, accel sample rate divided by ABW_divx
	_MOpMode = Regular;    // Select magnetometer perfomance mode
	_Modr = MODR_10Hz;     // Select magnetometer ODR when in BNO055 bypass mode
	_PWRMode = Normalpwr;  // Select BNO055 power mode Normalpwr is default
	
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
	
	//read Power On Self Test(POST) results
	byte a = readByte(BNO055_ST_RESULT);
	if (a != 0x0F)
		return a;
	byte c = readByte(BNO055_CHIP_ID);  
	if (c != CHIP_ID) // BNO055_CHIP_ID should always be 0xA0
		return -1;
	byte d = readByte(BNO055_ACC_ID);  
	if (d != 0xFB) // BNO055_ACC_ID should always be 0xFB
		return -2;
	// Read the WHO_AM_I register of the magnetometer
	byte e = readByte(BNO055_MAG_ID);  
	if (e != 0x32) // BNO055_MAG_ID should always be 0x32
		return -3;
	// Read the WHO_AM_I register of the gyroscope
	byte f = readByte(BNO055_GYRO_ID);  
	if (f != 0x0F) // BNO055_GYRO_ID should always be 0x0F
		return -4;

	// Select BNO055 config mode
	setMode(CONFIGMODE);
	// Select BNO055 gyro temperature source 
	writeByte(BNO055_TEMP_SOURCE, 0x01);
	// Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
	writeByte(BNO055_UNIT_SEL, 0x01);

	setMode(_OPRMode);
	// Select page 1 to configure sensors
	writeByte(BNO055_PAGE_ID, 0x01);
	// Configure ACC can only be changed in sensor mode
	writeByte(BNO055_ACC_CONFIG, _APwrMode << 5 | _Abw << 2 | _Ascale);
	// Configure GYR can only be changed in sensor mode
	writeByte(BNO055_GYRO_CONFIG_0, _Gbw << 3 | _Gscale);
	writeByte(BNO055_GYRO_CONFIG_1, _GPwrMode);
	// Configure MAG can only be changed in sensor mode
	writeByte(BNO055_MAG_CONFIG, _MPwrMode << 5 | _MOpMode << 3 | _Modr);
	writeByte(BNO055_PAGE_ID, 0x00);

	setClockExt(_Xtal);

	return 0;
}

uint8_t BNO055::getMode() {
	return readByte(BNO055_OPR_MODE);
}

// Sensor calibration data
void BNO055::readCalibration(accOffset * accoffset, gyroOffset * gyroffset, magOffset * magoffset)
{
	setMode(OPRMode::CONFIGMODE);
	accoffset->bias_x = (int16_t)((int16_t)readByte(BNO055_ACC_OFFSET_X_MSB) << 8) | (readByte(BNO055_ACC_OFFSET_X_LSB));
	accoffset->bias_y = (int16_t)((int16_t)readByte(BNO055_ACC_OFFSET_Y_MSB) << 8) | (readByte(BNO055_ACC_OFFSET_Y_LSB));
	accoffset->bias_z = (int16_t)((int16_t)readByte(BNO055_ACC_OFFSET_Z_MSB) << 8) | (readByte(BNO055_ACC_OFFSET_Z_LSB));
	accoffset->radius = (int16_t)((int16_t)readByte(BNO055_ACC_RADIUS_MSB  ) << 8) | (readByte(BNO055_ACC_RADIUS_LSB  ));
	gyroffset->bias_x = (int16_t)((int16_t)readByte(BNO055_GYR_OFFSET_X_MSB) << 8) | (readByte(BNO055_GYR_OFFSET_X_LSB));
	gyroffset->bias_y = (int16_t)((int16_t)readByte(BNO055_GYR_OFFSET_Y_MSB) << 8) | (readByte(BNO055_GYR_OFFSET_Y_LSB));
	gyroffset->bias_z = (int16_t)((int16_t)readByte(BNO055_GYR_OFFSET_Z_MSB) << 8) | (readByte(BNO055_GYR_OFFSET_Z_LSB));
	magoffset->bias_x = (int16_t)((int16_t)readByte(BNO055_MAG_OFFSET_X_MSB) << 8) | (readByte(BNO055_MAG_OFFSET_X_LSB));
	magoffset->bias_y = (int16_t)((int16_t)readByte(BNO055_MAG_OFFSET_Y_MSB) << 8) | (readByte(BNO055_MAG_OFFSET_Y_LSB));
	magoffset->bias_z = (int16_t)((int16_t)readByte(BNO055_MAG_OFFSET_Z_MSB) << 8) | (readByte(BNO055_MAG_OFFSET_Z_LSB));
	magoffset->radius = (int16_t)((int16_t)readByte(BNO055_MAG_RADIUS_MSB  ) << 8) | (readByte(BNO055_MAG_RADIUS_LSB  ));
	setMode(_OPRMode);
}

void BNO055::readAccelData(int16_t * destination) {
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];      // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void BNO055::readGyroData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

int8_t BNO055::readGyroTempData()
{
	return readByte(BNO055_TEMP);  // Read the two raw data registers sequentially into data array 
}

void BNO055::readMagData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void BNO055::readQuatData(int16_t * destination)
{
	uint8_t rawData[8];  // x/y/z gyro register data stored here
	readBytes(BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	destination[3] = ((int16_t)rawData[7] << 8) | rawData[6];
}

void BNO055::readEulData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_EUL_HEADING_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void BNO055::readLIAData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void BNO055::readGRVData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];       // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

// I2C read/write functions for the BNO055 sensor
void BNO055::writeByte(uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(_address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

void BNO055::readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(_address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(_address, (size_t)count);  // Read bytes from slave register address 
	while (Wire.available()) {
		dest[i++] = Wire.read();
	}         // Put read results in the Rx buffer
}

uint8_t BNO055::readByte(uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data   
	Wire.beginTransmission(_address);         // Initialize the Tx buffer
	Wire.write(subAddress);                  // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
											 //  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
											 //  Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(_address, (size_t)1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void BNO055::setAccOffsets(accOffset offset) {
	setMode(OPRMode::CONFIGMODE);
	writeByte(BNO055_ACC_OFFSET_X_LSB, (int16_t)offset.bias_x & 0xFF);
	writeByte(BNO055_ACC_OFFSET_X_MSB, ((int16_t)offset.bias_x >> 8) & 0xFF);
	writeByte(BNO055_ACC_OFFSET_Y_LSB, (int16_t)offset.bias_y & 0xFF);
	writeByte(BNO055_ACC_OFFSET_Y_MSB, ((int16_t)offset.bias_y >> 8) & 0xFF);
	writeByte(BNO055_ACC_OFFSET_Z_LSB, (int16_t)offset.bias_z & 0xFF);
	writeByte(BNO055_ACC_OFFSET_Z_MSB, ((int16_t)offset.bias_z >> 8) & 0xFF);
	writeByte(BNO055_ACC_RADIUS_LSB, (int16_t)offset.radius & 0xFF);
	writeByte(BNO055_ACC_RADIUS_MSB, ((int16_t)offset.radius >> 8) & 0xFF);
	setMode(_OPRMode);
}

void BNO055::setGyroOffsets(gyroOffset offset) {
	setMode(OPRMode::CONFIGMODE);
	writeByte(BNO055_GYR_OFFSET_X_LSB, (int16_t)offset.bias_x & 0xFF);
	writeByte(BNO055_GYR_OFFSET_X_MSB, ((int16_t)offset.bias_x >> 8) & 0xFF);
	writeByte(BNO055_GYR_OFFSET_Y_LSB, (int16_t)offset.bias_y & 0xFF);
	writeByte(BNO055_GYR_OFFSET_Y_MSB, ((int16_t)offset.bias_y >> 8) & 0xFF);
	writeByte(BNO055_GYR_OFFSET_Z_LSB, (int16_t)offset.bias_z & 0xFF);
	writeByte(BNO055_GYR_OFFSET_Z_MSB, ((int16_t)offset.bias_z >> 8) & 0xFF);
	setMode(_OPRMode);
}

void BNO055::setMagOffsets(magOffset offset) {
	setMode(OPRMode::CONFIGMODE);
	writeByte(BNO055_MAG_OFFSET_X_LSB, (int16_t)offset.bias_x & 0xFF);
	writeByte(BNO055_MAG_OFFSET_X_MSB, ((int16_t)offset.bias_x >> 8) & 0xFF);
	writeByte(BNO055_MAG_OFFSET_Y_LSB, (int16_t)offset.bias_y & 0xFF);
	writeByte(BNO055_MAG_OFFSET_Y_MSB, ((int16_t)offset.bias_y >> 8) & 0xFF);
	writeByte(BNO055_MAG_OFFSET_Z_LSB, (int16_t)offset.bias_z & 0xFF);
	writeByte(BNO055_MAG_OFFSET_Z_MSB, ((int16_t)offset.bias_z >> 8) & 0xFF);
	writeByte(BNO055_MAG_RADIUS_LSB, (int16_t)offset.radius & 0xFF);
	writeByte(BNO055_MAG_RADIUS_MSB, ((int16_t)offset.radius >> 8) & 0xFF);
	setMode(_OPRMode);
}

uint32_t BNO055::GetSampleCount() {
	return _samplecount;
}


// User initiated mag cal for slow turning vessel.
// Begin mag cal, circling a boat could take a while
void BNO055::startMagCal() {
	_samplecount = 0;
	mag_bias[0] = 0, mag_bias[1] = 0, mag_bias[2] = 0;
	mag_max[0] = mag_max[1] = mag_max[2] = 0;
	mag_min[0] = mag_min[1] = mag_min[2] = 0;
	//setMode(OPRMode::CONFIGMODE);
	//writeByte(BNO055_PAGE_ID, 1);
	//writeByte(BNO055_MAG_CONFIG, _MPwrMode << 5 | _MOpMode << 3 | _Modr);
	//writeByte(BNO055_PAGE_ID, 0);
	//setMode(_OPRMode);
}

// at 10 Hz ODR, new mag data is available every 100 ms
// Call updateMagCal as ofen as necessary. 
void BNO055::updateMagCal() {
	uint8_t data[6]; // data array to hold mag x, y, z, data
	int16_t mag_temp[3] = { 0, 0, 0 };
	readBytes(BNO055_MAG_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
	mag_temp[0] = (int16_t)(((int16_t)data[1] << 8) | data[0]);   // Form signed 16-bit integer for each sample in FIFO
	mag_temp[1] = (int16_t)(((int16_t)data[3] << 8) | data[2]);
	mag_temp[2] = (int16_t)(((int16_t)data[5] << 8) | data[4]);
	for (int jj = 0; jj < 3; jj++) {
		if (_samplecount == 0) { //first pass 
			mag_max[jj] = mag_temp[jj]; // Offsets may be large enough that mag_temp[i] may not be bipolar! 
			mag_min[jj] = mag_temp[jj]; // This prevents max or min being pinned to 0 if the values are unipolar...
		}
		else {
			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
	}
	_samplecount++;
}

// complete user initiated calibratation
BNO055::calResult BNO055::finishMagCal() {
	calResult _magCalResult;
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	_magCalResult.max[0] = mag_max[0];
	_magCalResult.max[1] = mag_max[1];
	_magCalResult.max[2] = mag_max[2];
	_magCalResult.min[0] = mag_min[0];
	_magCalResult.min[1] = mag_min[1];
	_magCalResult.min[2] = mag_min[2];

	_magCalResult.bias[0] = (float)mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
	_magCalResult.bias[1] = (float)mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
	_magCalResult.bias[2] = (float)mag_bias[2] / 1.6;

	// Return to config mode to write mag biases in offset register
	// This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
	//setMode(CONFIGMODE);
	////write biases to accelerometer offset registers as 16 LSB/microTesla
	//writeByte(BNO055_MAG_OFFSET_X_LSB, (int16_t)mag_bias[0] & 0xFF);
	//writeByte(BNO055_MAG_OFFSET_X_MSB, ((int16_t)mag_bias[0] >> 8) & 0xFF);
	//writeByte(BNO055_MAG_OFFSET_Y_LSB, (int16_t)mag_bias[1] & 0xFF);
	//writeByte(BNO055_MAG_OFFSET_Y_MSB, ((int16_t)mag_bias[1] >> 8) & 0xFF);
	//writeByte(BNO055_MAG_OFFSET_Z_LSB, (int16_t)mag_bias[2] & 0xFF);
	//writeByte(BNO055_MAG_OFFSET_Z_MSB, ((int16_t)mag_bias[2] >> 8) & 0xFF);
	//setMode(_OPRMode);
	return _magCalResult;
}

//one shot mag cal, turm sensor through all orientations to populate min max
BNO055::calResult BNO055::magCal()
{
	calResult _magCalResult;
	uint8_t data[6]; // data array to hold mag x, y, z, data
	uint16_t ii = 0, sample_count = 256;
	int32_t mag_bias[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { 0, 0, 0 }, mag_min[3] = { 0, 0, 0 };

	//setMode(OPRMode::AMG);
	// In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
	for (ii = 0; ii < sample_count; ii++) {
		int16_t mag_temp[3] = { 0, 0, 0 };
		readBytes(BNO055_MAG_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		mag_temp[0] = (int16_t)(((int16_t)data[1] << 8) | data[0]);   // Form signed 16-bit integer for each sample in FIFO
		mag_temp[1] = (int16_t)(((int16_t)data[3] << 8) | data[2]);
		mag_temp[2] = (int16_t)(((int16_t)data[5] << 8) | data[4]);
		for (int jj = 0; jj < 3; jj++) {
			if (ii == 0) {
				mag_max[jj] = mag_temp[jj]; // Offsets may be large enough that mag_temp[i] may not be bipolar! 
				mag_min[jj] = mag_temp[jj]; // This prevents max or min being pinned to 0 if the values are unipolar...
			}
			else {
				if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
		}
		delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
	}

	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	_magCalResult.max[0] = mag_max[0];
	_magCalResult.max[1] = mag_max[1];
	_magCalResult.max[2] = mag_max[2];
	_magCalResult.min[0] = mag_min[0];
	_magCalResult.min[1] = mag_min[1];
	_magCalResult.min[2] = mag_min[2];

	_magCalResult.bias[0] = (float)mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
	_magCalResult.bias[1] = (float)mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
	_magCalResult.bias[2] = (float)mag_bias[2] / 1.6;

	// Return to config mode to write mag biases in offset register
	setMode(CONFIGMODE);
	// This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
	writeByte(BNO055_MAG_OFFSET_X_LSB, (int16_t)mag_bias[0] & 0xFF);
	writeByte(BNO055_MAG_OFFSET_X_MSB, ((int16_t)mag_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_MAG_OFFSET_Y_LSB, (int16_t)mag_bias[1] & 0xFF);
	writeByte(BNO055_MAG_OFFSET_Y_MSB, ((int16_t)mag_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_MAG_OFFSET_Z_LSB, (int16_t)mag_bias[2] & 0xFF);
	writeByte(BNO055_MAG_OFFSET_Z_MSB, ((int16_t)mag_bias[2] >> 8) & 0xFF);
	setMode(_OPRMode);
	return _magCalResult;
}

/*
Interrupt Pin config 
TBD untested
Accelerometer Slow/No Motion Interrupt
Any Motion Interrupt
Gyro High Rate Interrupt
Gyroscope Any Motion Interrupt
*/
void BNO055::enableInterrupt(uint8_t mask)
{
	setMode(CONFIGMODE);
	writeByte(BNO055_PAGE_ID, 1);
	writeByte(BNO055_SYS_TRIGGER, mask);
	writeByte(BNO055_PAGE_ID, 0);
	setMode(_OPRMode);
}

//TBD untested
void BNO055::ackInterrupt()
{
	setMode(CONFIGMODE);
	writeByte(BNO055_PAGE_ID, 0);
	writeByte(BNO055_SYS_TRIGGER, RST_INT | _Xtal << 7); // 0x80
	if (_Xtal == Xtal::internal) {
		writeByte(BNO055_SYS_TRIGGER, RST_INT | _Xtal << 7); // 0x80
	}
	else {
		writeByte(BNO055_SYS_TRIGGER, RST_INT);
	}
	delay(1);
	setMode(_OPRMode);
}

/* Soft Reset */
void BNO055::reset()
{
	_samplecount = 0;
	writeByte(BNO055_PAGE_ID, 0);
	writeByte(BNO055_SYS_TRIGGER, RST_SYS);
	//while (readByte(BNO055_CHIP_ID) != CHIP_ID)
	//{
	//	delay(1);
	//	_samplecount++;
	//}
	writeByte(BNO055_SYS_TRIGGER, CLK_SEL);
	setClockExt(_Xtal);
	//init();
	delay(500);
}

/* allows for ANY orientation
* 0X21 | REMAP_X_Y | Z = Z; X = Y; Y = X
* 0X18 | REMAP_Y_Z | X = X; Y = Z; Z = Y
* 0X06 | REMAP_Z_X | Y = Y; X = Z; Z = X
* 0X12 | REMAP_X_Y_Z_TYPE0 | X = Z; Y = X; Z = Y
* 0X09 | REMAP_X_Y_Z_TYPE1 | X = Y; Y = Z; Z = X
* 0X24 | DEFAULT_AXIS | X = X; Y = Y; Z = Z
*/
void BNO055::axisRemap(uint8_t REMAP_CONFIG, uint8_t REMAP_SIGN)
{
	setMode(OPRMode::CONFIGMODE);
	writeByte(BNO055_AXIS_MAP_CONFIG, REMAP_CONFIG);
	writeByte(BNO055_AXIS_MAP_SIGN, REMAP_SIGN);
	setMode(_OPRMode);
}

// Set operationg mode
void BNO055::setMode(OPRMode mode)
{
	writeByte(BNO055_OPR_MODE, mode);
}

void BNO055::ChangeMode(OPRMode mode)
{
	_OPRMode = mode;
	setMode(mode);
}

// Check bootloader version
byte BNO055::bootloaderVersion() {
	setMode(CONFIGMODE);
	byte c =	 readByte(BNO055_BL_REV_ID);
	setMode(_OPRMode);
	return c;
}

uint8_t BNO055::getSystemStatus()
{
	_sysstat = readByte(BNO055_SYS_STATUS); // check system status
	if (_sysstat == 0x01) {
		_syserr = readByte(BNO055_SYS_ERR);
	}
	else _syserr = 0;
	return _sysstat;
}

uint8_t BNO055::selfTest()
{
	//force self test
	writeByte(BNO055_SYS_STATUS, SELF_TEST);
	delay(5);
	uint8_t result = readByte(BNO055_ST_RESULT); // check system status
	return result;
}

uint8_t BNO055::getSystemError()
{
	return _syserr;
}

//set this after reset
void BNO055::setClockExt(boolean usextal)
{
	if (usextal) {
		writeByte(BNO055_OPR_MODE, CONFIGMODE);
		writeByte(BNO055_SYS_TRIGGER, CLK_SEL);
		writeByte(BNO055_OPR_MODE, _OPRMode);
	}
}

void BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
	uint8_t calData = readByte(BNO055_CALIB_STAT);
	if (sys != NULL) {
		*sys = (calData >> 6) & 0x03;
	}
	if (gyro != NULL) {
		*gyro = (calData >> 4) & 0x03;
	}
	if (accel != NULL) {
		*accel = (calData >> 2) & 0x03;
	}
	if (mag != NULL) {
		*mag = calData & 0x03;
	}
}
