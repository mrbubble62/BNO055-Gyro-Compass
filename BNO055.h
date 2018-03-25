// BNO055.h

#ifndef _BNO055_h
#define _BNO055_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
//
// page 0
#define BNO055_CHIP_ID          0x00    // should be 0xA0              
#define BNO055_ACC_ID           0x01    // should be 0xFB              
#define BNO055_MAG_ID           0x02    // should be 0x32              
#define BNO055_GYRO_ID          0x03    // should be 0x0F              
#define BNO055_SW_REV_ID_LSB    0x04                                                                          
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F
#define BNO055_UNIQUEID         0x50
//
#define SELF_TEST 0x01
#define RST_SYS 0x20
#define RST_INT 0x40
#define CLK_SEL 0x80



class BNO055
{
public:
	uint8_t CHIP_ID = 0xA0;
	// Set initial input parameters
	enum Ascale {  // ACC Full Scale
		AFS_2G = 0,
		AFS_4G,
		AFS_8G,
		AFS_18G
	};

	enum Abw { // ACC Bandwidth
		ABW_7_81Hz = 0,
		ABW_15_63Hz,
		ABW_31_25Hz,
		ABW_62_5Hz,
		ABW_125Hz,
		ABW_250Hz,
		ABW_500Hz,
		ABW_1000Hz,    //0x07
	};

	enum APwrMode { // ACC Pwr Mode
		NormalA = 0,
		SuspendA,
		LowPower1A,
		StandbyA,
		LowPower2A,
		DeepSuspendA
	};

	enum Gscale {  // gyro full scale
		GFS_2000DPS = 0,
		GFS_1000DPS,
		GFS_500DPS,
		GFS_250DPS,
		GFS_125DPS    // 0x04
	};

	enum GPwrMode { // GYR Pwr Mode
		NormalG = 0,
		FastPowerUpG,
		DeepSuspendedG,
		SuspendG,
		AdvancedPowerSaveG
	};

	enum Gbw { // gyro bandwidth
		GBW_523Hz = 0,
		GBW_230Hz,
		GBW_116Hz,
		GBW_47Hz,
		GBW_23Hz,
		GBW_12Hz,
		GBW_64Hz,
		GBW_32Hz
	};

	enum OPRMode {  // BNO-55 operation modes
		CONFIGMODE = 0x00,
		// Sensor Mode
		ACCONLY,
		MAGONLY,
		GYROONLY,
		ACCMAG,
		ACCGYRO,
		MAGGYRO,
		AMG,     // 0x07
				 // Fusion Mode
				 IMU,
				 COMPASS,
				 M4G,
				 NDOF_FMC_OFF,
				 NDOF            // 0x0C
	};

	enum PWRMode {
		Normalpwr = 0,
		Lowpower,
		Suspendpwr
	};
	enum Xtal {
		internal=0,
		external
	};

	enum Modr {         // magnetometer output data rate  
		MODR_2Hz = 0,
		MODR_6Hz,
		MODR_8Hz,
		MODR_10Hz,
		MODR_15Hz,
		MODR_20Hz,
		MODR_25Hz,
		MODR_30Hz
	};

	enum MOpMode { // MAG Op Mode
		LowPower = 0,
		Regular,
		EnhancedRegular,
		HighAccuracy
	};

	enum MPwrMode { // MAG power mode
		Normal = 0,
		Sleep,
		Suspend,
		ForceMode
	};

	//enum
	//{
	//	AXIS_REMAP_CONFIG_P0 = 0x21,
	//	AXIS_REMAP_CONFIG_P1 = 0x24, // default
	//	AXIS_REMAP_CONFIG_P2 = 0x24,
	//	AXIS_REMAP_CONFIG_P3 = 0x21,
	//	AXIS_REMAP_CONFIG_P4 = 0x24,
	//	AXIS_REMAP_CONFIG_P5 = 0x21,
	//	AXIS_REMAP_CONFIG_P6 = 0x21,
	//	AXIS_REMAP_CONFIG_P7 = 0x24
	//};
	//enum
	//{
	//	AXIS_REMAP_SIGN_P0 = 0x04,
	//	AXIS_REMAP_SIGN_P1 = 0x00, // default
	//	AXIS_REMAP_SIGN_P2 = 0x06,
	//	AXIS_REMAP_SIGN_P3 = 0x02,
	//	AXIS_REMAP_SIGN_P4 = 0x03,
	//	AXIS_REMAP_SIGN_P5 = 0x01,
	//	AXIS_REMAP_SIGN_P6 = 0x07,
	//	AXIS_REMAP_SIGN_P7 = 0x05
	//};

public:
	typedef struct
	{
		int16_t bias_x;
		int16_t bias_y;
		int16_t bias_z;
		int16_t radius;
	} accOffset;

	typedef struct
	{
		int16_t bias_x;
		int16_t bias_y;
		int16_t bias_z;
	} gyroOffset;

	typedef struct
	{
		int16_t bias_x;
		int16_t bias_y;
		int16_t bias_z;
		int16_t radius;
	} magOffset;

	typedef struct
	{
		uint32_t bias[3];
		uint32_t max[3];
		uint32_t min[3];
	} calResult;

	int begin(uint8_t, OPRMode, Xtal externalXtal);
	void init();
	uint8_t getMode();
	int8_t readGyroTempData();
	void readCalibration(accOffset *, gyroOffset *, magOffset *);
	void readAccelData(int16_t *);
	void readGyroData(int16_t *);
	void readMagData(int16_t *);
	void readGRVData(int16_t *);
	void readLIAData(int16_t *);
	void readEulData(int16_t *);
	void readQuatData(int16_t *);
	void readBytes(uint8_t, uint8_t, uint8_t *);
	void writeByte(uint8_t, uint8_t);
	uint8_t readByte(uint8_t);
	calResult calibrateAccel();
	calResult calibrateGyro();
	calResult magCal();
	void startMagCal();
	void updateMagCal();
	calResult finishMagCal();
	uint32_t GetSampleCount();
	void enableInterrupt(uint8_t);
	void ackInterrupt();
	void reset();
	void axisRemap(uint8_t config, uint8_t sign);
	uint8_t _sysstat = 0;
	uint8_t _syserr = 0;
	uint8_t getSystemStatus();
	uint8_t getSystemError();
	byte bootloaderVersion();
	void setAccOffsets(accOffset);
	void setGyroOffsets(gyroOffset);
	void setMagOffsets(magOffset);
	void setClockExt(boolean);
	void  getCalibration(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
	float gyroOffset_x;
	float gyroOffset_y;
	float gyroOffset_z;
	uint8_t selfTest();
	void ChangeMode(OPRMode);

private:
	void setMode(OPRMode);
	int32_t _acc_offset_x;
	int32_t _acc_offset_y;
	int32_t _acc_offset_z;
	int32_t _gyr_offset_x;
	int32_t _gyr_offset_y;
	int32_t _gyr_offset_z;
	int32_t _mag_offset_x;
	int32_t _mag_offset_y;
	int32_t _mag_offset_z;
	int16_t _accel_radius;
	int16_t _mag_radius;
	uint8_t _address;
	OPRMode  _OPRMode;
	APwrMode _APwrMode;
	Abw _Abw;
	Ascale _Ascale;
	Gbw _Gbw;
	Gscale _Gscale;
	GPwrMode _GPwrMode;
	MPwrMode _MPwrMode;
	MOpMode _MOpMode;
	Modr _Modr;
	PWRMode _PWRMode;
	Xtal _Xtal;
	float aRes, gRes, mRes;       // scale resolutions per LSB for the sensors
	int32_t mag_bias[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { 0, 0, 0 }, mag_min[3] = { 0, 0, 0 };
	uint32_t  _samplecount;
};

#endif

