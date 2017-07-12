#include "AttitudeUtils.h"
#include "app_ao_config.h"

// QP/Framework
#include "bsp.h"
#include "active_events.h"
#include "active_log.h"

// STM32 Supporting Libs
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include <stdbool.h>
#include <math.h>

#include "LSM6DS0_ACC_GYRO_driver.h"
#include "LSM6DS0_ACC_GYRO_driver_HL.h"

#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"

#include "x_nucleo_iks01a1_magneto.h"
#include "LIS3MDL_MAG_driver.h"
#include "LIS3MDL_MAG_driver_HL.h"
#include "arm_math.h"

#define NUM_TAPS_19              	19
#define DIRTY_MEASUREMENT_TIME	 	100
#define TIME_FOR_MEAN_MEASUREMENT 	1000

static uint16_t numTaps19 = NUM_TAPS_19;

static float32_t FILTER_COEFF_LPF_50HZ[NUM_TAPS_19] = {
	0.0006,    0.0021,    0.0063,    0.0149,	0.0288,		0.0471,    0.0673,    0.0858,	 0.0988,
	0.1035,
	0.0988,    0.0858,    0.0673,    0.0471,	0.0288,		0.0149,    0.0063,    0.0021,    0.0006,
};

static float32_t FILTER_STATE_ACC_X[NUM_TAPS_19],  FILTER_STATE_ACC_Y[NUM_TAPS_19], FILTER_STATE_ACC_Z[NUM_TAPS_19];
static arm_fir_instance_f32 accFilterX, accFilterY, accFilterZ;

static float32_t FILTER_STATE_GYRO_X[NUM_TAPS_19],  FILTER_STATE_GYRO_Y[NUM_TAPS_19], FILTER_STATE_GYRO_Z[NUM_TAPS_19];
static arm_fir_instance_f32 gyroFilterX, gyroFilterY, gyroFilterZ;

static float32_t FILTER_STATE_MAG_X[NUM_TAPS_19],  FILTER_STATE_MAG_Y[NUM_TAPS_19], FILTER_STATE_MAG_Z[NUM_TAPS_19];
static arm_fir_instance_f32 magFilterX, magFilterY, magFilterZ;


using namespace StdEvents;

namespace Attitude {

float AttitudeUtils::cycleCounterAvg = 0;
int AttitudeUtils::cycleCounterCount = 0;
int AttitudeUtils::counter = 0;
float AttitudeUtils::accSensitivity = -1.0f;
float AttitudeUtils::gyroSensitivity = -1.0f;
float AttitudeUtils::magSensitivity = -1.0f;
Q_cxyz AttitudeUtils::q = {1.0f, 0.0f, 0.0f, 0.0f};

#define STATUS_T_SET(status, fn)  status =  ((fn) == MEMS_ERROR) ? MEMS_ERROR: status
#define HAL_STATUS_SET(status, fn)  status = (status == HAL_OK) ? (fn) : status;

#define MAG_OFFSET_X  -0.0217
#define MAG_OFFSET_Y  -0.0852
#define MAG_OFFSET_Z  0.4549

#define MAG_RADIUS_X  0.5070
#define MAG_RADIUS_Y  0.4698
#define MAG_RADIUS_Z  0.4546

const float32_t mag_rot_f32[9] = {	0.9673f, 	0.1121f, 	-0.2277f,
	    							0.0189f,  	-0.9266f,	-0.3756f,
									0.2530,		-0.3590,    0.8984};
arm_matrix_instance_f32 mag_rot = {3, 3, (float32_t *)mag_rot_f32};

const float32_t acc_rot_f32[9] = {	0.9673f, 	0.1121f, 	-0.2277f,
	    							0.0189f,  	-0.9266f,	-0.3756f,
									0.2530,		-0.3590,    0.8984};
arm_matrix_instance_f32 acc_rot = {3, 3, (float32_t *)acc_rot_f32};


static void extractXYZ(u8_t *regValue, AttitudeDim &dims, arm_fir_instance_f32 &filterX,
	arm_fir_instance_f32 &filterY, arm_fir_instance_f32 &filterZ, float sensitivity);
static void extractXYZNoFilter(u8_t *regValue, AttitudeDim &dims,  float sensitivity);

static void normalizeQ(Q_cxyz &q);
static void updateMagVector(MagneticField &magField);

static void normalizeQ(Q_cxyz &q) {
	float norm = q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
	q.q0 = q.q0/sqrtf(norm);
	q.q1 = q.q1/sqrtf(norm);
	q.q2 = q.q2/sqrtf(norm);
	q.q3 = q.q3/sqrtf(norm);
}

static void updateMagVector(MagneticField &magField) {

	// Center
	magField.x -= MAG_OFFSET_X;
	magField.y -= MAG_OFFSET_Y;
	magField.z -= MAG_OFFSET_Z;

	// Rotate
	//float32_t vect_members[3] = { magField.x, magField.y, magField.z };
	//arm_matrix_instance_f32 vect = {3, 1, vect_members};
	//float32_t result[3];
	//arm_matrix_instance_f32 resultVector = {3, 1, result};
	//arm_mat_mult_f32(&mag_rot, &vect, &resultVector);
	//magField.x = resultVector.pData[0];
	//magField.y = resultVector.pData[1];
	//magField.z = resultVector.pData[2];

	// Scale
	magField.x /= MAG_RADIUS_X;
	magField.y /= MAG_RADIUS_Y;
	magField.z /= MAG_RADIUS_Z;
}

static void extractXYZ(u8_t *regValue, AttitudeDim &dims, arm_fir_instance_f32 &filterX,
		arm_fir_instance_f32 &filterY, arm_fir_instance_f32 &filterZ, float sensitivity) {

	float filterOutput;

	int16_t dataRaw[3];
	dataRaw[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
	dataRaw[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
	dataRaw[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

	dims.x = dataRaw[0] * sensitivity;
	arm_fir_f32(&accFilterX, &dims.x, &filterOutput, 1);
	dims.x = filterOutput;

	dims.y = dataRaw[1] * sensitivity;
	arm_fir_f32(&accFilterY, &dims.y, &filterOutput, 1);
	dims.y = filterOutput;

	dims.z = dataRaw[2] * sensitivity;
	arm_fir_f32(&accFilterZ, &dims.z, &filterOutput, 1);
	dims.z = filterOutput;
}

static void extractXYZNoFilter(u8_t *regValue, AttitudeDim &dims, float sensitivity) {

	int16_t dataRaw[3];
	dataRaw[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
	dataRaw[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
	dataRaw[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

	dims.x = dataRaw[0] * sensitivity;
	dims.y = dataRaw[1] * sensitivity;
	dims.z = dataRaw[2] * sensitivity;
}


/**
 * Copied from LSM6DS0_ACC_GYRO_driver_HL.c (method: LSM6DS0_X_Get_Sensitivity). Make sure this is in sync with method.
 */
status_t AttitudeUtils::ResetAccSensitivity(void *handle) {
	assert_param(handle != NULL);
	LSM6DS0_ACC_GYRO_FS_XL_t fullScale;

	/* Read actual full scale selection from sensor. */
	if ( LSM6DS0_ACC_GYRO_R_AccelerometerFullScale(handle, &fullScale ) == MEMS_ERROR )	{
		return MEMS_ERROR;
	}

	/* Store the sensitivity based on actual full scale. */
	switch( fullScale )	{
	case LSM6DS0_ACC_GYRO_FS_XL_2g:
		accSensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_2G;
		break;
	case LSM6DS0_ACC_GYRO_FS_XL_4g:
		accSensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_4G;
		break;
	case LSM6DS0_ACC_GYRO_FS_XL_8g:
		accSensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_8G;
		break;
	case LSM6DS0_ACC_GYRO_FS_XL_16g:
		accSensitivity = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_16G;
		break;
	default:
		accSensitivity = -1.0f;
		return MEMS_ERROR;
	}

	return MEMS_SUCCESS;
}

/**
 * Copied from LSM6DS0_ACC_GYRO_driver_HL.c (method: LSM6DS0_G_Get_Sensitivity). Make sure this is in sync with method.
 */
status_t AttitudeUtils::ResetGyroSensitivity( void *handle ) {
	assert_param(handle != NULL);
	LSM6DS0_ACC_GYRO_FS_G_t fullScale;

	/* Read actual full scale selection from sensor. */
	if ( LSM6DS0_ACC_GYRO_R_GyroFullScale(handle, &fullScale ) == MEMS_ERROR ){
		return MEMS_ERROR;
	}

	/* Store the sensitivity based on actual full scale. */
	switch( fullScale )	{
	case LSM6DS0_ACC_GYRO_FS_G_245dps:
		gyroSensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_245DPS;
		break;
	case LSM6DS0_ACC_GYRO_FS_G_500dps:
		gyroSensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_500DPS;
		break;
	case LSM6DS0_ACC_GYRO_FS_G_2000dps:
		gyroSensitivity = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_2000DPS;
		break;
	default:
		gyroSensitivity = -1.0f;
		return MEMS_ERROR;
	}

	return MEMS_SUCCESS;
}

/**
 * Copied from LSM6DS0_ACC_GYRO_driver_HL.c (method: LSM6DS0_G_Get_Sensitivity). Make sure this is in sync with method.
 */
status_t AttitudeUtils::ResetMagSensitivity( void *handle ) {
	assert_param(handle != NULL);
	//TODO Make this dynamic; read the device register to get the full scale and then get corresponding sensitivity.
	//If the scale is changed from the default of +/- 4G, this method will need to change.
	magSensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4G;

	return MEMS_SUCCESS;
}

status_t AttitudeUtils::Initialize(DrvStatusTypeDef &result, void **hhandle, void **hmagHandle) {

	/*Configure GPIO pin : PB5 for interrupts from LSM6DS0 */
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	cycleCounterAvg = 0;
	cycleCounterCount = 0;
	static uint8_t isInitialized = 0, isMagInitialized = 0;

	//QF_CRIT_ENTRY(0);

	volatile status_t status = MEMS_SUCCESS;

	if (isInitialized == 1) {
		result = BSP_ACCELERO_DeInit(hhandle);
		isInitialized = 0;
	}
	result = BSP_ACCELERO_Init(LSM6DS0_X_0, hhandle);
	isInitialized = 1;
	void *handle = *hhandle;
	if (result == COMPONENT_OK) {
		result = BSP_ACCELERO_Sensor_Enable(handle);
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != GPIO_PIN_RESET) {
			LSM6DS0_ACC_GYRO_W_ResetSW(handle, LSM6DS0_ACC_GYRO_SW_RESET_YES);
		}
	}
	if (result != COMPONENT_OK) {
		//QF_CRIT_EXIT(0);
		return Initialize(result, hhandle, hmagHandle);
	}

	if (isMagInitialized == 1) {
		result = BSP_MAGNETO_DeInit(hmagHandle);
		isMagInitialized = 0;
	}
	result = BSP_MAGNETO_Init(LIS3MDL_0, hmagHandle);
	void *magHandle = *hmagHandle;
	if (result == COMPONENT_OK) {
		isMagInitialized = 1;
		result = BSP_MAGNETO_Sensor_Enable(magHandle);
	}
	if (result != COMPONENT_OK) {
		//QF_CRIT_EXIT(0);
		return Initialize(result, hhandle, hmagHandle);
	}

	// CTRL_REG1_G Register
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroDataRate(handle, LSM6DS0_ACC_GYRO_ODR_G_952Hz));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroBandwidthSelection(handle, LSM6DS0_ACC_GYRO_BW_G_ULTRA_HIGH));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroFullScale(handle, LSM6DS0_ACC_GYRO_FS_G_245dps));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AutoIndexOnMultiAccess(handle, LSM6DS0_ACC_GYRO_IF_ADD_INC_ENABLE));

	// CTRL_REG4 Register
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerDataRate(handle, LSM6DS0_ACC_GYRO_ODR_XL_952Hz));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerFullScale(handle, LSM6DS0_ACC_GYRO_FS_XL_2g));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerBandWitdthSelection(handle, LSM6DS0_ACC_GYRO_BW_SCAL_ODR_WITH_BW_XL));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerFilterBandwidth(handle, LSM6DS0_ACC_GYRO_BW_XL_105Hz));

	// CTRL_REG7_XL Register
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerAxisX(handle, LSM6DS0_ACC_GYRO_XEN_XL_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerAxisY(handle, LSM6DS0_ACC_GYRO_YEN_XL_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerAxisZ(handle, LSM6DS0_ACC_GYRO_ZEN_XL_ENABLE));

	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroAxisX(handle, LSM6DS0_ACC_GYRO_XEN_G_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroAxisY(handle, LSM6DS0_ACC_GYRO_YEN_G_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroAxisZ(handle, LSM6DS0_ACC_GYRO_ZEN_G_ENABLE));

	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerHighResolutionMode(handle, LSM6DS0_ACC_GYRO_HR_ENABLE));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerCutOff_filter(handle, LSM6DS0_ACC_GYRO_DCF_ODR_DIV_9));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_AccelerometerFilteredDataSelection(handle, LSM6DS0_ACC_GYRO_FDS_FILTER_ENABLE));

	STATUS_T_SET(status, ResetAccSensitivity(handle));
	STATUS_T_SET(status, ResetGyroSensitivity(handle));
	STATUS_T_SET(status, ResetMagSensitivity(handle));

	/////// 	INT_CTRL Register 	//////////
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_XL_DataReadyOnINT(handle, LSM6DS0_ACC_GYRO_INT_DRDY_XL_ENABLE));
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GYRO_DataReadyOnINT(handle, LSM6DS0_ACC_GYRO_INT_DRDY_G_ENABLE));

	STATUS_T_SET(status, LIS3MDL_MAG_W_OutputDataRate(magHandle, LIS3MDL_MAG_DO_80Hz)); // About 1/10th the gyro/acc rate.

	//QF_CRIT_EXIT(0);

	arm_fir_init_f32(&accFilterX, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_ACC_X[0], 1);
	arm_fir_init_f32(&accFilterY, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_ACC_Y[0], 1);
	arm_fir_init_f32(&accFilterZ, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_ACC_Z[0], 1);

	arm_fir_init_f32(&gyroFilterX, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_GYRO_X[0], 1);
	arm_fir_init_f32(&gyroFilterY, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_GYRO_Y[0], 1);
	arm_fir_init_f32(&gyroFilterZ, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_GYRO_Z[0], 1);

	arm_fir_init_f32(&magFilterX, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_MAG_X[0], 1);
	arm_fir_init_f32(&magFilterY, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_MAG_Y[0], 1);
	arm_fir_init_f32(&magFilterZ, numTaps19, &FILTER_COEFF_LPF_50HZ[0], &FILTER_STATE_MAG_Z[0], 1);

	return status;
}

status_t  AttitudeUtils::GetAttitude(Acceleration &acc, AngularRate &angRate, MagneticField &magField, void *handle, void *magHandle) {

	uint32_t marker = GetPerfCycle();
	u8_t value;

	LSM6DS0_ACC_GYRO_ReadReg(handle, LSM6DS0_ACC_GYRO_STATUS_REG, &value, 1);

	if ((value & LSM6DS0_ACC_GYRO_GDA_MASK) != LSM6DS0_ACC_GYRO_GDA_UP) {
		return MEMS_SUCCESS;
	}

	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
	HAL_StatusTypeDef status = HAL_OK;

	u8_t acc_8_t[6] = {0, 0, 0, 0, 0, 0};
	static float angRateMeanX = 0, angRateMeanY = 0, angRateMeanZ = 0;
	HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_G /* X Axis, low bit, gyro register */,
		I2C_MEMADD_SIZE_8BIT /* MemAddress Size */, acc_8_t /* Data */, 6 /* Data Size */, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
#if defined(GYRO_LPF)
	extractXYZ(acc_8_t, angRate, gyroFilterX, gyroFilterY, gyroFilterZ, gyroSensitivity/* mg/LSb */ * (1.0f/1000));
#else
	extractXYZNoFilter(acc_8_t, angRate, gyroSensitivity/* mg/LSb */ * (1.0f/1000));
#endif

#if defined(GYRO_MEAN_ADJUST)
	if (counter > DIRTY_MEASUREMENT_TIME && counter < TIME_FOR_MEAN_MEASUREMENT) {
		angRateMeanX += angRate.x; angRateMeanY += angRate.y; angRateMeanZ += angRate.z;
	} else if (counter == TIME_FOR_MEAN_MEASUREMENT) {
		int divisor = counter - DIRTY_MEASUREMENT_TIME;
		angRateMeanX /= divisor; angRateMeanY /= divisor; angRateMeanZ /= divisor;
	}
	if (counter > TIME_FOR_MEAN_MEASUREMENT) {
		angRate.x -= angRateMeanX; angRate.y -= angRateMeanY; angRate.z -= angRateMeanZ;
	}
#endif

#if defined(OUTPUT_GYRO_MEASUREMENTS)
	PRINT("gyro,%f,%f,%f\r\n", angRate.x, angRate.y, angRate.z);
#endif

	static Acceleration cachedAcc;
	static MagneticField cachedMagField;
	//if (counter % 10 == 0) { // Sample acc and mag field at about 100Hz, or every 10 ms. Only works if Gyro freq is set to 952 Hz.

		HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_XL /* X Axis, low bit, gyro register */,
			I2C_MEMADD_SIZE_8BIT /* MemAddress Size */, acc_8_t /* Data */, 6 /* Data Size */, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
#if defined(ACC_LPF)
		extractXYZ(acc_8_t, cachedAcc, accFilterX, accFilterY, accFilterZ, accSensitivity/* mdps/LSb */ * (1.0f/1000) * ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SEC_SQ);
#else
		extractXYZNoFilter(acc_8_t, cachedAcc, accSensitivity/* mdps/LSb */ * (1.0f/1000) * ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SEC_SQ);
#endif
#if defined(OUTPUT_ACC_MEASUREMENTS)
		PRINT("acc,%f,%f,%f\r\n", cachedAcc.x, cachedAcc.y, cachedAcc.z);
#endif
	//}

	if (counter % 10 == 0) { // Sample acc and mag field at 80Hz, or every 10 ms. Only works if Gyro freq is set to 952 Hz.
		ctx = (DrvContextTypeDef *)magHandle;
		HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LIS3MDL_MAG_OUTX_L /* X Axis, low bit, gyro register */,
				I2C_MEMADD_SIZE_8BIT /* MemAddress Size */, acc_8_t /* Data */, 6 /* Data Size */, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
#if defined(MAG_LPF)
		extractXYZ(acc_8_t, cachedMagField, magFilterX, magFilterY, magFilterZ, magSensitivity/* gauss/LSb */ * (1.0f/1000));
#else
		extractXYZNoFilter(acc_8_t, cachedMagField, magSensitivity/* gauss/LSb */ * (1.0f/1000));
#endif

#if defined(OUTPUT_MAG_MEASUREMENTS)
		PRINT("mag,%f,%f,%f\r\n", cachedMagField.x, cachedMagField.y, cachedMagField.z);
#endif
		// Emperically, zeroing out z axis mag field seems to reduce slow rotations of the attitude when the board is stationary.
		// TODO: Don't read this in the first place if you are going to zero it out.
#if defined(MAG_FIELD_ELLIPSOID_FIT)
		updateMagVector(cachedMagField);
#endif
#if defined(ZERO_Z_AXIS_MAG_FIELD)
		cachedMagField.z = 0.0f;
#endif

	}
	magField = cachedMagField;
	acc = cachedAcc;
	float mult = 3.14159265358979323846f/180;
	MahonyAHRSupdate(&q, angRate.x * mult, angRate.y * mult, angRate.z * mult, acc.x, acc.y, acc.z, magField.x, magField.y, magField.z);
	//MahonyAHRSupdateIMU(&q, angRate.x * mult, angRate.y * mult, angRate.z * mult, acc.x, acc.y, acc.z);
	normalizeQ(q);

#if defined(OUTPUT_ATTITUDE_QUATERNION)
	// Print attitude quaternion for matlab etc to read over serial and display
	if (counter > TIME_FOR_MEAN_MEASUREMENT) {
		PRINT("orientation,%f,%f,%f,%f\r\n", q.q0, q.q1, q.q2, q.q3);
	}
#endif
	counter++;
	uint32_t cycleCounter = GetElapsedCycles(marker);
	cycleCounterAvg = (cycleCounterAvg * ((float)cycleCounterCount)/(cycleCounterCount + 1));
	cycleCounterAvg += ((float)cycleCounter)/(cycleCounterCount + 1);
	cycleCounterCount++;
	return (status == HAL_OK) ? MEMS_SUCCESS : MEMS_ERROR;
}
}// namespace
