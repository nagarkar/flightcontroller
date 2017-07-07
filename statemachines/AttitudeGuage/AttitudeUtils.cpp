#include "AttitudeUtils.h"

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

static void extractXYZ(u8_t *regValue, float &x, float &y, float &z, float sensitivity);
static void normalizeQ(Q_cxyz &q);

static void normalizeQ(Q_cxyz &q) {
	float norm = q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
	q.q0 = q.q0/sqrtf(norm);
	q.q1 = q.q1/sqrtf(norm);
	q.q2 = q.q2/sqrtf(norm);
	q.q3 = q.q3/sqrtf(norm);
}

static void extractXYZ(u8_t *regValue, float &x, float &y, float &z, float sensitivity) {
	SensorAxes_t acceleration;
	int16_t dataRaw[3];
	dataRaw[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
	dataRaw[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
	dataRaw[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

	acceleration.AXIS_X = ( int32_t )( dataRaw[0] * sensitivity );
	acceleration.AXIS_Y = ( int32_t )( dataRaw[1] * sensitivity );
	acceleration.AXIS_Z = ( int32_t )( dataRaw[2] * sensitivity );

	x = dataRaw[0] * sensitivity;
	y = dataRaw[1] * sensitivity;
	z = dataRaw[2] * sensitivity;
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

	QF_CRIT_ENTRY(0);

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
		QF_CRIT_EXIT(0);
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
		QF_CRIT_EXIT(0);
		return Initialize(result, hhandle, hmagHandle);
	}

	// CTRL_REG1_G Register
	STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroDataRate(handle, LSM6DS0_ACC_GYRO_ODR_G_952Hz));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroBandwidthSelection(handle, LSM6DS0_ACC_GYRO_BW_G_ULTRA_HIGH));
	//STATUS_T_SET(status, LSM6DS0_ACC_GYRO_W_GyroFullScale(handle, LSM6DS0_ACC_GYRO_FS_G_2000dps));
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

	QF_CRIT_EXIT(0);

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

	// Read 12 registers at a time via I2C containing gyro and acc data.
	HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_G /* X Axis, low bit, gyro register */,
		I2C_MEMADD_SIZE_8BIT /* MemAddress Size */, acc_8_t /* Data */, 6 /* Data Size */, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
	extractXYZ(acc_8_t, angRate.x, angRate.y, angRate.z, gyroSensitivity/* mg/LSb */ * (1.0f/1000));

	static Acceleration cachedAcc;
	static MagneticField cachedMagField;
	if (counter % 10 == 0) { // Sample acc and mag field at about 100Hz, or every 10 ms

		HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LSM6DS0_ACC_GYRO_OUT_X_L_XL /* X Axis, low bit, gyro register */,
			I2C_MEMADD_SIZE_8BIT /* MemAddress Size */, acc_8_t /* Data */, 6 /* Data Size */, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
		extractXYZ(acc_8_t, cachedAcc.x, cachedAcc.y, cachedAcc.z, accSensitivity/* mdps/LSb */ * (1.0f/1000) * ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SEC_SQ);

		ctx = (DrvContextTypeDef *)magHandle;
		HAL_STATUS_SET(status, HAL_I2C_Mem_Read(GetI2CHandle(), ctx->address, LIS3MDL_MAG_OUTX_L /* X Axis, low bit, gyro register */,
				I2C_MEMADD_SIZE_8BIT /* MemAddress Size */, acc_8_t /* Data */, 6 /* Data Size */, NUCLEO_I2C_EXPBD_TIMEOUT_MAX));
		extractXYZ(acc_8_t, cachedMagField.x, cachedMagField.y, cachedMagField.z, magSensitivity/* gauss/LSb */ * (1.0f/1000));
	}
	magField = cachedMagField;
	acc = cachedAcc;
	float mult = 3.14159265358979323846f/180;
	MahonyAHRSupdate(&q, angRate.x * mult, angRate.y * mult, angRate.z * mult, acc.x, acc.y, acc.z, magField.x, magField.y, magField.z);
	normalizeQ(q);
	counter++;
	uint32_t cycleCounter = GetElapsedCycles(marker);
	cycleCounterAvg = (cycleCounterAvg * ((float)cycleCounterCount)/(cycleCounterCount + 1));
	cycleCounterAvg += ((float)cycleCounter)/(cycleCounterCount + 1);
	cycleCounterCount++;

	return (status == HAL_OK) ? MEMS_SUCCESS : MEMS_ERROR;
}
}// namespace
