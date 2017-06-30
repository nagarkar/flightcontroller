#include "../../api_fc/inc/controller.h"


static void zeroFloat32(float32_t *arry, uint16_t n) {
	for(uint16_t i = 0; i < n; i++)
		arry[i] = 0.00;
	return;
}

static float32_t normFloat32(float32_t *arry, uint16_t n) {
	float32_t sum = 0.00;
	float32_t sqRoot = 0.00;

	for(uint16_t i = 0; i < n; i++)
		sum += (arry[i] * arry[i]);

	arm_sqrt_f32(sum, &sqRoot);
	return sqRoot;
}

static void unitFloat32(float32_t *vetArryResult, float32_t *vetArry, uint8_t n) {

	float32_t norm = normFloat32(vetArry, n);

	for(uint8_t i = 0; i < n; i++)
		vetArryResult[i] = (vetArry[i] / norm);

	return;
}

static void crossFloat32(float32_t *vetArryResult, float32_t *vetArry1 ,float32_t *vetArry2) {
	vetArryResult[I] = (vetArry1[J] * vetArry2[K]) -  (vetArry1[K] * vetArry2[J]);
	vetArryResult[J] = (vetArry1[K] * vetArry2[I]) -  (vetArry1[I] * vetArry2[K]);
	vetArryResult[K] = (vetArry1[I] * vetArry2[J]) -  (vetArry1[J] * vetArry2[I]);
	return;
}


static float32_t dotFloat32(float32_t *vetArry1 ,float32_t *vetArry2) {
	return ((vetArry1[I] * vetArry2[I]) + (vetArry1[J] * vetArry2[J]) + (vetArry1[K] * vetArry2[K]));
}

static void subFloat32(float32_t *vetArryResult, float32_t *vetArry1 ,float32_t *vetArry2) {
	vetArryResult[I] = vetArry1[I] - vetArry2[I];
	vetArryResult[J] = vetArry1[J] - vetArry2[J];
	vetArryResult[K] = vetArry1[K] - vetArry2[K];
	return;
}

static void addFloat32(float32_t *vetArryResult, float32_t *vetArry1 ,float32_t *vetArry2) {
	vetArryResult[I] = vetArry1[I] + vetArry2[I];
	vetArryResult[J] = vetArry1[J] + vetArry2[J];
	vetArryResult[K] = vetArry1[K] + vetArry2[K];
	return;
}

static void mulFloat32(float32_t *vetArry, float32_t mulVal, uint8_t n) {
	for(uint8_t i = 0; i < n; i++) {
		vetArry[i] *= mulVal;
	}
	return;
}


void controller(float32_t t, stateStruct *state, destStateStruct *destState, robotParamsStruct *robotParams, resultFM *result) {

	float32_t g = robotParams->gravity;
	float32_t m = robotParams->mass;

	float32_t F = m * g;

	float32_t M[DIM];
	zeroFloat32(M, DIM);

	float32_t unitTangentV[DIM];

	if(normFloat32(destState->vel, DIM) == 0.00)
		zeroFloat32(unitTangentV, DIM);
	else
		unitFloat32(unitTangentV, destState->vel, DIM);

	float32_t nHat[DIM];
	if(normFloat32(destState->acc, DIM) == 0.00)
		zeroFloat32(nHat, DIM);
	else
		unitFloat32(nHat, destState->acc, DIM);

	float32_t bHat[DIM];
	crossFloat32(bHat, unitTangentV, nHat);

	float32_t posErrorNominal[DIM];
	subFloat32(posErrorNominal, destState->pos, state->pos);

	mulFloat32(nHat, dotFloat32(posErrorNominal, nHat), DIM);

	mulFloat32(bHat, dotFloat32(posErrorNominal, bHat), DIM);

	float32_t errorPosition[DIM];
	addFloat32(errorPosition, nHat, bHat);

	float32_t errorVelocity[DIM];
	subFloat32(errorVelocity, destState->vel, state->vel);


	float32_t gainMulTmp[DIM];
	float32_t gainAddTmp1[DIM];
	float32_t gainAddTmp2[DIM];

	// gainKd Kd1 = gainKd[0]

	float32_t gainKd[] = {1000.0, 1000.0, 1000.0};
	float32_t gainKp[] = {600.0, 600.0, 600.0};

	float32_t commandedRDotDot[] = {0.0, 0.0, 0.0};


	arm_mult_f32(gainKd, errorVelocity, gainMulTmp, DIM);
	arm_add_f32(destState->acc, gainMulTmp, gainAddTmp1, DIM);

	arm_mult_f32(gainKp, errorPosition, gainMulTmp, DIM);
	arm_add_f32(gainAddTmp1, gainMulTmp, commandedRDotDot, DIM);

//	arm_add_f32(gainAddTmp1, gainAddTmp2, commandedRDotDot, DIM);

	F = (commandedRDotDot[K] + g) * m;

	float32_t aX = commandedRDotDot[I];
	float32_t aY = commandedRDotDot[J];

	float32_t txDes = - (aY * arm_cos_f32(destState->yaw) - aX * arm_sin_f32(destState->yaw)) / g;
	float32_t tyDes = 	(aX * arm_cos_f32(destState->yaw) - aY * arm_sin_f32(destState->yaw)) / g;

	float32_t KpX = 2.0;
	float32_t KdX = 0.5;

	float32_t KpY = 2.0;
	float32_t KdY = 0.5;

	float32_t KpZ = 2.0;
	float32_t KdZ = 0.5;

//	float32_t rowMatArryTmp[] = {KpX, KdX};
//
//	float32_t colMatArryTmp[] = {
//			txDes - state->rot[I],
//			0.0 - state->omega[I]
//	};
//
//	float32_t resultArryMat[] = {0.0};
//
//	arm_matrix_instance_f32 rowMatTmp;
//	arm_matrix_instance_f32 colMatTmp;
//	arm_matrix_instance_f32 resultMatrix;
//
//	arm_mat_init_f32(&rowMatTmp, 1, 2, rowMatArryTmp);
//	arm_mat_init_f32(&colMatTmp, 2, 1, colMatArryTmp);
//	arm_mat_init_f32(&resultMatrix, 1, 1, resultArryMat);
//
//	arm_mat_mult_f32(&rowMatTmp, &colMatTmp, &resultMatrix);

//	M[I] = resultArryMat[0];

	M[I] = (KpX * (txDes - state->rot[I])) + (KdX * (0 - state->omega[I]));
	M[J] = (KpY * (tyDes - state->rot[J])) + (KdY * (0 - state->omega[J]));
	M[K] = (KpZ * (destState->yaw - state->rot[K])) + (KdZ * (destState->yawdot - state->omega[K]));

	result->F = F;
	result->M[I] = M[I];
	result->M[J] = M[J];
	result->M[K] = M[K];

	return;
}

