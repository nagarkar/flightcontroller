#include "../../api_fc/inc/controller.h"


static float32_t normFloat32(float32_t *arry, uint16_t n) {
	float32_t dotResult = 0.00;
	float32_t sqRoot = 0.00;

	arm_dot_prod_f32(arry, arry, DIM, &dotResult);

	arm_sqrt_f32(dotResult, &sqRoot);
	return sqRoot;
}

static void unitFloat32(float32_t *vetArryResult, float32_t *vetArry, uint8_t n) {

	float32_t norm = normFloat32(vetArry, n);
	norm = 1 / norm;

	arm_scale_f32(vetArry, norm, vetArryResult, DIM);

	return;
}

static void crossFloat32(float32_t *vetArryResult, float32_t *vetArry1 ,float32_t *vetArry2) {

	float32_t matArry3x3Tmp[] = {
			0.0, 			- vetArry2[K],		vetArry2[J],
			vetArry2[K], 	0.0,				- vetArry2[I],
			- vetArry2[J], 	vetArry2[I],		0.0
	};

	vetArryResult[I] = 0.0;
	vetArryResult[J] = 0.0;
	vetArryResult[K] = 0.0;

	arm_matrix_instance_f32 matA;
	arm_matrix_instance_f32 matB;
	arm_matrix_instance_f32 resultMatrix;

	// Create a 1 x 3 row Column matrix as matrix a = [aI, aJ, aK]
	arm_mat_init_f32(&matA, 1, 3, vetArry1);

	arm_mat_init_f32(&matB, 3, 3, matArry3x3Tmp);

	arm_mat_init_f32(&resultMatrix, 1, 3, vetArryResult);

	arm_mat_mult_f32(&matA, &matB, &resultMatrix);

	return;
}

resultFM controller(float32_t t, stateStruct *state, destStateStruct *destState, robotParamsStruct *robotParams) {

	resultFM result;

	float32_t g = robotParams->gravity;
	float32_t m = robotParams->mass;

	float32_t F = m * g;

	float32_t M[DIM];

	arm_fill_f32(ZERO, M, DIM);

	float32_t unitTangentV[DIM];

	if(normFloat32(destState->vel, DIM) == 0.00)
		arm_fill_f32(ZERO, unitTangentV, DIM);
	else
		unitFloat32(unitTangentV, destState->vel, DIM);

	float32_t nHat[DIM];
	if(normFloat32(destState->acc, DIM) == 0.00)
		arm_fill_f32(ZERO, nHat, DIM);
	else
		unitFloat32(nHat, destState->acc, DIM);

	float32_t bHat[DIM];
	crossFloat32(bHat, unitTangentV, nHat);

	float32_t posErrorNominal[DIM];
	arm_sub_f32(destState->pos, state->pos, posErrorNominal, DIM);

	float32_t dotResult = 0.0;
	arm_dot_prod_f32(posErrorNominal, nHat, DIM, &dotResult);
	arm_scale_f32(nHat, dotResult, nHat, DIM);

	arm_dot_prod_f32(posErrorNominal, bHat, DIM, &dotResult);
	arm_scale_f32(bHat, dotResult, bHat, DIM);

	float32_t errorPosition[DIM];
	arm_add_f32(nHat, bHat, errorPosition, DIM);

	float32_t errorVelocity[DIM];
	arm_sub_f32(destState->vel, state->vel, errorVelocity, DIM);

	float32_t gainMulTmp[DIM];
	float32_t gainAddTmp[DIM];

	// gainKd Kd1 = gainKd[0], gainKd Kd2 = gainKd[1] and so on.
	float32_t gainKd[] = {1000.0, 1000.0, 1000.0};

	// gainKd Kp1 = gainKp[0], gainKd Kp2 = gainKp[1] and so on.
	float32_t gainKp[] = {600.0, 600.0, 600.0};

	float32_t commandedRDotDot[DIM];
	arm_fill_f32(ZERO, commandedRDotDot, DIM);

	arm_mult_f32(gainKd, errorVelocity, gainMulTmp, DIM);
	arm_add_f32(destState->acc, gainMulTmp, gainAddTmp, DIM);

	arm_mult_f32(gainKp, errorPosition, gainMulTmp, DIM);
	arm_add_f32(gainAddTmp, gainMulTmp, commandedRDotDot, DIM);

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

	float32_t rowMatArryTmp[] = {KpX, KdX};
	float32_t colMatArryTmp[] = {
			txDes - state->rot[I],
			0.0 - state->omega[I]
	};
	float32_t resultArryMat[] = {0.0};

	arm_matrix_instance_f32 rowMatTmp;
	arm_matrix_instance_f32 colMatTmp;
	arm_matrix_instance_f32 resultMatrix;

	arm_mat_init_f32(&rowMatTmp, 1, 2, rowMatArryTmp);
	arm_mat_init_f32(&colMatTmp, 2, 1, colMatArryTmp);
	arm_mat_init_f32(&resultMatrix, 1, 1, resultArryMat);
	arm_mat_mult_f32(&rowMatTmp, &colMatTmp, &resultMatrix);
	M[I] = resultArryMat[0];


	rowMatArryTmp[0] = KpY;
	rowMatArryTmp[1] = KdY;
	colMatArryTmp[0] = tyDes - state->rot[J];
	colMatArryTmp[1] = 0.0 - state->omega[J];

	arm_mat_init_f32(&rowMatTmp, 1, 2, rowMatArryTmp);
	arm_mat_init_f32(&colMatTmp, 2, 1, colMatArryTmp);
	arm_mat_init_f32(&resultMatrix, 1, 1, resultArryMat);
	arm_mat_mult_f32(&rowMatTmp, &colMatTmp, &resultMatrix);
	M[J] = resultArryMat[0];


	rowMatArryTmp[0] = KpZ;
	rowMatArryTmp[1] = KdZ;
	colMatArryTmp[0] = destState->yaw - state->rot[K];
	colMatArryTmp[1] = destState->yawdot - state->omega[K];

	arm_mat_init_f32(&rowMatTmp, 1, 2, rowMatArryTmp);
	arm_mat_init_f32(&colMatTmp, 2, 1, colMatArryTmp);
	arm_mat_init_f32(&resultMatrix, 1, 1, resultArryMat);
	arm_mat_mult_f32(&rowMatTmp, &colMatTmp, &resultMatrix);
	M[K] = resultArryMat[0];

	//	 Results
	result.F = F;
	result.M[I] = M[I];
	result.M[J] = M[J];
	result.M[K] = M[K];

	return result;
}

