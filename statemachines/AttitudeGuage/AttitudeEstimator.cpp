/*
 * AttitudeEstimator.cpp
 *
 *  Created on: Jul 27, 2017
 *      Author: chinm_000
 */
#include "AltitudeEstimator.h"
#include <stdio.h>

/**
 *  A = [1, T, 0.5*T2, 0; 0, 1, T, 0; 0, 0, 1, 0; 0, 0, 0, 1];
	P = eye(4);
	syms Q_std Q_bias;
	assume([Q_std Q_bias], 'real');
	assume([Q_std Q_bias], 'positive');
	Q = [Q_std, 0, 0, 0; 0, Q_std, 0, 0; 0, 0, Q_std, 0; 0, 0, 0, Q_bias];
	C = [1, 0, 0, 0; 0, 0, 1, 1];
 */
static float32_t A_m_def[STATE_N * STATE_N] = {
		1.0f, T, 	0.5f*powf(T,2), 0.0f,
		0.0f, 1.0f, T, 				0.0f,
		0.0f, 0.0f,	1.0f, 			0.0f,
		0.0f, 0.0f, 0.0f, 			1.0f};

static float32_t A_m_t_def[STATE_N * STATE_N];


static float32_t Q_m_def[STATE_N * STATE_N] = {
		0.000000001f, 	0.0f, 			0.0f, 			0.0f,
		0.0f, 			0.000000001f,	0.0f, 			0.0f,
		0.0f, 			0.0f,			0.000000001f,	0.0f,
		0.0f, 			0.0f, 			0.0f, 			0.00000000000000000000001f};

static float32_t P_m_def[STATE_N * STATE_N] = {
		0.0f, 	0.0f, 	0.0f, 	0.0f,
		0.0f,	0.0f,	0.0f, 	0.0f,
		0.0f, 	0.0f,	0.0f,	0.0f,
		0.0f, 	0.0f, 	0.0f, 	0.0f};


static float32_t C_m_def[OBS_M 	 * STATE_N] = {
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 1.0f};

static float32_t C_m_t_def[STATE_N * OBS_M];


static float32_t R_m_def[OBS_M 	 * OBS_M] 	= {
		0.06f, 		0.0088f,
		0.0088f,	0.275f};

static float32_t Z_v_def[OBS_M 	 * 1] 	= { 0.0f, 0.0f };

static float32_t X_v_def[STATE_N * 1] 	= { 0.0f, 0.0f, 0.0f, 0.0f };

namespace Attitude {

#define MAT_PROC(statusVar, armCall) \
	do {							\
		(statusVar) = (armCall); 	\
		if ((statusVar) < 0) {		\
			return (statusVar);		\
		}							\
	} while(0)


AltitudeEstimator::AltitudeEstimator(): printMatrices(false) {
	A_m = {STATE_N, STATE_N, A_m_def};
	A_m_t = {STATE_N, STATE_N, A_m_t_def};
	arm_mat_trans_f32(&A_m, &A_m_t);
	Q_m = {STATE_N, STATE_N, Q_m_def};
	C_m = {OBS_M, STATE_N, 	 C_m_def};
	C_m_t = {STATE_N, OBS_M, C_m_t_def};
	arm_mat_trans_f32(&C_m, &C_m_t);
	Z_v = {OBS_M, 1, 	 Z_v_def};
	R_m = {OBS_M, OBS_M,	 R_m_def};
	altitudeState.P_m = {STATE_N, STATE_N, P_m_def};
	altitudeState.X_v = {STATE_N, 1, X_v_def};
}

static char printbuff[STATE_N * STATE_N * sizeof(float)];
static char * printMat(arm_matrix_instance_f32 mat) {
	uint16_t nrows = mat.numRows;
	uint16_t ncols = mat.numCols;
	int maxSize = (STATE_N) * (STATE_N) * 15; // 9 = num chars in string of form "x.yz+e+ab", corresponding to format %1.2e with padding.
	char *nextPrintLoc = printbuff;
	int charsWritten  = 0;

	charsWritten += sprintf(nextPrintLoc + charsWritten, "[\r\n");
	for (int i = 0; i < nrows * ncols; i++) {
		if (i % ncols == 0) {
			charsWritten += sprintf(nextPrintLoc + charsWritten, "\r\n");
			if (charsWritten + 2 > maxSize) { return printbuff; }
		}
		float32_t val = mat.pData[i];
		charsWritten += sprintf(nextPrintLoc + charsWritten, "%1.2e, ", val);
		if (charsWritten + 2 > maxSize) { return printbuff; }
	}
	charsWritten += sprintf(nextPrintLoc + charsWritten, "]\r\n");
	return printbuff;
}

char * AltitudeEstimator::printSys() {
	char * str = printMat(A_m);
	printf("Sys.A:\r\n%s", str);
	str = printMat(A_m_t);
	printf("Sys.A':\r\n%s", str);
	str = printMat(C_m);
	printf("Sys.C:\r\n%s", str);
	str = printMat(C_m_t);
	printf("Sys.C':\r\n%s", str);
	str = printMat(Q_m);
	printf("Sys.Q:\r\n%s", str);
	str = printMat(R_m);
	printf("Sys.R:\r\n%s", str);
	str = printMat(Z_v);
	printf("Sys.Z:\r\n%s", str);
}

arm_status AltitudeEstimator::filter2(float32_t z_p, float32_t z_a) {

	float32_t * state = altitudeState.X_v.pData;
	float32_t * Pdata = altitudeState.P_m.pData;
	float32_t p = state[0];
	float32_t v = state[1];
	float32_t a = state[2];
	float32_t bias= state[3];
	float32_t pNew, vNew, aNew, biasNew;

	pNew = p + T*v + (T2*a)/2 + ((a + bias - z_a)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (((a*T2)/2 + v*T + p - z_p)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	vNew = v + T*a - (2*(a + bias - z_a)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*((a*T2)/2 + v*T + p - z_p)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	aNew = a - (2*((a*T2)/2 + v*T + p - z_p)*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((a + bias - z_a)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	biasNew = bias - ((a + bias - z_a)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*((a*T2)/2 + v*T + p - z_p))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);

	state[0] = pNew;
	state[1] = vNew;
	state[2] = aNew;
	state[3] = biasNew;

	Pdata[0] = Q_std + T2 + T4/4 - ((((T4/4 + T2 + Q_std + R_bar + 1)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((T2/2 + R_bar_acc)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((((Q_bias + Q_std + R_acc + 2)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((T2/2 + R_bar_acc)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + 1;
	Pdata[1] = T + T3/2 - (2*(((T4/4 + T2 + Q_std + R_bar + 1)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((T2/2 + R_bar_acc)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(((Q_bias + Q_std + R_acc + 2)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((T2/2 + R_bar_acc)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[2] = T2/2 - (2*(((T4/4 + T2 + Q_std + R_bar + 1)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((T2/2 + R_bar_acc)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + ((((Q_bias + Q_std + R_acc + 2)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((T2/2 + R_bar_acc)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[3] = ((((Q_bias + Q_std + R_acc + 2)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((T2/2 + R_bar_acc)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(((T4/4 + T2 + Q_std + R_bar + 1)*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((T2/2 + R_bar_acc)*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(Q_bias + 1)*(T2 + 2*R_bar_acc))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);

	Pdata[4] = T + T3/2 - (((2*(T4/4 + T2 + Q_std + R_bar + 1)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (((2*(Q_bias + Q_std + R_acc + 2)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[5] = Q_std + T2 - (2*((2*(T4/4 + T2 + Q_std + R_bar + 1)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*((2*(Q_bias + Q_std + R_acc + 2)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + 1;
	Pdata[6] = T - (2*((2*(T4/4 + T2 + Q_std + R_bar + 1)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (((2*(Q_bias + Q_std + R_acc + 2)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[7] = (2*((2*(T4/4 + T2 + Q_std + R_bar + 1)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(Q_bias + 1)*(T2 + 2*R_bar_acc))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (((2*(Q_bias + Q_std + R_acc + 2)*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);

	Pdata[8] = T2/2 + ((((Q_bias + Q_std + R_acc + 2)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((((T2/2 + R_bar_acc)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2)*(T4/4 + T2 + Q_std + R_bar + 1))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[9] = T - (2*(((Q_bias + Q_std + R_acc + 2)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(((T2/2 + R_bar_acc)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2)*(T4/4 + T2 + Q_std + R_bar + 1))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[10] = Q_std - ((((Q_bias + Q_std + R_acc + 2)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(((T2/2 + R_bar_acc)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2)*(T4/4 + T2 + Q_std + R_bar + 1))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + 1;
	Pdata[11] = (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(((T2/2 + R_bar_acc)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2)*(T4/4 + T2 + Q_std + R_bar + 1))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8)))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((((Q_bias + Q_std + R_acc + 2)*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(T2/2 + R_bar_acc)*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);

	Pdata[12] = ((((Q_bias + Q_std + R_acc + 2)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(T2/2 + R_bar_acc))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*R_bar_acc + 4*Q_std_x_R_bar_acc - 2*R_bar_x_T2 + 4*R_bar_acc_x_T2 + R_bar_acc*T4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((((T2/2 + R_bar_acc)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(T4/4 + T2 + Q_std + R_bar + 1))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*Q_bias + 12*Q_std + 4*R_acc + 4*Q_bias_x_Q_std + 4*Q_std_x_R_acc + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 8*T2 + T4 + 8))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[13] = - (2*(((Q_bias + Q_std + R_acc + 2)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(T2/2 + R_bar_acc))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(2*T + 2*Q_std_x_T + 2*R_bar*T - 2*R_bar_acc*T - R_bar_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(((T2/2 + R_bar_acc)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(T4/4 + T2 + Q_std + R_bar + 1))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*T + 2*Q_bias_x_T + 2*Q_std_x_T + 2*R_acc*T - 2*R_bar_acc*T + Q_bias_x_T3 + Q_std_x_T3 + R_acc*T3 + T3))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[14] = - (2*(((T2/2 + R_bar_acc)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(T4/4 + T2 + Q_std + R_bar + 1))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(Q_bias_x_T2 - 2*Q_std_x_R_bar_acc - 2*R_bar_acc + R_acc*T2 + T2))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - ((((Q_bias + Q_std + R_acc + 2)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(T2/2 + R_bar_acc))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(8*Q_std + 4*R_bar + 4*Q_std_x_R_bar + 4*Q_std_x_T2 + Q_std_x_T4 - 2*R_bar_acc_x_T2 + 4*Q_std2 + 4*T2 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8);
	Pdata[15] = Q_bias - ((((Q_bias + Q_std + R_acc + 2)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(T2/2 + R_bar_acc))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + (2*(((T2/2 + R_bar_acc)*(4*Q_bias + 4*Q_std + 4*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*T2 + T4 + 4))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) - (2*(Q_bias + 1)*(T2 + 2*R_bar_acc)*(T4/4 + T2 + Q_std + R_bar + 1))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8))*(Q_bias + 1)*(T2 + 2*R_bar_acc))/(4*Q_bias + 12*Q_std + 4*R_acc + 8*R_bar + 4*Q_bias_x_Q_std + 4*Q_bias_x_R_bar + 4*Q_std_x_R_acc + 4*Q_std_x_R_bar + 4*R_acc_x_R_bar + 4*Q_bias_x_T2 + Q_bias_x_T4 + 4*Q_std_x_T2 + Q_std_x_T4 + 4*R_acc*T2 + R_acc*T4 - 4*R_bar_acc_x_T2 + 4*Q_std2 - 4*R_bar_acc2 + 8*T2 + T4 + 8) + 1;

}
/**
 *  % Time Update
    x_apriori = A * xPrev + B * u;
    P_apriori = A * PPrev * A' + F * Q * F';

    yPredicted = C * x_apriori + D * u;
    YPredicted = C * P_apriori * C';

    % Compute Measurement Error
    e = y - yPredicted;
    E = YPredicted + G * R * G';

    % Compute Kalman Gain & Estimate State
    K = P_apriori * C' / E;

    xNew = x_apriori + K * e;
    PNew = P_apriori - K * E * K';
 */

arm_status AltitudeEstimator::filter(float32_t posZ, float accZ) {
	arm_status status = ARM_MATH_SUCCESS;

	// x_apriori = A * xPrev + B * u; with u = 0
	MAT_PROC(status, arm_mat_mult_f32(&A_m, &altitudeState.X_v, &xApriori));
	if (printMatrices) printf("xApriori:\r\n%s", printMat(xApriori));

	// P_apriori = A * PPrev * A' + F * Q * F';	with F = 1
	MAT_PROC(status, arm_mat_mult_f32(&A_m, 	&altitudeState.P_m, &tmpNxN));
	MAT_PROC(status, arm_mat_mult_f32(&tmpNxN,	&A_m_t,    			&tmp2NxN));
	MAT_PROC(status, arm_mat_add_f32(&tmp2NxN, 	&Q_m, 				&PApriori));

	if (printMatrices) printf("PApriori:\r\n%s", printMat(PApriori));

	// zPredicted = C * x_apriori + D * u;	with u = 0
	MAT_PROC(status, arm_mat_mult_f32(&C_m, &xApriori, &zPredicted));
	if (printMatrices) printf("zPred:\r\n%s", printMat(zPredicted));

	// ZPredicted = C * P_apriori * C';
	MAT_PROC(status, arm_mat_mult_f32(&C_m,  &PApriori, &tmpMxN));
	MAT_PROC(status, arm_mat_mult_f32(&tmpMxN, &C_m_t, 	&ZPredicted));
	if (printMatrices) printf("ZPred:\r\n%s", printMat(ZPredicted));

	// e = z - zPredicted;
	Z_v.pData[0] = posZ; Z_v.pData[1] = accZ;
	MAT_PROC(status, arm_mat_sub_f32(&Z_v, &zPredicted, &e));
	if (printMatrices) printf("e:\r\n%s", printMat(e));

	// E = ZPredicted + G * R * G';		with G = 1
	MAT_PROC(status, arm_mat_add_f32(&ZPredicted, &R_m, &E));
	if (printMatrices) printf("E:\r\n%s", printMat(E));

	// K = P_apriori * C' / E;
	MAT_PROC(status, arm_mat_mult_f32(	 &PApriori, &C_m_t, &tmpNxM));
	MAT_PROC(status, arm_mat_inverse_f32(&E, 		&tmpMxM));
	MAT_PROC(status, arm_mat_mult_f32(	 &tmpNxM, 	&tmpMxM,  &K));
	if (printMatrices) printf("K:\r\n%s", printMat(K));

	// xNew = x_apriori + K * e;
	arm_mat_mult_f32(&K, &e, &tmpNx1);
	MAT_PROC(status, arm_mat_add_f32(&xApriori, &tmpNx1, &tmp2Nx1));
	if (printMatrices) printf("xNew:\r\n%s", printMat(tmp2Nx1));
	memcpy(altitudeState.X_v.pData, tmp2Nx1.pData, STATE_N * sizeof(float));

	// PNew = P_apriori - K * E * K';
//	MAT_PROC(status, arm_mat_mult_f32( &K, 			&E, 	&tmpNxM));
//	MAT_PROC(status, arm_mat_trans_f32(&K, 			&tmpMxN));
//	MAT_PROC(status, arm_mat_mult_f32( &tmpNxM,		&tmpMxN, 	&tmpNxN));
//	MAT_PROC(status, arm_mat_sub_f32(  &PApriori, 	&tmpNxN, 	&tmp2NxN));
//	if (printMatrices) printf("PNew:\r\n%s", printMat(tmp2NxN));
//	memcpy(altitudeState.P_m.pData, tmp2NxN.pData, STATE_N * STATE_N * sizeof(float));

	// PNew = P_apriori - K * C * P_apriori
	MAT_PROC(status, arm_mat_mult_f32( &K, 			&C_m, 	&tmpNxN));
	MAT_PROC(status, arm_mat_mult_f32( &tmpNxN, 	&PApriori, 	&tmp2NxN));
	MAT_PROC(status, arm_mat_sub_f32( &PApriori, 	&tmp2NxN,	&tmpNxN));
	if (printMatrices) printf("PNew:\r\n%s", printMat(tmpNxN));
	memcpy(altitudeState.P_m.pData, tmpNxN.pData, STATE_N * STATE_N * sizeof(float));

	// PNew = (I - K*C) * P_apriori
/*	MAT_PROC(status, arm_mat_mult_f32( &K, 			&C_m, 	&tmpNxN));
	tmpNxN.pData[0]  = 1.0f - tmpNxN.pData[0];
	tmpNxN.pData[4]  = 1.0f - tmpNxN.pData[4];
	tmpNxN.pData[8]  = 1.0f - tmpNxN.pData[8];
	tmpNxN.pData[12] = 1.0f - tmpNxN.pData[12];
	MAT_PROC(status, arm_mat_mult_f32( &tmpNxN, 	&PApriori, 	&tmp2NxN));
	if (printMatrices) printf("PNew:\r\n%s", printMat(tmpNxN));
	memcpy(altitudeState.P_m.pData, tmpNxN.pData, STATE_N * STATE_N * sizeof(float));*/

	return status;

}


} // Namespace
