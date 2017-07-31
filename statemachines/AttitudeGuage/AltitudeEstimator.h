#ifndef _AO_ALTITUDE_ESTIMATOR_H
#define _AO_ALTITUDE_ESTIMATOR_H

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

// Static (global) variables for faster processing https://people.cs.clemson.edu/~dhouse/courses/405/papers/optimize.pdf
// static => only available in this class, const: will not change.
static const int STATE_N = 4;
static const int OBS_M = 2;

static float32_t tmpNxN_def[STATE_N * STATE_N];
static float32_t tmp2NxN_def[STATE_N * STATE_N];

static float32_t tmpNxM_def[STATE_N * OBS_M];
static float32_t tmpMxM_def[OBS_M   * OBS_M];
static float32_t tmpMxN_def[OBS_M   * STATE_N];

static float32_t tmpNx1_def[STATE_N * 1];
static float32_t tmp2Nx1_def[STATE_N * 1];
static float32_t tmpMx1_def[OBS_M	 * 1];

static float32_t xApriori_def[STATE_N * 1];
static float32_t PApriori_def[STATE_N * STATE_N];
static float32_t zPredicted_def[OBS_M * 1];
static float32_t ZPredicted_def[OBS_M * OBS_M];
static float32_t e_def[OBS_M * 1];
static float32_t E_def[OBS_M * OBS_M];
static float32_t K_def[STATE_N * OBS_M];

static arm_matrix_instance_f32 tmpNxN = 	{STATE_N, STATE_N, tmpNxN_def};
static arm_matrix_instance_f32 tmp2NxN = 	{STATE_N, STATE_N, tmp2NxN_def};

static arm_matrix_instance_f32 tmpMx1 = 	{OBS_M,   1, 	   tmpMx1_def};
static arm_matrix_instance_f32 tmpNx1 = 	{STATE_N, 1, 	   tmpNx1_def};
static arm_matrix_instance_f32 tmp2Nx1 = 	{STATE_N, 1, 	   tmp2Nx1_def};
static arm_matrix_instance_f32 tmpNxM = 	{STATE_N, OBS_M,   tmpNxM_def};
static arm_matrix_instance_f32 tmpMxN = 	{OBS_M,   STATE_N, tmpMxN_def};
static arm_matrix_instance_f32 tmpMxM = 	{OBS_M,   OBS_M,   tmpMxM_def};

static arm_matrix_instance_f32 xApriori = {STATE_N, 1, xApriori_def};;
static arm_matrix_instance_f32 PApriori = {STATE_N, STATE_N, PApriori_def};;
static arm_matrix_instance_f32 zPredicted = {OBS_M, 1, zPredicted_def};
static arm_matrix_instance_f32 ZPredicted = {OBS_M, OBS_M, ZPredicted_def};
static arm_matrix_instance_f32 e = {OBS_M, 1, e_def};;
static arm_matrix_instance_f32 E = {OBS_M, OBS_M, E_def};;
static arm_matrix_instance_f32 K = {STATE_N, OBS_M, 	K_def};;


const uint8_t ALTITUDE_KALMAN_SAMPLING_FACTOR = 15;
static const float32_t T = (1.0f * ALTITUDE_KALMAN_SAMPLING_FACTOR)/930;
static const float32_t T2 = powf(T, 2);
static const float32_t T3 = powf(T, 3);
static const float32_t T4 = powf(T, 4);
//static const float32_t R_bar_acc = -0.0003f;
//static const float32_t R_acc = 0.00005f;
//static const float32_t R_bar = 0.06f;

//static const float32_t R_bar_acc = 0.0003f;
//static const float32_t R_acc = 0.00002f;
//static const float32_t R_bar = 0.006f;

static const float32_t R_bar = 		2.0e-1;
static const float32_t R_acc = 		5.0e-7;
static const float32_t R_bar_acc =  3.0e-5;


static const float32_t Q_std = 		1.0e-9;
static const float32_t Q_bias = 	1.0e-25;


static const float32_t R_bar_acc2 = powf(R_bar_acc, 2);
static const float32_t Q_std2 = powf(Q_std, 2);

static const float32_t R_bar_acc_x_T2 = R_bar_acc * T2;
static const float32_t R_bar_x_T2 = R_bar * T2;
static const float32_t Q_std_x_R_bar_acc = Q_std * R_bar_acc;

static const float32_t Q_bias_x_T = Q_bias * T;
static const float32_t Q_bias_x_T3 = Q_bias * T3;
static const float32_t Q_std_x_T4 = Q_bias * T4;
static const float32_t Q_std_x_T2 = Q_std * T2;
static const float32_t Q_std_x_R_bar = Q_std * R_bar;
static const float32_t Q_bias_x_Q_std = Q_bias * Q_std;
static const float32_t Q_std_x_R_acc = Q_std * R_acc;
static const float32_t Q_bias_x_R_bar = Q_bias * R_bar;
static const float32_t Q_bias_x_T2 = Q_bias * T2;
static const float32_t Q_bias_x_T4 = Q_bias * T4;
static const float32_t Q_std_x_T = Q_std * T;

static const float32_t Q_std_x_T3 = Q_std * T3;
static const float32_t R_acc_x_R_bar = R_acc * R_bar;


namespace Attitude {

struct AltitudeState {
	arm_matrix_instance_f32 X_v;
	arm_matrix_instance_f32 P_m;
};

class AltitudeEstimator {

private:
	arm_matrix_instance_f32 A_m;
	arm_matrix_instance_f32 A_m_t;
	arm_matrix_instance_f32 Z_v;
	arm_matrix_instance_f32 Q_m;
	arm_matrix_instance_f32 C_m;
	arm_matrix_instance_f32 C_m_t;
	arm_matrix_instance_f32 R_m;

public:
	AltitudeState altitudeState;
	bool printMatrices;
	arm_status filter(float32_t posZ, float32_t accZ);
	arm_status filter2(float32_t posZ, float32_t accZ);
	char * printSys();
	AltitudeEstimator(void);
};

}// namespace

#ifdef __cplusplus
}
#endif

#endif // _AO_ALTITUDE_ESTIMATOR_H
