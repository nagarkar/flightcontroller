/*
 * coefnts.h
 *
 *  Created on: Jun 26, 2017
 *      Author: LOMAS
 */

#ifndef COEFNTS_H_
#define COEFNTS_H_

#include "arm_math.h"

#define 	DIM				3
#define		VECT_WIDTH		4

#define 	I		0
#define 	J		1
#define 	K		2

#define MATRIX_DIM_WAYPOINTS_ROW	5
#define MATRIX_DIM_WAYPOINTS_COL	3

#define TRUE	1
#define FALSE	0

#define EPSILON 	0.00000001F

#define ZERO	0

typedef struct coeffProperties {
	float32_t mT[VECT_WIDTH];
	float32_t mS[VECT_WIDTH];

	arm_matrix_instance_f32 mWaypoints;
	float32_t mWaypointsBuffr[15];

	uint16_t mNcoeff;

	unsigned char *mSubscripts[DIM];

	float32_t mVelocity;

	float32_t mLhsV;
	float32_t mRhsV;

	uint16_t eqnIndex;
	float32_t mAllCoeffs;
	float32_t mAllPaths;
	float32_t t;
} coefficientsStruct;

void initCoeff(float32_t *waypointsRaw, uint8_t ncoeff, unsigned char *subscripts, float32_t *const_vel);

#endif /* COEFNTS_H_ */
