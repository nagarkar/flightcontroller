#include "coefnts.h"
#include <math.h>
#include <float.h>

coefficientsStruct coeffStruct;

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

static uint8_t nearlyEqual(float32_t a, float32_t b, float32_t epsilon) {

	float32_t absA = 0.0;
	float32_t absB = 0.0;
	float32_t diff = 0.0;
	float32_t absDiff = 0.0;

	arm_abs_f32 (&absA, &a, 1);
	arm_abs_f32 (&absB, &b, 1);
	diff = absA - absB;
	arm_abs_f32 (&absDiff, &diff, 1);

	if (a == b) { // shortcut, handles infinities
		return TRUE;

	} else if (a == 0 || b == 0 || absDiff < FLT_MIN ) {
		// a or b is zero or both are extremely close to it
		// relative error is less meaningful here
		return absDiff < (epsilon * FLT_MIN);
	} else { // use relative error
			return absDiff / fmin((absA + absB), FLT_MAX) < epsilon;
	}
}

 void initCoeff(float32_t *waypointsRaw, uint8_t nCoeff, unsigned char *subscripts, const uint8_t nSubscripts, float32_t *constVel) {

	coeffStruct.mNcoeff = nCoeff;
	coeffStruct.mVelocity = 2;
	coeffStruct.mSubscripts = subscripts;

	uint32_t srcRows, srcColumns;   /* Temporary variables */

	float32_t firstWayPoints[DIM];

	float32_t matWayPointsBuffer[15];
	arm_matrix_instance_f32 matWayPoint;				// Waypoint matrix

	arm_copy_f32(waypointsRaw, firstWayPoints, DIM);

	uint8_t nnzCount = 0;						// Number of non-zero count

	for(uint8_t i = 0; i < DIM; i++)
		if(!(nearlyEqual(firstWayPoints[i], 0, FLT_MIN)))	nnzCount++;

	if(nnzCount < DIM) {

		// Subtract the initial Row from the waypoints data
		arm_copy_f32(&waypointsRaw[DIM], matWayPointsBuffer, ((MATRIX_DIM_WAYPOINTS_ROW - 1) * MATRIX_DIM_WAYPOINTS_COL));

		 // Now there are 4 rows
		 srcRows = 4;
		 srcColumns = 3;
		} else {
		 nnzCount = 0;
		 arm_copy_f32(waypointsRaw, matWayPointsBuffer, (MATRIX_DIM_WAYPOINTS_ROW  * MATRIX_DIM_WAYPOINTS_COL));
		 srcRows = 5;
		 srcColumns = 3;
	}

	//---- Create waypoints matrix, coeffStruct.mWaypoints is wquivalent to obj.m_waypoints in matlab
	arm_mat_init_f32(&matWayPoint, srcRows, srcColumns, matWayPointsBuffer);

	// Copy the same matrix
	arm_copy_f32(matWayPointsBuffer, &coeffStruct.mWaypointsBuffr, (srcRows * srcColumns));
	arm_mat_init_f32(&coeffStruct.mWaypoints, srcRows, srcColumns, &coeffStruct.mWaypointsBuffr);

	//arm_mat_trans_f32(&matWayPoint, &coeffStruct.mWaypoints); // Transpose might not required as in matlab


	// ----------------- Calculate Difference ---------------------
	// Subtracter Raw Matrix
	srcRows = 4;
	srcColumns = 3;
	float32_t matWayPointSubtracterRaw[srcRows * srcColumns];
	arm_matrix_instance_f32 matWayPointSubtracter;
	arm_copy_f32(&waypointsRaw[3], matWayPointSubtracterRaw, (srcRows * srcColumns));
	arm_mat_init_f32(&matWayPointSubtracter, srcRows, srcColumns, matWayPointSubtracterRaw);

	// Subtracter Transpose Matrix
	srcRows = 3;
	srcColumns = 4;
	float32_t matWayPointSubtracterTRaw[srcRows * srcColumns];
	arm_matrix_instance_f32 matWayPointSubtracterT;
	arm_mat_init_f32(&matWayPointSubtracterT, srcRows, srcColumns, matWayPointSubtracterTRaw);

	// Calculate Transpose for the Subtracter
	arm_mat_trans_f32(&matWayPointSubtracter, &matWayPointSubtracterT);

	// Subtrahend Raw Matrix
	srcRows = 4;
	srcColumns = 3;
	float32_t matWayPointSubtrahendRaw[srcRows * srcColumns];
	arm_matrix_instance_f32 matWayPointSubtrahend;
	arm_copy_f32(waypointsRaw, matWayPointSubtrahendRaw, (srcRows * srcColumns));
	arm_mat_init_f32(&matWayPointSubtrahend, srcRows, srcColumns, matWayPointSubtrahendRaw);

	// Subtrahend Transpose Matrix
	srcRows = 3;
	srcColumns = 4;
	float32_t matWayPointSubtrahendTRaw[srcRows * srcColumns];
	arm_matrix_instance_f32 matWayPointSubtrahendT;
	arm_mat_init_f32(&matWayPointSubtrahendT, srcRows, srcColumns, matWayPointSubtrahendTRaw);

	// Calculate Transpose for the Subtrahend
	arm_mat_trans_f32(&matWayPointSubtrahend, &matWayPointSubtrahendT);

	// Calculate Difference
	srcRows = 3;
	srcColumns = 4;
	float32_t matDiffBuffr[srcRows * srcColumns];
	arm_matrix_instance_f32 matDiff;
	arm_mat_init_f32(&matDiff, srcRows, srcColumns, matDiffBuffr);

	arm_mat_sub_f32(&matWayPointSubtracterT, &matWayPointSubtrahendT, &matDiff);


	// Calculation of wp_diff_norms
	srcRows = 4;
	srcColumns = 3;
	float32_t matDiffBuffrT[srcRows * srcColumns];
	arm_matrix_instance_f32 matDiffT;
	arm_mat_init_f32(&matDiffT, srcRows, srcColumns, matDiffBuffrT);

	arm_mat_trans_f32(&matDiff, &matDiffT);

	// A vector for storing wp_diff_norm
	float32_t wpDiffNorms[srcRows];

	for(uint8_t i = 0, j = 0; i < (srcRows * srcColumns); i += srcColumns, j++) {
		arm_dot_prod_f32(&matDiffBuffrT[i], &matDiffBuffrT[i], srcColumns, &wpDiffNorms[j]);
		arm_sqrt_f32(wpDiffNorms[j], &wpDiffNorms[j]);
	}

	// Calculation of m_T and m_S
	uint8_t sizeOfVels = 4;
	float32_t constVelTmp[sizeOfVels];

	for(uint8_t i = 0; i < sizeOfVels; i++) {
		constVelTmp[i] = 1 / constVel[i];		// Calculation of reciprocal
	}

	arm_mult_f32(wpDiffNorms, constVelTmp, coeffStruct.mT, sizeOfVels);

	for(uint8_t i = 0; i < 4; i++) {
		int8_t j = i;
		while(j >= 0) {
			coeffStruct.mS[i] += coeffStruct.mT[j];
			j--;
		}
	}

	// Initialize obj.m_all_coeffs
	srcRows = nSubscripts;
	srcColumns = coeffStruct.mNcoeff;
	float32_t mAllCoeffBuffr[srcRows * srcColumns];

	arm_fill_f32(0.0, mAllCoeffBuffr, (srcRows * srcColumns));
	coeffStruct.mAllCoeffsBuffr = mAllCoeffBuffr;
	arm_mat_init_f32(&coeffStruct.mAllCoeffs, srcRows, srcColumns, coeffStruct.mAllCoeffsBuffr);


	// Create a column matrix for obj.m_all_paths
	srcRows = 8;
	srcColumns = 1;
	float32_t mAllPathsBuffr[srcRows * srcColumns];

	arm_fill_f32(0.0, mAllPathsBuffr, (srcRows * srcColumns));
	coeffStruct.mAllPathsBuffr = mAllPathsBuffr;
	arm_mat_init_f32(&coeffStruct.mAllPaths, srcRows, srcColumns, coeffStruct.mAllPathsBuffr);

	coeffStruct.t = 't';
	return;
}


void getWayPoint(float32_t *wayPoint, uint8_t index) {
	// index starts with 0
	if(index <= 3) {
		index *= 3;
		arm_copy_f32(&(coeffStruct.mWaypointsBuffr[index]), wayPoint, DIM);
//		arm_copy_f32((coeffStruct.mWaypointsBuffr + index), wayPoint, DIM);
		return;
	} else {
		return;
	}
}

