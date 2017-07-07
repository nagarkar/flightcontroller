#ifndef _madgwick_mahony_common_h
#define _madgwick_mahony_common_h

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------------------------------
// Variable declaration
typedef struct {
		float q0;
		float q1;
		float q2;
		float q3;
} Q_cxyz;


//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif

//=====================================================================================================
// End of file
//=====================================================================================================
