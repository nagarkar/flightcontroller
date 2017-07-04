#ifndef _madgwick_mahony_common_h
#define _madgwick_mahony_common_h

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------------------------------
// Variable declaration
typedef struct {
		float q0;// = 1.0f;
		float q1;// = 0.0f;
		float q2;// = 0.0f;
		float q3;// = 0.0f;
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
