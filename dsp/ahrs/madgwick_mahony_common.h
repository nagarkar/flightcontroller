#ifndef _madgwick_mahony_common_h
#define _madgwick_mahony_common_h

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------------------------------
// Variable declaration
typedef struct {
		float c;
		float x;
		float y;
		float z;
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
