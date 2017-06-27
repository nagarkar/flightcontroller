#ifndef __rotation_h
#define __rotation_h
#include "coefficents.h"
class Rotation {
private:
	/*TYPEOF: tx*/ m_tx;
	/*TYPEOF: tx*/ m_ty;
	/*TYPEOF: tx*/ m_tz;
	/*TYPEOF: Rx*/ m_Rx;
	/*TYPEOF: Rx*/ m_Ry;
	/*TYPEOF: Rx*/ m_Rz;
	/*TYPEOF: Rzyx*/ m_Rzyx;
	/*TYPEOF: Rxyz*/ m_Rxyz;
	/*TYPEOF: tolerance*/ tolerance;
	// Symbolic Euler Angle Vector composed from tx, ty, tz
	/*TYPEOF: sym_eu_ang_v*/ sym_eu_ang_v;
public:
	Rotation();
	bool AisalmostB(/*TYPEOF A:*/A, /*TYPEOF A:*/ B);
	bool assertPropertRotationMatrix(/*TYPEOF: X*/ X);
	/*TYPEOF: matrix*/ getDesiredRotation3D(/*TYPEOF: type*/type, /*TYPEOF: t*/t, 
									/*TYPEOF: v*/v, struct TVVector tv_desired);
	/*TYPEOF: matrix*/ getOmegaRotation(/*TYPEOF: type*/type, /*TYPEOF: tx*/tx,
		 									/*TYPEOF: tx*/ty, /*TYPEOF: tx*/tz);
	/*TYPEOF: matrix*/ getVectorRotation(/*TYPEOF: type*/type, /*TYPEOF: tx*/tx,
		 									/*TYPEOF: tx*/ty, /*TYPEOF: tx*/tz);
	/*TYPEOF: ang_vel_body_vec*/ getAngularBodyVelFromEulerRates(/*TYPEOF: type*/type,
									/*TYPEOF euler_rate_vec*/ euler_rate_vec,
									/*TYPEOF eu_ang_v*/ eu_ang_vec);
	/*TYPEOF: ang_vel_body_vec*/ getAngularFixedVelFromEulerRates(/*TYPEOF: type*/type,
									/*TYPEOF euler_rate_vec*/ euler_rate_vec,
									/*TYPEOF eu_ang_v*/ eu_ang_vec);	
};
#endif //__rotation_h