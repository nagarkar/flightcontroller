#ifndef __snap_trajectory_generator_h
#define __snap_trajectory_generator_h
#include "coefficients.h"
struct generator_solution{
	Coeffectients coeffs;
	/*TYPEOF: Soultion*/ solution;
}
class SnapTrajectoryGenerator{
public:
	static struct generator_solution generateTrajectorySolution(point[] waypoints,
		 				char[] subscripts, /*TYPEOF: veolocity_fn*/ velocity_fn);
};
#endif //__snap_trajectory_generator_h 