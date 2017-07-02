#ifndef __controller_h
#define __controller_h

struct controller_ret {
	Double F;
	Double[3] M;
};

static struct *controller_ret controller(double t, struct state_vector *state,
					struct state_vector *des_state, struct system_params params);

#endif //__controller_h