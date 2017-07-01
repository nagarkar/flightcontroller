#ifndef __utils_h
#define __utils_h

struct system_params{
	double mass;
	double gravity;
	double[3][3] I;
	double[3][3] invI;
	double arm_length;
	double minF;
	double maxF;
};

struct qd{
	double[3] pos;
	double[3] vel;
};

// typdeef for controller handle function pointer
typedef struct *controller_ret (*ControllerHandle)(double t, struct state_vector *current state,
								struct state_vector desired_state*, struct system_params *params);

typedef struct *state_vector (*TrajectoryHandle)(double t, struct state_vector *current_state);												

static struct *system_params sys_params();

static struct stateToQd(struct state_vector x);

static double** RPYtoRot_ZXY(double phi, double theta, double psi);

static double** rotToQuat(double[3][3] r);

static double** quatToRot(double[4][4] q);

static struct *state_vector quadEOM(double time, struct state_vector *s,
								ControllerHandle controller_handle,
								TrajectoryHandle trajHandle);

static struct *state_vector quadEOM_readonly(double t, struct state_vector *s, double* F, double[3] M
											struct system_params *params);

static double** quad_pos(double[3] pos, double[3][3] rot, double L, double H=0.05);				

// termination criteria, including position, velocity and time
static int terminate_check(struct state_vector x, double time, double[3] stop,
						double pos_tol, double vel_tol, double time_tol);


#endif //__utils_h
