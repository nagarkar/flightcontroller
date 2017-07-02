#ifndef __coefficents_h
#define __coefficents_h

struct TVVector{
	T;
	S;
};

typedef double (*vel_function)(struct point[], int num_points);

class Coefficents {
private:
	/* TYPEOF: T */ m_T;
	/* TYPEOF: S */ m_S;
	struct point[] m_waypoints;
	uint32_t m_ncoeff;
	char[] m_subscripts;
	// m/s
	int m_velocity = 2;
	vel_function m_lhs_v;
	vel_function m_rhs_v;
	uint32_t eqn_index;
	int[][] m_all_coeff;
	m_all_paths;
	t;
public:
	Coefficents(struct point[] waypoints, int ncoeff,char[] subscripts,
									vel_function velocity_fn);

	struct point getWaypoint(int index) 
		{return m_waypoints[index];}

	struct TVVector get_TV_vector() {
		return struct TVVector{T = m_T, S = m_S};
	}

	/* TYPEOF coeff_m */ get_coeff_matrix(int i);

	/*TYPEOF path*/ get_path(int i, double time, /*TYPEOF:deriv*/ deriv);

	/*TYPEOF path*/ get_path_at_time(double time);

	void initialize_equations();

	void add_equation(vel_function lhs, vel_function rhs);

	int get_num_paths();

	/*TYPEOF solution*/ solve();

	static get_trajectory(int i, /*TYPEOF: t*/ t,/* TYPEOF: T */T, 
						/* TYPEOF: S */S);
	
};
#endif //__coefficents_h