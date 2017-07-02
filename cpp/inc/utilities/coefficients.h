#ifndef __coefficents_h
#define __coefficents_h

struct TVVector{
	T;
	S;
};

class Coefficents {
private:
	/* TYPEOF: T */ m_T;
	/* TYPEOF: S */ m_S;
	struct point[] m_waypoints;
	int m_ncoeff;
	char[] m_subscripts;
	// m/s
	int m_velocity = 2;
	/* TYPEOF: V */ m_lhs_v;
	/* TYPEOF: V */ m_rhs_v;
	int eqn_index;
	int[][] m_all_coeff;
	m_all_paths;
	t;
public:
	Coefficents(struct point[] waypoints, int ncoeff,char[] subscripts,
									/*TYPEOF: veolocity_fn*/ velocity_fn);
	WayPoint wp getWaypoint(int index) 
		{return m_waypoints[index];}
	struct TVVector get_TV_vector() {
		return struct TVVector{T = m_T, S = m_S};
	}
	get_coeff_matrix(int i);
	get_path(int i, double time, /*TYPEOF:deriv*/ deriv);
	get_path_at_time(double time);
	initialize_equations();
	add_equation(/*typeof: eq*/lhs, /*typeof: eq*/rhs);
	get_num_paths();
	solve(bool precision_not_used);
	static get_trajectory(int i, /*TYPEOF: t*/ t,/* TYPEOF: T */T, 
						/* TYPEOF: S */S);
};
#endif //__coefficents_h