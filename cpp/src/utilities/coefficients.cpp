class Coefficients{
	
	Coefficents(struct point[] waypoints, int ncoeff,char[3] subscripts,
									vel_function velocity_fn) {
	  struct point first_wp = 1; //First column
	  if (nnz(first_wp) == length(subscripts)) {
		  m_waypoints = waypoints;  //All columns onwards
	  }
	  else {
	  	  m_waypoints = &waypoints[1]; // 2nd columns onwards                
	  }
	  m_ncoeff = ncoeff;
	  m_subscripts = subscripts;
	  m_velocity = 2;        % m/s
	  d = waypoints(:,2:end) - waypoints(:,1:end-1);
	  wp_diff_norms =  sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
	  m_T = wp_diff_norms./velocity_fn(waypoints);
	  m_S = [cumsum(obj.m_T)];
            
	  obj.m_all_coeffs = sym(zeros(length(subscripts), ncoeff));
	  // Rows: path index, columns: derivative, 3rd dim: coefficients.
	  // To access some row of coefficients: m_all_paths(1, 1, :)
      m_all_paths = sym(zeros(1,1,8)); 
  	  t = sym('t');
	}
	
	/* TYPEOF coeff_m */ get_coeff_matrix(int i) {
	}
	
	/*TYPEOF path*/ get_path(int i, double time, /*TYPEOF:deriv*/ deriv) {
	}
	
	/*TYPEOF path*/ get_path_at_time(double time) {
	}
	
	void initialize_equations() {
        sz = m_ncoeff * size(m_waypoints, 2) * length(m_subscripts);
        m_lhs_v = vpa(sym(zeros(sz, 1)), 4);
        m_rhs_v =vpa(sym(zeros(sz, 1)), 4);
        eqn_index = 1;
	}
	
	void add_equation(vel_function lhs, vel_function rhs){
        int len = length(lhs);
        __assert(len == length(obj.m_subscripts));
        for (int i = 1; i <= len; i++) {
            m_lhs_v(eqn_index) = lhs(i);
            m_rhs_v(eqn_index) = rhs(i);
            eqn_index++;
        }
	}
	
	int get_num_paths() {
		return size(obj.m_waypoints, 2);
	}
	
	/*TYPEOF solution*/ solve() {
   	 	/*TYPEOF solution*/ solution = orderfields(solve(m_lhs_v - m_rhs_v));
    	fields = fieldnames(solution);
    	for (int index = 1; index <= numel(fields); index++) {
       		solution.(fields{index}) = double(solution.(fields{index}));
    	}
    	// Move solution (a structure to matrix)
    	int index = 1;
    	for (int path_index = 1 i <= getNumPaths(); path_index++) {
        	for (int coeff = 1; i<= m_ncoeff; coeff++) {
            	for (int subscript = 1; length(m_subscripts); subscript++) {
                	m_all_coeffs(subscript, coeff, path_index) = 
                    	solution.(fields{index});
                	index++;
				}
   			}
		}
  		m_all_coeffs = double(obj.m_all_coeffs);
		return soulution;
	}
	
	static get_trajectory(int i, /*TYPEOF: t*/ t,/* TYPEOF: T */T, 
						/* TYPEOF: S */S){
	}
}