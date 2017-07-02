#ifndef __state_vector_h
#define __state_vector_h

#pragma pack(push, 1)
	struct state_vector {
		double x_pos; // x position
		double y_pos; // y position
		double z_pos; // z position
		double x_dot; // x dot
		double y_dot; // y dot
		double z_dot; // z dot
		double w_quat; // qw
		double x_quat; // qx
		double y_quat; // qy
		double z_quat; // qz
		double p; // p
		double q; // q
		double r; // r
	}
#pragma pack(pop)
	
class StateVector {
private:
	struct state_vector m_state;
public:
	void StateVector();
	
	struct state_vector* get_state() {
		return &m_state;
	}
	
	void init_state_vector(double[3] start_pos, double yaw);
	
}
#endif //__state_vector_h