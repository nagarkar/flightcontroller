#ifndef __state_vector_h
#define __state_vector_h

#pragma pack(push, 1)
	struct state_vector {
		double x_pos = 0; // x position
		double y_pos = 0; // y position
		double z_pos = 0; // z position
		double x_dot = 0; // x dot
		double y_dot = 0; // y dot
		double z_dot = 0; // z dot
		double w_quat = 0; // qw
		double x_quat = 0; // qx
		double y_quat = 0; // qy
		double z_quat = 0; // qz
		double p = 0; // p
		double q = 0; // q
		double r = 0; // r
	}
#pragma pack(pop)
	
class StateVector {
private:
	struct state_vector m_state;
public:
	struct state_vector* get_state() {
		return &m_state;
	}
	
	void init_state_vector(double[3] start_pos, double yaw);
	
}
#endif //__state_vector_h