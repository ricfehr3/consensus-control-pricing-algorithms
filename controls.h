// controls.h
// Ric Fehr, March 2017
#ifndef controls_h
#define controls_h

#include "math.h"
/*
 * each agent has controller information 
 * desired position (x,y,z,yaw)
 * Actual position (x,y,z,yaw)
 * current sample position (x,y,z,yaw)
 * previous sample position (x,y,z,yaw)
 * pos error (x,y,z,yaw)
 * vel error (x,y,z,yaw)
 * absisiton error (x,y,z,yaw)
*/ 

class myStates
{
	public:
		double my_sr; // my sample rate
		// Rows. Axis [0]->xy [1]->z 2->yaw
		// Cols. PID gains [0]->kp [1]->kd [2]->ki
		double k[3][3]; 
		struct pos
		{
			/*
			 * get position data in array 
			 * values will be held in this order
			 * vals[0]->x, vals[1]->y, vals[2]->z, vals[3]->yaw
			 ***** Quaternion *****
			 * ori[0]->x, ori[1]->y, ori[2]->z, ori[w]->w
			 ******** Euler *******
			*/ 
			double vals[4]; 
			double ori[4];
			double roll, pitch, yaw_RAD;
		} desPos, actPos, actPos_old, pos_error, vel_error, abs_error, output;
		void get_values(double (*)[3],double*,double*,double*,double); //actPos, //actPos_old
	private:
		void get_pos_error(double*,double*);
		void get_vel_error(double*,double*,double);
		void get_abs_error(double*,double*,double);
		void get_output(double*,double*,double*,double (*)[3],double*);
};

//double tmp_pos_error[4], double tmp_vel_error[4], double tmp_abs_error[4], double tmp_k[3], 
void myStates::get_values(double tmp_k[3][3], double tmp_pos[4], double tmp_pos_old[4], double tmp_des[4], double tmp_my_sr)
{
	for (int i=0; i<4; i++)
	{
		// Get current sample position values
		actPos.vals[i] = tmp_pos[i];
		// Get previous sample position values
		actPos_old.vals[i] = tmp_pos_old[i];
		// Get desired position for drone n
		desPos.vals[i] = tmp_des[i];
	}
	
	// Get actual sample rate in s
	my_sr = tmp_my_sr;
	
	//get_pos_error(actPos.vals, actPos_old.vals);
	//get_vel_error(actPos.vals, actPos_old.vals, my_sr);//desired velocity is "0" for sample based positional controller 
	get_abs_error(actPos.vals, actPos_old.vals, my_sr);
	get_output(pos_error.vals, vel_error.vals, abs_error.vals, tmp_k, actPos.vals);
}

/*
 * End of public function
 * Beginning of private functions
*/

/*
void myStates::get_pos_error(double tmp_pos[4], double tmp_pos_old[4])
{
	for (int i=0; i<4; i++)
	{
		pos_error.vals[i] = desPos.vals[i]-actPos.vals[i];
	}
}

void myStates::get_vel_error(double tmp_pos[4], double tmp_pos_old[4], double my_dt)
{
	for (int i=0; i<4; i++)
	{
		vel_error.vals[i] = -(tmp_pos[i]-tmp_pos_old[i])/my_dt;
	}
}
*/

// Get absition using Simpsons method
void myStates::get_abs_error(double tmp_pos[4], double tmp_pos_old[4], double my_dt)
{
	double a = 0.0;
	double b = my_dt;	
	
	for (int i=0; i<4; i++)
	{
		pos_error.vals[i] = desPos.vals[i]-actPos.vals[i];
		
		vel_error.vals[i] = -(tmp_pos[i]-tmp_pos_old[i])/my_dt;
		
		double s = (tmp_pos_old[i]+tmp_pos_old[i])/2;
		double q = (b-a)/6;
		abs_error.vals[i] = q*(tmp_pos_old[i]+4*s+tmp_pos_old[i]); 
	}
}

void myStates::get_output(double tmp_pos_error[4], double tmp_vel_error[4], double tmp_abs_error[4], double tmp_k[3][3], double tmp_pos[4])
{
	double tmp_yaw = tmp_pos[3]*(3.141592653589793238462/180);
	
	double tmp_x = tmp_k[0][0]*tmp_pos_error[0]+tmp_k[0][1]*tmp_vel_error[0]+tmp_k[0][2]*tmp_abs_error[0]; // x output value
	double tmp_y = tmp_k[0][0]*tmp_pos_error[1]+tmp_k[0][1]*tmp_vel_error[1]+tmp_k[0][2]*tmp_abs_error[1]; // y output value
	output.vals[2] = tmp_k[1][0]*tmp_pos_error[2]+tmp_k[1][1]*tmp_vel_error[2]+tmp_k[1][2]*tmp_abs_error[2]; // z output value
	output.vals[3] = tmp_k[2][0]*tmp_pos_error[3]+tmp_k[2][1]*tmp_vel_error[3]+tmp_k[2][2]*tmp_abs_error[3]; // yaw output value
	
	output.vals[0] = tmp_x*cos(tmp_yaw) + tmp_y*sin(tmp_yaw);
	output.vals[1] = tmp_x*sin(tmp_yaw) - tmp_y*cos(tmp_yaw);
}

/*
 * End of private functions
*/

#endif
