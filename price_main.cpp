// Ric Fehr, March 2017

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <fstream>
#include <sstream>

#include "controls.h"
#include "consensus.h"
#include "adjacency.h"

#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;

#define PI 3.141592653589793238462

// Joystick Buttons Correspond to numbers on logitech dual action controller
int button[7];
// number of agents
const int n = 4;

// number of active agents
int num_fly = 0;

// construct drone classes for my n word
myStates drone[n]; // Declare n number of drone states


Mat q_act[n] = Mat(1, 3, CV_64FC1);
		
Mat p_act[n] = Mat(1, 3, CV_64FC1);

Mat u_a[n] = Mat(1, 3, CV_64FC1);
Mat u_g[n] = Mat(1, 3, CV_64FC1);
Mat u[n] = Mat(1, 3, CV_64FC1);
Mat pkk[n] = Mat(1, 3, CV_64FC1);

// Desired Position
Mat Qd[n] = Mat(1, 3, CV_64FC1);

struct states
{
	// Translational position in meters
	double pos_x, pos_y, pos_z;
	// Orientation position in quaternions
	double ori_x, ori_y, ori_z, ori_w;
	// Orientation position in euler angles
	double roll, pitch, yaw, yaw_RAD;
	// Values for control class header thing
	double gains[3];
	double my_pos[4];
	double my_pos_old[4];
	double my_desired[4];
} bebop[n];

// Get joystick controller info
void getJoy(const sensor_msgs::Joy::ConstPtr& button_)
{
	/*
	button[0] = button->buttons[0];
	button[1] = button->buttons[1];
	button[2] = button->buttons[2];
	//lb, rb, lt, rt
	button[3] = button->buttons[4];
	button[4] = button->buttons[5];
	button[5] = button->buttons[6];
	button[6] = button->buttons[7];
	*/
	for (int i=0; i<7; i++)
	{
		button[i] = button_->buttons[i];	
	}
}

// Get position info from mocap for bebop[n]
void getPosBebop1(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	bebop[0].pos_x = pos->pose.position.x;
	bebop[0].pos_y = pos->pose.position.y;
	bebop[0].pos_z = pos->pose.position.z;
	bebop[0].ori_x = pos->pose.orientation.x;
	bebop[0].ori_y = pos->pose.orientation.y;
	bebop[0].ori_z = pos->pose.orientation.z;
	bebop[0].ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop[0].roll, bebop[0].pitch, bebop[0].yaw_RAD);
	bebop[0].yaw = bebop[0].yaw_RAD*(180/PI);
}

void getPosBebop2(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	bebop[1].pos_x = pos->pose.position.x;
	bebop[1].pos_y = pos->pose.position.y;
	bebop[1].pos_z = pos->pose.position.z;
	bebop[1].ori_x = pos->pose.orientation.x;
	bebop[1].ori_y = pos->pose.orientation.y;
	bebop[1].ori_z = pos->pose.orientation.z;
	bebop[1].ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop[1].roll, bebop[1].pitch, bebop[1].yaw_RAD);
	bebop[1].yaw = bebop[1].yaw_RAD*(180/PI);
}

void getPosBebop3(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	bebop[2].pos_x = pos->pose.position.x;
	bebop[2].pos_y = pos->pose.position.y;
	bebop[2].pos_z = pos->pose.position.z;
	bebop[2].ori_x = pos->pose.orientation.x;
	bebop[2].ori_y = pos->pose.orientation.y;
	bebop[2].ori_z = pos->pose.orientation.z;
	bebop[2].ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop[2].roll, bebop[2].pitch, bebop[2].yaw_RAD);
	bebop[2].yaw = bebop[2].yaw_RAD*(180/PI);
}

void getPosBebop4(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	bebop[3].pos_x = pos->pose.position.x;
	bebop[3].pos_y = pos->pose.position.y;
	bebop[3].pos_z = pos->pose.position.z;
	bebop[3].ori_x = pos->pose.orientation.x;
	bebop[3].ori_y = pos->pose.orientation.y;
	bebop[3].ori_z = pos->pose.orientation.z;
	bebop[3].ori_w = pos->pose.orientation.w;

	tf::Quaternion q(pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop[3].roll, bebop[3].pitch, bebop[3].yaw_RAD);
	bebop[3].yaw = bebop[3].yaw_RAD*(180/PI);
}

// Convert posiiton to array for controller
void getMyPos()
{
	for (int i=0; i<n; i++)
	{
		drone[i].actPos.vals[0] = bebop[i].pos_x;
		drone[i].actPos.vals[1] = bebop[i].pos_y;
		drone[i].actPos.vals[2] = bebop[i].pos_z;
		drone[i].actPos.vals[3] = bebop[i].yaw;
	}
}

Mat adjacency = Mat(n, n, CV_64FC1);;

void getPrice()
{
	// get price and communication quality
	double price_tmp[4][4];// =	{{1, 3, 4, 2},
							//	{7, 1, 4, 2},
							//	{2, 5, 1, 3},
							//	{2, 2, 9, 1}};
							
	for (int i=0; i<4; i++)
	{
		for (int j=0; j<4; j++)
		{
			if (i==j)
			{
				price_tmp[i][j] = 1; // diagonal has no self loops but assumes perfect communication on self loops                  
			}
			if (i>j)
			{
				price_tmp[i][j] = rand() % 10 + 1; // generate random number between 1 and 10
				price_tmp[j][i] = price_tmp[i][j];
				//printf ("i=%d j=%d\n",i,j);
				//price_tmp[i][j] = sqrt((drone[i].actPos.vals[0]-drone[j].actPos.vals[0])*(drone[i].actPos.vals[0]-drone[j].actPos.vals[0])+(drone[i].actPos.vals[1]-drone[j].actPos.vals[1])*(drone[i].actPos.vals[1]-drone[j].actPos.vals[1]));
			}
		}
	}
		
	int boi = ((n-1)*(n))/2;
		
	double p_array[6];
	int pos_array = 0;
		
	for (int i=0; i<4; i++)
	{
		for (int j=0; j<4; j++)
		{
			if(i>j)
			{
				p_array[pos_array] = price_tmp[i][j];
					
				pos_array = pos_array+1;
			}
		}
	}
		
	//cout << pos_array << "\n";
		
	//cout << p_array << "\n";
		
	double max_p = getPMax(p_array, n);
		
	//cout << max_p << "\n";
						
	Mat price = Mat(n, n, CV_64FC1, price_tmp);
		
	//Mat adjacency = getAdj(price, max_p, n);
	adjacency = getAdj(price, max_p, n);
		
	//cout << adjacency << "\n";
}

int main(int argc, char** argv)
{
	// Initialize desired position
	double qd[n][3];

	for (int i=0; i<n; i++)
	{
		for (int j=0; j<3; j++)
		{
			qd[i][j] = 0;
		}
	}
	
	// initialize time 
	float myTime = 0.0;
	
	// Maximum speed of each drone in m/s ir rad/s
	float speed = 0.69;
	
	// Initialize ros
	ros::init(argc, argv, "bebop_control"); 

	// Node handle used
	ros::NodeHandle nh_; 
	
	// Initialized publishers
	// For bebop1
	ros::Publisher takeoff_pub_drone[n];
	ros::Publisher land_pub_drone[n];	
	ros::Publisher cmd_vel_pub_drone[n];
	
	ros::Subscriber subPoseActual[n];
	
	// declare n number of strings to subscribe to for pos data
	stringstream pos_string[n];
	stringstream takeoff_str[n];
	stringstream land_str[n];
	stringstream cmd_str[n];
	
	// create subscription strings
	for (int i=0; i<n; i++)
	{
		// subscriber string topics
		pos_string[i] << "/UAV_IP" << (i+5) << "/pose";
		// publisher string topics
		takeoff_str[i] << "/bebop_IP" << (i+5) << "/takeoff";
		land_str[i] << "/bebop_IP" << (i+5) << "/land";
		cmd_str[i] << "/bebop_IP" << (i+5) << "/cmd_vel";
		// publisher creation
		takeoff_pub_drone[i] = nh_.advertise<std_msgs::Empty>(takeoff_str[i].str(), 1000);
		land_pub_drone[i] = nh_.advertise<std_msgs::Empty>(land_str[i].str(), 1000);
		cmd_vel_pub_drone[i] = nh_.advertise<geometry_msgs::Twist>(cmd_str[i].str(), 1000);
	}
	
	// Subscribe to joy node
	ros::Subscriber joy_controller = nh_.subscribe("/joy", 1000, getJoy);
	
	subPoseActual[0] = nh_.subscribe(pos_string[0].str(), 1000, getPosBebop1);
	subPoseActual[1] = nh_.subscribe(pos_string[1].str(), 1000, getPosBebop2);
	subPoseActual[2] = nh_.subscribe(pos_string[2].str(), 1000, getPosBebop3);
	subPoseActual[3] = nh_.subscribe(pos_string[3].str(), 1000, getPosBebop4);
	
	
	// Velocity data to be sent to individual drones
	geometry_msgs::Twist cmd_vel_bebop[n];
	
	// std_msgs::Empty "takeoff" & "land"
	std_msgs::Empty msg_takeoff, msg_land; 

	// PID gains
	double k[3][3];
	k[0][0] = 0.85; //kp_xy
	k[0][1] = 0.625; //kd_xy
	k[0][2] = 0.6; //ki_xy
	k[1][0] = 1.2; //kp_z
	k[1][1] = 0.1; //kd_z
	k[1][2] = 0.4; //ki_z
	k[2][0] = 0.05; //kp_yaw
	k[2][1] = 0.01; //kd_yaw
	k[2][2] = 0.01; //ki_yaw
	
	// Consensus Gains
	double c1_g = 30.0;//5;
	double c2_g = 0.0;
	double c1_a = 1.25;//2;
	double c2_a = 0.0;//1.0;//0.5;
	// Consensus Parameters
	double d = 2.0; // 7 in simulation
	double r = 1.25*d;//1.2*d;
	double r_prime = 0.6*r;
	double kappa = 1.2;
	double d_p = 0.6*d;
	double r_p = 1.2*d_p;
	double epsilon = 0.1; // For sigma_norm //1
	double aa = 5; 
	double bb = aa; // For phi
	double h_a = 0.2; // For phi_alpha
	double h_b = 0.9; // For phi_beta
	double N = 4; // Number of agents
	double m = 3; // m=2 for 2-D and m=3 for 3-D
	
	// Sampling rate in Hz and s
	ros::Rate loop_rate(100);
	double Ts = 0.01;
	
	double pd[3] = {0,0,0}; // Target Speed  
	Mat Pd = Mat(1, 3, CV_64FC1, pd);
			
		/****************************************************************************************************
		 ************************************* O U T P U T  F I L E S ***************************************
		 ****************************************************************************************************/	

	ofstream bebop1Position_x;
	bebop1Position_x.open ("bebop1Position_x.txt");
	ofstream bebop1Position_y;
	bebop1Position_y.open ("bebop1Position_y.txt");
	ofstream bebop1Position_z;
	bebop1Position_z.open ("bebop1Position_z.txt");
	
	ofstream bebop2Position_x;
	bebop2Position_x.open ("bebop2Position_x.txt");
	ofstream bebop2Position_y;
	bebop2Position_y.open ("bebop2Position_y.txt");
	ofstream bebop2Position_z;
	bebop2Position_z.open ("bebop2Position_z.txt");
	
	ofstream bebop3Position_x;
	bebop3Position_x.open ("bebop3Position_x.txt");
	ofstream bebop3Position_y;
	bebop3Position_y.open ("bebop3Position_y.txt");
	ofstream bebop3Position_z;
	bebop3Position_z.open ("bebop3Position_z.txt");

	ofstream bebop4Position_x;
	bebop4Position_x.open ("bebop4Position_x.txt");
	ofstream bebop4Position_y;
	bebop4Position_y.open ("bebop4Position_y.txt");
	ofstream bebop4Position_z;
	bebop4Position_z.open ("bebop4Position_z.txt");
	
	ofstream price_shit;
	price_shit.open ("fucking_ass_tits.txt");


		/****************************************************************************************************
		 **************************************** M A I N  L O O P ******************************************
		 ****************************************************************************************************/	
	short testyb = 0;
	while(nh_.ok())
	{
		//ros::Time lasttime=ros::Time::now();
		
		getMyPos();
		
		float getchTime = fmod(myTime, 5);
		
		//printf("myTime|%f|\n",myTime);
		//printf("modulo|%f|\n",goat);
		if (getchTime > 0 && getchTime < 0.01)
		{
			testyb++;
			printf("Random Price Iteration|%d|\n", testyb);
			getPrice();
		}
		
		for (int i=0; i<4; i++)
		{
			drone[i].get_values(k, drone[i].actPos.vals, drone[i].actPos_old.vals, qd[i], Ts);
		}
		
		/****************************************************************************************************
		 **************************************** I N P U T  B O I ******************************************
		 ****************************************************************************************************/		
	 
		if (button[0]==1)
		{
			num_fly = 0;
			
			printf("u pressed da button 0\n");
			
			// For bebop1
			takeoff_pub_drone[0].publish(msg_takeoff);
			// For bebop2
			land_pub_drone[1].publish(msg_takeoff);
			// For bebop3
			land_pub_drone[2].publish(msg_takeoff);
			// For bebop4
			land_pub_drone[3].publish(msg_takeoff);
		}
		if (button[1]==1)
		{
			num_fly = 1;
			// For bebop1
			takeoff_pub_drone[0].publish(msg_takeoff);
			// For bebop2
			takeoff_pub_drone[1].publish(msg_takeoff);
			// For bebop3
			land_pub_drone[2].publish(msg_takeoff);
			// For bebop4
			land_pub_drone[3].publish(msg_takeoff);
		}
		if (button[2]==1)
		{
			num_fly = 2;
			
			// For bebop1
			takeoff_pub_drone[0].publish(msg_takeoff);
			// For bebop2
			takeoff_pub_drone[1].publish(msg_takeoff);
			// For bebop3
			takeoff_pub_drone[2].publish(msg_takeoff);
			// For bebop4
			land_pub_drone[3].publish(msg_takeoff);
		}
		if (button[3]==1)
		{
			num_fly = 3;
			
			// For bebop1
			takeoff_pub_drone[0].publish(msg_takeoff);
			// For bebop2
			takeoff_pub_drone[1].publish(msg_takeoff);
			// For bebop3
			takeoff_pub_drone[2].publish(msg_takeoff);
			// For bebop4
			takeoff_pub_drone[3].publish(msg_takeoff);
		}
		if (button[6]==1)
		{
			num_fly = -1;
			// For bebop1
			land_pub_drone[0].publish(msg_takeoff);
			// For bebop2
			land_pub_drone[1].publish(msg_takeoff);
			// For bebop3
			land_pub_drone[2].publish(msg_takeoff);
			// For bebop4
			land_pub_drone[3].publish(msg_takeoff);		
			printf("land\n");
		}
	 
		/****************************************************************************************************
		 ****************************** C O N S E N S U S  C O N T R O L L E R ******************************
		 ****************************************************************************************************/
		
		// Position Matrix
		double q[4][3] = {{bebop[0].pos_x, bebop[0].pos_y, bebop[0].pos_z},
						  {bebop[1].pos_x, bebop[1].pos_y, bebop[1].pos_z},
						  {bebop[2].pos_x, bebop[2].pos_y, bebop[2].pos_z},
						  {bebop[3].pos_x, bebop[3].pos_y, bebop[3].pos_z}};
		Mat Q = Mat(4, 3, CV_64FC1, q);	
		
		// Velocity Matrix
		double p[4][3] = {{drone[0].vel_error.vals[0], drone[0].vel_error.vals[1], drone[0].vel_error.vals[2]},
						  {drone[1].vel_error.vals[0], drone[1].vel_error.vals[1], drone[1].vel_error.vals[2]},
						  {drone[2].vel_error.vals[0], drone[2].vel_error.vals[1], drone[2].vel_error.vals[2]},
						  {drone[3].vel_error.vals[0], drone[3].vel_error.vals[1], drone[3].vel_error.vals[2]}}; 
		Mat P = Mat(4, 3, CV_64FC1, p);	

		// Get individual position and velocity arrays for computation
		for (int i=0; i<4; i++)
		{
			for (int j=0; j<4; j++)
			{
				q_act[i].at<double>(j) = Q.at<double>(i,j);
				p_act[i].at<double>(j) = P.at<double>(i,j);
			} 
		}
		
		if (num_fly == 0)
		{
			qd[0][0] = 0; // Target Position
			qd[0][1] = 0;
			qd[0][2] = 1;
		
			Qd[0] = Mat(1, 3, CV_64FC1, qd[0]);
		}

		if (num_fly == 1)
		{
			qd[0][0] = -1; // Target Position
			qd[0][1] = 0;
			qd[0][2] = 1;
			
			qd[1][0] = 1; // Target Position
			qd[1][1] = 0;
			qd[1][2] = 1;

			Qd[0] = Mat(1, 3, CV_64FC1, qd[0]);
	
			Qd[1] = Mat(1, 3, CV_64FC1, qd[0]);
		}

		if (num_fly == 2)
		{
			qd[0][0] = 0; // Target Position
			qd[0][1] = 1;
			qd[0][2] = 1;
			
			qd[1][0] = sin(PI/3); // Target Position
			qd[1][1] = -cos(PI/3);
			qd[1][2] = 1;
			
			qd[2][0] = -sin(PI/3); // Target Position
			qd[2][1] = -cos(PI/3);
			qd[2][2] = 1;
			
			
			Qd[0] = Mat(1, 3, CV_64FC1, qd[0]);
	
			Qd[1] = Mat(1, 3, CV_64FC1, qd[1]);
	
			Qd[2] = Mat(1, 3, CV_64FC1, qd[2]);
			// Q t 3.14 goes here
		}
		
		if (num_fly == 3)
		{
			qd[0][0] = -1.2; // Target Position
			qd[0][1] = 1.2;
			qd[0][2] = 1;
			
			qd[1][0] = 1.2; // Target Position
			qd[1][1] = -1.2;
			qd[1][2] = 1;
			
			qd[2][0] = -1.2; // Target Position
			qd[2][1] = -1.2;
			qd[2][2] = 1;
			
			qd[3][0] = 1.2; // Target Position
			qd[3][1] = 1.2;
			qd[3][2] = 1;
			
			Qd[0] = Mat(1, 3, CV_64FC1, qd[0]);
			
			Qd[1] = Mat(1, 3, CV_64FC1, qd[1]);
			
			Qd[2] = Mat(1, 3, CV_64FC1, qd[2]);
			// Q t 3.14 goes here
			Qd[3] = Mat(1, 3, CV_64FC1, qd[3]);
		}
		
		for (int i=0; i<4; i++)
		{
			if (i <= num_fly)
			{
				// Consensus Controller Output u_{\alpha}
				u_a[i] = fi_alpha(c1_a, c2_a, 1, Q, P, r, d, h_a, aa, bb, epsilon, adjacency);
				u_a[i] = fi_alpha(c1_a, c2_a, 1, Q, P, r, d, h_a, aa, bb, epsilon, adjacency);
	
				// Navigational Feedback Controller Output u_{\gamma}
				u_g[i] = fi_gamma(q_act[i], p_act[i], Qd[i], Pd, c1_g, c2_g);
			}
			if (i > num_fly)
			{
				u_a[i] = 0;
				u_g[i] = 0;
			}
			// Total output matrix for 
			u[i] = u_a[i] + u_g[i];
			
			// Velocity output for Controller
			pkk[i] = Ts*u[i];
		}
		
		
		/****************************************************************************************************
		 ************************************* S A T U R A T I O N ******************************************
		 ****************************************************************************************************/
		
		for (int i=0; i<n; i++)
		{
			for (int j=0; j<4; j++)
			{
				// set bebop output values to PID + u[i]
				
				// comment this if no need for consensus controller
				
				// TODO check for axis issues with this bad boi
				
				drone[i].output.vals[j] = drone[i].output.vals[j] + pkk[i].at<double>(j);
				
				
				// saturate output speed to j
				if (drone[i].output.vals[j] > speed) {drone[i].output.vals[j] = speed;}
				else if (drone[i].output.vals[j] < -speed) {drone[i].output.vals[j] = -speed;}
				else {drone[i].output.vals[j] = drone[i].output.vals[j];}
			}
			
			if (i <= num_fly)
			{	
				// Set data to publish
				cmd_vel_bebop[i].linear.x = -drone[i].output.vals[1];
				cmd_vel_bebop[i].linear.y = -drone[i].output.vals[0];
				cmd_vel_bebop[i].linear.z = drone[i].output.vals[2];
				cmd_vel_bebop[i].angular.z = drone[i].output.vals[3];
			}
			
			if (i > num_fly)
			{	
				// Set data to publish
				cmd_vel_bebop[i].linear.x = 0;//-drone[i].output.vals[1];
				cmd_vel_bebop[i].linear.y = 0;//-drone[i].output.vals[0];
				cmd_vel_bebop[i].linear.z = 0;//drone[i].output.vals[2];
				cmd_vel_bebop[i].angular.z = 0;//drone[i].output.vals[3];
			}
				
			// Publish the assembled command
			cmd_vel_pub_drone[i].publish(cmd_vel_bebop[i]);
		}

		/****************************************************************************************************
		 ********************************** U P D A T E  S T A T E S ****************************************
		 ****************************************************************************************************/
		
		// Run subscriber update function
		ros::spinOnce();
		// Delay loop to keep 100 Hz rate
		loop_rate.sleep();
		
		// save old values
		for (int i=0; i<n; i++)
		{
			drone[i].actPos_old.vals[0] = drone[i].actPos.vals[0];
			drone[i].actPos_old.vals[1] = drone[i].actPos.vals[1];
			drone[i].actPos_old.vals[2] = drone[i].actPos.vals[2];
			drone[i].actPos_old.vals[3] = drone[i].actPos.vals[3];
		}
		
		//ros::Time currtime=ros::Time::now();
		//ros::Duration diff=currtime-lasttime;
		
		//cout<<"diff: "<<diff<<endl;	
		
		myTime = myTime + Ts;
		
		price_shit.seekp(0,std::ios::end);
		price_shit << "" << myTime << "\t" << adjacency << "\n"; 
		
		bebop1Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop1Position_x << "" << myTime << "\t" << qd[0][0] << "\t" << bebop[0].pos_x << "\t" << cmd_vel_bebop[0].linear.x << "\t" << u_a[0].at<double>(1) << "\t" << u_g[0].at<double>(1) << "\t" << u[0].at<double>(1) << "\t" << pkk[0].at<double>(1) << "\n";
		bebop1Position_y << "" << myTime << "\t" << qd[0][1] << "\t" << bebop[0].pos_y << "\t" << cmd_vel_bebop[0].linear.y << "\t" << u_a[0].at<double>(0) << "\t" << u_g[0].at<double>(0) << "\t" << u[0].at<double>(0) << "\t" << pkk[0].at<double>(0) << "\n";
		bebop1Position_z << "" << myTime << "\t" << qd[0][2] << "\t" << bebop[0].pos_z << "\t" << cmd_vel_bebop[0].linear.z << "\t" << u_a[0].at<double>(2) << "\t" << u_g[0].at<double>(2) << "\t" << u[0].at<double>(2) << "\t" << pkk[0].at<double>(2) << "\n";
		
		bebop2Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop2Position_x << "" << myTime << "\t" << qd[1][0] << "\t" << bebop[1].pos_x << "\t" << cmd_vel_bebop[1].linear.x << "\t" << u_a[1].at<double>(1) << "\t" << u_g[1].at<double>(1) << "\t" << u[1].at<double>(1) << "\t" << pkk[1].at<double>(1) << "\n";
		bebop2Position_y << "" << myTime << "\t" << qd[1][1] << "\t" << bebop[1].pos_y << "\t" << cmd_vel_bebop[1].linear.y << "\t" << u_a[1].at<double>(0) << "\t" << u_g[1].at<double>(0) << "\t" << u[1].at<double>(0) << "\t" << pkk[1].at<double>(0) << "\n";
		bebop2Position_z << "" << myTime << "\t" << qd[1][2] << "\t" << bebop[1].pos_z << "\t" << cmd_vel_bebop[1].linear.z << "\t" << u_a[1].at<double>(2) << "\t" << u_g[1].at<double>(2) << "\t" << u[1].at<double>(2) << "\t" << pkk[1].at<double>(2) << "\n";
		
		bebop3Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop3Position_x << "" << myTime << "\t" << qd[2][0] << "\t" << bebop[2].pos_x << "\t" << cmd_vel_bebop[2].linear.x << "\t" << u_a[2].at<double>(1) << "\t" << u_g[2].at<double>(1) << "\t" << u[2].at<double>(1) << "\t" << pkk[2].at<double>(1) << "\n";
		bebop3Position_y << "" << myTime << "\t" << qd[2][1] << "\t" << bebop[2].pos_y << "\t" << cmd_vel_bebop[2].linear.y << "\t" << u_a[2].at<double>(0) << "\t" << u_g[2].at<double>(0) << "\t" << u[2].at<double>(0) << "\t" << pkk[2].at<double>(0) << "\n";
		bebop3Position_z << "" << myTime << "\t" << qd[2][2] << "\t" << bebop[2].pos_z << "\t" << cmd_vel_bebop[2].linear.z << "\t" << u_a[2].at<double>(2) << "\t" << u_g[2].at<double>(2) << "\t" << u[2].at<double>(2) << "\t" << pkk[2].at<double>(2) << "\n";
		
		bebop4Position_x.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop4Position_y.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop4Position_z.seekp(0,std::ios::end); // To ensure the put pointer is at the end
		bebop4Position_x << "" << myTime << "\t" << qd[3][0] << "\t" << bebop[3].pos_x << "\t" << cmd_vel_bebop[3].linear.x << "\t" << u_a[3].at<double>(1) << "\t" << u_g[3].at<double>(1) << "\t" << u[3].at<double>(1) << "\t" << pkk[3].at<double>(1) << "\n";
		bebop4Position_y << "" << myTime << "\t" << qd[3][1] << "\t" << bebop[3].pos_y << "\t" << cmd_vel_bebop[3].linear.y << "\t" << u_a[3].at<double>(0) << "\t" << u_g[3].at<double>(0) << "\t" << u[3].at<double>(0) << "\t" << pkk[3].at<double>(0) << "\n";
		bebop4Position_z << "" << myTime << "\t" << qd[3][2] << "\t" << bebop[3].pos_z << "\t" << cmd_vel_bebop[3].linear.z << "\t" << u_a[3].at<double>(2) << "\t" << u_g[3].at<double>(2) << "\t" << u[3].at<double>(2) << "\t" << pkk[3].at<double>(2) << "\n";
	}
	
	bebop1Position_x.close();
	bebop1Position_y.close();
	bebop1Position_z.close();
	
	bebop2Position_x.close();
	bebop2Position_y.close();
	bebop2Position_z.close();

	bebop3Position_x.close();
	bebop3Position_y.close();
	bebop3Position_z.close();

	bebop4Position_x.close();
	bebop4Position_y.close();
	bebop4Position_z.close();
	
	price_shit.close();

	printf("hello world!\n");
	
	return 0;
}

/****************************************************************************************************
 ***************************************** T O D O **************************************************
 ****************************************************************************************************/
// fly
