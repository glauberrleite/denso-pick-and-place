/* !<
 * Author: Israel Vasconcelos
 * Date: Oct/2018
 * Computing Institute - Federal University of Alagoas
 * DENSO's Arm Simulation: Model VP6242
 *
 * This application brings an inverse kinematics evaluation 
 * alongside a model running on Gazebo Simulator from the arm.
 *
 * \brief Description:
 * Evaluates the inverse kinematics of 3 first joints.
 * Managing a console application, grab the ROS topics which controls the arm.
 * Publishes the joint states following the x, y, z coordinates parsed.
 */

#include <stdlib.h>
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <tuple>
#include <cmath>
#include <stdlib.h> 

using namespace std;

#define PI 3.14159265

#define Z_OFFSET 280
#define GRIPPER_OFFSET 180

#define GRIPPER_OPEN 0
#define GRIPPER_CLOSE 0.365

#define GO_TO_ZERO_STATE() inverse_kinematics(210,0,385,0);

#define L1 210
#define L2 75
#define L3 210

#define joint1_limit 2.79
#define joint2_limit 2.09
#define joint3_limit 2.79
#define joint4_limit 2.79
#define joint5_limit 2.09
#define joint6_limit 6.28

tuple<double, double, double, double> current_position;
double current_gripper_position;

//! Handles the arm and publish the joint states to the simulator.
void handle_arm(double angle_j1, double angle_j2, double angle_j3, 
		double angle_j4=0, double angle_j5=0, double angle_j6=0) {

	const char * console_publish;

	//! Composing string for bash
	string str_handle = "rostopic pub -1 /ik_joint_states  sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ""},\n"
	"  name: [\"joint1\", \"joint2\", \"joint3\", \"joint4\", \"joint5\", \"joint6\"],\n"
	"  position: ["  + to_string(angle_j1) + ", "
			 + to_string(angle_j2) + ", "
			 + to_string(angle_j3) + ", "
			 + to_string(angle_j4) + ", "
			 + to_string(angle_j5) + ", "
			 + to_string(angle_j6) + "]}'";

	
	//! Conversion to const char *
	console_publish = str_handle.c_str();

	//! Publishing on respectives ROS topics the evaluated kinematics.

	cout << "Handling joint 1 by " << angle_j1 << " rad." << endl;
	cout << "Handling joint 2 by " << angle_j2 << " rad." << endl;
	cout << "Handling joint 3 by " << angle_j3 << " rad." << endl;
	cout << "Handling joint 4 by " << angle_j4 << " rad." << endl;
	cout << "Handling joint 5 by " << angle_j5 << " rad." << endl;
	cout << "Handling joint 6 by " << angle_j6 << " rad." << endl;

	system(console_publish);

	cout << "-------------------------------------------" << endl << endl;

}

void gripper_grab(double gripper_gap) {

	const char * console_publish;

	//! Composing string for bash
	string str_handle = "rostopic pub -1 /ik_joint_states  sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ""},\n"
	"  name: [\"gripper_finger1_joint\"],\n"
	"  position: ["  + to_string(gripper_gap) + "]}'";
	
	//! Conversion to const char *
	console_publish = str_handle.c_str();

	//! Publishing on respectives ROS topics the evaluated kinematics.
	if(gripper_gap > 0)
		cout << "Closing gripper by " << gripper_gap << endl;
	else 
		cout << "Opening gripper " << endl;
	

	system(console_publish);
	current_gripper_position = gripper_gap;
	cout << "-------------------------------------------" << endl << endl;

}

//! Evaluate the kinematics and call the arm handler.
void inverse_kinematics(double x, double y, double z, 
			double w=0,
			bool publish = true,
			bool display_angles=false,
			bool display_misc= false) {

		double true_z = z;

		double joint1_arg, joint2_arg, joint3_arg, joint4_arg, joint5_arg, joint6_arg;
		double L, Lf, xy;

		double alpha, beta, kappa;
		double phi, phi1, phi2;
		double c_phi, s_phi;
		double c_phi2,s_phi2;
		double theta_L, theta_L1;

		bool x_out_of_limits = x<210  || x>300;
		bool y_out_of_limits = y<-300 || y>300;
		bool z_out_of_limits = z< 20  || z>285;

		z = z - Z_OFFSET; //! Fixing z reference
		z = z + GRIPPER_OFFSET; //! Fixing reference to center of gripper
		
		if(0){//x_out_of_limits || y_out_of_limits  || z_out_of_limits) { //! Setting the spatial thresholds
			cout << "[WARNING!!]: Avoiding goal to an invalid position." << endl;
			publish = false;
			GO_TO_ZERO_STATE();
		} else {

			xy = sqrt(pow(x,2) + pow(y,2));
			L = sqrt(pow(xy,2) + pow(z,2));
			Lf= sqrt(pow(L2,2) + pow(L3,2));

			c_phi = (pow(L1,2) + pow(Lf,2) - pow(L,2)) / (2*L1*Lf);
			s_phi = sqrt(1 - pow(c_phi, 2));
			phi = atan2(s_phi,c_phi); //! When phi>PI/2, arm reaches an inadequate position

			c_phi2 = (pow(Lf,2) + pow(L,2) - pow(L1,2)) / (2*Lf*L);
			s_phi2 = sqrt(1 - pow(c_phi2,2));
			phi2 = atan2(s_phi2,c_phi2);

			phi1 = PI - phi - phi2;
			theta_L = atan2(z,xy);

			theta_L1 = theta_L + phi1;
			alpha = 2*PI - atan2(L3,L2) - phi;

			beta = atan2(L3,L2);
			kappa = (PI/2)-beta;

			joint1_arg = atan2(y,x);
			joint2_arg = ((PI/2)-theta_L1);
			joint3_arg = alpha-(PI/2);
			joint4_arg = 0;
			joint5_arg = PI-(kappa+phi2)-(PI/2-theta_L);
			joint6_arg = atan2(y,x) - fmod((2*acos(abs(w))) , PI/2);

			if(display_angles) {

				cout << "Joint angle 1 = " << joint1_arg*180/PI << endl << endl;
				cout << "Joint angle 2 = " << joint2_arg*180/PI << endl << endl;
				cout << "Joint angle 3 = " << joint3_arg*180/PI << endl << endl;
				cout << "Joint angle 4 = " << joint4_arg*180/PI << endl << endl;
				cout << "Joint angle 5 = " << joint5_arg*180/PI << endl << endl;
				cout << "Joint angle 6 = " << joint6_arg*180/PI << endl << endl;
			}

			if(display_misc) {
				cout << "L = " << L << endl;
				cout << "xy plan = " << xy << endl;
				cout << "Lf = " << Lf << endl;

				cout << "Phi = " << phi*180/PI << endl;
				cout << "Phi1 = " << phi1*180/PI << endl;
				cout << "Phi2 = " << phi2*180/PI << endl;

				cout << "Beta = " << atan2(L3,L2)*180/PI << endl << endl;	

				cout << "Alpha = " << alpha*180/PI << endl << endl;

				cout << "phi " << phi*180/PI << endl;

				cout << "L1*s_theta_L1 = " << sin(theta_L1)*L1 << endl;
				cout << "L1*c_theta_L1 = " << cos(theta_L1)*L1 << endl;
			} 

			if(publish) { //<! Publish on the ROS Topics, disable it for debug the calculations.
				handle_arm(joint1_arg, joint2_arg, joint3_arg, 
					   joint4_arg, joint5_arg, joint6_arg);

				current_position = make_tuple(x, y, true_z, w);
			}
	}
}

//! Discretize the trajectory through a straight line in microsteps 
void straight_trajectory_stepper(double x_goal , double y_goal , double z_goal , double w_goal, 
				 bool step_info = false, bool planner_info = false) {

	double x,y,z,w;

	double x_start, y_start, z_start, w_start;
	double step=3, step_to_x, step_to_y, step_to_z, step_to_w, z_offset_fix;

	x_start = get<0>(current_position);
	y_start = get<1>(current_position);
	z_start = get<2>(current_position);
	w_start = get<3>(current_position);

	step_to_x = (x_goal - x_start)/step;
	step_to_y = (y_goal - y_start)/step;
	step_to_z = (z_goal - z_start)/step;
	step_to_w = (w_goal - w_start)/step;

	x = x_start;
	y = y_start;
	z = z_start;
	w = w_start;

	for(int i=0; i<step; i++) {
		x = x + step_to_x;
		y = y + step_to_y;
		w = w + step_to_w;
		z = z + step_to_z;
		inverse_kinematics(x,y,z,w);

		if(step_info) {
			cout << "----- GOAL -----" << endl;
			cout << 
			"  x=" << x << 
			"; y=" << y << 
			"; z=" << z << endl;
			cout << "----- GOAL -----" << endl << endl;
		}
	}

	if(planner_info) {
		cout << "----- PLANNER INFO -----" << endl;
		cout << "X-Axis step " << step_to_x << "mm" << endl;
		cout << "Y-Axis step " << step_to_y << "mm" << endl;
		cout << "Z-Axis step " << step_to_z << "mm" << endl;
		cout << "----- PLANNER INFO -----" << endl << endl;
	}
}


//! Path planner for a move which grab the cube and take it back to its starting position.
void planner_grab_and_move(double x_start, double y_start, double z_start, double w_start,
				double x_goal , double y_goal , double z_goal , double w_goal ) {

	double x, y, z, w, gripper_position, height_offset;
	vector < tuple<double, double, double, double, double> > goals;

	height_offset=150;

	//! Align at x/y axis and adjust the rotation of joint6/gripper
	goals.push_back( make_tuple( x_goal, y_goal  , height_offset , w_goal, GRIPPER_OPEN) );

	//! Lift down the arm with the gripper opened	
	goals.push_back( make_tuple( x_goal, y_goal  , z_goal       , w_goal, GRIPPER_OPEN)  );

	//! Close the gripper and grab the object
	goals.push_back( make_tuple( x_goal, y_goal  , z_goal       , w_goal, GRIPPER_CLOSE) );

	//! Lift up the arm with the gripper closed	
	goals.push_back( make_tuple( x_goal, y_goal  , height_offset, w_goal, GRIPPER_CLOSE) );		

	//! Move arm to the targeted position
	goals.push_back( make_tuple( x_start, y_start, height_offset, w_start, GRIPPER_CLOSE));

	//<! Open gripper to release the object
	goals.push_back( make_tuple( x_start, y_start, z_start	    , w_start, GRIPPER_OPEN) );

	//<! Lift up the arm 
	goals.push_back( make_tuple( x_start, y_start, height_offset, w_start, GRIPPER_OPEN));

	//<! Back to the zero state
	goals.push_back( make_tuple( 210, 0  , 415 , 0, GRIPPER_OPEN) );

	GO_TO_ZERO_STATE();
	gripper_grab(GRIPPER_OPEN);

	for(int i=0; i<goals.size(); i++) {
		x = get<0>(goals.at(i));
		y = get<1>(goals.at(i));
		z = get<2>(goals.at(i));
		w = get<3>(goals.at(i));
		gripper_position = get<4>(goals.at(i));

		//straight_trajectory_stepper(x, y, z, w, true);
		inverse_kinematics(x, y, z, w, true);

		if(i > 0 && gripper_position != get<4>(goals.at(i-1))) {
			gripper_grab(gripper_position);
		}
	}

}


//! Main execution
int main() {

	double x_start, y_start, z_start, w_start;
	double x_goal , y_goal , z_goal , w_goal;

	tuple<double, double, double, double> start_position;
	tuple<double, double, double, double> goal_position;

	start_position = make_tuple( 220, -300, 25, 0);
	goal_position  = make_tuple( 280, 175, 25, 0 );

	x_start = get<0>(start_position);
	y_start = get<1>(start_position);
	z_start = get<2>(start_position);
	w_start = get<3>(start_position);

	x_goal = get<0>(goal_position);
	y_goal = get<1>(goal_position);
	z_goal = get<2>(goal_position);
	w_goal = get<3>(goal_position);

	planner_grab_and_move(x_start, y_start, z_start, w_start, x_goal , y_goal , z_goal , w_goal);

	GO_TO_ZERO_STATE();

	return 0;
}
