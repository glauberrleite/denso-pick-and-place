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

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tfMessage.h>


#include <stdlib.h>
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <tuple>
#include <cmath>

using namespace std;

#define PI 3.14159265

#define Z_OFFSET 280
#define GRIPPER_OFFSET 180

#define GRIPPER_OPEN 0
#define GRIPPER_CLOSE 0.42

#define GO_TO_ZERO_STATE() inverse_kinematics(210,0,385,0,0);

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
double current_gripper_position = GRIPPER_OPEN;

tuple<double, double, double, double> goal_position;
tuple<double, double, double, double> start_position;

vector < tuple<double, double, double, double> > goal_position_samples;

bool start_position_ready = true;
bool goal_position_ready = false;

//* Toggle the variable "simulation" true/false to define the source of data. 
/* If true, set an arbitrary goal. 
/* If false, assign the goal with data listened from topic /denso_cube_tf
*/
bool simulation = true; 

//! Evaluate the kinematics and call the arm handler.
tuple<double, double, double, double, double, double, double> inverse_kinematics(double x, double y, 
										 double z,  double w,
										 double gripper_position,
										 bool display_angles=false,
									 	 bool display_misc= false) 
{

		tuple<double, double, double, double, double, double, double> current_joint_state;
		double joint1_arg, joint2_arg, joint3_arg, joint4_arg, joint5_arg, joint6_arg;
		double L, Lf, xy;

		double alpha, beta, kappa;
		double phi, phi1, phi2;
		double c_phi, s_phi;
		double c_phi2,s_phi2;
		double theta_L, theta_L1;

		double true_z = z;
		bool x_out_of_limits = x<210  || x>300;
		bool y_out_of_limits = y<-300 || y>300;
		bool z_out_of_limits = z< 20  || z>415;
		bool gripper_limits  = gripper_position < 0 || gripper_position > 0.8;
		bool joints_nan_flag, joints_inf_flag;

		z = z - Z_OFFSET; //<! Fixing z reference
		z = z + GRIPPER_OFFSET; //<! Fixing reference to center of gripper
		
		if(x_out_of_limits || y_out_of_limits  || z_out_of_limits || gripper_limits) { //<! Setting the spatial thresholds
			cout << "Avoiding goal to an invalid position. Execution halted." << endl;

			cout << "----- GOAL -----" << endl;
			cout << 
			"  x=" << x << 
			"; y=" << y << 
			"; z=" << true_z << endl;
			cout << "----- GOAL -----" << endl << endl;

			exit (EXIT_FAILURE);

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
			joint6_arg = atan2(y,x) - fmod((2*acos(abs(w))) , PI);

			joints_nan_flag = isnan(joint1_arg) || isnan(joint2_arg) || isnan(joint3_arg) || isnan(joint4_arg) || isnan(joint5_arg) || isnan(joint6_arg);

			joints_inf_flag = isinf(joint1_arg) || isinf(joint2_arg) || isinf(joint3_arg)|| isinf(joint4_arg) || isinf(joint5_arg) || isinf(joint6_arg);

			if(joints_nan_flag || joints_inf_flag) { //<! Setting the angular thresholds
				cout << "Avoiding goal to an invalid position. Execution halted." << endl;
				exit (EXIT_FAILURE);
			}

			if(display_angles) {

				cout << "-------------------------------------------" << endl;
				cout << "Handling joint 1 by " << joint1_arg << " rad." << endl;
				cout << "Handling joint 2 by " << joint2_arg << " rad." << endl;
				cout << "Handling joint 3 by " << joint3_arg << " rad." << endl;
				cout << "Handling joint 4 by " << joint4_arg << " rad." << endl;
				cout << "Handling joint 5 by " << joint5_arg << " rad." << endl;
				cout << "Handling joint 6 by " << joint6_arg << " rad." << endl;
				cout << "-------------------------------------------" << endl << endl;

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
	}

	current_joint_state = make_tuple(joint1_arg, joint2_arg, joint3_arg, 
					 joint4_arg, joint5_arg, joint6_arg, gripper_position);

	//<! Updating global variables
	current_position = make_tuple(x, y, true_z, w);
	current_gripper_position = gripper_position;

	return current_joint_state;
}

//! Discretize the trajectory through a straight line in microsteps 
vector< tuple<double, double, double, double, double, double, double> > straight_trajectory_stepper(double x_goal , double y_goal , double z_goal , double w_goal, double gripper_position, bool step_info = true, bool planner_info=false) {

	vector< tuple<double, double, double, double, double, double, double> > joint_trajectory;
	double x,y,z,w;
	double x_start, y_start, z_start, w_start;
	double step_to_x, step_to_y, step_to_z, step_to_w, z_offset_fix;
	double step=20;


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

	for(int i=0; i<step; i++) { //! Moving throug xyz-axis and rotating joint6/gripper by w arg
		x = x + step_to_x;
		y = y + step_to_y;
		w = w + step_to_w;
		z = z + step_to_z;

		if(step_info) {
			cout << "----- GOAL -----" << endl;
			cout << 
			"  x=" << x << 
			"; y=" << y << 
			"; z=" << z << endl;
			cout << "----- GOAL -----" << endl << endl;
		}

		joint_trajectory.push_back(inverse_kinematics(x,y,z,w,gripper_position,false));
	}

	if(planner_info) {
		cout << "----- PLANNER INFO -----" << endl;
		cout << "X-Axis step " << step_to_x << "mm" << endl;
		cout << "Y-Axis step " << step_to_y << "mm" << endl;
		cout << "Z-Axis step " << step_to_z << "mm" << endl;
		cout << "----- PLANNER INFO -----" << endl << endl;
	}

	return joint_trajectory;
}


//<! Path planner for a move which grab the cube and take it back to its starting position.
vector< tuple<double, double, double, double, double, double, double> > planner_grab_and_move(
	double x_start, double y_start, double z_start, double w_start,
	double x_goal , double y_goal , double z_goal , double w_goal ) {

	double x, y, z, w, gripper_position, height_offset;
	vector < tuple<double, double, double, double, double> > goals;
	vector< tuple<double, double, double, double, double, double, double> > overall_joint_trajectory;
	vector< tuple<double, double, double, double, double, double, double> > step_joint_trajectory;

	height_offset=150;

	goals.push_back( make_tuple( 210, 0  , 385 , 0, GRIPPER_OPEN) );

	//<! Align at x/y axis and adjust the rotation of joint6/gripper
	goals.push_back( make_tuple( x_goal, y_goal  , height_offset, w_goal, GRIPPER_OPEN) );

	//<! Lift down the arm with the gripper opened	
	goals.push_back( make_tuple( x_goal, y_goal  , z_goal       , w_goal, GRIPPER_OPEN)  );

	//<! Close the gripper and grab the object
	goals.push_back( make_tuple( x_goal, y_goal  , z_goal       , w_goal, GRIPPER_CLOSE) );

	//<! Lift up the arm with the gripper closed	
	goals.push_back( make_tuple( x_goal, y_goal  , height_offset, w_goal, GRIPPER_CLOSE) );		

	//<! Move arm to the targeted position
	goals.push_back( make_tuple( x_start, y_start, height_offset, w_start, GRIPPER_CLOSE));

	//<! Lifting down arm to the targeted position
	goals.push_back( make_tuple( x_start, y_start, z_start	    , w_start, GRIPPER_CLOSE) );

	//<! Open gripper to release the object
	goals.push_back( make_tuple( x_start, y_start, z_start	    , w_start, GRIPPER_OPEN) );

	//<! Lift up the arm 
	goals.push_back( make_tuple( x_start, y_start, height_offset, w_start, GRIPPER_OPEN));

	//<! Back to the zero state
	goals.push_back( make_tuple( 210, 0  , 385 , 0, GRIPPER_OPEN) );

	GO_TO_ZERO_STATE();

	for(int i=0; i<goals.size(); i++) {
		x = get<0>(goals.at(i));
		y = get<1>(goals.at(i));
		z = get<2>(goals.at(i));
		w = get<3>(goals.at(i));
		gripper_position = get<4>(goals.at(i));

		step_joint_trajectory = straight_trajectory_stepper(x, y, z, w, gripper_position);

		overall_joint_trajectory.insert(overall_joint_trajectory.end(),
						step_joint_trajectory.begin() ,
						step_joint_trajectory.end()  );
	}

	return overall_joint_trajectory;

}

void goal_position_callback(const tf::tfMessage::ConstPtr& msg) {

	double x, y, z, w;
	tuple<double, double, double, double> current_position;


	if(goal_position_samples.size() < 10) {

			x = msg->transforms[0].transform.translation.x*1000;
			y = msg->transforms[0].transform.translation.y*1000;
			z = msg->transforms[0].transform.translation.z*1000;
			w = msg->transforms[0].transform.rotation.w;

			ROS_INFO("Buffering.");

			/*ROS_INFO("I heard x_t: [%f]", msg->transforms[0].transform.translation.x);
			ROS_INFO("I heard y_t: [%f]", msg->transforms[0].transform.translation.y);
			ROS_INFO("I heard z_t: [%f]", msg->transforms[0].transform.translation.z);
			ROS_INFO("I heard x_r: [%f]", msg->transforms[0].transform.rotation.x);
			ROS_INFO("I heard y_r: [%f]", msg->transforms[0].transform.rotation.y);
			ROS_INFO("I heard z_r: [%f]", msg->transforms[0].transform.rotation.z);
			ROS_INFO("I heard w_r: [%f]", msg->transforms[0].transform.rotation.w);*/
	
			current_position = make_tuple( x, y, z, w );

			goal_position_samples.push_back(current_position);

	} else {
		goal_position = goal_position_samples.back();
		goal_position_ready = true;
	}
}



//<! Main execution
int main(int argc, char** argv) {

	ros::init(argc, argv, "vp6242_state_publisher");
	ros::NodeHandle node;
	ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("ik_joint_states", 1);
	ros::Rate loop_rate(10);

	ros::Subscriber sub = node.subscribe("/denso_cube_tf", 10, goal_position_callback);

	sensor_msgs::JointState joint_state;
	int trajectory_step=0;

	double x_goal , y_goal , z_goal , w_goal;
	double x_start, y_start, z_start, w_start;

	vector< tuple<double, double, double, double, double, double, double> > overall_joint_trajectory;

	start_position = make_tuple( 250, 0, 30, 0 );

	x_start = get<0>(start_position);
	y_start = get<1>(start_position);
	z_start = get<2>(start_position);
	w_start = get<3>(start_position);


	if(simulation) {
		goal_position = make_tuple( 250, 250, 25, .69 );
		goal_position_ready = true;

	} else
		while(!goal_position_ready)
			ros::spinOnce();

	if(goal_position_ready) {

		x_goal = get<0>(goal_position);
		y_goal = get<1>(goal_position);
		z_goal = get<2>(goal_position);
		w_goal = get<3>(goal_position);

		overall_joint_trajectory = planner_grab_and_move(x_start, y_start, z_start, w_start, 
							 x_goal , y_goal , z_goal , w_goal);
	
		while (ros::ok() && trajectory_step < overall_joint_trajectory.size()) {
			//update joint_state
			joint_state.header.stamp = ros::Time::now();
			joint_state.name.resize(7);
			joint_state.position.resize(7);
			joint_state.name[0] ="joint1";
			joint_state.position[0] = get<0>(overall_joint_trajectory.at(trajectory_step));
			joint_state.name[1] ="joint2";
			joint_state.position[1] = get<1>(overall_joint_trajectory.at(trajectory_step));
			joint_state.name[2] ="joint3";
			joint_state.position[2] = get<2>(overall_joint_trajectory.at(trajectory_step));
			joint_state.name[3] ="joint4";
			joint_state.position[3] = get<3>(overall_joint_trajectory.at(trajectory_step));
			joint_state.name[4] ="joint5";
			joint_state.position[4] = get<4>(overall_joint_trajectory.at(trajectory_step));
			joint_state.name[5] ="joint6";
			joint_state.position[5] = get<5>(overall_joint_trajectory.at(trajectory_step));
			joint_state.name[6] ="gripper_finger1_joint";
			joint_state.position[6] = get<6>(overall_joint_trajectory.at(trajectory_step));

			joint_pub.publish(joint_state);  //! Send the joint state

			loop_rate.sleep(); //! This will adjust as needed per iteration
			trajectory_step=trajectory_step+1;
		}
	}

	cout << "Trajectory finished." << endl;

	return 0;
}
