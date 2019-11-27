#include <LWRCartImpedanceController.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include <pthread.h>
#include <mutex>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <kdl/frames_io.hpp>
#include <thread>
#include <unistd.h>
#include "RLUtils.hpp"

#include <boost/math/distributions/normal.hpp>
#include <boost/math/constants/constants.hpp>


#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS			7
#endif

#ifndef NUMBER_OF_CART_DOFS
#define NUMBER_OF_CART_DOFS			6
#endif

#ifndef NUMBER_OF_FRAME_ELEMENTS
#define NUMBER_OF_FRAME_ELEMENTS	12
#endif

#define RUN_TIME_IN_SECONDS			10800.0


//*******************************************************************************************
// main()
//
int main(int argc, char *argv[])
{
	unsigned int CycleCounter = 0, i = 0;
	int ResultValue	= 0;

	LWRCartImpedanceController *Robot;
	KDL::Frame ee_frame;
	double roll, pitch, yaw;
	double qx, qy, qz, qw;
	// ROS INITIALIZATION
	rclcpp::init(argc, argv);
  	node = rclcpp::Node::make_shared("kuka_cartesian_control");
	// ros::init(argc, argv, "kuka_cartesian_control");
	// ros::NodeHandle nh;
  	joint_state_pub =  node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
  	cart_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/cartesian_pose", 1);
  	wrench_pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>("/wrench", 1);
	auto cmd_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 100, cmd_vel_cb);
	auto event_in_sub = node->create_subscription<std_msgs::msg::String>("/event_in", 100, event_in_cb);

    // ROBOT AND FRI INITILIZATION
	Robot	=	new LWRCartImpedanceController("/home/lwr-station/ros2/src/kuka_lwr_fri_library/config/980039-FRI-Driver.init");
	fprintf(stdout, "RobotCartImpedanceController object created. Starting the robot...\n");
	ResultValue	=	Robot->StartRobot();
	if (ResultValue == EOK)
	{
		fprintf(stdout, "Robot successfully started.\n");
	}
	else
	{
		fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
	}
	// fprintf(stdout, "Current system state:\n%s\n", Robot->GetCompleteRobotStateAndInformation());

	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		CommandedStiffness[i] =	(float)50.0 * ((i <= 2)?(100.0):(100.0));
		CommandedDamping[i]	= (float)0.7;
		CommandedForcesAndTorques[i] = (float)0.0;
	}
	Robot->GetMeasuredCartPose(MeasuredPose);
	Robot->GetMeasuredJointPositions(JointValuesInRad);
	Robot->SetCommandedCartStiffness(CommandedStiffness);
	Robot->SetCommandedCartDamping(CommandedDamping);

	setup_original_pose();

	previous_time = (float)CycleCounter * Robot->GetCycleTime();
	current_time = (float)CycleCounter * Robot->GetCycleTime();
	start_time = current_time;

	// KDL INITIALIZATION

	KDL::Rotation ee_rot(MeasuredPose[0], MeasuredPose[1], MeasuredPose[2],
						 MeasuredPose[4], MeasuredPose[5], MeasuredPose[6],
				         MeasuredPose[8], MeasuredPose[9], MeasuredPose[10]);
	KDL::Vector ee_trans(MeasuredPose[3], MeasuredPose[7], MeasuredPose[11]);
	ee_frame = KDL::Frame(ee_rot, ee_trans);
	ee_rot.GetRPY(roll, pitch, yaw);
	ee_rot.GetQuaternion(qx, qy, qz, qw);
	current_angular_pos[0] = roll;
	current_angular_pos[1] = pitch;
	current_angular_pos[2] = yaw;
	float new_pitch = pitch, new_roll = roll, new_yaw = yaw; 

	calculate_guassian_param();

	std::thread joint_state_pub_tread(robot_state_publisher_function);
	// start_execution = true;
	// task_mode = JOY_CONTROL;
	while (((float)CycleCounter * Robot->GetCycleTime() < RUN_TIME_IN_SECONDS) and (not stop_program) and rclcpp::ok())
	{
		Robot->WaitForKRCTick();
		if (!Robot->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			stop_program = true;
		}
		KDL::Rotation ee_rot(MeasuredPose[0], MeasuredPose[1], MeasuredPose[2],
							 MeasuredPose[4], MeasuredPose[5], MeasuredPose[6],
							 MeasuredPose[8], MeasuredPose[9], MeasuredPose[10]);
		KDL::Vector ee_trans(MeasuredPose[3], MeasuredPose[7], MeasuredPose[11]);
		ee_frame = KDL::Frame(ee_rot, ee_trans);
		ee_rot.GetRPY(roll, pitch, yaw);
		ee_rot.GetQuaternion(qx, qy, qz, qw);
		auto ros_time = rclcpp::Duration(current_time);
		geometry_msgs::msg::PoseStamped posestamped_;
		posestamped_.header.frame_id = "/tcp";
		posestamped_.header.stamp.sec = ros_time.seconds();
		posestamped_.header.stamp.nanosec = ros_time.nanoseconds();
		posestamped_.pose.position.x = MeasuredPose[3];
		posestamped_.pose.position.y = MeasuredPose[7];
		posestamped_.pose.position.z = MeasuredPose[11];
		posestamped_.pose.orientation.x = qx;
		posestamped_.pose.orientation.y = qy;
		posestamped_.pose.orientation.z = qz;
		posestamped_.pose.orientation.w = qw;
		cart_pose_pub->publish(posestamped_);
		Robot->SetCommandedCartStiffness(CommandedStiffness);

		switch (task_mode)
		{

		case POSITIONING:
		{
			// std::cout << MeasuredPose[3] << " " << MeasuredPose[7] << " " << MeasuredPose[11] << " " << roll
							// << " " << pitch << " " << yaw << std::endl;
			if (fabs(pitch) < 0.01 and fabs(-pi/2 - yaw) < 0.01 and fabs(fabs(-pi) - roll) < 0.01)
			{
				task_mode = IDLE;
				execution_finished = true;
			}
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			if (roll < 0.0 and roll > -pi)
			{
				new_roll = new_roll - 0.1 * (current_time - previous_time);
			}
			else
			{
				new_roll = new_roll + 0.1 * (current_time - previous_time);
			}
			if (pitch > 0.0)
			{
				new_pitch = new_pitch - 0.1 * (current_time - previous_time);
			}
			else
			{
				new_pitch = new_pitch + 0.1 * (current_time - previous_time);
			}
			if (yaw > -pi/2)
			{
				new_yaw = new_yaw - 0.1 * (current_time - previous_time);
			}
			else
			{
				new_yaw = new_yaw + 0.1 * (current_time - previous_time);
			}

			previous_time = current_time;
			KDL::Rotation new_rot = KDL::Rotation::RPY(new_roll, new_pitch, new_yaw);
			CommandedPose[0] = new_rot(0,0);
			CommandedPose[1] = new_rot(0,1);
			CommandedPose[2] = new_rot(0,2);
			CommandedPose[4] = new_rot(1,0);
			CommandedPose[5] = new_rot(1,1);
			CommandedPose[6] = new_rot(1,2);
			CommandedPose[8] = new_rot(2,0);
			CommandedPose[9] = new_rot(2,1);
			CommandedPose[10] = new_rot(2,2);
			Robot->SetCommandedCartPose(CommandedPose);
		}
			break;

		case GRAV_COMP:
			Robot->SetCommandedCartPose(MeasuredPose);
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			previous_time = current_time;
			break;



		case JOY_CONTROL:
			// std::cout << "in joy control" << std::endl;
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			CommandedPose[3] = CommandedPose[3] + linear_velocity[0] * (current_time - previous_time);
			CommandedPose[7] = CommandedPose[7] + linear_velocity[1] * (current_time - previous_time);
			CommandedPose[11] = CommandedPose[11] + linear_velocity[2] * (current_time - previous_time);
			previous_time = current_time;
			Robot->SetCommandedCartPose(CommandedPose);
			break;

		case IDLE:
		default:
			// std::cout << "Entered in case: " << IDLE << std::endl;
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			previous_time = current_time;
			new_pitch = pitch, new_roll = roll, new_yaw = yaw;
			break;
		}
		// Reading Robot state
		Robot->GetMeasuredJointTorques(MeasuredJointTorques);
		Robot->GetMeasuredJointPositions(JointValuesInRad);
		Robot->GetMeasuredCartPose(MeasuredPose);
		Robot->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesAndTorques);
		CycleCounter++;
	}

	// RCLCPP_INFO("Stopping the robot...");
	ResultValue	=	Robot->StopRobot();

	if (ResultValue != EOK)
	{
		// RCLCPP_INFO("An error occurred during stopping the robot...");
	}
	else
	{
		// RCLCPP_INFO("Robot successfully stopped.");
	}

	// RCLCPP_INFO("Deleting the object...");
	delete Robot;
	// RCLCPP_INFO("Object deleted...");
	joint_state_pub_tread.join();
	return(EXIT_SUCCESS);
}

