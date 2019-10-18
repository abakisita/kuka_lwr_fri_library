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



void state_coordinator()
{
	int i = 0;
	episode_data.clear();
	while (true)
	{
		single_time_step_data.clear();
		roll_out_data.clear();
		// Update policy params
		if (not policy_params_updated)
		{
			policy.update_params(updated_policy_params[0], updated_policy_params[1], updated_policy_params[2]);
			policy_params_updated = true;
			std::cout << "Updated policy parameters" << std::endl;
			std::cout << "New policy params " << policy.A << " " << policy.B << " " << policy.C << std::endl;
		}
		if (start_execution and policy_params_updated)
		{

			start_execution = false;
			// Go to start
			std::cout << "GOING TO START" << std::endl;
			execution_finished = false;
			setup_original_pose();
			setup_new_start_pose();
			task_mode = GO_TO_START;
			while (!execution_finished)
			{
				usleep(1000);
			}
			std::cout << "Robot at start position" << std::endl << std::endl;
			usleep(2000000);

			// Positioning robot above the vegetable
			std::cout << "POSITIONING THE ROBOT" << std::endl;
			execution_finished = false;
			setup_original_pose();
			task_mode = POSITIONING;
			while (!execution_finished)
			{
				usleep(1000);
			}
			std::cout << "FINISHED POSITIONING THE ROBOT" << std::endl << std::endl;
			usleep(2000000);

			// Touching down until some force is felt at TCP
			std::cout << "TOUCH DOWN" << std::endl;
			execution_finished = false;
			setup_touch_down(-0.10, 0.01);
			setup_original_pose();
			// std::cout << target_linear_pos[2] << " " << MeasuredPose[11] << std::endl;
			task_mode = TOUCH_DOWN;
			while (!execution_finished)
			{
				usleep(1000);
			}
			std::cout << "FINISHED TOUCH DOWN" << std::endl << std::endl;
			usleep(2000000);


			// Cutting action execution
			std::cout << "CUTTING" << std::endl;
			execution_finished = false;
			force_z_offset = EstimatedExternalCartForcesAndTorques[2];
			setup_original_pose();
			CommandedStiffness[2] = 15.0;
			set_gaussian_velocity_params(0.05, 0.10);
			calculate_guassian_param();
			task_mode = CUTTING;
			while (!execution_finished)
			{
				usleep(1000);
			}
			CommandedStiffness[2] = 50000;

			std::cout << "FINISHED CUTTING" << std::endl << std::endl;
			usleep(2000000);

			// Going up
			std::cout << "GOING UP" << std::endl;
			execution_finished = false;
			setup_touch_down(0.07, 0.05);
			setup_original_pose();
			std::cout << target_linear_pos[2] << " " << MeasuredPose[11] << std::endl;
			task_mode = GO_UP;
			while (!execution_finished)
			{
				usleep(1000);
			}
			std::cout << "FINISHED GOING UP" << std::endl << std::endl;
			usleep(2000000);


		}
		usleep(1000);

	}

}


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
	int sec, nsec;

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
	float original_pose = CommandedPose[3];

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
	std::thread state_coordinator_thread(state_coordinator);
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
		case CUTTING:
		{
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			float x_vel = gaussian_velocity(current_time - start_time);
			CommandedPose[7] = CommandedPose[7] - x_vel * (current_time - previous_time);
			Robot->GetMeasuredCartPose(MeasuredPose);
			CommandedPose[11] = MeasuredPose[11];
 			previous_time = current_time;
			float a = 0.0;
			float u = 0.0;
			float state[3];
			state[0] = -x_vel;
			state[1] = (LearningReferencePose[7] - MeasuredPose[7])/x_dist;
			state[2] = (LearningReferencePose[11] - MeasuredPose[11])/z_dist;
			policy.action(a, u, state);
			float original_a = a;
			// std::cout << LearningReferencePose[11] << " " << MeasuredPose[11] << " " << state[2] << " " << z_dist << std::endl;
			float excessive_force_penalty = 0.0;
			if (a < 0.0)
			{
				CommandedForcesAndTorques[2] = -1.7 ;
			}
			else if (a > max_force)
			{
				CommandedForcesAndTorques[2] = max_force;
				a = a;
			}
			else
			{
				CommandedForcesAndTorques[2] = a;
				a = a;
			}

			Robot->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques);
			Robot->SetCommandedCartPose(CommandedPose);
			if (roll_out_data.size() >= 1)
			{
				float reward = (100 * state[2] * state[2] - 1.0 * std::pow(roll_out_data[roll_out_data.size() - 1][1], 2));
				roll_out_data[roll_out_data.size()-1].push_back(reward);
			}
			single_time_step_data.clear();
			std::cout << u << " " << a << " " << original_a << " " << EstimatedExternalCartForcesAndTorques[2] 
									<< " " << CommandedForcesAndTorques[2] << std::endl;
 			single_time_step_data = std::vector<float> ({u, a, state[0], state[1], state[2]});
			roll_out_data.push_back(single_time_step_data);
			geometry_msgs::msg::TwistStamped twist_msg_;
			twist_msg_.header.frame_id = "base_link";
			twist_msg_.header.stamp.sec = ros_time.seconds(); 
			twist_msg_.header.stamp.nanosec = ros_time.nanoseconds();
			twist_msg_.twist.linear.y = -x_vel;
			// cart_twist_pub->publish(twist_msg_);
			
			geometry_msgs::msg::WrenchStamped wrench_cmd;
			wrench_cmd.header.stamp.sec = ros_time.seconds();
			wrench_cmd.header.stamp.nanosec = ros_time.nanoseconds();
			wrench_cmd.wrench.force.z = a;
			// force_command_pub.publish(wrench_cmd);
			if ((current_time - start_time) >= (gaussian_mu + 2 * gaussian_sigma))
			{
				float termination_reward = 0.0;
				if (state[2] > 0.995)
				{
					termination_reward = 100;
				}
				roll_out_data[roll_out_data.size()-1].push_back(termination_reward);
				task_mode = IDLE;
				execution_finished = true;
			}

		}
			break;

		case POSITIONING:
		{
			// std::cout << MeasuredPose[3] << " " << MeasuredPose[7] << " " << MeasuredPose[11] << " " << roll
							// << " " << pitch << " " << yaw << std::endl;
			float angular_setpoint = pi/4;
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

		case GO_TO_START:
		{
			float velocity[3];
			if (get_velocity(new_start_pose, velocity, 0.05))
			{
				task_mode = IDLE;
				execution_finished = true;
				std::cout << "Robot is at start position" << std::endl;
			}
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			CommandedPose[3] = CommandedPose[3] + velocity[0] * (current_time - previous_time);
			CommandedPose[7] = CommandedPose[7] + velocity[1] * (current_time - previous_time);
			CommandedPose[11] = CommandedPose[11] + velocity[2] * (current_time - previous_time);
			previous_time = current_time;
			Robot->SetCommandedCartPose(CommandedPose);
		}
			break;

		case REPOSITIONING:
		{
			float velocity[3];
			if (get_velocity(start_pose, velocity, 0.05))
			{
				task_mode = IDLE;
				execution_finished = true;
			}
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			CommandedPose[3] = OriginalPose[3] + velocity[0] * (current_time - previous_time);
			CommandedPose[7] = OriginalPose[7] + velocity[1] * (current_time - previous_time);
			CommandedPose[11] = OriginalPose[11] + velocity[2] * (current_time - previous_time);
			previous_time = current_time;
			Robot->SetCommandedCartPose(CommandedPose);
		}
			break;

		case GRAV_COMP:
			Robot->SetCommandedCartPose(MeasuredPose);
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			previous_time = current_time;
			break;

		case TOUCH_DOWN:
		{
			// std::cout << "Entered in case: " << TOUCH_DOWN << std::endl;
			float error = target_linear_pos[2] - MeasuredPose[11];
			if (fabs(error) < 0.001 or fabs(EstimatedExternalCartForcesAndTorques[2]) > 2.5)
			{
				std::cout << EstimatedExternalCartForcesAndTorques[2] << std::endl;
				task_mode = IDLE;
				CommandedStiffness[2] = 5.0;

				execution_finished = true;
			}
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			if (error < 0.0)
			{
				CommandedPose[11] = CommandedPose[11] - max_vertical_vel * (current_time - previous_time);
			}
			else
			{
				CommandedPose[11] = CommandedPose[11] + max_vertical_vel * (current_time - previous_time);
			}
			previous_time = current_time;
			Robot->SetCommandedCartPose(CommandedPose);
		}
			break;

		case GO_UP:
		{
			// std::cout << "Entered in case: " << TOUCH_DOWN << std::endl;
			float error = target_linear_pos[2] - MeasuredPose[11];
			if (fabs(error) < 0.001)
			{
				task_mode = IDLE;
				execution_finished = true;
			}
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			if (error < 0.0)
			{
				CommandedPose[11] = CommandedPose[11] - 0.05 * (current_time - previous_time);
			}
			else
			{
				CommandedPose[11] = CommandedPose[11] + 0.05 * (current_time - previous_time);
			}
			previous_time = current_time;
			Robot->SetCommandedCartPose(CommandedPose);
		}
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
			// std::cout << "Entered in case: " << IDLE << std::endl;
			current_time = (float)CycleCounter * Robot->GetCycleTime();
			previous_time = current_time;
			new_pitch = pitch, new_roll = roll, new_yaw = yaw;
			break;

		default:
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
	state_coordinator_thread.join();
	return(EXIT_SUCCESS);
}

