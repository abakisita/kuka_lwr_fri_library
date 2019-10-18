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
#include <vector>
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// #include <sensor_msgs/Joy.h>
// #include <std_msgs/String.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <kdl/frames_io.hpp>
#include <thread>

#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
#include <chrono>
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

#define IDLE 0 
#define CUTTING 1
#define POSITIONING 2
#define GRAV_COMP 3
#define TOUCH_DOWN 4
#define REPOSITIONING 5
#define GO_TO_START 6
#define JOY_CONTROL 7
#define GO_UP 8


#ifndef __RLUtils__
#define __RLUtils__
using namespace std::chrono_literals;

int task_mode = IDLE;
bool stop_program = false;

int state_machine_status = 0;
bool execution_finished = true;
bool start_execution = false;

float EstimatedExternalCartForcesAndTorques[NUMBER_OF_CART_DOFS], 
	  JointValuesInRad[NUMBER_OF_JOINTS],
	  CommandedForcesAndTorques[NUMBER_OF_CART_DOFS],
	  CommandedStiffness[NUMBER_OF_CART_DOFS],
	  CommandedDamping[NUMBER_OF_CART_DOFS],
	  CommandedPose[NUMBER_OF_FRAME_ELEMENTS],
	  OriginalPose[NUMBER_OF_FRAME_ELEMENTS],
	  LearningReferencePose[NUMBER_OF_FRAME_ELEMENTS],
	  MeasuredPose[NUMBER_OF_FRAME_ELEMENTS],
	  MeasuredJointTorques[NUMBER_OF_JOINTS];

float target_linear_pos[3],
	  target_angular_pos[3],
	  current_linear_pos[3],
	  current_angular_pos[3];


float start_pose[3] = {-0.05, 0.729, 0.0};
float new_start_pose[3] = {-0.05, 0.729, 0.0};
float linear_velocity[3] = {0.0, 0.0, 0.0};

rclcpp::Node::SharedPtr node;

class LwrFriInterface : public rclcpp::Node
{
public:
	LwrFriInterface()
	: Node("minimal_publisher"), count_(0)
	{
		publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
		timer_ = this->create_wall_timer(
		500ms, std::bind(&LwrFriInterface::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto message = std_msgs::msg::String();
		message.data = "Hello, world! " + std::to_string(count_++);
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		publisher_->publish(message);
	}


  	rclcpp::TimerBase::SharedPtr timer_;
  	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  	size_t count_;
};

rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cart_pose_pub;
rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub;

double gaussian_mu = 0.0, gaussian_sigma_square = 1.0, gaussian_sigma = 1.0, target_distance = 0.012, gaussian_a = 1.0;
double scaling_factor = 1.0, curve_integration = 0.95, v_max = 0.05, gaussian_peak = v_max;
float start_time, previous_time, current_time;
double pi = boost::math::constants::pi<double>();

// environment variables
float platform_height = -0.08; // in base frame of robot
float z_dist = 0.0;
float x_dist = target_distance;
float slice_size = 0.0025;

// Episode data collection
std::vector< std::vector< std::vector<float> > > episode_data;
std::vector < std::vector<float> > roll_out_data;
std::vector <float> single_time_step_data;

// Policy Param update variables
float updated_policy_params[8];
bool policy_params_updated = true;

// Safety params

float max_force = 12, min_force = -7;

float max_vertical_vel = 0.01;



float force_z_offset = 0.0;

/*************************************************************
                Setup functions
*/

void setup_touch_down(float touch_down_distance, float max_vel)
{
	for (int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
	{
		CommandedPose[i]	=	MeasuredPose[i];
		OriginalPose[i]	=	MeasuredPose[i];
	}
	target_linear_pos[0] = MeasuredPose[3]; 
	target_linear_pos[1] = MeasuredPose[7]; 
	target_linear_pos[2] = MeasuredPose[11] + touch_down_distance;
	max_vertical_vel = max_vel;
}

void set_gaussian_velocity_params(double v_max_, double target_distance_)
{
	v_max = v_max_;
	target_distance = target_distance_;
	start_time = current_time;
	x_dist = target_distance;
}

void setup_new_start_pose()
{
	new_start_pose[0] = new_start_pose[0] - slice_size;
}

void setup_original_pose()
{
	for (int i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
	{
		CommandedPose[i]	=	MeasuredPose[i];
		OriginalPose[i]	=	MeasuredPose[i];
	}
}

/*************************************************************
                Callbacks
*/

void event_in_cb(const std_msgs::msg::String::SharedPtr msg)
{
	if (msg->data == "e_record_sp")
	{
		start_pose[0] = MeasuredPose[3];
		start_pose[1] = MeasuredPose[7];
		start_pose[2] = MeasuredPose[11];
		new_start_pose[0] = start_pose[0];
		new_start_pose[1] = start_pose[1];
		new_start_pose[2] = start_pose[2];
		// RCLCPP_INFO("New start pose Recorded");
	}
	else if (msg->data == "e_start")
	{
		start_execution = true;
		// RCLCPP_INFO("Starting cutting rollout");			
	}
	else if (msg->data == "e_joy_control")
	{
		task_mode = JOY_CONTROL;
		// RCLCPP_INFO("Joy control enabled");		

	}
	else if (msg->data == "e_stop")
	{
		stop_program = true;
	}
	else if (msg->data == "e_go_to_start")
	{
		task_mode = GO_TO_START;
		// RCLCPP_INFO("Going to start pose");		

	}
	
}

void cmd_vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
	linear_velocity[0] = msg->twist.linear.x / 5;
	linear_velocity[1] = msg->twist.linear.y / 5;
	linear_velocity[2] = msg->twist.angular.z / 10;
	std::cout << linear_velocity[0] << " " << linear_velocity[1] << " " << linear_velocity[2] << std::endl;
}


bool get_velocity(float target_pos[3], float *velocity, float max_velocity)
{
	velocity[0] = target_pos[0] - MeasuredPose[3];
	velocity[1] = target_pos[1] - MeasuredPose[7];
	velocity[2] = target_pos[2] - MeasuredPose[11];
	float norm = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]);
	if (norm)
	{
		velocity[0] = velocity[0] * max_velocity / norm;
		velocity[1] = velocity[1] * max_velocity / norm;
		velocity[2] = velocity[2] * max_velocity / norm;
	}
	if (norm < 0.001)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

void calculate_guassian_param()
{
	scaling_factor = target_distance / curve_integration;
	gaussian_peak = v_max / scaling_factor;
	gaussian_sigma_square = 1 / (2 * pi * gaussian_peak * gaussian_peak);
	gaussian_mu = 2 * std::sqrt(gaussian_sigma_square);
	gaussian_sigma = std::sqrt(gaussian_sigma_square);
	gaussian_a = 1/std::sqrt(2 * pi * gaussian_sigma_square);

	// std::cout << "scaling_factor " << scaling_factor << std::endl;
	// std::cout << "gaussian_mu " << gaussian_mu << std::endl;
	// std::cout << "gaussian_sigma_square " << gaussian_sigma_square << std::endl;
	// RCLCPP_INFO("Gaussian velocity parameters calculated");
}

double gaussian_velocity(double time)
{
	return scaling_factor * gaussian_a * exp(- ((time - gaussian_mu) * (time - gaussian_mu)) / (2 * gaussian_sigma_square));
}

void robot_state_publisher_function()
{
	while (rclcpp::ok())
	{
		// Publisheing joint states
		
		sensor_msgs::msg::JointState js;
		js.header.stamp = node->get_clock()->now();
		js.name = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
		js.position = {JointValuesInRad[0], JointValuesInRad[1], JointValuesInRad[2], JointValuesInRad[3],
					   JointValuesInRad[4], JointValuesInRad[5], JointValuesInRad[6]};
		joint_state_pub->publish(js);
		
		geometry_msgs::msg::WrenchStamped wr;
		wr.header.stamp = node->get_clock()->now();
		wr.header.frame_id = "link_7";
		wr.wrench.force.x = EstimatedExternalCartForcesAndTorques[0];
		wr.wrench.force.y = EstimatedExternalCartForcesAndTorques[1];
		wr.wrench.force.z = EstimatedExternalCartForcesAndTorques[2];
		wr.wrench.torque.x = EstimatedExternalCartForcesAndTorques[3];
		wr.wrench.torque.y = EstimatedExternalCartForcesAndTorques[4];
		wr.wrench.torque.z = EstimatedExternalCartForcesAndTorques[5];
		wrench_pub->publish(wr);
	}

}




/*************************************************************
                Policy
*/

class CuttingPolicy
{
    public:

    float A = 0, 
          B = 0, 
          C = 0;

	// Gaussian policy params
	float var = 0.55;
	float mean = 0.0;
	std::random_device rd{};
	std::mt19937 gen{rd()};

    CuttingPolicy()
    {

    }

    void action(float& a, float& u, float s[])
    {
        u =  - A*s[0] + B* (0.25 - std::pow((0.5 - s[1]), 2)) + C*(1 - s[2]);
		std::normal_distribution<float> d(u, var); 
		a = u;
    }

	void update_params(float a, float b, float c)
	{
		A = a;
		B = b;
		C = c;
	}
}policy;
#endif