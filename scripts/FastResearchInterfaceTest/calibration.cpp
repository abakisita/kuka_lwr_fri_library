#include <FastResearchInterface.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <ft_calib.h>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK		2000

#define SIZE_OF_TRANSFER_STRING					32

//*******************************************************************************************
// main()
FastResearchInterface *FRI;

double joint_7_values[10][7] = {{0.0, 45.0, 0.0, 72.0, 0.0, -61.0, 0.0},
								{0.0, 20.0, 40.0, -20.0, 0.0, 20.0, 0.0},
								{0.0, 20.0, 20.0, -20.0, 0.0, 20.0, 20.0},
								{45.0, 20.0, 40.0, -20.0, 0.0, 20.0, 20.0},
								{45.0, 20.0, 20.0, -20.0, 0.0, 20.0, 45.0},
								{45.0, 20.0, 40.0, -20.0, 0.0, 20.0, 90.0},
								{45.0, 20.0, 20.0, -20.0, 0.0, 45.0, 0.0},
								{45.0, 20.0, 40.0, -20.0, 0.0, 0.0, 45.0},
								{45.0, 20.0, 20.0, -20.0, 0.0, -60.0, -45.0},
								{45.0, 20.0, 20.0, -20.0, 0.0, -60.0, -45.0},
								};

int pos_cnt = 0;
bool move_ = true;

int joint_increments[7] = {0, 0, 0, 0, 0, 0, 0};
bool joy_control = false;
bool fast_mode = false;

ros::Publisher joint_state_pub;

void move_next(FastResearchInterface *FRI)
{
		float joint_values[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		if (pos_cnt < 10)
		{
			std::cout << "Position " << pos_cnt << std::endl;
			for (int itrc = 0; itrc < 7; itrc++)
			{
				joint_values[itrc] = joint_7_values[pos_cnt][itrc];

			}
			RunTrajectorySimple(FRI, joint_values);
			std::cout << "Done " << std::endl; 
		}
		pos_cnt += 1;
}

void move_next_cb(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "We are Here" << msg->data << std::endl;
	if (msg->data == "go")
		move_ = true;
}


void joint_state_publisher_function()
{
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		float joint_positions[7];
		FRI->GetMeasuredJointPositions(joint_positions);

		// Publisheing joint states
		
		sensor_msgs::JointState js;
		js.header.stamp = ros::Time::now();
		js.name = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
		js.position = {joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3],
			joint_positions[4], joint_positions[5], joint_positions[6]};
		joint_state_pub.publish(js);
		loop_rate.sleep();
	}

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "calib");
	ros::NodeHandle nh;
	float gravity_vector[3];
	float pose_vector[12];
	float joint_positions[7];
	float ft[6];
	
	ros::Subscriber move_sub = nh.subscribe("/move_next", 10, move_next_cb);
  	joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  	ros::Publisher move_done_pub = nh.advertise<std_msgs::String>("/move_done", 1);
	ros::Duration(3.0).sleep();
	ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/wrench_raw", 1);

	Calibration::FTCalib ft_cal;
	std::cout << "BreakPoint 1" << std::endl;
	KDL::Rotation ee_rot(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	KDL::Vector ee_trans(0.0, 0.0, 0.0);
	KDL::Vector base_gravity(0.0, 0.0, -9.810834);
	KDL::Vector ee_gravity(0.0, 0.0, -9.810834);
	KDL::Frame ee_frame(ee_rot, ee_trans);
  	ros::Rate loop_rate(200);

	bool Run = true, 
		 StartRobotCalled = false;
	char c = 0,	
		 d = 0, 
		 TransferString[SIZE_OF_TRANSFER_STRING];

	unsigned int ControlScheme = FastResearchInterface::JOINT_POSITION_CONTROL,	
				 i = 0,	
				 LoopValue = 0;

	int	ResultValue = 0;

	float FloatValues[SIZE_USER_DATA], 
		  TmpFloatValues[SIZE_USER_DATA], 
		  DesiredTorqueValues[NUMBER_OF_JOINTS], 
		  JointStiffnessValues[NUMBER_OF_JOINTS],	
		  JointDampingValues[NUMBER_OF_JOINTS], 
		  CartStiffnessValues[NUMBER_OF_CART_DOFS], 
		  CartDampingValues[NUMBER_OF_CART_DOFS];
	

	memset(TransferString, 0x0, SIZE_OF_TRANSFER_STRING * sizeof(char));
	memset(FloatValues, 0x0, SIZE_USER_DATA * sizeof(float));
	memset(TmpFloatValues, 0x0, SIZE_USER_DATA * sizeof(float));
	memset(DesiredTorqueValues, 0x0, NUMBER_OF_JOINTS * sizeof(float));


	fprintf(stdout, "You may need superuser permission to run this program.\n");
	fflush(stdout);
	FRI = new FastResearchInterface("/home/padalkar/FRILibrary/src/FastResearchInterfaceLibrary/etc/980039-FRI-Driver.init");
	std::cout << "BreakPoint 4" << std::endl;
		ros::spinOnce();

	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		JointStiffnessValues	[i] =	(float)10.0;
		JointDampingValues		[i]	=	(float)0.7;
	}
		ros::spinOnce();

	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		CartStiffnessValues		[i]	=	(float)10.0;
		CartDampingValues		[i]	=	(float)0.7;
	}
	FRI->SetCommandedCartDamping(CartStiffnessValues);
	FRI->SetCommandedCartStiffness(CartDampingValues);
	FRI->SetCommandedJointDamping(JointDampingValues);
	FRI->SetCommandedJointStiffness(JointStiffnessValues);
	std::cout << "BreakPoint 5" << std::endl;
		ros::spinOnce();

	ControlScheme =	FastResearchInterface::JOINT_POSITION_CONTROL;
	ResultValue	= FRI->StartRobot(ControlScheme);
	std::cout << "BreakPoint 6" << std::endl;
		ros::spinOnce();

	if (ResultValue != EOK)
	{
		std::cout << "something went wrong" << std::endl;
	}
	std::thread joint_state_publisher_thread(joint_state_publisher_function);

	float joint_[7];
	while (ros::ok())
	{
		ros::spinOnce();
		FRI->WaitForKRCTick();
		if (true)
		{
			if (move_ == true)
			{
				move_next(FRI);
				move_ = false;
				ros::Duration(2.0).sleep();
				std_msgs::String str_msg;
				str_msg.data = "done";
				move_done_pub.publish(str_msg);
			}
			FRI->GetEstimatedExternalCartForcesAndTorques(ft);
			std::cout << "FT data " ;
			for (int i = 0; i<6; i++)
			{
				std::cout  << ft[i]<< "   ";
			}
			std::cout << std::endl; 
			geometry_msgs::WrenchStamped wr;
			wr.header.frame_id = "link_7";
			wr.wrench.force.x = ft[0];
			wr.wrench.force.y = ft[1];
			wr.wrench.force.z = ft[2];
			wr.wrench.torque.x = ft[3];
			wr.wrench.torque.y = ft[4];
			wr.wrench.torque.z = ft[5];
			wrench_pub.publish(wr);
		}
	}
	delete FRI;
	printf("\nGood bye.\n\n");
	joint_state_publisher_thread.join();
	return(EXIT_SUCCESS);
}
