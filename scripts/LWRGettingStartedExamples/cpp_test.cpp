//  ---------------------- Doxygen info ----------------------
//! \file LWRCartImpedanceControlExample.cpp
//!
//! \brief
//! Sample application for the class LWRCartImpedanceController
//!
//! \details
//! This simple application feature a sample of how to use the
//! Cartesian impedance controller of the KUKA Fast Research Interface
//! for the Light-Weight Robot IV. For details about the actual
//! interface class (i.e., class LWRCartImpedanceController), please
//! refer to the file LWRCartImpedanceController.h.
//!
//! \date December 2014
//!
//! \version 1.2
//!
//!	\author Torsten Kroeger, tkr@stanford.edu\n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//! \n
//! \n
//! \copyright Copyright 2014 Stanford University\n
//! \n
//! Licensed under the Apache License, Version 2.0 (the "License");\n
//! you may not use this file except in compliance with the License.\n
//! You may obtain a copy of the License at\n
//! \n
//! http://www.apache.org/licenses/LICENSE-2.0\n
//! \n
//! Unless required by applicable law or agreed to in writing, software\n
//! distributed under the License is distributed on an "AS IS" BASIS,\n
//! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n
//! See the License for the specific language governing permissions and\n
//! limitations under the License.\n
//! 
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include <pthread.h>
#include <mutex>
#include <thread>
// #include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iomanip>
#include <chrono>
#include <ctime>

#include <boost/math/distributions/normal.hpp>
#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>
#include "matplotlibcpp.h"

#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS			7
#endif

#ifndef NUMBER_OF_CART_DOFS
#define NUMBER_OF_CART_DOFS			6
#endif

#ifndef NUMBER_OF_FRAME_ELEMENTS
#define NUMBER_OF_FRAME_ELEMENTS	12
#endif

#define RUN_TIME_IN_SECONDS			30.0

float JointValuesInRad[NUMBER_OF_JOINTS];

double gaussian_mu = 0.0, gaussian_sigma_square = 1.0, gaussian_sigma = 1.0, target_distance = 0.15, gaussian_a = 1.0;
double scaling_factor = 1.0, curve_integration = 0.95, v_max = 0.1, gaussian_peak = v_max;
double pi = boost::math::constants::pi<double>();
void calculate_guassian_param()
{
	scaling_factor = target_distance / curve_integration;
	gaussian_peak = v_max / scaling_factor;
	gaussian_sigma_square = 1 / (2 * pi * gaussian_peak * gaussian_peak);
	gaussian_mu = 2 * std::sqrt(gaussian_sigma_square);
	gaussian_sigma = std::sqrt(gaussian_sigma_square);
	gaussian_a = 1/std::sqrt(2 * pi * gaussian_sigma_square);
	std::cout << "scaling_factor " << scaling_factor << std::endl;
	std::cout << "gaussian_mu " << gaussian_mu << std::endl;
	std::cout << "gaussian_sigma_square " << gaussian_sigma_square << std::endl;
}

double gaussian_velocity(double time)
{
	return gaussian_a * exp(- ((time - gaussian_mu) * (time - gaussian_mu)) / (2 * gaussian_sigma_square));
}

class RadialBasisFunction
{
public:
	int number_of_basis_functions;
	float variance;
	float amplitude;
	int number_of_inputs;
	std::vector<float> mean;
	Eigen::MatrixXf input_layer_weights;
	Eigen::MatrixXf output_layer_weights;

	RadialBasisFunction(int  number_of_basis_functions, float variance, int number_of_inputs) : number_of_basis_functions(number_of_basis_functions), 
																		  variance(variance),
																		  number_of_inputs(number_of_inputs)

	{
		srand((unsigned int) time(0));
		mean.resize(number_of_basis_functions);
		input_layer_weights = Eigen::MatrixXf::Random(number_of_basis_functions, number_of_inputs);
    	output_layer_weights = Eigen::MatrixXf::Random(number_of_basis_functions, 1);

		for (size_t i = 0; i < number_of_basis_functions; i++)
		{
			mean[i] = (i + 1) / (number_of_basis_functions + 1);

		}
		amplitude = 1/std::sqrt(2 * pi * variance * variance);

	}

	double gaussian(double mean, double x)
	{
		return amplitude * exp(- ((x - mean) * (x - mean)) / (2 * variance * variance));
	}

	float evaluate(Eigen::VectorXf input)
	{
		Eigen::MatrixXf first_layer_ip = (input_layer_weights * input);
		float sum = first_layer_ip.sum();
		first_layer_ip = first_layer_ip / sum;
		Eigen::MatrixXf fisrt_layer_output(1, number_of_basis_functions);
		for (size_t i = 0; i < number_of_basis_functions; i++)
		{
			fisrt_layer_output(0, i) = gaussian(mean[i], first_layer_ip(i, 0));
		}
		Eigen::MatrixXf output = fisrt_layer_output * output_layer_weights;
		return output(0, 0);
	}
};





//*******************************************************************************************
// main()
//
int main(int argc, char *argv[])
{
	// calculate_guassian_param();
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;
	// ros::Publisher gauss_pub = nh.advertise<std_msgs::Float64>("/gauss_value", 1);
	// auto t_start = std::chrono::high_resolution_clock::now();
	// auto t_end = std::chrono::high_resolution_clock::now();
	// while(std::chrono::duration<double, std::milli>(t_end-t_start).count() < (gaussian_mu + 2 * gaussian_sigma) * 1000)
	// {
	// 	t_end = std::chrono::high_resolution_clock::now();
	// 	std_msgs::Float64 msg;
	// 	double dt = std::chrono::duration<double, std::milli>(t_end-t_start).count() / 1000;
	// 	msg.data = gaussian_velocity(dt) * scaling_factor;
	// 	std::cout << msg.data << std::endl;
	// 	gauss_pub.publish(msg);
	// }
	RadialBasisFunction rbf(10, 1, 4);
	std::vector <float> v1;
	std::vector <float> v2;
	std::vector <float> v3;
	std::vector <float> v4;

	for (size_t i = 0; i < 200; i++)
	{	
		// ip = Eigen::MatrixXf::Random(4,1);
		Eigen::MatrixXf ip(4,1);
		ip << i * 0.001, i * 0.001, i * 0.001, i * 0.001;
		float val = rbf.evaluate(ip);
		std::cout << ip(0,0) << std::endl;
		std::cout << val << std::endl;
		v1.push_back(val);
	}
	
	matplotlibcpp::plot(v1);
    matplotlibcpp::show();
}
