/*
 * generator/ecp_g_en_get_position.cc
 *
 *Author: Emil Natil
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <iostream>

#include "base/lib/typedefs.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_en_get_position.h"


namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

en_get_position::en_get_position(common::task::task& _ecp_task) :
		common::generator::get_position(_ecp_task, lib::ECP_XYZ_EULER_ZYZ, 6)

{
	generator_name = ecp_mp::generator::ECP_GEN_EN_GET_POSITION;
	calibration_points = 0;
}

void en_get_position::conditional_execution()
{
	Move();

	std::vector <double> position_vector;
	position_vector = get_position_vector();

	if(!position_vector.size()) {
		sr_ecp_msg.message("get_position_vector is empty!");
	}
	std::string position_str;
	position_str = "XYZ_EULER_ZYZ\n1\nABSOLUTE\n0.1 0.1 0.1 0.1 0.1 0.1\n0.07 0.07 0.07 0.07 0.07 0.07\n";
	for(size_t i = 0; i < position_vector.size(); ++i)
	{
		std::stringstream stream;
		stream << position_vector[i];
		position_str += stream.str();
		position_str += " ";
	}
	position_str += "\n";


	char* filename;
	if(calibration_points == 0)
	{
		std::cout << "First calibration point:" << std::endl << position_str << std::endl;
		filename = "en_first_point.trj";
		calibration_points++;
	}
	else if(calibration_points == 1)
	{
		std::cout << "Second calibration point:" << std::endl << position_str << std::endl;
		filename = "en_second_point.trj";
		calibration_points++;
	}
	else if(calibration_points == 2)
	{
		std::cout << "Third calibration point:" << std::endl << position_str << std::endl;
		filename = "en_third_point.trj";
		calibration_points++;
	}
	else
	{
		std::cout << "Fourth calibration point:" << std::endl << position_str << std::endl;
		filename = "en_fourth_point.trj";
		calibration_points = 0;
	}


	std::ofstream to_file;
	to_file.open(filename);
	to_file << position_str;
	to_file.close();

}


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

