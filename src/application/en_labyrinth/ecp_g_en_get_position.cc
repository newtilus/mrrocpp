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
		common::generator::get_position(_ecp_task, lib::ECP_XYZ_ANGLE_AXIS, 6)

{
	generator_name = ecp_mp::generator::ECP_GEN_EN_GET_POSITION;
}

void en_get_position::conditional_execution()
{
	Move();

	std::vector <double> position_vector;
	position_vector = get_position_vector();

	if(!position_vector.size()) {
		sr_ecp_msg.message("get_position_vector is empty!");
	}

	std::cout << "get_position generator:" << std::endl;
	for(size_t i = 0; i < position_vector.size(); ++i) {
		std::cout << position_vector[i] << std::endl;
	}
	std::cout << std::endl;
}


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

