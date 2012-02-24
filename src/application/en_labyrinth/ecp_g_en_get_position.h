/*
 * generator/ecp_g_en_get_position.h
 *
 *Author: Emil Natil
 */

#ifndef ECP_G_EN_GET_POSITION_H_
#define ECP_G_EN_GET_POSITION_H_

#include <ctime>

#include "base/ecp/ecp_generator.h"
#include "generator/ecp/get_position/ecp_g_get_position.h"
#include "ecp_mp_g_en_get_position.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

class en_get_position : public common::generator::get_position
{
public:
	en_get_position(common::task::task& _ecp_task); //constructor

	void conditional_execution();
private:
	int calibration_points; // the number of calibration points set for the labyrinth
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* ECP_G_EN_GET_POSITION_H_ */
