/*
 * ecp_t_pb_sac_calibration.h
 *
 *  Created on: 14-12-2010
 *      Author: mboryn
 */

#ifndef ECP_T_SAC_CALIBRATION_H_
#define ECP_T_SAC_CALIBRATION_H_

#include "sensor/discode/discode_sensor.h"
#include <boost/shared_ptr.hpp>
#include "../single_visual_servo_manager.h"

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace task {

class ecp_t_pb_sac_calibration : public mrrocpp::ecp::common::task::task
{
public:
	ecp_t_pb_sac_calibration(mrrocpp::lib::configurator& config);
	virtual ~ecp_t_pb_sac_calibration();

	void main_task_algorithm();

private:
	boost::shared_ptr<mrrocpp::ecp_mp::sensor::discode::discode_sensor> ds;
	boost::shared_ptr <mrrocpp::ecp::common::generator::single_visual_servo_manager> sm;
	boost::shared_ptr <mrrocpp::ecp::servovision::visual_servo> vs;
};

} // namespace task
}
}
}

#endif /* ECP_T_SAC_CALIBRATION_H_ */
