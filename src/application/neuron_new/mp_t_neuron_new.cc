/**
 * @file mp_t_Neuron_new.h
 * @brief Header file for Neuron_new class.
 * @author Tomasz Bem (mebmot@wp.pl)
 * @ingroup Neuron_new
 * @date 25.06.2010
 */

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/mp/mp_task.h"

#include "mp_t_neuron_new.h"
#include "ecp_mp_t_neuron_new.h"

#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

namespace mrrocpp {
namespace mp {
namespace task {

/*=======================return_created_mp_task===========================*//**
 * @brief returns inherited task pointer.
 * @param _config configurator object reference.
 * @return inherited task pointer.
 */
task* return_created_mp_task(lib::configurator& _config)
{
	return new Neuron_new(_config);
}

/*================================constructor=============================*//**
 * @brief Constructor, with task configurator.
 * @param _config configurator object reference.
 */
Neuron_new::Neuron_new(lib::configurator &_config) :
		task(_config)
{
}

/*================================create_robots===========================*//**
 * @brief Brings robots into being.
 * @details Which robots are bring into being depends on Neuron_new.ini
 * configuration file.
 */
void Neuron_new::create_robots()
{
	//ACTIVATE_MP_ROBOT(conveyor);
	//
	//
	//ACTIVATE_MP_ROBOT(polycrank);
	//ACTIVATE_MP_ROBOT(bird_hand);
	//ACTIVATE_MP_ROBOT(spkm);
	//ACTIVATE_MP_ROBOT(smb);
	//ACTIVATE_MP_ROBOT(shead);
	//ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT( irp6ot_m);
	//ACTIVATE_MP_ROBOT(irp6p_tfg);
	//ACTIVATE_MP_ROBOT(irp6p_m);
	//ACTIVATE_MP_ROBOT(sarkofag);

	//ACTIVATE_MP_DEFAULT_ROBOT(electron);
	//ACTIVATE_MP_DEFAULT_ROBOT(speechrecognition);
	//ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

/*===============================main_task_algorithm======================*//**
 * @brief Main taks algorithm.
 * @details It initializes Neuron_new subtask.
 */
void Neuron_new::main_task_algorithm(void)
{
	sr_ecp_msg->message("Neuron task initialization");

	set_next_ecp_state(ecp_mp::task::ECP_T_NEURON_NEW, 5, "", lib::irp6ot_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6ot_m::ROBOT_NAME);

	sr_ecp_msg->message("END");
}

Neuron_new::~Neuron_new()
{
}

} //task
} //mp
} //mrrocpp
