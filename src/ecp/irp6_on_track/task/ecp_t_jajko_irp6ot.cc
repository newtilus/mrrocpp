// ------------------------------------------------------------------------
//   task/ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/generator/ecp_g_force.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"
#include "ecp/irp6_on_track/task/ecp_t_jajko_irp6ot.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


// KONSTRUKTORY
jajko::jajko(lib::configurator &_config) : task(_config)
{
	ecp_m_robot = new robot (*this);

	// Powolanie czujnikow
	sensor_m[lib::SENSOR_FORCE_ON_TRACK] =
		new ecp_mp::sensor::schunk (lib::SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);

	// Konfiguracja wszystkich czujnikow
	for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);
	sr_ecp_msg->message("ECP jajeczne irp6ot loaded");
};


void jajko::main_task_algorithm(void)
{
	for(;;)
	{
		uint8_t tryb;

		// Wybor trybu zadania - rozbijanie jajka badz nie
		tryb = choose_option ("1 - Egg destroying, 2 - Egg is safe", 2);
		if ( tryb == lib::OPTION_ONE )
		{
			tryb=0;
		} else if ( tryb == lib::OPTION_TWO )
		{
			tryb=1;
		} else if ( tryb == lib::QUIT)
		{
			break;
		}

		if (yefg!=NULL) delete yefg;
		yefg = new common::generator::y_egg_force (*this, 8, tryb);
		yefg->sensor_m = sensor_m;

		sr_ecp_msg->message("NOWA SERIA");
		yefg->Move();
	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::jajko(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
