#include <cstdio>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <string>

#include "mp_en_labyrinth.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"
#include "generator/ecp/bias_edp_force/ecp_g_bias_edp_force.h"
#include "generator/ecp/tff_nose_run/ecp_g_tff_nose_run.h"

#include "base/mp/mp_task.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/irp6_tfg/dp_tfg.h"
#include "ecp_mp_g_en_labyrinth.h"
#include "ecp_mp_g_en_get_position.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"

#include "sensor/discode/discode_sensor.h"
#include <boost/shared_ptr.hpp>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "EN_Labyrinth_Reading.hpp"

using mrrocpp::ecp_mp::sensor::discode::discode_sensor;

using namespace std;

namespace mrrocpp {
namespace mp {
namespace task {
task* return_created_mp_task(lib::configurator &_config)
{
	return new mp_en_labyrinth(_config);
}

void mp_en_labyrinth::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
}

mp_en_labyrinth::mp_en_labyrinth(lib::configurator &_config) :
		task(_config)
{
	sr_ecp_msg->message("EN_Labirynth");

	char config_section_name[] = { "[EN_Labirynth]" };
	discode =
			boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor>(new mrrocpp::ecp_mp::sensor::discode::discode_sensor(config, config_section_name));

	sr_ecp_msg->message("Configuring DisCODe sensor");
	discode->configure_sensor();

	ERROR = false;
}

void mp_en_labyrinth::main_task_algorithm(void)
{
	sr_ecp_msg->message("mp start");

	lib::robot_name_t manipulator_name;
	lib::robot_name_t gripper_name;

	manipulator_name = lib::irp6p_m::ROBOT_NAME;
	gripper_name = lib::irp6p_tfg::ROBOT_NAME;


	Types::Mrrocpp_Proxy::EN_Labyrinth_Reading reading;

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "XYZ_ANGLE_AXIS 0.608 2.457 0.15 -1.517 2.7 -0.094", lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

	cout << endl << endl << "Do you want to calibrate manipulator with the labyrinth?" << endl;
	char reply;
	while (true) {
	    cout << "Calibrate manipulator with the labyrinth? [y/n]" << endl;
	    cin >> reply;

	    if ((reply == 'y') || (reply == 'n')) {
	        break;
	    }
	}

	if(reply == 'y')
	{
		// ENABLING MANIPULATOR TO BE MOVED TO EACH CORNER

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_BIAS_EDP_FORCE, (int) 5, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		sr_ecp_msg->message("Move the manipulator to the first corner.");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, true, true, true, true), lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_GET_POSITION, 0, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		sr_ecp_msg->message("Move the manipulator to the second corner.");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, true, true, true, true), lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_GET_POSITION, 0, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		sr_ecp_msg->message("Move the manipulator to the third corner.");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, true, true, true, true), lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_GET_POSITION, 0, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		sr_ecp_msg->message("Move the manipulator to the fourth corner.");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN, (int) ecp_mp::generator::tff_nose_run::behaviour_specification, ecp_mp::generator::tff_nose_run::behaviour_specification_data_type(true, true, true, true, true, true), lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_GET_POSITION, 0, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
	}



	// READ CORNERS FROM FILES
	string position_corner1, position_corner2, position_corner3, position_corner4;

	std::ifstream from_file;

	from_file.open("en_first_point.trj");
	if(!from_file)
	{
		sr_ecp_msg->message("Cannot open file en_first_point.trj! Aborting.");
		return;
	}
	getline(from_file, position_corner1); getline(from_file, position_corner1); getline(from_file, position_corner1); getline(from_file, position_corner1); getline(from_file, position_corner1); getline(from_file, position_corner1);
	cout << position_corner1 << endl;
	from_file.close();

	from_file.open("en_second_point.trj");
	if(!from_file)
	{
		sr_ecp_msg->message("Cannot open file en_second_point.trj! Aborting.");
		return;
	}
	getline(from_file, position_corner2); getline(from_file, position_corner2); getline(from_file, position_corner2); getline(from_file, position_corner2); getline(from_file, position_corner2); getline(from_file, position_corner2);
	cout << position_corner2 << endl;
	from_file.close();

	from_file.open("en_third_point.trj");
	if(!from_file)
	{
		sr_ecp_msg->message("Cannot open file en_third_point.trj! Aborting.");
		return;
	}
	getline(from_file, position_corner3); getline(from_file, position_corner3); getline(from_file, position_corner3); getline(from_file, position_corner3); getline(from_file, position_corner3); getline(from_file, position_corner3);
	cout << position_corner3 << endl;
	from_file.close();

	from_file.open("en_fourth_point.trj");
	if(!from_file)
	{
		sr_ecp_msg->message("Cannot open file en_fourth_point.trj! Aborting.");
		return;
	}
	getline(from_file, position_corner4); getline(from_file, position_corner4); getline(from_file, position_corner4); getline(from_file, position_corner4); getline(from_file, position_corner4); getline(from_file, position_corner4);
	cout << position_corner4 << endl;
	from_file.close();


	// CONVERTING CORNERS COORDINATES FROM STRING TO DOUBLES
	std::vector<double> position_corner1_v, position_corner2_v, position_corner3_v, position_corner4_v;
	istringstream iss1(position_corner1);
	do
	{
		double coordinate;
		iss1 >> coordinate;
		position_corner1_v.push_back(coordinate);
	} while(iss1);
	istringstream iss2(position_corner2);
	do
	{
		double coordinate;
		iss2 >> coordinate;
		position_corner2_v.push_back(coordinate);
	} while(iss2);
	istringstream iss3(position_corner3);
	do
	{
		double coordinate;
		iss3 >> coordinate;
		position_corner3_v.push_back(coordinate);
	} while(iss3);
	istringstream iss4(position_corner4);
	do
	{
		double coordinate;
		iss4 >> coordinate;
		position_corner4_v.push_back(coordinate);
	} while(iss4);


	// CALCULATING RELATIVE MOVES BASED ON COORDINATES OF THE CORNERS
//	double relative_move_right_x = (position_corner2_v[0]-position_corner1_v[0]) / DIMENSION_X;
//	double relative_move_left_x = -1 * relative_move_right_x;
//	double relative_move_right_y = (position_corner2_v[1]-position_corner1_v[1]) / DIMENSION_X;
//	double relative_move_left_y = -1 * relative_move_right_y;
//	double relative_move_up_x = (position_corner4_v[0]-position_corner1_v[0]) / DIMENSION_Y;
//	double relative_move_down_x = -1 * relative_move_up_x;
//	double relative_move_up_y = (position_corner4_v[1]-position_corner1_v[1]) / DIMENSION_Y;
//	double relative_move_down_y = -1 * relative_move_up_y;

	double relative_move_down_x = (position_corner2_v[0]-position_corner1_v[0]) / (DIMENSION_Y-1);
	double relative_move_down_y = (position_corner2_v[1]-position_corner1_v[1]) / (DIMENSION_Y-1);
	double relative_move_up_x = -1 * relative_move_down_x;
	double relative_move_up_y = -1 * relative_move_down_y;
	double relative_move_left_x = (position_corner4_v[0]-position_corner1_v[0]) / (DIMENSION_X-1);
	double relative_move_left_y = (position_corner4_v[1]-position_corner1_v[1]) / (DIMENSION_X-1);
	double relative_move_right_x = -1 * relative_move_left_x;
	double relative_move_right_y = -1 * relative_move_left_y;

	stringstream ss1, ss2, ss3, ss4, ss5, ss6, ss7, ss8;
	string tmp, relative_move_up, relative_move_down, relative_move_left, relative_move_right;
	ss1 << relative_move_right_x;
	ss1 >> tmp;
	relative_move_right = tmp + " ";
	tmp = "";
	ss2 << relative_move_right_y;
	ss2 >> tmp;
	relative_move_right += tmp;
	tmp = "";
	relative_move_right += " 0.0 0.0 0.0 0.0";

	ss3 << relative_move_left_x;
	ss3 >> tmp;
	relative_move_left = tmp + " ";
	tmp = "";
	ss4 << relative_move_left_y;
	ss4 >> tmp;
	relative_move_left += tmp;
	relative_move_left += " 0.0 0.0 0.0 0.0";

	ss5 << relative_move_up_x;
	ss5 >> tmp;
	relative_move_up = tmp + " ";
	tmp = "";
	ss6 << relative_move_up_y;
	ss6 >> tmp;
	relative_move_up += tmp;
	relative_move_up += " 0.0 0.0 0.0 0.0";

	ss7 << relative_move_down_x;
	ss7 >> tmp;
	relative_move_down = tmp + " ";
	tmp = "";
	ss8 << relative_move_down_y;
	ss8 >> tmp;
	relative_move_down += tmp;
	relative_move_down += " 0.0 0.0 0.0 0.0";

	cout << "Generated relative down right:"<< endl << relative_move_right_x << " " << relative_move_right_y << endl;
	cout << relative_move_right << endl;
	cout << "Generated relative up up:"<< endl << relative_move_up_x << " " << relative_move_up_y << endl;
	cout << relative_move_up << endl;
	cout << "Generated relative up left:"<< endl << relative_move_left_x << " " << relative_move_left_y << endl;
	cout << relative_move_left << endl;
	cout << "Generated relative down down:"<< endl << relative_move_down_x << " " << relative_move_down_y << endl;
	cout << relative_move_down << endl;


	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "XYZ_EULER_ZYZ "+position_corner2, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
//	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "XYZ_EULER_ZYZ "+position_corner3, lib::irp6p_m::ROBOT_NAME);
//	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
//	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "XYZ_EULER_ZYZ "+position_corner4, lib::irp6p_m::ROBOT_NAME);
//	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "XYZ_EULER_ZYZ "+position_corner1, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);


//	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "RELATIVE_EULER "+relative_move_right, lib::irp6p_m::ROBOT_NAME);
//	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
//	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "RELATIVE_EULER "+relative_move_up, lib::irp6p_m::ROBOT_NAME);
//	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
//	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "RELATIVE_EULER "+relative_move_left, lib::irp6p_m::ROBOT_NAME);
//	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
//	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "RELATIVE_EULER "+relative_move_down, lib::irp6p_m::ROBOT_NAME);
//	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);



	sr_ecp_msg->message("reading init");
	reading = discode->call_remote_procedure <Types::Mrrocpp_Proxy::EN_Labyrinth_Reading>(double(29.0386));
	sr_ecp_msg->message("reading received");

//	while(1)
//	{

//		if(!reading.labyrinth_solved)
//			continue;

	cout << "Reading info: " << endl;
	cout << "Solved: " << reading.labyrinth_solved << endl;
	cout << "Path Size: " << reading.path_size << endl;
	cout << "Start_pt: (" << reading.start_point_x << "," << reading.start_point_y << ")" << endl;
	cout << "End_pt (" << reading.end_point_x << "," << reading.end_point_y << ")" << endl;
	cout << "Path: ";
	for (int i = 0; i < reading.path_size; ++i)
		cout << reading.path[i] << " ";
	cout << endl;

//		// TODO better read the data then make up...
//		bool labyrinth_solved_static = true;
//		int path_size_static = 10;
//		int start_point_x_static = 1;
//		int start_point_y_static = 1;
//		int end_point_x_static = 8;
//		int end_point_y_static = 8;
//		int path_static[10] = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1};
//


	sr_ecp_msg->message("Moving to the ending point");
	cout << "Ending point: (" << reading.end_point_x << "," << reading.end_point_y << ")" << endl;
	std::string relative_move;
	bool error = false;
	for (int i = 0; i < reading.path_size && !error; ++i)
	{
		cout << "Point " << i << " is " << reading.path[i] << endl;
		switch (reading.path[i])
		{
			case UP:
				relative_move = "RELATIVE_EULER "+relative_move_up;
				break;
			case DOWN:
				relative_move = "RELATIVE_EULER "+relative_move_down;
				break;
			case LEFT:
				relative_move = "RELATIVE_EULER "+relative_move_left;
				break;
			case RIGHT:
				relative_move = "RELATIVE_EULER "+relative_move_right;
				break;
			default:
				error = true;
				sr_ecp_msg->message("Path reading from DisCODe contains error in direction description!");
				break;
		}
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, relative_move, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);
	}

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_EN_LABYRINTH, (int) 5, "XYZ_EULER_ZYZ "+position_corner1, lib::irp6p_m::ROBOT_NAME);
	wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);


//	}

	sr_ecp_msg->message("mp end");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp

