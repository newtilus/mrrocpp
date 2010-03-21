/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/netmgr.h>
#include <errno.h>
#include <process.h>
#include <math.h>

#include <boost/bind.hpp>

#include "lib/srlib.h"
#include "ui/ui_const.h"
// #include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"
#include "ui/ui_ecp.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern function_execution_buffer edp_irp6p_tfg_eb;
extern ui_state_def ui_state;
extern lib::configurator* config;
extern ui_msg_def ui_msg;
extern ui_robot_def ui_robot;

double irp6p_tfg_current_pos[IRP6P_TFG_NUM_OF_SERVOS];// pozycja biezaca
double irp6p_tfg_desired_pos[IRP6P_TFG_NUM_OF_SERVOS]; // pozycja zadana



int
close_wind_irp6p_tfg_moves( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_irp6p_tfg_moves_open)
	{
		PtDestroyWidget( ABW_wnd_irp6p_tfg_moves );
	}

	return( Pt_CONTINUE );

	}

int
close_wnd_irp6p_tfg_servo_algorithm( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (ui_state.is_wind_irp6p_tfg_servo_algorithm_open)
	{
		PtDestroyWidget ( ABW_wnd_irp6p_tfg_servo_algorithm );
	}

	return( Pt_CONTINUE );

	}


int EDP_irp6p_tfg_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	edp_irp6p_tfg_eb.command(boost::bind(EDP_irp6p_tfg_create_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_irp6p_tfg_create_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	//	sleep(10);
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_PROCESS_CREATION);

	try { // dla bledow robot :: ECP_error

		// dla robota irp6p_tfg
		if (ui_state.irp6p_tfg.edp.state == 0) {

			ui_state.irp6p_tfg.edp.state = 0;
			ui_state.irp6p_tfg.edp.is_synchronised = false;

			std::string tmp_string("/dev/name/global/");
			tmp_string += ui_state.irp6p_tfg.edp.hardware_busy_attach_point;

			std::string tmp2_string("/dev/name/global/");
			tmp2_string += ui_state.irp6p_tfg.edp.network_resourceman_attach_point;

			// sprawdzenie czy nie jest juz zarejestrowany zarzadca zasobow
			if ((!(ui_state.irp6p_tfg.edp.test_mode)) && (access(tmp_string.c_str(), R_OK) == 0)
					|| (access(tmp2_string.c_str(), R_OK) == 0)) {
				ui_msg.ui->message(lib::NON_FATAL_ERROR, "edp_irp6p_tfg already exists");
			} else if (check_node_existence(ui_state.irp6p_tfg.edp.node_name, std::string("edp_irp6p_tfg"))) {

				ui_state.irp6p_tfg.edp.node_nr = config->return_node_number(ui_state.irp6p_tfg.edp.node_name);

				ui_state.irp6p_tfg.edp.state = 1;

				ui_robot.irp6p_tfg = new ui_tfg_and_conv_robot(*config, *ui_msg.all_ecp, lib::ROBOT_IRP6P_TFG);

				ui_state.irp6p_tfg.edp.pid = ui_robot.irp6p_tfg->ecp->get_EDP_pid();

				if (ui_state.irp6p_tfg.edp.pid < 0) {

					ui_state.irp6p_tfg.edp.state = 0;
					fprintf(stderr, "EDP spawn failed: %s\n", strerror(errno));
					delete ui_robot.irp6p_tfg;
				} else { // jesli spawn sie powiodl

					short tmp = 0;
					// kilka sekund  (~1) na otworzenie urzadzenia

					while ((ui_state.irp6p_tfg.edp.reader_fd
							= name_open(ui_state.irp6p_tfg.edp.network_reader_attach_point.c_str(), NAME_FLAG_ATTACH_GLOBAL))
							< 0)
						if ((tmp++) < CONNECT_RETRY) {
							delay(CONNECT_DELAY);
						} else {
							perror("blad odwolania do READER_OT");
							break;
						}

					// odczytanie poczatkowego stanu robota (komunikuje sie z EDP)
					lib::controller_state_t robot_controller_initial_state_tmp;

					ui_robot.irp6p_tfg->get_controller_state(robot_controller_initial_state_tmp);

					//ui_state.irp6p_tfg.edp.state = 1; // edp wlaczone reader czeka na start

					ui_state.irp6p_tfg.edp.is_synchronised = robot_controller_initial_state_tmp.is_synchronised;
				}
			}
		}

	} // end try

	CATCH_SECTION_UI

	manage_interface();

	return 1;
}

int EDP_irp6p_tfg_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	edp_irp6p_tfg_eb.command(boost::bind(EDP_irp6p_tfg_slay_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_irp6p_tfg_slay_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{
	int pt_res;
	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;
	// dla robota irp6p_tfg
	if (ui_state.irp6p_tfg.edp.state > 0) { // jesli istnieje EDP
		if (ui_state.irp6p_tfg.edp.reader_fd >= 0) {
			if (name_close(ui_state.irp6p_tfg.edp.reader_fd) == -1) {
				fprintf(stderr, "UI: EDP_irp6ot, %s:%d, name_close(): %s\n", __FILE__, __LINE__, strerror(errno));
			}
		}
		delete ui_robot.irp6p_tfg;
		ui_state.irp6p_tfg.edp.state = 0; // edp wylaczone
		ui_state.irp6p_tfg.edp.is_synchronised = false;

		ui_state.irp6p_tfg.edp.pid = -1;
		ui_state.irp6p_tfg.edp.reader_fd = -1;
		pt_res = PtEnter(0);
		close_all_irp6ot_windows(NULL, NULL, NULL);
		if (pt_res >= 0)
			PtLeave(0);
	}

	// modyfikacja menu

	manage_interface();

	return (Pt_CONTINUE);

}

int EDP_irp6p_tfg_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	edp_irp6p_tfg_eb.command(boost::bind(EDP_irp6p_tfg_synchronise_int, widget, apinfo, cbinfo));

	return (Pt_CONTINUE);

}

int EDP_irp6p_tfg_synchronise_int(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	set_ui_state_notification(UI_N_SYNCHRONISATION);

	// wychwytania ew. bledow ECP::robot
	try {
		// dla robota irp6p_tfg_

		if ((ui_state.irp6p_tfg.edp.state > 0) && (ui_state.irp6p_tfg.edp.is_synchronised == false)) {
			ui_robot.irp6p_tfg->ecp->synchronise();
			ui_state.irp6p_tfg.edp.is_synchronised = ui_robot.irp6p_tfg->ecp->is_synchronised();
		} else {
			// 	printf("EDP irp6p_tfg niepowolane, synchronizacja niedozwolona\n");
		}

	} // end try
	CATCH_SECTION_UI

	// modyfikacje menu
	manage_interface();

	return (Pt_CONTINUE);

}

int start_wind_irp6p_tfg_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!ui_state.is_wind_irp6p_tfg_moves_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6p_tfg_moves, widget, cbinfo);
		ui_state.is_wind_irp6p_tfg_moves_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6p_tfg_moves);
	}

	return (Pt_CONTINUE);

}

int irp6p_tfg_move_to_preset_position(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	PhKeyEvent_t *my_data = NULL;

	if (cbinfo->event->type == Ph_EV_KEY) {
		my_data = (PhKeyEvent_t *) PhGetData(cbinfo->event);
	}

	if (ui_state.irp6p_tfg.edp.pid != -1) {

		if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6p_tfg_preset_position_synchro) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_synchro)) || ((cbinfo->event->type == Ph_EV_KEY)
				&& (my_data->key_cap == 0x73))) && (ui_state.irp6p_tfg.edp.is_synchronised)) {// powrot do pozycji synchronizacji
			for (int i = 0; i < IRP6P_TFG_NUM_OF_SERVOS; i++) {
				irp6p_tfg_desired_pos[i] = 0.0;
			}
			edp_irp6p_tfg_eb.command(boost::bind(irp6p_tfg_execute_motor_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6p_tfg_preset_position_0) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_0)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x30))) && (ui_state.irp6p_tfg.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < IRP6P_TFG_NUM_OF_SERVOS; i++) {
				irp6p_tfg_desired_pos[i] = ui_state.irp6p_tfg.edp.preset_position[0][i];
			}
			edp_irp6p_tfg_eb.command(boost::bind(irp6p_tfg_execute_joint_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6p_tfg_preset_position_1) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_1)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x31))) && (ui_state.irp6p_tfg.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < IRP6P_TFG_NUM_OF_SERVOS; i++) {
				irp6p_tfg_desired_pos[i] = ui_state.irp6p_tfg.edp.preset_position[1][i];
			}
			edp_irp6p_tfg_eb.command(boost::bind(irp6p_tfg_execute_joint_motion));
		} else if ((((ApName(ApWidget(cbinfo)) == ABN_mm_irp6p_tfg_preset_position_2) || (ApName(ApWidget(cbinfo))
				== ABN_mm_all_robots_preset_position_2)) || ((cbinfo->event->type == Ph_EV_KEY) && (my_data->key_cap
				== 0x32))) && (ui_state.irp6p_tfg.edp.is_synchronised)) {// ruch do pozycji zadania (wspolrzedne przyjete arbitralnie)
			for (int i = 0; i < IRP6P_TFG_NUM_OF_SERVOS; i++) {
				irp6p_tfg_desired_pos[i] = ui_state.irp6p_tfg.edp.preset_position[2][i];
			}
			edp_irp6p_tfg_eb.command(boost::bind(irp6p_tfg_execute_joint_motion));
		}

		//	ui_robot.irp6p_tfg->move_motors(irp6p_tfg_desired_pos);

	} // end if (ui_state.irp6p_tfg.edp.pid!=-1)


	return (Pt_CONTINUE);

}

int irp6p_tfg_execute_motor_motion()
{
	try {

		ui_robot.irp6p_tfg->move_motors(irp6p_tfg_desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int irp6p_tfg_execute_joint_motion()
{
	try {

		ui_robot.irp6p_tfg->move_joints(irp6p_tfg_desired_pos);

	} // end try
	CATCH_SECTION_UI

	return 1;
}

int start_wnd_irp6p_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!ui_state.is_wind_irp6p_tfg_servo_algorithm_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_irp6p_tfg_servo_algorithm, widget, cbinfo);
		ui_state.is_wind_irp6p_tfg_servo_algorithm_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_irp6p_tfg_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int init_wnd_irp6p_tfg_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t servo_alg_no[IRP6P_TFG_NUM_OF_SERVOS];
	uint8_t servo_par_no[IRP6P_TFG_NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (ui_state.irp6p_tfg.edp.pid != -1) {
			if (ui_state.irp6p_tfg.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				ui_robot.irp6p_tfg->get_servo_algorithm(servo_alg_no, servo_par_no);

				PtSetResource(ABW_PtNumericInteger_wnd_irp6p_tfg_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, servo_alg_no[0] , 0);

				PtSetResource(ABW_PtNumericInteger_wnd_irp6p_tfg_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, servo_par_no[0] , 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int irp6p_tfg_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *servo_alg_no_tmp[IRP6P_TFG_NUM_OF_SERVOS];
	uint8_t servo_alg_no_output[IRP6P_TFG_NUM_OF_SERVOS];
	uint8_t *servo_par_no_tmp[IRP6P_TFG_NUM_OF_SERVOS];
	uint8_t servo_par_no_output[IRP6P_TFG_NUM_OF_SERVOS];

	// wychwytania ew. bledow ECP::robot
	try {
		if (ui_state.irp6p_tfg.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_tfg_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0 );

			PtGetResource(ABW_PtNumericInteger_wnd_irp6p_tfg_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0 );

			for (int i = 0; i < IRP6P_TFG_NUM_OF_SERVOS; i++) {
				servo_alg_no_output[i] = *servo_alg_no_tmp[i];
				servo_par_no_output[i] = *servo_par_no_tmp[i];
			}

			// zlecenie wykonania ruchu
			ui_robot.irp6p_tfg->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int wind_irp6p_tfg_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if ((ui_state.irp6p_tfg.edp.pid != -1) && (ui_state.is_wind_irp6p_tfg_moves_open)) {
			if (ui_state.irp6p_tfg.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				unblock_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_pos);
				unblock_widget(ABW_PtButton_wind_irp6p_tfg_moves_inc_exec);

				unblock_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_left);
				unblock_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_right);
				unblock_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_step);
				unblock_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_pos);
				unblock_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_exec);

				ui_robot.irp6p_tfg->read_motors(irp6p_tfg_current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_read_motor_pos, Pt_ARG_NUMERIC_VALUE, &irp6p_tfg_current_pos[0] , 0);

				ui_robot.irp6p_tfg->read_joints(irp6p_tfg_current_pos);

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_read_int_pos, Pt_ARG_NUMERIC_VALUE, &irp6p_tfg_current_pos[0] , 0);

			} else {
				block_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_pos);
				block_widget(ABW_PtButton_wind_irp6p_tfg_moves_inc_exec);

				block_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_left);
				block_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_right);
				block_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_step);
				block_widget(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_pos);
				block_widget(ABW_PtButton_wind_irp6p_tfg_moves_int_exec);
			}
			PtDamageWidget(ABW_wnd_irp6p_tfg_moves);
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int clear_wind_irp6p_tfg_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6p_tfg_moves_open = false;

	return (Pt_CONTINUE);

}

int wind_irp6p_tfg_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor_ptgr, irp6p_tfg_desired_pos_motors[6], irp6p_tfg_desired_pos_int[6];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {

		if (ui_state.irp6p_tfg.edp.pid != -1) {

			// incremental
			if ((widget == ABW_PtButton_wind_irp6p_tfg_moves_inc_left) || (widget
					== ABW_PtButton_wind_irp6p_tfg_moves_inc_right) || (widget
					== ABW_PtButton_wind_irp6p_tfg_moves_inc_exec)) {

				if (ui_state.irp6p_tfg.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0 );
					irp6p_tfg_desired_pos_motors[0] = (*wektor_ptgr);
				} else {
					irp6p_tfg_desired_pos_motors[0] = 0.0;
				}

				PtGetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0 );

				if (widget == ABW_PtButton_wind_irp6p_tfg_moves_inc_left) {
					irp6p_tfg_desired_pos_motors[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_irp6p_tfg_moves_inc_right) {
					irp6p_tfg_desired_pos_motors[0] += (*step1);
				}

				ui_robot.irp6p_tfg->move_motors(irp6p_tfg_desired_pos_motors);

			}

			// internal
			if ((widget == ABW_PtButton_wind_irp6p_tfg_moves_int_left) || (widget
					== ABW_PtButton_wind_irp6p_tfg_moves_int_right) || (widget
					== ABW_PtButton_wind_irp6p_tfg_moves_int_exec)) {
				if (ui_state.irp6p_tfg.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0 );
					irp6p_tfg_desired_pos_int[0] = (*wektor_ptgr);
				}

				PtGetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_step, Pt_ARG_NUMERIC_VALUE, &step1, 0 );

				if (widget == ABW_PtButton_wind_irp6p_tfg_moves_int_left) {
					irp6p_tfg_desired_pos_int[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_irp6p_tfg_moves_int_right) {
					irp6p_tfg_desired_pos_int[0] += (*step1);
				}
				ui_robot.irp6p_tfg->move_joints(irp6p_tfg_desired_pos_int);
			}

			// odswierzenie pozycji robota
			if ((ui_state.irp6p_tfg.edp.is_synchronised) && (ui_state.is_wind_irp6p_tfg_moves_open)) {

				PtSetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &irp6p_tfg_desired_pos_motors[0] , 0);
				PtSetResource(ABW_PtNumericFloat_wind_irp6p_tfg_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &irp6p_tfg_desired_pos_int[0] , 0);

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int clear_wnd_irp6p_tfg_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	ui_state.is_wind_irp6p_tfg_servo_algorithm_open = false;

	return (Pt_CONTINUE);

}

int reload_irp6p_tfg_configuration()
{
	// jesli IRP6 on_track ma byc aktywne
	if ((ui_state.irp6p_tfg.is_active = config->value <int> ("is_irp6p_tfg_active")) == 1) {
		// ini_con->create_ecp_irp6p_tfg (ini_con->ui->ecp_irp6p_tfg_section);
		//ui_state.is_any_edp_active = true;
		if (ui_state.is_mp_and_ecps_active) {
			ui_state.irp6p_tfg.ecp.network_trigger_attach_point
					= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "trigger_attach_point", ui_state.irp6p_tfg.ecp.section_name);

			ui_state.irp6p_tfg.ecp.pid = -1;
			ui_state.irp6p_tfg.ecp.trigger_fd = -1;
		}

		switch (ui_state.irp6p_tfg.edp.state)
		{
			case -1:
			case 0:
				// ini_con->create_edp_irp6p_tfg (ini_con->ui->edp_irp6p_tfg_section);

				ui_state.irp6p_tfg.edp.pid = -1;
				ui_state.irp6p_tfg.edp.reader_fd = -1;
				ui_state.irp6p_tfg.edp.state = 0;

				for (int i = 0; i < 3; i++) {
					char tmp_string[50];
					sprintf(tmp_string, "preset_position_%d", i);

					if (config->exists(tmp_string, ui_state.irp6p_tfg.edp.section_name)) {
						char* tmp, *tmp1;
						tmp1
								= tmp
										= strdup(config->value <std::string> (tmp_string, ui_state.irp6p_tfg.edp.section_name).c_str());
						char* toDel = tmp;
						for (int j = 0; j < IRP6P_TFG_NUM_OF_SERVOS; j++) {

							ui_state.irp6p_tfg.edp.preset_position[i][j] = strtod(tmp1, &tmp1);

						}
						free(toDel);
					} else {
						for (int j = 0; j < IRP6P_TFG_NUM_OF_SERVOS; j++) {

							ui_state.irp6p_tfg.edp.preset_position[i][j] = 0.074;

						}
					}
				}

				if (config->exists("test_mode", ui_state.irp6p_tfg.edp.section_name))
					ui_state.irp6p_tfg.edp.test_mode
							= config->value <int> ("test_mode", ui_state.irp6p_tfg.edp.section_name);
				else
					ui_state.irp6p_tfg.edp.test_mode = 0;

				ui_state.irp6p_tfg.edp.hardware_busy_attach_point
						= config->value <std::string> ("hardware_busy_attach_point", ui_state.irp6p_tfg.edp.section_name);

				ui_state.irp6p_tfg.edp.network_resourceman_attach_point
						= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "resourceman_attach_point", ui_state.irp6p_tfg.edp.section_name);

				ui_state.irp6p_tfg.edp.network_reader_attach_point
						= config->return_attach_point_name(lib::configurator::CONFIG_SERVER, "reader_attach_point", ui_state.irp6p_tfg.edp.section_name);

				ui_state.irp6p_tfg.edp.node_name
						= config->value <std::string> ("node_name", ui_state.irp6p_tfg.edp.section_name);
				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}

	} else // jesli  irp6 on_track ma byc nieaktywne
	{
		switch (ui_state.irp6p_tfg.edp.state)
		{
			case -1:
			case 0:
				ui_state.irp6p_tfg.edp.state = -1;
				break;
			case 1:
			case 2:
				// nie robi nic bo EDP pracuje
				break;
			default:
				break;
		}
	} // end irp6p_tfg

	return 1;
}

int manage_interface_irp6p_tfg()
{
	switch (ui_state.irp6p_tfg.edp.state)
	{
		case -1:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg, NULL);
			break;
		case 0:
			ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_edp_unload, ABN_mm_irp6p_tfg_synchronisation, ABN_mm_irp6p_tfg_move, ABN_mm_irp6p_tfg_preset_positions, ABN_mm_irp6p_tfg_servo_algorithm, NULL);
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg, ABN_mm_irp6p_tfg_edp_load, NULL);

			break;
		case 1:
		case 2:
			ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg, NULL);

			// jesli robot jest zsynchronizowany
			if (ui_state.irp6p_tfg.edp.is_synchronised) {
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_synchronisation, NULL);
				ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_preset_positions, NULL);

				switch (ui_state.mp.state)
				{
					case UI_MP_NOT_PERMITED_TO_RUN:
					case UI_MP_PERMITED_TO_RUN:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg_edp_unload, ABN_mm_irp6p_tfg_move, ABN_mm_irp6p_tfg_preset_positions, ABN_mm_irp6p_tfg_servo_algorithm, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_edp_load, NULL);
						break;
					case UI_MP_WAITING_FOR_START_PULSE:
						ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg_move, ABN_mm_irp6p_tfg_preset_positions, ABN_mm_irp6p_tfg_servo_algorithm, NULL);
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_edp_load, ABN_mm_irp6p_tfg_edp_unload, NULL);
						break;
					case UI_MP_TASK_RUNNING:
					case UI_MP_TASK_PAUSED:
						ApModifyItemState(&robot_menu, AB_ITEM_DIM, // modyfikacja menu - ruchy reczne zakazane
						ABN_mm_irp6p_tfg_move, ABN_mm_irp6p_tfg_preset_positions, ABN_mm_irp6p_tfg_servo_algorithm, NULL);
						break;
					default:
						break;
				}
			} else // jesli robot jest niezsynchronizowany
			{
				ApModifyItemState(&robot_menu, AB_ITEM_NORMAL, ABN_mm_irp6p_tfg_edp_unload, ABN_mm_irp6p_tfg_synchronisation, ABN_mm_irp6p_tfg_move, NULL);
				ApModifyItemState(&robot_menu, AB_ITEM_DIM, ABN_mm_irp6p_tfg_edp_load, NULL);
				ApModifyItemState(&all_robots_menu, AB_ITEM_NORMAL, ABN_mm_all_robots_synchronisation, NULL);
			}
			break;
		default:
			break;
	}

	return 1;
}