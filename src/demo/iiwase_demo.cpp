/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Brendan Pousett and Louis Hanut
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
	* @file iiwase_demo.cpp
	* @date November 14, 2022
 **/

#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

// AACAL
#include <five_c/thread/thread.h>
#include <five_c/activity/activity.h>

#include <iiwa_activity.hpp>

#include "iiwa/iiwa_controller.hpp"
#include "task_mediator/task_mediator.hpp"


bool *deinitialisation_request;
bool *deinit_controller;
double jnt_pos_save[7];
FILE *fpt, *fpt2;
double traj_time;
int thread_time = 4; //ms

static void sigint_handler(int sig){
	if (deinitialisation_request==NULL){
		printf("Ops.. deinitalisation request is a NULL pointer.\n");
	}else{
		printf("\nDeinitialising iiwa activity\n");
		*deinitialisation_request = true;
		*deinit_controller = true;
	}
}

void* set_actuation(void* activity){
	activity_t *iiwa_controller = (activity_t*) activity; 
	iiwa_controller_params_t* params = (iiwa_controller_params_t *) iiwa_controller->conf.params;
	iiwa_controller_coordination_state_t *coord_state =
	(iiwa_controller_coordination_state_t *) iiwa_controller->state.coordination_state;  

	int dt = 500; // ms
	double t = 0;
	double local_time = 0;
	
	while(!(*deinitialisation_request)){
		usleep(1000*dt);  // time in microseconds
		if (iiwa_controller->lcsm.state == RUNNING){
			// Copying data
			pthread_mutex_lock(&coord_state->goal_lock);
			params->goal_jnt_pos[0] = 0;
			params->goal_jnt_pos[1] = 0;
			params->goal_jnt_pos[2] = 0;
			params->goal_jnt_pos[3] = 0;
			params->goal_jnt_pos[4] = 0;
			params->goal_jnt_pos[5] = 0;
			if (local_time < 1){
				params->goal_jnt_pos[6] = 0;
			}else{
				params->goal_jnt_pos[6] = 1;
				// if (t<5){
				// 	params->goal_jnt_pos[6] = 1+ (t-1)/10;
				// }
				// else{
				// 	params->goal_jnt_pos[6] = 1;
				// }
			}

            fpt2 = fopen("setpoint.csv", "a+");
			fprintf(fpt2, " %f, %f \n", t, params->goal_jnt_pos[6]);
			fclose(fpt2);

			pthread_mutex_unlock(&coord_state->goal_lock);
			local_time += (double)dt/1000;

			// printf("%f\n", t);
		}
		t += (double)dt/1000;
	}
	return 0;
}

void* set_petrinet(void* activity){
	activity_t *mediator_activity = (activity_t*) activity; 
	task_mediator_coordination_state_t *coord_state =
	(task_mediator_coordination_state_t *) mediator_activity->state.coordination_state;  

	int dt = 500; // ms
	double t = 0;

	bool board_in_range = false; //need to allocate memory for those
	bool board_dirty = false;
	bool start_vel_transition = false;
	bool end_vel_transition = false;
	bool contact_detected = false;

	coord_state->board_in_range = &board_in_range;
	coord_state->board_dirty = &board_dirty;
	coord_state->start_vel_transition = &start_vel_transition;
	coord_state->end_vel_transition = &end_vel_transition;
	coord_state->contact_detected = &contact_detected;
	
	while(!(*deinitialisation_request)){
		usleep(1000*dt);  // time in microseconds
		printf("Time %f: \n", t);
		if (mediator_activity->lcsm.state == RUNNING){
			// Copying data
			if (t < 4){
				if (t>2){
					*coord_state->board_in_range = true;
				}
			}else{
				*coord_state->board_dirty = true;
			}
		}
		t += (double)dt/1000;
	}
	return 0;
}

// void* save_sensor_data(void* activity){
// 	activity_t *iiwa_virtual = (activity_t*) activity; 
// 	iiwa_virtual_params_t* params = (iiwa_virtual_params_t *) iiwa_virtual->conf.params;
// 	iiwa_virtual_continuous_state_t *continuous_state =
// 	(iiwa_virtual_continuous_state_t *) iiwa_virtual->state.computational_state.continuous;
// 	iiwa_virtual_coordination_state_t *coord_state =
// 	(iiwa_virtual_coordination_state_t *) iiwa_virtual->state.coordination_state;  

// 	while(!(*deinitialisation_request)){
// 		printf("%f \n",continuous_state->iiwa_state.iiwa_sensors.meas_ext_torques[0]);
// 		if (iiwa_virtual->lcsm.state == RUNNING){
// 			fpt = fopen("jnt_pos.csv", "a+");
// 			pthread_mutex_lock(&coord_state->sensor_lock);
// 			memcpy(jnt_pos_save, continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos, 7 * sizeof(double));
// 			pthread_mutex_unlock(&coord_state->sensor_lock);
// 			fprintf(fpt, " %f, %f, %f, %f, %f, %f, %f \n", jnt_pos_save[0], jnt_pos_save[1], jnt_pos_save[2],
// 															jnt_pos_save[3], jnt_pos_save[4], jnt_pos_save[5], jnt_pos_save[6]);
// 			fclose(fpt);
// 		}
// 	}
// }

int main(int argc, char**argv){
	signal(SIGINT, sigint_handler);
	
	// ### ACTIVITIES ### //     
	activity_t iiwa_activity; 
	activity_t iiwa_controller;
	activity_t mediator_activity;
	// iiwa Lidar
	ec_iiwa_activity.create_lcsm(&iiwa_activity, "iiwa_activity");   
	ec_iiwa_activity.resource_configure_lcsm(&iiwa_activity);

	ec_iiwa_controller.create_lcsm(&iiwa_controller, "iiwa_controller");
	ec_iiwa_controller.resource_configure_lcsm(&iiwa_controller);

	ec_task_mediator.create_lcsm(&mediator_activity, "mediator_activity");
	ec_task_mediator.resource_configure_lcsm(&mediator_activity);

	// Share memory
	iiwa_activity_params_t* iiwa_activity_params = (iiwa_activity_params_t *) iiwa_activity.conf.params;
	iiwa_activity_continuous_state_t *iiwa_activity_continuous_state = (iiwa_activity_continuous_state_t *) iiwa_activity.state.computational_state.continuous;
	iiwa_activity_coordination_state_t *iiwa_activity_coord_state = (iiwa_activity_coordination_state_t *) iiwa_activity.state.coordination_state;

	deinitialisation_request = &iiwa_activity_coord_state->deinitialisation_request;

	strcpy(iiwa_activity_params->iiwa_params.fri_ip,"192.168.1.50");
	iiwa_activity_params->iiwa_params.fri_port = 30100;
	iiwa_activity_params->iiwa_params.cmd_mode = POSITION;
	
	iiwa_controller_params_t* iiwa_controller_params = (iiwa_controller_params_t *) iiwa_controller.conf.params;
	iiwa_controller_continuous_state_t *iiwa_controller_continuous_state = (iiwa_controller_continuous_state_t *) iiwa_controller.state.computational_state.continuous;
	iiwa_controller_coordination_state_t *iiwa_controller_coord_state = (iiwa_controller_coordination_state_t *) iiwa_controller.state.coordination_state;

	deinit_controller= &iiwa_controller_coord_state->deinitialisation_request;

	task_mediator_coordination_state_t* task_coord_state = (task_mediator_coordination_state_t *) mediator_activity.state.coordination_state;

	// Iiwa activity <-> Controller
	iiwa_controller_coord_state->sensor_lock = &iiwa_activity_coord_state->sensor_lock;
	iiwa_controller_coord_state->actuation_lock = &iiwa_activity_coord_state->actuation_lock;

	iiwa_controller_params->iiwa_controller_params = &iiwa_activity_continuous_state->iiwa_state;
	iiwa_controller_continuous_state->iiwa_controller_state = &iiwa_activity_params->iiwa_params;

	// Task <-> Controller  
	task_coord_state->initiate_motion = &iiwa_controller_coord_state->execution_request;

	// Manually 
	iiwa_activity_coord_state->execution_request = true;

	// Configure the Controller Parameters
	switch(iiwa_activity_params->iiwa_params.cmd_mode){
		case POSITION:
		{
			iiwa_controller_params->max_jnt_vel[6] = 1;
			iiwa_controller_params->slow_jnt_vel[6] = 0.1;
			iiwa_controller_params->jnt_accel[6] = 1; //10
			iiwa_controller_params->approach_buffer[6] = 0.2;
			iiwa_controller_params->slow_buffer[6] = 0.05;
			iiwa_controller_params->goal_buffer[6] = 0.01;
			break;
		}
		case WRENCH:
		{
			// Set the required controller parameters
			break;
		}
	}
	

	// ### THREADS ### //
	thread_t thread_iiwa;
	thread_t thread_iiwa_controller;
	thread_t thread_mediator;

	// Create thread: data structure, thread name, cycle time in milliseconds
	create_thread(&thread_iiwa, "thread_iiwa", thread_time); // 4 ms
	create_thread(&thread_iiwa_controller, "thread_iiwa_controller", 5*thread_time);
	create_thread(&thread_mediator, "thread_mediator", 100);

	// Register activities in threads
	register_activity(&thread_iiwa, &iiwa_activity, "iiwa_activity");
	register_activity(&thread_iiwa_controller, &iiwa_controller, "iiwa_controller");
	register_activity(&thread_mediator, &mediator_activity, "mediator_activity");

	// ### SHARED MEMORY ### //

	// Create POSIX threads   
	pthread_t pthread_iiwa, pthread_actuation, phtread_saving, pthread_iiwa_controller, pthread_mediator, pthread_petrinet;

	pthread_create( &pthread_iiwa, NULL, do_thread_loop, ((void*) &thread_iiwa));
	pthread_create( &pthread_actuation, NULL, set_actuation, (void*) &iiwa_controller);
	pthread_create(&pthread_iiwa_controller, NULL, do_thread_loop, ((void*) &thread_iiwa_controller));
	pthread_create( &pthread_mediator, NULL, do_thread_loop, ((void*) &thread_mediator));
	pthread_create( &pthread_petrinet, NULL, set_petrinet, (void*) &mediator_activity);
	// pthread_create( &phtread_saving, NULL, save_sensor_data, (void*) &iiwa_activity);

	// Wait for threads to finish, which means all activities must properly finish and reach the dead LCSM state
	pthread_join(pthread_iiwa, NULL);
	pthread_join(pthread_actuation, NULL);
	pthread_join(pthread_iiwa_controller, NULL);
	pthread_join(pthread_mediator, NULL);
	pthread_join(pthread_petrinet, NULL);
	// pthread_join(phtread_saving, NULL);
	
	// Freeing memory
	ec_iiwa_activity.destroy_lcsm(&iiwa_activity);
	ec_iiwa_controller.destroy_lcsm(&iiwa_controller);
	ec_task_mediator.destroy_lcsm(&mediator_activity);
	return 0;
}