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
#include "data_logger.hpp"

bool *deinitialisation_request;
bool *deinit_controller;
bool *deinit_logger;

static void sigint_handler(int sig){
	if (deinitialisation_request==NULL){
		printf("Ops.. deinitalisation request is a NULL pointer.\n");
	}else{
		printf("\nDeinitialising iiwa activity\n");
		*deinitialisation_request = true;
		*deinit_controller = true;
		*deinit_logger = true;
	}
}

// void* set_actuation(void* activity){
// 	activity_t *iiwa_controller = (activity_t*) activity; 
// 	iiwa_controller_params_t* params = (iiwa_controller_params_t *) iiwa_controller->conf.params;
// 	iiwa_controller_coordination_state_t *coord_state =
// 	(iiwa_controller_coordination_state_t *) iiwa_controller->state.coordination_state;  

// 	int dt = 500; // ms
// 	double t = 0;
// 	double local_time = 0;
	
// 	while(!(*deinitialisation_request)){
// 		usleep(1000*dt);  // time in microseconds
// 		if (iiwa_controller->lcsm.state == RUNNING){
// 			// Copying data
// 			pthread_mutex_lock(&coord_state->goal_lock);
// 			params->goal_jnt_pos[0] = 0;
// 			params->goal_jnt_pos[1] = 0;
// 			params->goal_jnt_pos[2] = 0;
// 			params->goal_jnt_pos[3] = 0;
// 			params->goal_jnt_pos[4] = 0;
// 			params->goal_jnt_pos[5] = 0;
// 			if (local_time < 1){
// 				params->goal_jnt_pos[6] = 0;
// 			}else{
// 				params->goal_jnt_pos[6] = 1;
// 				// if (t<5){
// 				// 	params->goal_jnt_pos[6] = 1+ (t-1)/10;
// 				// }
// 				// else{
// 				// 	params->goal_jnt_pos[6] = 1;
// 				// }
// 			}

//             fpt2 = fopen("setpoint.csv", "a+");
// 			fprintf(fpt2, " %f, %f \n", t, params->goal_jnt_pos[6]);
// 			fclose(fpt2);

// 			pthread_mutex_unlock(&coord_state->goal_lock);
// 			local_time += (double)dt/1000;

// 			// printf("%f\n", t);
// 		}
// 		t += (double)dt/1000;
// 	}
// 	return 0;
// }

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
		// printf("Time %f: \n", t);
		if (mediator_activity->lcsm.state == RUNNING){
			// Copying data
			if (t < 2){
				if (t>1){
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

// void* set_wrench_actuation(void* activity){
// 	activity_t *iiwa_controller = (activity_t*) activity; 
// 	iiwa_controller_params_t* params = (iiwa_controller_params_t *) iiwa_controller->conf.params;
// 	iiwa_controller_coordination_state_t *coord_state =
// 	(iiwa_controller_coordination_state_t *) iiwa_controller->state.coordination_state;  

// 	int dt = 500; // ms
// 	double t = 0;
// 	double local_time = 0;
	
// 	while(!(*deinitialisation_request)){
// 		usleep(1000*dt);  // time in microseconds
// 		if (iiwa_controller->lcsm.state == RUNNING){
// 			// Copying data
// 			pthread_mutex_lock(&coord_state->goal_lock);
// 			params->goal_wrench[0] = 0;
// 			params->goal_wrench[1] = 0;
// 			params->goal_wrench[2] = 0;
// 			params->goal_wrench[3] = 0;
// 			params->goal_wrench[4] = 0;
// 			if (local_time < 1){
// 				params->goal_wrench[5] = 0;
// 			}else{
// 				params->goal_wrench[5] = 1;
// 			}

//             fpt3 = fopen("setpoint_wrench.csv", "a+");
// 			fprintf(fpt3, " %f, %f \n", t, params->goal_wrench[5]);
// 			fclose(fpt3);

// 			pthread_mutex_unlock(&coord_state->goal_lock);
// 			local_time += (double)dt/1000;

// 			// printf("%f\n", t);
// 		}
// 		t += (double)dt/1000;
// 	}
// 	return 0;
// }

int main(int argc, char**argv){
	signal(SIGINT, sigint_handler);
	
	// ### ACTIVITIES ### //     
	activity_t iiwa_activity; 
	activity_t iiwa_controller;
	activity_t mediator_activity;
	activity_t data_logger;

	ec_iiwa_activity.create_lcsm(&iiwa_activity, "iiwa_activity");   
	ec_iiwa_activity.resource_configure_lcsm(&iiwa_activity);

	ec_iiwa_controller.create_lcsm(&iiwa_controller, "iiwa_controller");
	ec_iiwa_controller.resource_configure_lcsm(&iiwa_controller);

	ec_task_mediator.create_lcsm(&mediator_activity, "mediator_activity");
	ec_task_mediator.resource_configure_lcsm(&mediator_activity);

	ec_data_logger.create_lcsm(&data_logger, "data_logger");
	ec_data_logger.resource_configure_lcsm(&data_logger);

	// Initialize Vars: iiwa_activity
	iiwa_activity_params_t* iiwa_activity_params = (iiwa_activity_params_t *) iiwa_activity.conf.params;
	iiwa_activity_continuous_state_t *iiwa_activity_continuous_state = (iiwa_activity_continuous_state_t *) iiwa_activity.state.computational_state.continuous;
	iiwa_activity_coordination_state_t *iiwa_activity_coord_state = (iiwa_activity_coordination_state_t *) iiwa_activity.state.coordination_state;

	deinitialisation_request = &iiwa_activity_coord_state->deinitialisation_request;

	strcpy(iiwa_activity_params->iiwa_params.fri_ip,"192.168.1.50");
	iiwa_activity_params->iiwa_params.fri_port = 30100;
	iiwa_activity_params->iiwa_params.cmd_mode = TORQUE;
	
	// Initialize Vars: iiwa_controller
	iiwa_controller_params_t* iiwa_controller_params = (iiwa_controller_params_t *) iiwa_controller.conf.params;
	iiwa_controller_continuous_state_t *iiwa_controller_continuous_state = (iiwa_controller_continuous_state_t *) iiwa_controller.state.computational_state.continuous;
	iiwa_controller_coordination_state_t *iiwa_controller_coord_state = (iiwa_controller_coordination_state_t *) iiwa_controller.state.coordination_state;

	deinit_controller= &iiwa_controller_coord_state->deinitialisation_request;

    // Initialize Vars: task_mediator
	task_mediator_coordination_state_t* task_coord_state = (task_mediator_coordination_state_t *) mediator_activity.state.coordination_state;

	// Initialize Vars: data_logger
	data_logger_params_t *data_logger_params = (data_logger_params_t *) data_logger.conf.params;
	data_logger_continuous_state_t *data_logger_state = (data_logger_continuous_state_t *) data_logger.state.computational_state.continuous;
	data_logger_coordination_state_t *data_logger_coord_state = (data_logger_coordination_state_t *) data_logger.state.coordination_state;

	deinit_logger = &data_logger_coord_state->deinitialisation_request;

    // Share Memory
	iiwa_controller_coord_state->sensor_lock = &iiwa_activity_coord_state->sensor_lock;
	iiwa_controller_coord_state->actuation_lock = &iiwa_activity_coord_state->actuation_lock;
	
	data_logger_coord_state->sensor_lock = &iiwa_activity_coord_state->sensor_lock;
	data_logger_coord_state->actuation_lock = &iiwa_activity_coord_state->actuation_lock;
	data_logger_coord_state->goal_lock = &iiwa_controller_coord_state->goal_lock;

	iiwa_controller_params->iiwa_controller_params = &iiwa_activity_continuous_state->iiwa_state;
	iiwa_controller_continuous_state->iiwa_controller_state = &iiwa_activity_params->iiwa_params;

	data_logger_state->controller_continuous_state = iiwa_controller_continuous_state;
	data_logger_state->iiwa_continuous_state = iiwa_activity_continuous_state;

	// Task <-> Controller  
	task_coord_state->initiate_motion = &iiwa_controller_coord_state->execution_request;

	// Manually 
	iiwa_activity_coord_state->execution_request = true;
	data_logger_coord_state->execution_request = true;
	data_logger_params->fname = "logging.csv";

	// Configure the Controller Parameters
	// switch(iiwa_activity_params->iiwa_params.cmd_mode){
	// 	case POSITION:
	// 	{
	// 		iiwa_controller_params->max_jnt_vel[6] = 1;
	// 		iiwa_controller_params->slow_jnt_vel[6] = 0.1;
	// 		iiwa_controller_params->jnt_accel[6] = 1; //10
	// 		iiwa_controller_params->approach_buffer[6] = 0.2;
	// 		iiwa_controller_params->slow_buffer[6] = 0.05;
	// 		iiwa_controller_params->goal_buffer[6] = 0.01;
	// 		break;
	// 	}
	// 	case WRENCH:
	// 	{
	// 		// Set the required controller parameters
	// 		iiwa_controller_params->max_wrench_step = 0.01;
	// 		break;
	// 	}
	// }
	

	// ### THREADS ### //
	thread_t thread_iiwa;
	thread_t thread_iiwa_controller;
	thread_t thread_mediator;
	thread_t thread_logger;

	// Create thread: data structure, thread name, cycle time in milliseconds
	create_thread(&thread_iiwa, "thread_iiwa", 4); // 4 ms = 250 Hz
	create_thread(&thread_iiwa_controller, "thread_iiwa_controller", 12);
	create_thread(&thread_mediator, "thread_mediator", 100);
	create_thread(&thread_logger, "thread_logger", 10);

	// Register activities in threads
	register_activity(&thread_iiwa, &iiwa_activity, "iiwa_activity");
	register_activity(&thread_iiwa_controller, &iiwa_controller, "iiwa_controller");
	register_activity(&thread_mediator, &mediator_activity, "mediator_activity");
	register_activity(&thread_logger, &data_logger, "data_logger");

	// ### SHARED MEMORY ### //

	// Create POSIX threads   
	pthread_t pthread_iiwa, pthread_iiwa_controller, pthread_mediator, pthread_petrinet, pthread_logger;

	pthread_create( &pthread_iiwa, NULL, do_thread_loop, ((void*) &thread_iiwa));
	// pthread_create( &pthread_actuation, NULL, set_actuation, (void*) &iiwa_controller);
	pthread_create( &pthread_iiwa_controller, NULL, do_thread_loop, ((void*) &thread_iiwa_controller));
	pthread_create( &pthread_mediator, NULL, do_thread_loop, ((void*) &thread_mediator));
	pthread_create( &pthread_petrinet, NULL, set_petrinet, (void*) &mediator_activity);
	pthread_create( &pthread_logger, NULL, do_thread_loop, ((void*) &thread_logger));

	// Wait for threads to finish, which means all activities must properly finish and reach the dead LCSM state
	pthread_join(pthread_iiwa, NULL);
	// pthread_join(pthread_actuation, NULL);
	pthread_join(pthread_iiwa_controller, NULL);
	pthread_join(pthread_mediator, NULL);
	pthread_join(pthread_petrinet, NULL);
	pthread_join(pthread_logger, NULL);
	
	// Freeing memory
	ec_iiwa_activity.destroy_lcsm(&iiwa_activity);
	ec_iiwa_controller.destroy_lcsm(&iiwa_controller);
	ec_task_mediator.destroy_lcsm(&mediator_activity);
	ec_data_logger.destroy_lcsm(&data_logger);
	return 0;
}