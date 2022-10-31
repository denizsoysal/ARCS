/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
	* @file iiwa_demo.cpp
	* @date September 16, 2022
 **/

#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <math.h>

// AACAL
#include <five_c/thread/thread.h>
#include <five_c/activity/activity.h>

#include "iiwa_virtual.hpp"

#include <pthread.h>
#include <unistd.h>


bool *deinitialisation_request;
double jnt_pos_save[7];
FILE *fpt;
double traj_time;
int thread_time = 10; //ms

static void sigint_handler(int sig){
	if (deinitialisation_request==NULL){
		printf("Ops.. deinitalisation request is a NULL pointer.\n");
	}else{
		printf("\nDeinitialising iiwa activity\n");
		*deinitialisation_request = true;
	}
}

void* set_actuation(void* activity){
	activity_t *iiwa_virtual = (activity_t*) activity; 
	iiwa_virtual_params_t* params = (iiwa_virtual_params_t *) iiwa_virtual->conf.params;
	iiwa_virtual_continuous_state_t *continuous_state =
	(iiwa_virtual_continuous_state_t *) iiwa_virtual->state.computational_state.continuous;
	iiwa_virtual_coordination_state_t *coord_state =
	(iiwa_virtual_coordination_state_t *) iiwa_virtual->state.coordination_state;  

	int dt = 100; // ms
	double t = 0;
	
	while(!(*deinitialisation_request)){
		usleep(1000*dt);  // time in microseconds
		if (iiwa_virtual->lcsm.state == RUNNING){
			// Copying data
			pthread_mutex_lock(&coord_state->actuation_lock);
			params->iiwa_params.cmd_jnt_vel[0] = 0;
			params->iiwa_params.cmd_jnt_vel[1] = 0;
			params->iiwa_params.cmd_jnt_vel[2] = 0;
			params->iiwa_params.cmd_jnt_vel[3] = 0;
			params->iiwa_params.cmd_jnt_vel[4] = 0;
			params->iiwa_params.cmd_jnt_vel[5] = 0;
			params->iiwa_params.cmd_jnt_vel[6] = -(3.1416*2.0*3.1416*0.3/4.0)*sin(0.3*2.0*3.1416*t);
			pthread_mutex_unlock(&coord_state->actuation_lock);
			t += (double)dt/1000;
			// printf("%f\n", t);
		}
	}
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
	activity_t iiwa_virtual;    
	// iiwa Lidar
	ec_iiwa_virtual.create_lcsm(&iiwa_virtual, "iiwa_virtual");   
	ec_iiwa_virtual.resource_configure_lcsm(&iiwa_virtual);

	// Share memory
	iiwa_virtual_coordination_state_t *iiwa_virtual_coord_state =
			(iiwa_virtual_coordination_state_t *) iiwa_virtual.state.coordination_state;

	deinitialisation_request = &iiwa_virtual_coord_state->deinitialisation_request;

	iiwa_virtual_params_t* params = (iiwa_virtual_params_t *) iiwa_virtual.conf.params;

	params->iiwa_params.cmd_mode = POSITION;
	params->thread_time = thread_time; //ms

	// Manually 
	iiwa_virtual_coord_state->execution_request = true;

	// ### THREADS ### //
	thread_t thread_iiwa;

	// Create thread: data structure, thread name, cycle time in milliseconds
	create_thread(&thread_iiwa, "thread_iiwa", thread_time); // 4 ms

	// Register activities in threads
	register_activity(&thread_iiwa, &iiwa_virtual, "iiwa_virtual");

	// ### SHARED MEMORY ### //

	// Create POSIX threads   
	pthread_t pthread_iiwa, pthread_actuation, phtread_saving;

	pthread_create( &pthread_iiwa, NULL, do_thread_loop, ((void*) &thread_iiwa));
	pthread_create( &pthread_actuation, NULL, set_actuation, (void*) &iiwa_virtual);
	// pthread_create( &phtread_saving, NULL, save_sensor_data, (void*) &iiwa_virtual);

	// Wait for threads to finish, which means all activities must properly finish and reach the dead LCSM state
	pthread_join(pthread_iiwa, NULL);
	pthread_join(pthread_actuation, NULL);
	// pthread_join(phtread_saving, NULL);
	
	// Freeing memory
	ec_iiwa_virtual.destroy_lcsm(&iiwa_virtual);
	return 0;
}