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
#include <aacal/thread/thread.h>
#include <aacal/activity/activity.h>
#include <coordination-libs/lcsm/lcsm.h>

#include <iiwa_activity.hpp>

#include <pthread.h>
#include <unistd.h>


bool *deinitialisation_request;
int RUN = 1;
int Prev = 1;

static void sigint_handler(int sig){
	if (deinitialisation_request==NULL){
		printf("Ops.. deinitalisation request is a NULL pointer.\n");
	}else{
		printf("\nDeinitialising iiwa activity\n");
		*deinitialisation_request = true;
	}
	
	RUN = 0;
}

void* set_actuation(void* activity){
	activity_t *iiwa_activity = (activity_t*) activity; 
	iiwa_activity_params_t* params = (iiwa_activity_params_t *) iiwa_activity->conf.params;
	iiwa_activity_continuous_state_t *continuous_state =
	(iiwa_activity_continuous_state_t *) iiwa_activity->state.computational_state.continuous;
	iiwa_activity_coordination_state_t *coord_state =
	(iiwa_activity_coordination_state_t *) iiwa_activity->state.coordination_state;  

	while(!(*deinitialisation_request)){
	if (iiwa_activity->lcsm.state == RUNNING){
		// Copying data
		pthread_mutex_lock(&coord_state->actuation_lock);
		for (int i=0; i<5; i++){
			params->iiwa_params.cmd_torques[i] = 0*Prev;
		}
		params->iiwa_params.cmd_torques[5] = 0;
		params->iiwa_params.cmd_torques[6] = 0;
		pthread_mutex_unlock(&coord_state->actuation_lock);
		// Prev = Prev*-1;
	}
	sleep(2);  // time in seconds
	}
}

int main(int argc, char**argv){
	signal(SIGINT, sigint_handler);
	
	// ### ACTIVITIES ### //   
	activity_t iiwa_activity;    
	// iiwa Lidar
	ec_iiwa_activity.create_lcsm(&iiwa_activity, "iiwa_activity");   
	ec_iiwa_activity.resource_configure_lcsm(&iiwa_activity);

	// Share memory
	iiwa_activity_coordination_state_t *iiwa_activity_coord_state =
			(iiwa_activity_coordination_state_t *) iiwa_activity.state.coordination_state;

	deinitialisation_request = &iiwa_activity_coord_state->deinitialisation_request;

	iiwa_activity_params_t* params = (iiwa_activity_params_t *) iiwa_activity.conf.params;

	strcpy(params->iiwa_params.fri_ip,"192.168.1.50");
	params->iiwa_params.fri_port = 30100;
	params->iiwa_params.cmd_mode = POSITION;

	// Manually 
	iiwa_activity_coord_state->execution_request = true;

	// ### THREADS ### //
	thread_t thread_iiwa;

	// Create thread: data structure, thread name, cycle time in milliseconds
	create_thread(&thread_iiwa, "thread_iiwa", 4); // 4 ms

	// Register activities in threads
	register_activity(&thread_iiwa, &iiwa_activity, "iiwa_activity");

	// ### SHARED MEMORY ### //

	// Create POSIX threads   
	pthread_t pthread_iiwa, pthread_actuation;

	pthread_create( &pthread_iiwa, NULL, do_thread_loop, ((void*) &thread_iiwa));
	pthread_create( &pthread_actuation, NULL, set_actuation, (void*) &iiwa_activity);

	// Wait for threads to finish, which means all activities must properly finish and reach the dead LCSM state
	pthread_join(pthread_iiwa, NULL);
	pthread_join(pthread_actuation, NULL);
	
	// Freeing memory
	// ec_iiwa_activity.destroy_lcsm(&iiwa_activity);
	return 0;
}