/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Louis Hanut and Brendan Pousett
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_controller.cpp
 * @date September 15, 2022
 **/

#include "string.h"
#include <time.h>
#include <math.h>

#include "iiwa/iiwa_controller.hpp"
#include <iostream>

// TODO create local fsm states
#define WAIT 0
#define APPROACH 1
#define BLEND 2
#define SLOW 3
#define STOP 4

/** 
 * The config() has to be scheduled everytime a change in the LCSM occurs, 
 * so it properly configures the schedule for the next iteration according
 * to the LCSM state, resources, task, ..  
 * @param[in] activity data structure for the hokuyo activity
*/
void iiwa_controller_config(activity_t *activity){
	// Remove config() from the eventloop schedule in the next iteration
	remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
	// Deciding which schedule to add
	switch (activity->lcsm.state){
		case CREATION:
			printf("In creation state Controller \n");
			add_schedule_to_eventloop(&activity->schedule_table, "creation");
			break;
		case RESOURCE_CONFIGURATION:
			printf("In resource configuration state Controller\n");
			add_schedule_to_eventloop(&activity->schedule_table, "resource_configuration");
			break;
		case CAPABILITY_CONFIGURATION:
			printf("In capability configuration state Controller\n");
			add_schedule_to_eventloop(&activity->schedule_table, "capability_configuration");
            break;
		case PAUSING:
			printf("In pausing state Controller\n");
			add_schedule_to_eventloop(&activity->schedule_table, "pausing");
			break;
		case RUNNING:
			printf("In running state Controller\n");
			add_schedule_to_eventloop(&activity->schedule_table, "running");
			break;
		case CLEANING:
			printf("In cleaning state Controller\n");
			add_schedule_to_eventloop(&activity->schedule_table, "cleaning");
			break;
		case DONE:
			break;
	}
};

// Creation
void iiwa_controller_creation_coordinate(activity_t *activity){
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;

	// Coordinating own activity
	if (activity->state.lcsm_flags.creation_complete)
		activity->lcsm.state = RESOURCE_CONFIGURATION;
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_controller_creation_configure(activity_t *activity){
	if (activity->lcsm.state != CREATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "creation");
	}
}

// Allocating memory here
void iiwa_controller_creation_compute(activity_t *activity){
    activity->fsm = (FSM_t *) malloc(sizeof(FSM_t));
	activity->state.lcsm_flags.creation_complete = true;
}

void iiwa_controller_creation(activity_t *activity){
	iiwa_controller_creation_compute(activity);
	iiwa_controller_creation_coordinate(activity);
	iiwa_controller_creation_configure(activity);
}

// Cleaning
void iiwa_controller_cleaning_coordinate(activity_t *activity){
    iiwa_controller_coordination_state_t * coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
    // Coordinating own activity
    if (activity->state.lcsm_flags.deletion_complete)
        activity->lcsm.state = DONE;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_controller_cleaning_configure(activity_t *activity){
    if (activity->lcsm.state != CLEANING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "cleaning");
    }
}

void iiwa_controller_cleaning_compute(activity_t *activity){
    activity->state.lcsm_flags.deletion_complete = true;
}

void iiwa_controller_cleaning(activity_t *activity){
    iiwa_controller_cleaning_compute(activity);
    iiwa_controller_cleaning_coordinate(activity);
    iiwa_controller_cleaning_configure(activity);
}

// Resource configuration
void iiwa_controller_resource_configuration_coordinate(activity_t *activity){
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;
	
	// Coordinating own activity
	if (activity->state.lcsm_flags.creation_complete)
		switch (activity->state.lcsm_protocol){ 
			case INITIALISATION:
				activity->lcsm.state = CAPABILITY_CONFIGURATION;
				break;
			case EXECUTION:
				activity->lcsm.state = CAPABILITY_CONFIGURATION;
				break;
			case DEINITIALISATION:
				activity->lcsm.state = CLEANING;
				break;
		}
		update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_controller_resource_configuration_configure(activity_t *activity){
	if (activity->lcsm.state != RESOURCE_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
		// Update flags for next visit to the resource configuration LCS 
		activity->state.lcsm_flags.resource_configuration_complete = false;
	}
}

//I need to update this function 
void iiwa_controller_resource_configuration_compute(activity_t *activity){
	iiwa_controller_coordination_state_t * coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;

	activity->state.lcsm_flags.resource_configuration_complete = true;

	// TODO Update this
	activity->fsm[0].state = WAIT;
}

void iiwa_controller_resource_configuration(activity_t *activity){
	iiwa_controller_resource_configuration_compute(activity);
	iiwa_controller_resource_configuration_coordinate(activity);
	iiwa_controller_resource_configuration_configure(activity);
}

// capability configuration
void iiwa_controller_capability_configuration_coordinate(activity_t *activity){
	iiwa_controller_coordination_state_t * coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	// if (coord_state->execution_request)
	// 	activity->state.lcsm_protocol = EXECUTION;
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;

	if (activity->state.lcsm_flags.capability_configuration_complete){
		switch (activity->state.lcsm_protocol){ 
			case INITIALISATION:
				activity->lcsm.state = PAUSING;
				break;
			case EXECUTION:
				activity->lcsm.state = RUNNING;
				break;
			case DEINITIALISATION:
				activity->lcsm.state = RESOURCE_CONFIGURATION;
				break;
		}
		update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
	}
}

void iiwa_controller_capability_configuration_configure(activity_t *activity){
	if (activity->lcsm.state != CAPABILITY_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "capability_configuration");
		// Update flags for next visit to the capability configuration LCS 
		activity->state.lcsm_flags.capability_configuration_complete = false;
	}
}

void iiwa_controller_capability_configuration_compute(activity_t *activity){
	iiwa_controller_params_t* params = (iiwa_controller_params_t *) activity->conf.params;
	// TODO bugfix the controller never leaves the capability configuration state
	if (activity->state.lcsm_protocol == DEINITIALISATION){
		activity->state.lcsm_flags.capability_configuration_complete = true;
	}
	activity->state.lcsm_flags.capability_configuration_complete = true;
}

void iiwa_controller_capability_configuration(activity_t *activity){
	iiwa_controller_capability_configuration_compute(activity);
	iiwa_controller_capability_configuration_coordinate(activity);
	iiwa_controller_capability_configuration_configure(activity);
}

//PAUSING
void iiwa_controller_pausing_coordinate(activity_t *activity){
	iiwa_controller_coordination_state_t * coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	iiwa_controller_continuous_state_t *continuous_state = (iiwa_controller_continuous_state_t *) activity->state.computational_state.continuous;
	// Coordinating with other activities
	if (coord_state->execution_request)
		activity->state.lcsm_protocol = EXECUTION;
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;
	// Coordinating own activity
	switch (activity->state.lcsm_protocol){ 
		case EXECUTION:
			activity->lcsm.state = RUNNING;
			break;
		case DEINITIALISATION:
			activity->lcsm.state = CAPABILITY_CONFIGURATION;
			break;
	}
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_controller_pausing_configure(activity_t *activity){
	if (activity->lcsm.state != PAUSING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "pausing");
	}
}

void iiwa_controller_pausing(activity_t *activity){
	iiwa_controller_pausing_coordinate(activity);
	iiwa_controller_pausing_configure(activity);
}

// Running
void iiwa_controller_running_communicate_read(activity_t *activity){
	iiwa_controller_params_t* params = (iiwa_controller_params_t *) activity->conf.params;
	iiwa_controller_continuous_state_t* state = (iiwa_controller_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;

    // Read the sensors from iiwa
	pthread_mutex_lock(coord_state->sensor_lock);
	for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++){
		params->local_sensors.meas_torques[i] = params->iiwa_controller_params->iiwa_sensors.meas_torques[i];
		params->local_sensors.meas_jnt_pos[i] = params->iiwa_controller_params->iiwa_sensors.meas_jnt_pos[i];
		params->local_sensors.meas_ext_torques[i] = params->iiwa_controller_params->iiwa_sensors.meas_ext_torques[i];
	}
	pthread_mutex_unlock(coord_state->sensor_lock);

	// Read the goal from OTHER activity 
	pthread_mutex_lock(&coord_state->goal_lock);
	for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++){
		params->local_goal_jnt_pos[i] = params->goal_jnt_pos[i];
		state->jnt_pos_error[i] = params->goal_jnt_pos[i] - params->local_sensors.meas_jnt_pos[i];
	}
	pthread_mutex_unlock(&coord_state->goal_lock);
}

void iiwa_controller_running_communicate_write(activity_t *activity){
    iiwa_controller_continuous_state_t* state = (iiwa_controller_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	
	// Write the commanded velocity
	pthread_mutex_lock(coord_state->actuation_lock);
	for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++){
		state->iiwa_controller_state->cmd_jnt_vel[i] = state->local_cmd_jnt_vel[i];
	}
	pthread_mutex_unlock(coord_state->actuation_lock);
}

void iiwa_controller_running_coordinate(activity_t *activity){
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	iiwa_controller_continuous_state_t* continuous_state = (iiwa_controller_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_controller_params_t* params = (iiwa_controller_params_t *) activity->conf.params;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;
	
	// Go to pausing when not commanding active and wait there until active
	if (coord_state->commanding_not_active)
		activity->lcsm.state = PAUSING;

	switch (activity->state.lcsm_protocol){ 
		case DEINITIALISATION:
			activity->lcsm.state = RESOURCE_CONFIGURATION;
			break;
	}
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);

	double error = continuous_state->jnt_pos_error[6];
	double cmd_vel = continuous_state->iiwa_controller_state->cmd_jnt_vel[6];


	// FSM TRANSITIONS
	switch (activity->fsm[0].state){
		case WAIT:
		    if (fabs(error) > params->goal_buffer[6]){
				activity->fsm[0].state = APPROACH;
			}
		case APPROACH:
		    if (fabs(error) < params->approach_buffer[6] && sgn(cmd_vel) == sgn(error)){
				// What to do here;
				params->approach_jnt_vel[6] = continuous_state->local_cmd_jnt_vel[6];
                activity->fsm[0].state = BLEND;
			}
		case BLEND:
		    if (fabs(error) > params->approach_buffer[6] || sgn(cmd_vel) != sgn(error)){
				activity->fsm[0].state = APPROACH;
			}else if (fabs(error) < params->slow_buffer[6]){
                activity->fsm[0].state = SLOW;
			}
		case SLOW:
		    if (fabs(error) > params->slow_buffer[6] || sgn(cmd_vel) != sgn(error)){
				activity->fsm[0].state = APPROACH;
			}else if (fabs(error) < params->goal_buffer[6]){
				activity->fsm[0].state = STOP;
			}
		case STOP:
		    break;
	}

	printf("The controller state is: %d \n", activity->fsm[0].state);
}

void iiwa_controller_running_configure(activity_t *activity){
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	if (activity->lcsm.state != RUNNING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "running");
		// Update flags
		coord_state->commanding_not_active = false;
	}
}

void iiwa_controller_running_compute(activity_t *activity){
	iiwa_controller_params_t* params = (iiwa_controller_params_t *) activity->conf.params;
	iiwa_controller_continuous_state_t *continuous_state = (iiwa_controller_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	iiwa_controller_discrete_state_t *discrete_state = (iiwa_controller_discrete_state_t *) activity->state.computational_state.discrete;

	long cycle_time; // cycle time in secondefined types if operator '>' is overloaded

	double cmd_vel;
	double prev_cmd_vel = continuous_state->local_cmd_jnt_vel[6];
	double alpha;

	// compute the current timespec, time difference, and then the previous timespec
	// TODO move this time tracking into the activity.h data structure
	timespec_get(&continuous_state->current_timespec, TIME_UTC);
	if (discrete_state->first_run_compute_cycle){
		cycle_time = 0.0;
		discrete_state->first_run_compute_cycle = FALSE;
	}else{
	    cycle_time = (continuous_state->current_timespec.tv_nsec - continuous_state->prev_timespec.tv_nsec) / 1000000000.0;
	}
	memcpy(&continuous_state->prev_timespec, &continuous_state->current_timespec, sizeof(continuous_state->current_timespec));

	double error = params->local_goal_jnt_pos[6] - params->local_sensors.meas_jnt_pos[6];
	int direction = sgn(error); 

	switch (activity->fsm[0].state){
		case WAIT:
			cmd_vel = 0.0;
		case APPROACH:
			cmd_vel = prev_cmd_vel + direction * params->jnt_accel[6] * cycle_time;
			if (fabs(cmd_vel) >= params->max_jnt_vel[6]){
                cmd_vel = direction * params->max_jnt_vel[6];
			}
		case BLEND:
            alpha = fabs(error) - params->slow_buffer[6] / (params->approach_buffer[6] - params->slow_buffer[6]);
			cmd_vel = direction * (params->slow_jnt_vel[6] + alpha * (params->approach_jnt_vel[6] - params->slow_jnt_vel[6]));
		case SLOW:
			cmd_vel = params->slow_jnt_vel[6] * direction;
		case STOP:
		    cmd_vel = 0;
	}

    // write the command velocity to the local variable
	continuous_state->local_cmd_jnt_vel[6] = cmd_vel;
}

void iiwa_controller_running(activity_t *activity){
	iiwa_controller_running_communicate_read(activity);
	iiwa_controller_running_coordinate(activity);
	iiwa_controller_running_compute(activity);
	iiwa_controller_running_communicate_write(activity);
	iiwa_controller_running_configure(activity);
}

// SCHEDULER 
void iiwa_controller_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t) iiwa_controller_config, 
        activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");
    
    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t) iiwa_controller_creation, 
        activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, "creation");

    schedule_t schedule_resource_config = {.number_of_functions = 0};
    register_function(&schedule_resource_config, (function_ptr_t) iiwa_controller_resource_configuration, 
        activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_config, "resource_configuration");

	schedule_t schedule_capability_config = {.number_of_functions = 0};
    register_function(&schedule_capability_config, (function_ptr_t) iiwa_controller_capability_configuration, 
        activity, "capability_configuration");
    register_schedule(&activity->schedule_table, schedule_capability_config, "capability_configuration");
    
    schedule_t schedule_pausing = {.number_of_functions = 0};
    register_function(&schedule_pausing, (function_ptr_t) iiwa_controller_pausing, 
        activity, "pausing");
    register_schedule(&activity->schedule_table, schedule_pausing, "pausing");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t) iiwa_controller_running, 
        activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, "running");

	schedule_t schedule_cleaning = {.number_of_functions = 0};
    register_function(&schedule_cleaning, (function_ptr_t) iiwa_controller_cleaning, 
        activity, "cleaning");
    register_schedule(&activity->schedule_table, schedule_cleaning, 
        "cleaning");
}

void iiwa_controller_create_lcsm(activity_t* activity, const char* name_activity){
    activity->conf.params = malloc(sizeof(iiwa_controller_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(iiwa_controller_continuous_state_t));
    activity->state.computational_state.discrete = malloc(sizeof(iiwa_controller_discrete_state_t));
    activity->state.coordination_state = malloc(sizeof(iiwa_controller_coordination_state_t));
}

void iiwa_controller_resource_configure_lcsm(activity_t *activity){
    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;

    // Schedule table (adding config() for the first eventloop iteration)
    iiwa_controller_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void iiwa_controller_destroy_lcsm(activity_t* activity){
    destroy_activity(activity);
}

const iiwa_controller_t ec_iiwa_controller ={
    .create_lcsm = iiwa_controller_create_lcsm,
    .resource_configure_lcsm = iiwa_controller_resource_configure_lcsm,
    .destroy_lcsm = iiwa_controller_destroy_lcsm,
};