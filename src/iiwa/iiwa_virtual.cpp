/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_virtual.c
 * @date September 15, 2022
 **/

#include <string.h>
#include <time.h>

#include "iiwa/iiwa_virtual.hpp"
#include <iostream>

FILE *fpt;
/** 
 * The config() has to be scheduled everytime a change in the LCSM occurs, 
 * so it properly configures the schedule for the next iteration according
 * to the LCSM state, resources, task, ..  
 * @param[in] activity data structure for the hokuyo activity
*/
void iiwa_virtual_config(activity_t *activity){
	// Remove config() from the eventloop schedule in the next iteration
	remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
	// Deciding which schedule to add
	switch (activity->lcsm.state){
		case CREATION:
			printf("iiwa virtual in creation state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "creation");
			break;
		case RESOURCE_CONFIGURATION:
			printf("iiwa virtual in resource configuration state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "resource_configuration");
			break;
		case CAPABILITY_CONFIGURATION:
			printf("iiwa virtual in capability configuration state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "capability_configuration");
            break;
		case PAUSING:
			printf("iiwa virtual in pausing state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "pausing");
			break;
		case RUNNING:
			printf("iiwa virtual in running state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "running");
			break;
		case CLEANING:
			printf("iiwa virtual in cleaning state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "cleaning");
			break;
		case DONE:
			break;
	}
};

// Creation
void iiwa_virtual_creation_coordinate(activity_t *activity){
	iiwa_virtual_coordination_state_t *coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;

	// Coordinating own activity
	if (activity->state.lcsm_flags.creation_complete)
		activity->lcsm.state = RESOURCE_CONFIGURATION;
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_virtual_creation_configure(activity_t *activity){
	if (activity->lcsm.state != CREATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "creation");
	}
}

void iiwa_virtual_creation_compute(activity_t *activity){
	/// @brief Set the creation_complete flag to true without connecting to any physical
	/// iiwa robot. 
	/// @param activity 
	activity->state.lcsm_flags.creation_complete = true;
}

void iiwa_virtual_creation(activity_t *activity){
	iiwa_virtual_creation_compute(activity);
	iiwa_virtual_creation_coordinate(activity);
	iiwa_virtual_creation_configure(activity);
}

// Cleaning
void iiwa_virtual_cleaning_coordinate(activity_t *activity){
    iiwa_virtual_coordination_state_t * coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
    // Coordinating own activity
    if (activity->state.lcsm_flags.deletion_complete)
        activity->lcsm.state = DONE;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_virtual_cleaning_configure(activity_t *activity){
    if (activity->lcsm.state != CLEANING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "cleaning");
    }
}

void iiwa_virtual_cleaning_compute(activity_t *activity){
    activity->state.lcsm_flags.deletion_complete = true;
}

void iiwa_virtual_cleaning(activity_t *activity){
    iiwa_virtual_cleaning_compute(activity);
    iiwa_virtual_cleaning_coordinate(activity);
    iiwa_virtual_cleaning_configure(activity);
}

// Resource configuration
void iiwa_virtual_resource_configuration_coordinate(activity_t *activity){
	iiwa_virtual_coordination_state_t *coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
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

void iiwa_virtual_resource_configuration_configure(activity_t *activity){
	if (activity->lcsm.state != RESOURCE_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
		// Update flags for next visit to the resource configuration LCS 
		activity->state.lcsm_flags.resource_configuration_complete = false;
	}
}

void iiwa_virtual_resource_configuration_compute(activity_t *activity){
    iiwa_virtual_coordination_state_t* coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
	iiwa_virtual_continuous_state_t* continuous_state = (iiwa_virtual_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_state_t *iiwa_state = (iiwa_state_t *) &continuous_state->iiwa_state;
	switch (activity->state.lcsm_protocol){
		case INITIALISATION:
		    // set the initial configuration of the virtual robot
			pthread_mutex_lock(&coord_state->sensor_lock);
			for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++){
				iiwa_state->iiwa_sensors.meas_jnt_pos[i] = 0;
			}
			pthread_mutex_unlock(&coord_state->sensor_lock);
			printf("[iiwa activity] Succesfully connected iiwa robot\n");
			break;
		case DEINITIALISATION:
			// do nothing 
			break;
	}
	activity->state.lcsm_flags.resource_configuration_complete = true;
}

void iiwa_virtual_resource_configuration(activity_t *activity){
	iiwa_virtual_resource_configuration_compute(activity);
	iiwa_virtual_resource_configuration_coordinate(activity);
	iiwa_virtual_resource_configuration_configure(activity);
}

// capability configuration
void iiwa_virtual_capability_configuration_coordinate(activity_t *activity){
	iiwa_virtual_coordination_state_t * coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
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

void iiwa_virtual_capability_configuration_configure(activity_t *activity){
	if (activity->lcsm.state != CAPABILITY_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "capability_configuration");
		// Update flags for next visit to the capability configuration LCS 
		activity->state.lcsm_flags.capability_configuration_complete = false;
	}
}

void iiwa_virtual_capability_configuration_compute(activity_t *activity){
	// Connect to iiwa robot
	iiwa_virtual_params_t* params = (iiwa_virtual_params_t *) activity->conf.params;
	iiwa_virtual_continuous_state_t* continuous_state = (iiwa_virtual_continuous_state_t *) activity->state.computational_state.continuous;

    // check if the command mode is set to POSITION...
	switch (activity->state.lcsm_protocol){
		case INITIALISATION:
		    // '1' is position as defined in EClientCommandMode
			if (params->iiwa_params.cmd_mode == POSITION){
				activity->state.lcsm_flags.capability_configuration_complete = true;
			}
			else{
				printf("[ERROR] Commanding mode different, deinitializing \n Virtual only valid for POSITION Mode");
				activity->state.lcsm_protocol = DEINITIALISATION;
			}
			break;
		case DEINITIALISATION:
			activity->state.lcsm_flags.capability_configuration_complete = true;
			break;
	}
}

void iiwa_virtual_capability_configuration(activity_t *activity){
	iiwa_virtual_capability_configuration_compute(activity);
	iiwa_virtual_capability_configuration_coordinate(activity);
	iiwa_virtual_capability_configuration_configure(activity);
}

void iiwa_virtual_pausing_coordinate(activity_t *activity){
	iiwa_virtual_coordination_state_t *coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
	iiwa_virtual_continuous_state_t *continuous_state = (iiwa_virtual_continuous_state_t *) activity->state.computational_state.continuous;
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

void iiwa_virtual_pausing_configure(activity_t *activity){
	if (activity->lcsm.state != PAUSING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "pausing");
	}
}

void iiwa_virtual_pausing(activity_t *activity){
	iiwa_virtual_pausing_coordinate(activity);
	iiwa_virtual_pausing_configure(activity);
}

// Running
void iiwa_virtual_running_communicate(activity_t *activity){
	iiwa_virtual_params_t* params = (iiwa_virtual_params_t *) activity->conf.params;
	iiwa_virtual_continuous_state_t *continuous_state = (iiwa_virtual_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_virtual_coordination_state_t *coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
	iiwa_state_t *iiwa_state = (iiwa_state_t *) &continuous_state->iiwa_state;

    // read the commanded set points
    pthread_mutex_lock(&coord_state->actuation_lock);
	for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++)
		{
			iiwa_state->iiwa_actuation_input.cmd_jnt_vel[i] = params->iiwa_params.cmd_jnt_vel[i];
		}
	pthread_mutex_unlock(&coord_state->actuation_lock);

}

void iiwa_virtual_running_coordinate(activity_t *activity){
	iiwa_virtual_coordination_state_t *coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
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
}

void iiwa_virtual_running_configure(activity_t *activity){
	iiwa_virtual_coordination_state_t *coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
	if (activity->lcsm.state != RUNNING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "running");
		// Update flags
		coord_state->commanding_not_active = false;
	}
}

void iiwa_virtual_running_compute(activity_t *activity){
	iiwa_virtual_params_t* params = (iiwa_virtual_params_t *) activity->conf.params;
	iiwa_virtual_continuous_state_t *continuous_state = (iiwa_virtual_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_virtual_coordination_state_t *coord_state = (iiwa_virtual_coordination_state_t *) activity->state.coordination_state;
	iiwa_state_t *iiwa_state = (iiwa_state_t *) &continuous_state->iiwa_state;


	pthread_mutex_lock(&coord_state->sensor_lock);

    // write the current set points to a text file
    fpt = fopen("meas_jnt_pos.csv", "a+");
	fprintf(fpt, "%f, %f, %f, %f, %f, %f, %f \n", 
	            iiwa_state->iiwa_sensors.meas_jnt_pos[0],
	            iiwa_state->iiwa_sensors.meas_jnt_pos[1],
	            iiwa_state->iiwa_sensors.meas_jnt_pos[2],
	            iiwa_state->iiwa_sensors.meas_jnt_pos[3],
	            iiwa_state->iiwa_sensors.meas_jnt_pos[4],
	            iiwa_state->iiwa_sensors.meas_jnt_pos[5],
	            iiwa_state->iiwa_sensors.meas_jnt_pos[6]);
	fclose(fpt);

    // update the meas_jnt_pos on the iiwa using cmd_jnt_vel
	for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++){
		iiwa_state->iiwa_sensors.meas_jnt_pos[i] += params->thread_time/1000.0 * iiwa_state->iiwa_actuation_input.cmd_jnt_vel[i];
	}
	pthread_mutex_unlock(&coord_state->sensor_lock);

}

void iiwa_virtual_running(activity_t *activity){
	iiwa_virtual_running_communicate(activity);
	iiwa_virtual_running_coordinate(activity);
	iiwa_virtual_running_configure(activity);
	iiwa_virtual_running_compute(activity);
}

// SCHEDULER 
void iiwa_virtual_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t) iiwa_virtual_config, 
        activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");
    
    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t) iiwa_virtual_creation, 
        activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, "creation");

    schedule_t schedule_resource_config = {.number_of_functions = 0};
    register_function(&schedule_resource_config, (function_ptr_t) iiwa_virtual_resource_configuration, 
        activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_config, "resource_configuration");

	schedule_t schedule_capability_config = {.number_of_functions = 0};
    register_function(&schedule_capability_config, (function_ptr_t) iiwa_virtual_capability_configuration, 
        activity, "capability_configuration");
    register_schedule(&activity->schedule_table, schedule_capability_config, "capability_configuration");
    
    schedule_t schedule_pausing = {.number_of_functions = 0};
    register_function(&schedule_pausing, (function_ptr_t) iiwa_virtual_pausing, 
        activity, "pausing");
    register_schedule(&activity->schedule_table, schedule_pausing, "pausing");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t) iiwa_virtual_running, 
        activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, "running");

	schedule_t schedule_cleaning = {.number_of_functions = 0};
    register_function(&schedule_cleaning, (function_ptr_t) iiwa_virtual_cleaning, 
        activity, "cleaning");
    register_schedule(&activity->schedule_table, schedule_cleaning, 
        "cleaning");
}

void iiwa_virtual_create_lcsm(activity_t* activity, const char* name_activity){
    activity->conf.params = malloc(sizeof(iiwa_virtual_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(iiwa_virtual_continuous_state_t));
    activity->state.coordination_state = malloc(sizeof(iiwa_virtual_coordination_state_t));
}

void iiwa_virtual_resource_configure_lcsm(activity_t *activity){
    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;

    // Schedule table (adding config() for the first eventloop iteration)
    iiwa_virtual_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void iiwa_virtual_destroy_lcsm(activity_t* activity){
    destroy_activity(activity);
}

const iiwa_virtual_t ec_iiwa_virtual ={
    .create_lcsm = iiwa_virtual_create_lcsm,
    .resource_configure_lcsm = iiwa_virtual_resource_configure_lcsm,
    .destroy_lcsm = iiwa_virtual_destroy_lcsm,
};