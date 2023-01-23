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

// Basic utilities
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

// Orocos KDL
#include <chain.hpp>
#include <models.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainfksolvervel_recursive.hpp>
#include <chainiksolver.hpp>
#include <chainidsolver_recursive_newton_euler.hpp>

// iiwa controller
#include "iiwa/iiwa_controller.hpp"

KDL::Chain iiwa_robot_kdl=KDL::KukaIIWA14();

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
		activity->lcsm.state = CAPABILITY_CONFIGURATION;
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
	iiwa_controller_continuous_state_t *cts_state = (iiwa_controller_continuous_state_t *) activity->state.computational_state.continuous;
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
	iiwa_controller_params_t *params = (iiwa_controller_params_t*) activity->conf.params;
	// flush the logger to ensure all data is written
	params->logger->flush();
    activity->state.lcsm_flags.deletion_complete = true;
}

void iiwa_controller_cleaning(activity_t *activity){
    iiwa_controller_cleaning_compute(activity);
    iiwa_controller_cleaning_coordinate(activity);
    iiwa_controller_cleaning_configure(activity);
}

// capability configuration
void iiwa_controller_capability_configuration_coordinate(activity_t *activity){
	iiwa_controller_coordination_state_t * coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
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
				activity->lcsm.state = CLEANING;
				break;
		}
		update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
	}
}

void iiwa_controller_capability_configuration_configure(activity_t *activity){
	iiwa_controller_params_t *params = (iiwa_controller_params_t *) activity->conf.params;
	iiwa_controller_continuous_state_t *cts_state = (iiwa_controller_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_controller_coordination_state_t * coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;

	if (activity->lcsm.state != CAPABILITY_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "capability_configuration");
		// Update flags for next visit to the capability configuration LCS 
		activity->state.lcsm_flags.capability_configuration_complete = false;

		// Configure controller for running state
		params->max_torque = 2.0;
		params->max_wrench = 5.0;

		// Configure ABAG Controller
		params->abag_params.sat_high = 1;
		params->abag_params.sat_low = -1;

        // parameters from paper
		params->abag_params.alpha = 0.75;
		params->abag_params.bias_thresh = 0.75;
		params->abag_params.delta_bias = 0.001;
		params->abag_params.gain_thresh = 0.5;
		params->abag_params.delta_gain = 0.001;
		
		// ABAG states need to be initialized to 0
		cts_state->abag_state.bias = 0.0;
		cts_state->abag_state.gain = 0.0;
		cts_state->abag_state.control = 0.0;
		cts_state->abag_state.ek_bar = 0.0;
	}
}

void iiwa_controller_capability_configuration_compute(activity_t *activity){
	iiwa_controller_params_t* params = (iiwa_controller_params_t *) activity->conf.params;

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

	// iiwa controller first run compute cycle
	coord_state->first_run_compute_cycle = TRUE;
    activity->fsm[0].state = WAIT;
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

    // Read the state estimation from the state estimation activity
	pthread_mutex_lock(coord_state->estimate_lock);
	memcpy(&state->local_cart_pos, state->cart_pos, sizeof(state->local_cart_pos));
	memcpy(&state->local_cart_vel, state->cart_vel, sizeof(state->local_cart_vel));
	memcpy(state->local_jnt_vel, state->jnt_vel, sizeof(state->local_jnt_vel));
	pthread_mutex_unlock(coord_state->estimate_lock);

    // read goals from other, depending on control mode
    pthread_mutex_lock(&coord_state->goal_lock);
	memcpy(params->local_goal_jnt_pos, params->goal_jnt_pos, sizeof(params->goal_jnt_pos));
	switch(params->cmd_mode){
		case(POSITION): break;
		case(WRENCH):
		{
			memcpy(params->local_goal_wrench, params->goal_wrench, sizeof(params->goal_wrench));
			break;
		}
		case(TORQUE): break;
	}
    pthread_mutex_unlock(&coord_state->goal_lock);
}

void iiwa_controller_running_communicate_write(activity_t *activity){
    iiwa_controller_continuous_state_t* state = (iiwa_controller_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;
	
	// Write the motion actuation commands back to the iiwa
	pthread_mutex_lock(coord_state->actuation_lock);
	// copy everything in every case, and relevant params will be set to 0
	memcpy(state->cmd_jnt_vel, state->local_cmd_jnt_vel, sizeof(state->local_cmd_jnt_vel));
	memcpy(state->cmd_wrench, state->local_cmd_wrench, sizeof(state->local_cmd_wrench));
	memcpy(state->cmd_torques, state->local_cmd_torques, sizeof(state->local_cmd_torques));
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
			activity->lcsm.state = CAPABILITY_CONFIGURATION;
			break;
	}
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
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

	double cmd_vel;
	double prev_cmd_vel = continuous_state->local_cmd_jnt_vel[6];
	KDL::Twist local_v;

	switch(params->cmd_mode){
		case(POSITION):
		{
			//do nothing
			break;
		}
		case(WRENCH):
		{	
            // extract the z component
			local_v = continuous_state->local_cart_vel.GetTwist();
			abag(&params->abag_params, &continuous_state->abag_state, 0.05, -1*local_v.vel(2));

			for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++)
			{
				// update the 'spring' position here. 
				continuous_state->local_cmd_jnt_vel[i] = continuous_state->local_jnt_vel[i];
			}
			for (unsigned int i=0;i<6;i++)
			{
				continuous_state->local_cmd_wrench[i] = 0.0;
			}
			continuous_state->local_cmd_wrench[2] = params->max_wrench * continuous_state->abag_state.control; 

            // LOGGING
			params->logger->info("wrench_z, {}", continuous_state->local_cmd_wrench[2]);

			params->logger->info("ek_bar, {}", continuous_state->abag_state.ek_bar);
			params->logger->info("bias, {}", continuous_state->abag_state.bias);
			params->logger->info("gain, {}", continuous_state->abag_state.gain);
			params->logger->info("control, {}", continuous_state->abag_state.control);

			params->logger->info("meas_jnt_pos[0], {}", continuous_state->local_meas_jnt_pos[0]);
			params->logger->info("meas_jnt_pos[1], {}", continuous_state->local_meas_jnt_pos[1]);
			params->logger->info("meas_jnt_pos[2], {}", continuous_state->local_meas_jnt_pos[2]);
			params->logger->info("meas_jnt_pos[3], {}", continuous_state->local_meas_jnt_pos[3]);
			params->logger->info("meas_jnt_pos[4], {}", continuous_state->local_meas_jnt_pos[4]);
			params->logger->info("meas_jnt_pos[5], {}", continuous_state->local_meas_jnt_pos[5]);
			params->logger->info("meas_jnt_pos[6], {}", continuous_state->local_meas_jnt_pos[6]);

			params->logger->info("meas_jnt_vel[0], {}", continuous_state->meas_jnt_vel[0]);
			params->logger->info("meas_jnt_vel[1], {}", continuous_state->meas_jnt_vel[1]);
			params->logger->info("meas_jnt_vel[2], {}", continuous_state->meas_jnt_vel[2]);
			params->logger->info("meas_jnt_vel[3], {}", continuous_state->meas_jnt_vel[3]);
			params->logger->info("meas_jnt_vel[4], {}", continuous_state->meas_jnt_vel[4]);
			params->logger->info("meas_jnt_vel[5], {}", continuous_state->meas_jnt_vel[5]);
			params->logger->info("meas_jnt_vel[6], {}", continuous_state->meas_jnt_vel[6]);

			params->logger->info("local_vx, {}", local_v.vel(0));
			params->logger->info("local_vy, {}", local_v.vel(1));
			params->logger->info("local_vz, {}", local_v.vel(2));
			params->logger->info("cycle_time_us, {}", continuous_state->cycle_time_us);

			break;
		}
		case(TORQUE):
		{
			// local cmd torque = max_torque * control E [-1, 1]
			abag(&params->abag_params, &continuous_state->abag_state, 0.1, continuous_state->local_jnt_vel[6]);

			for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS - 1;i++)
			{
				continuous_state->local_cmd_jnt_vel[i] = 0.0;
				continuous_state->local_cmd_torques[i] = 0.0;
			}
			continuous_state->local_cmd_jnt_vel[6] = 0.0; // TODO do I need to update this parameter
			continuous_state->local_cmd_torques[6] = params->max_torque * continuous_state->abag_state.control; 
			printf("In torque control, cmd: %f \n", continuous_state->local_cmd_torques[6]);
			break;
		}
	}
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
	iiwa_controller_coordination_state_t *coord_state = (iiwa_controller_coordination_state_t *) activity->state.coordination_state;

    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;
	coord_state->deinitialisation_request = false;
	coord_state->execution_request = false;

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

void abag(abag_params_t *params, abag_state_t *state, double setpoint, double val){
	state->ek_bar = params->alpha * state->ek_bar + (1 - params->alpha) * sgn(setpoint - val);
	state->bias = saturate(state->bias + 
	    params->delta_bias * hside(fabs(state->ek_bar)-params->bias_thresh) * sgn(state->ek_bar-params->bias_thresh), 
		params->sat_low, params->sat_high);
	state->gain = saturate(state->gain + 
	    params->delta_gain * sgn(fabs(state->ek_bar) - params->gain_thresh), 
		params->sat_low, params->sat_high);
	state->control = saturate(state->bias + state->gain * sgn(setpoint - val), 
	    params->sat_low, params->sat_high);

	return;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T> int hside(T val){
    return (val > T(0));
}

template <typename T> T saturate(T val, T sat_low, T sat_high){
    if (val > sat_high){
		val = sat_high;
	}else if(val < sat_low){
		val = sat_low;
	}
    return val;
}