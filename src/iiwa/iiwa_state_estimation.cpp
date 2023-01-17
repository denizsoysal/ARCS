/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Louis Hanut and Brendan Pousett
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_state_estimation.cpp
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

// iiwa state_estimation
#include "iiwa/iiwa_state_estimation.hpp"

KDL::Chain iiwa_robot_estimate=KDL::KukaIIWA14();
// Vector gravity = Vector(0.0,0.0,-9.81);
KDL::ChainFkSolverPos_recursive fksolver_estimate(iiwa_robot_estimate);
KDL::ChainFkSolverVel_recursive velksolver_estimate(iiwa_robot_estimate);

/** 
 * The config() has to be scheduled everytime a change in the LCSM occurs, 
 * so it properly configures the schedule for the next iteration according
 * to the LCSM state, resources, task, ..  
 * @param[in] activity data structure for the hokuyo activity
*/
void iiwa_state_estimation_config(activity_t *activity){
	// Remove config() from the eventloop schedule in the next iteration
	remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
	// Deciding which schedule to add
	switch (activity->lcsm.state){
		case CREATION:
			printf("In creation state State Estimation \n");
			add_schedule_to_eventloop(&activity->schedule_table, "creation");
			break;
		case CAPABILITY_CONFIGURATION:
			printf("In capability configuration state State Estimation\n");
			add_schedule_to_eventloop(&activity->schedule_table, "capability_configuration");
            break;
		case PAUSING:
			printf("In pausing state State Estimation\n");
			add_schedule_to_eventloop(&activity->schedule_table, "pausing");
			break;
		case RUNNING:
			printf("In running state State Estimation\n");
			add_schedule_to_eventloop(&activity->schedule_table, "running");
			break;
		case CLEANING:
			printf("In cleaning state State Estimation\n");
			add_schedule_to_eventloop(&activity->schedule_table, "cleaning");
			break;
		case DONE:
			break;
	}
};

// Creation
void iiwa_state_estimation_creation_coordinate(activity_t *activity){
	iiwa_state_estimation_coordination_state_t *coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;

	// Coordinating own activity
	if (activity->state.lcsm_flags.creation_complete)
		activity->lcsm.state = CAPABILITY_CONFIGURATION;
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_state_estimation_creation_configure(activity_t *activity){
	if (activity->lcsm.state != CREATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "creation");
	}
}

// Allocating memory here
void iiwa_state_estimation_creation_compute(activity_t *activity){
	iiwa_state_estimation_continuous_state_t *cts_state = (iiwa_state_estimation_continuous_state_t *) activity->state.computational_state.continuous;
    cts_state->local_q = KDL::JntArray(LBRState::NUMBER_OF_JOINTS);
	cts_state->local_qd = KDL::JntArrayVel(LBRState::NUMBER_OF_JOINTS);

	activity->state.lcsm_flags.creation_complete = true;
}

void iiwa_state_estimation_creation(activity_t *activity){
	iiwa_state_estimation_creation_compute(activity);
	iiwa_state_estimation_creation_coordinate(activity);
	iiwa_state_estimation_creation_configure(activity);
}

// Cleaning
void iiwa_state_estimation_cleaning_coordinate(activity_t *activity){
    iiwa_state_estimation_coordination_state_t * coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;
    // Coordinating own activity
    if (activity->state.lcsm_flags.deletion_complete)
        activity->lcsm.state = DONE;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_state_estimation_cleaning_configure(activity_t *activity){
    if (activity->lcsm.state != CLEANING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "cleaning");
    }
}

void iiwa_state_estimation_cleaning_compute(activity_t *activity){
	iiwa_state_estimation_params_t *params = (iiwa_state_estimation_params_t*) activity->conf.params;
	// flush the logger to ensure all data is written
	params->logger->flush();
    activity->state.lcsm_flags.deletion_complete = true;
}

void iiwa_state_estimation_cleaning(activity_t *activity){
    iiwa_state_estimation_cleaning_compute(activity);
    iiwa_state_estimation_cleaning_coordinate(activity);
    iiwa_state_estimation_cleaning_configure(activity);
	printf("finished cleaning state estimation\n");
}

// capability configuration
void iiwa_state_estimation_capability_configuration_coordinate(activity_t *activity){
	iiwa_state_estimation_coordination_state_t * coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;
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

void iiwa_state_estimation_capability_configuration_configure(activity_t *activity){
	iiwa_state_estimation_params_t *params = (iiwa_state_estimation_params_t *) activity->conf.params;
	iiwa_state_estimation_continuous_state_t *cts_state = (iiwa_state_estimation_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_state_estimation_coordination_state_t * coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;

	if (activity->lcsm.state != CAPABILITY_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "capability_configuration");
		// Update flags for next visit to the capability configuration LCS 
		activity->state.lcsm_flags.capability_configuration_complete = false;
	}

	params->logger->info("Capability configuration");
}

void iiwa_state_estimation_capability_configuration_compute(activity_t *activity){
	iiwa_state_estimation_params_t* params = (iiwa_state_estimation_params_t *) activity->conf.params;

	if (activity->state.lcsm_protocol == DEINITIALISATION){
		activity->state.lcsm_flags.capability_configuration_complete = true;
	}
	activity->state.lcsm_flags.capability_configuration_complete = true;
}

void iiwa_state_estimation_capability_configuration(activity_t *activity){
	iiwa_state_estimation_capability_configuration_compute(activity);
	iiwa_state_estimation_capability_configuration_coordinate(activity);
	iiwa_state_estimation_capability_configuration_configure(activity);
}

//PAUSING
void iiwa_state_estimation_pausing_coordinate(activity_t *activity){
	iiwa_state_estimation_coordination_state_t * coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;

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

	// iiwa state_estimation first run compute cycle
	coord_state->first_run_compute_cycle = TRUE;
}

void iiwa_state_estimation_pausing_configure(activity_t *activity){
	if (activity->lcsm.state != PAUSING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "pausing");
	}
}

void iiwa_state_estimation_pausing(activity_t *activity){
	iiwa_state_estimation_pausing_coordinate(activity);
	iiwa_state_estimation_pausing_configure(activity);
}

// Running
void iiwa_state_estimation_running_communicate(activity_t *activity){
	iiwa_state_estimation_params_t* params = (iiwa_state_estimation_params_t *) activity->conf.params;
	iiwa_state_estimation_continuous_state_t* state = (iiwa_state_estimation_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_state_estimation_coordination_state_t *coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;

    // cache the previous values of vars
	memcpy(state->jnt_pos_prev, state->local_meas_jnt_pos, sizeof(state->local_meas_jnt_pos));

    // Read the sensors from iiwa
	pthread_mutex_lock(coord_state->sensor_lock);
	memcpy(state->local_meas_torques, state->meas_torques, sizeof(state->local_meas_torques));
	memcpy(state->local_meas_jnt_pos, state->meas_jnt_pos, sizeof(state->local_meas_jnt_pos));
    memcpy(state->local_meas_ext_torques, state->meas_ext_torques, sizeof(state->local_meas_ext_torques));
	pthread_mutex_unlock(coord_state->sensor_lock);	
}

void iiwa_state_estimation_running_coordinate(activity_t *activity){
	iiwa_state_estimation_coordination_state_t *coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;
	iiwa_state_estimation_continuous_state_t* continuous_state = (iiwa_state_estimation_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_state_estimation_params_t* params = (iiwa_state_estimation_params_t *) activity->conf.params;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;

	switch (activity->state.lcsm_protocol){ 
		case DEINITIALISATION:
			activity->lcsm.state = CAPABILITY_CONFIGURATION;
			break;
	}
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_state_estimation_running_configure(activity_t *activity){
	iiwa_state_estimation_coordination_state_t *coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;
	if (activity->lcsm.state != RUNNING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "running");
	}
}

void iiwa_state_estimation_running_compute(activity_t *activity){
	iiwa_state_estimation_params_t* params = (iiwa_state_estimation_params_t *) activity->conf.params;
	iiwa_state_estimation_continuous_state_t *continuous_state = (iiwa_state_estimation_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_state_estimation_coordination_state_t *coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;

    // get the current timestamp and compute the current cycle time. 
	if (coord_state->first_run_compute_cycle){
		coord_state->first_run_compute_cycle = false;
		timespec_get(&continuous_state->current_timespec, TIME_UTC);
		continuous_state->cycle_time_us = 0;
		memcpy(&continuous_state->prev_timespec, &continuous_state->current_timespec,
		    sizeof(continuous_state->current_timespec));
	}else{
		timespec_get(&continuous_state->current_timespec, TIME_UTC);
		continuous_state->cycle_time_us = difftimespec_us_estimate(&continuous_state->current_timespec, 
		    &continuous_state->prev_timespec);
		memcpy(&continuous_state->prev_timespec, &continuous_state->current_timespec,
		    sizeof(continuous_state->current_timespec));
	}

	// compute the joint velocities
	for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++){
		continuous_state->meas_jnt_vel[i] = estimate_velocity(continuous_state->local_meas_jnt_pos[i], continuous_state->jnt_pos_prev[i], (double) continuous_state->cycle_time_us / 1000000.0);

		// write the joint positions and velocities to the JntArray
		continuous_state->local_qd.q(i) = continuous_state->local_meas_jnt_pos[i];
		continuous_state->local_qd.qdot(i) = continuous_state->meas_jnt_vel[i];
	}

    // Forward velocity kinematics
	velksolver_estimate.JntToCart(continuous_state->local_qd, continuous_state->local_cartvel);
	fksolver_estimate.JntToCart(continuous_state->local_q, continuous_state->local_cartpos);

	std::cout<< "Position in arm_base frame: " << continuous_state->local_cartpos.p <<std::endl;
	std::cout<< "Velocity in arm_base frame: " << continuous_state->local_cartvel.GetTwist() <<std::endl;
    //In which frame are they expressed ? Robot base frame?
    // Set the local_cartvel to a variable that can be used in other activities : Is the local_cartvel expressed in end_effector frame or base?
}

void iiwa_state_estimation_running(activity_t *activity){
	iiwa_state_estimation_running_communicate(activity);
	iiwa_state_estimation_running_coordinate(activity);
	iiwa_state_estimation_running_compute(activity);
	iiwa_state_estimation_running_configure(activity);
}

// SCHEDULER 
void iiwa_state_estimation_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t) iiwa_state_estimation_config, 
        activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");
    
    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t) iiwa_state_estimation_creation, 
        activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, "creation");

	schedule_t schedule_capability_config = {.number_of_functions = 0};
    register_function(&schedule_capability_config, (function_ptr_t) iiwa_state_estimation_capability_configuration, 
        activity, "capability_configuration");
    register_schedule(&activity->schedule_table, schedule_capability_config, "capability_configuration");
    
    schedule_t schedule_pausing = {.number_of_functions = 0};
    register_function(&schedule_pausing, (function_ptr_t) iiwa_state_estimation_pausing, 
        activity, "pausing");
    register_schedule(&activity->schedule_table, schedule_pausing, "pausing");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t) iiwa_state_estimation_running, 
        activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, "running");

	schedule_t schedule_cleaning = {.number_of_functions = 0};
    register_function(&schedule_cleaning, (function_ptr_t) iiwa_state_estimation_cleaning, 
        activity, "cleaning");
    register_schedule(&activity->schedule_table, schedule_cleaning, 
        "cleaning");
}

void iiwa_state_estimation_create_lcsm(activity_t* activity, const char* name_activity){
    activity->conf.params = malloc(sizeof(iiwa_state_estimation_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(iiwa_state_estimation_continuous_state_t));
    activity->state.computational_state.discrete = malloc(sizeof(iiwa_state_estimation_discrete_state_t));
    activity->state.coordination_state = malloc(sizeof(iiwa_state_estimation_coordination_state_t));
}

void iiwa_state_estimation_resource_configure_lcsm(activity_t *activity){
	iiwa_state_estimation_coordination_state_t *coord_state = (iiwa_state_estimation_coordination_state_t *) activity->state.coordination_state;

    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;
	coord_state->deinitialisation_request = false;
	coord_state->execution_request = false;

    // Schedule table (adding config() for the first eventloop iteration)
    iiwa_state_estimation_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void iiwa_state_estimation_destroy_lcsm(activity_t* activity){
    destroy_activity(activity);
}

const iiwa_state_estimation_t ec_iiwa_state_estimation ={
    .create_lcsm = iiwa_state_estimation_create_lcsm,
    .resource_configure_lcsm = iiwa_state_estimation_resource_configure_lcsm,
    .destroy_lcsm = iiwa_state_estimation_destroy_lcsm,
};

long difftimespec_us_estimate(struct timespec *current_timespec, struct timespec *prev_timespec){
    long difftimespec_us; // cycle time in microseconds
	double diff_s;
	long diff_ns;

	diff_s = difftime(current_timespec->tv_sec, prev_timespec->tv_sec);
	diff_ns = current_timespec->tv_nsec - prev_timespec->tv_nsec;

	difftimespec_us =  (long) diff_s * 1000000 + diff_ns / 1000;

	return difftimespec_us;
}

double estimate_velocity(double meas_jnt_pos, double prev_jnt_pos, double cycle_time){
	double vel;

	if (cycle_time == 0.0) vel = 0.0;
	else vel = (meas_jnt_pos - prev_jnt_pos) / cycle_time;

	return vel;
}

// // Function to get the cartesian position of the end_effector expressed in the inertial frame (located at the bottom of the bimanual support)
// void compute_inertial_pos(activity_t *activity){
// 	iiwa_state_estimation_continuous_state_t *continuous_state = (iiwa_state_estimation_continuous_state_t *) activity->state.computational_state.continuous;
// 	KDL::Frame arm_base_cartpos;
// 	fksolver_estimate.JntToCart(continuous_state->local_q, arm_base_cartpos);
// 	std::cout<< "Position in arm_base frame: " << arm_base_cartpos <<std::endl;
// 	//Need to map the coordinates from the arm_base frame to the inertial frame
// }