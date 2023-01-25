/* ----------------------------------------------------------------------------
 * IIWASE (ARCS),
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Louis Hanut and Brendan Pousett
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file navigation.cpp
 * @date January 16, 2023
 **/

#include "navigation.hpp"

//CONFIG
void navigation_config(activity_t *activity){
    remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
    
    switch(activity->lcsm.state){
        case CREATION:
            add_schedule_to_eventloop(&activity->schedule_table, "creation");
            spdlog::debug("In creation state navigation");
            break;
        case RESOURCE_CONFIGURATION:
            add_schedule_to_eventloop(&activity->schedule_table, "resource_configuration");
            spdlog::debug("In resource configuration state navigation");
            break;
        case RUNNING:
            add_schedule_to_eventloop(&activity->schedule_table, "running");
            spdlog::debug("In running state navigation");
            break;
        case CLEANING:
            add_schedule_to_eventloop(&activity->schedule_table, "cleaning");
            spdlog::debug("In cleaning state navigation");
            break;
        case DONE:
            break;
    }
}

void navigation_creation(activity_t *activity){
    // coordinate lcsm state with external activities
    activity->state.lcsm_flags.creation_complete = true;
    activity->lcsm.state = RESOURCE_CONFIGURATION;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    // configure activity schedule
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
    remove_schedule_from_eventloop(&activity->schedule_table, "creation");
}

void navigation_resource_configuration_compute(activity_t *activity){
    navigation_params_t *params = (navigation_params_t *) activity->conf.params;

    params->slow_vel = 0.02;
    params->fast_vel = 0.05;
    params->slowdown_threshold = 0.1;
    params->stop_threshold = 0.02;
}

void navigation_resource_configuration_coordinate(activity_t *activity){
    navigation_coordination_state_t *coord_state = 
        (navigation_coordination_state_t *) activity->state.coordination_state;
 
    if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;
    if (coord_state->execution_request)
        activity->state.lcsm_protocol = EXECUTION;

	if (activity->state.lcsm_flags.resource_configuration_complete){
		switch (activity->state.lcsm_protocol){ 
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

void navigation_resource_configuration_configure(activity_t *activity){
    navigation_coordination_state_t *coord_state = 
        (navigation_coordination_state_t *) activity->state.coordination_state;
 
	if (activity->lcsm.state != RESOURCE_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
	}
}

void navigation_resource_configuration(activity_t *activity){
    navigation_params_t *params = (navigation_params_t *) activity->conf.params;

    navigation_resource_configuration_compute(activity);
    activity->state.lcsm_flags.resource_configuration_complete = true;
    navigation_resource_configuration_coordinate(activity);
    navigation_resource_configuration_configure(activity);
    params->logger->debug("Navigation resource configuration");
}

void navigation_running_communicate(activity_t *activity){
    // communicate and compute
    navigation_continuous_state_t *cts_state = 
        (navigation_continuous_state_t *) activity->state.computational_state.continuous;
    navigation_coordination_state_t *coord_state = 
        (navigation_coordination_state_t *) activity->state.coordination_state;
    navigation_params_t *params = (navigation_params_t *) activity->conf.params;
    double heading_norm;
    KDL::Vector hardcode_pos(0, 0, 0);
    
    pthread_mutex_lock(coord_state->estimation_lock);
    // pthread_mutex_lock(coord_state->setpoint_lock);
    pthread_mutex_lock(&coord_state->navigation_lock);
    // cts_state->heading = *cts_state->set_pos - (*cts_state->end_effector_pos).p;

    // TODO Manually set the heading
    // cts_state->heading[0]=0; cts_state->heading[1]=0; cts_state->heading[2]=-1;

    // TODO Manually set the pos
    cts_state->heading = hardcode_pos - (*cts_state->end_effector_pos).p;
    pthread_mutex_unlock(coord_state->estimation_lock);
    // pthread_mutex_unlock(coord_state->setpoint_lock);

    heading_norm = cts_state->heading.Normalize();

    if(heading_norm > params->slowdown_threshold){
        cts_state->velocity_magnitude = params->fast_vel;
    }else if(heading_norm > params->stop_threshold){
        cts_state->velocity_magnitude = params->slow_vel;
    }else{
        cts_state->velocity_magnitude = 0;
    }
    
    // TODO Manually set the velocity magnitude 
    cts_state->velocity_magnitude = 0.05;

    pthread_mutex_unlock(&coord_state->navigation_lock);

    params->logger->info("headingx,{}", cts_state->heading[0]);
    params->logger->info("headingy,{}", cts_state->heading[1]);
    params->logger->info("headingz,{}", cts_state->heading[2]);
}

void navigation_running_coordinate(activity_t *activity){
    navigation_coordination_state_t *coord_state = 
        (navigation_coordination_state_t *) activity->state.coordination_state;
    // Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;
	
	switch (activity->state.lcsm_protocol){ 
		case DEINITIALISATION:
			activity->lcsm.state = CLEANING; 
			break;
	}
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void navigation_running_configure(activity_t *activity){
    navigation_coordination_state_t *coord_state = 
        (navigation_coordination_state_t *) activity->state.coordination_state;
 
	if (activity->lcsm.state != RUNNING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "running");
	}
}

void navigation_running(activity_t *activity){
    navigation_running_communicate(activity);
    navigation_running_coordinate(activity);
    navigation_running_configure(activity);
}

void navigation_cleaning_compute(activity_t *activity){
    navigation_params_t *params = (navigation_params_t *) activity->conf.params;
    // flush the logger to ensure all data is written
    if(params->logger){
        params->logger->flush();
    }
}

void navigation_cleaning(activity_t *activity){
    navigation_cleaning_compute(activity);
    // coordinate lcsm state
    activity->state.lcsm_flags.deletion_complete = true;
    activity->lcsm.state = DONE;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);

    // configure activity schedule
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
    remove_schedule_from_eventloop(&activity->schedule_table, "cleaning");
}

//SCHEDULER
void navigation_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, 
        (function_ptr_t) navigation_config, activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");

    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, 
        (function_ptr_t) navigation_creation, activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, "creation");

    schedule_t schedule_resource_configuration = {.number_of_functions = 0};
    register_function(&schedule_resource_configuration, 
        (function_ptr_t) navigation_resource_configuration, activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_configuration, "resource_configuration");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, 
        (function_ptr_t) navigation_running, activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, "running");

    schedule_t schedule_cleaning = {.number_of_functions = 0};
    register_function(&schedule_cleaning, 
        (function_ptr_t) navigation_cleaning, activity, "cleaning");
    register_schedule(&activity->schedule_table, schedule_cleaning, "cleaning");
}

// LCSM FUNCTIONS
void navigation_create_lcsm(activity_t* activity, const char* name_activity){
    activity->conf.params = malloc(sizeof(navigation_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(navigation_continuous_state_t));
    activity->state.coordination_state = malloc(sizeof(navigation_coordination_state_t));
}

void navigation_resource_configure_lcsm(activity_t *activity){
	navigation_coordination_state_t *coord_state = (navigation_coordination_state_t *) activity->state.coordination_state;
    navigation_params_t *params = (navigation_params_t *) activity->conf.params;

    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;
	coord_state->deinitialisation_request = false;
	coord_state->execution_request = false;

    // initialize the logger to the null pointer
    params->logger = NULL;

    // Schedule table (adding config() for the first eventloop iteration)
    navigation_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void navigation_destroy_lcsm(activity_t* activity){
    destroy_activity(activity);
}

const navigation_t ec_navigation ={
    .create_lcsm = navigation_create_lcsm,
    .resource_configure_lcsm = navigation_resource_configure_lcsm,
    .destroy_lcsm = navigation_destroy_lcsm,
};