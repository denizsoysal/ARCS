/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file data_logger.cpp
 * @date November 09, 2022
 **/

#include "data_logger.hpp"

void data_logger_config(activity_t* activity){
    // Remove config() from the eventloop schedule in the next iteration
    remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
    switch (activity->lcsm.state){
        case CREATION:
            add_schedule_to_eventloop(&activity->schedule_table, "creation");
            break;
        case RESOURCE_CONFIGURATION:
            add_schedule_to_eventloop(&activity->schedule_table, "resource_configuration");
            break;
        case RUNNING:
            add_schedule_to_eventloop(&activity->schedule_table, "running");
            break;
        case DONE:
			break;
    }
};

// Creation
void data_logger_creation_coordinate(activity_t *activity){
    // Coordinating own activity
    if (activity->state.lcsm_flags.creation_complete){
        activity->lcsm.state = RESOURCE_CONFIGURATION;
        update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    }
}

void data_logger_creation_configure(activity_t *activity){
    if (activity->lcsm.state != CREATION){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "creation");
    }
}

void data_logger_creation_compute(activity_t *activity){
    activity->state.lcsm_flags.creation_complete = true;
}

void data_logger_creation(activity_t *activity){
    data_logger_creation_compute(activity);
    data_logger_creation_coordinate(activity);
    data_logger_creation_configure(activity);
}

// Resource configuration
void data_logger_resource_configuration_coordinate(activity_t *activity){
    data_logger_coordination_state_t* coord_state = (data_logger_coordination_state_t*) activity->state.coordination_state;

    if (coord_state->execution_request)
        activity->state.lcsm_protocol = EXECUTION;

     // Internal coordination
    if (activity->state.lcsm_flags.resource_configuration_complete){
        switch (activity->state.lcsm_protocol){ 
            case INITIALISATION:
                break;
            case EXECUTION:
                activity->lcsm.state = RUNNING;
                break;
            case DEINITIALISATION:
                activity->lcsm.state = DONE;
                activity->state.lcsm_flags.deletion_complete = true;            
                break;
        }
        update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    }
}

void data_logger_resource_configuration_configure(activity_t *activity){
    if (activity->lcsm.state != RESOURCE_CONFIGURATION){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
        // Update flags for next visit to the resource configuration LCS 
        activity->state.lcsm_flags.resource_configuration_complete = false;
    }
}

void data_logger_resource_configuration_compute(activity_t *activity){
    data_logger_params_t *params = (data_logger_params_t *) activity->conf.params;
    data_logger_continuous_state_t *logger_state = (data_logger_continuous_state_t *) activity->state.computational_state.continuous;
    if (activity->state.lcsm_protocol == EXECUTION){
        params->fpt = fopen(params->fname.c_str(), "w");
        // make header
	    fprintf(params->fpt, "%s, %s, %s, %s, %s, %s\n", 
            "time_us",
            "abag_bias",
            "abag_gain", 
            "abag_control",
            "abag_ek_bar",
            "local_vz");
        fclose(params->fpt);

        timespec_get(&logger_state->initial_timespec, TIME_UTC);
    }
    activity->state.lcsm_flags.resource_configuration_complete  = true;
}

void data_logger_resource_configuration(activity_t *activity){
    data_logger_resource_configuration_coordinate(activity);
    data_logger_resource_configuration_compute(activity);
    data_logger_resource_configuration_configure(activity);
}

// Running
void data_logger_running_coordinate(activity_t *activity){
    if (activity->state.lcsm_flags.running_complete){
        activity->lcsm.state = RESOURCE_CONFIGURATION;
        activity->state.lcsm_protocol = DEINITIALISATION;
        update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    }
}

void data_logger_running_configure(activity_t *activity){
    if (activity->lcsm.state != RUNNING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "running");
        activity->state.lcsm_flags.running_complete = false;
    }
}

void data_logger_running_compute(activity_t *activity){
    data_logger_params_t *params = (data_logger_params_t *) activity->conf.params;
    data_logger_coordination_state_t *coord_state = (data_logger_coordination_state_t *) activity->state.coordination_state;
    data_logger_continuous_state_t *logger_state = (data_logger_continuous_state_t *) activity->state.computational_state.continuous;
    iiwa_controller_continuous_state_t *controller_state = (iiwa_controller_continuous_state_t *) logger_state->controller_continuous_state;
    iiwa_activity_continuous_state_t *iiwa_state = (iiwa_activity_continuous_state_t *) logger_state->iiwa_continuous_state;

    timespec_get(&logger_state->current_timespec, TIME_UTC);
    logger_state->time_us = difftimespec_us(&logger_state->current_timespec, 
        &logger_state->initial_timespec);

    pthread_mutex_lock(coord_state->actuation_lock);
    // iiwa
    // memcpy(logger_state->cmd_jnt_torque_iiwa, iiwa_state->iiwa_state.iiwa_actuation_input.cmd_torques,
    //     sizeof(iiwa_state->iiwa_state.iiwa_actuation_input.cmd_torques));

    // controller abag state
    memcpy(&logger_state->abag_state_controller, &controller_state->abag_state,
        sizeof(controller_state->abag_state));
    pthread_mutex_unlock(coord_state->actuation_lock);


	// pthread_mutex_lock(coord_state->sensor_lock);
    // memcpy(logger_state->meas_jnt_pos_iiwa, iiwa_state->iiwa_state.iiwa_sensors.meas_jnt_pos,
    //     sizeof(iiwa_state->iiwa_state.iiwa_sensors.meas_jnt_pos));
	// pthread_mutex_unlock(coord_state->sensor_lock);

    // // copy the estimated velocity without a mutex
    // memcpy(logger_state->meas_jnt_vel_controller, controller_state->meas_jnt_vel,
    //     sizeof(controller_state->meas_jnt_vel));

    // get the local cartesian velocity
    memcpy(&logger_state->cartvel_controller, &controller_state->local_cartvel,
        sizeof(controller_state->local_cartvel)); 
    
    params->fpt = fopen(params->fname.c_str(), "a");

	fprintf(params->fpt, "%lu, %f, %f, %f, %f, %f\n", 
        logger_state->time_us,
        logger_state->abag_state_controller.bias,
        logger_state->abag_state_controller.gain,
        logger_state->abag_state_controller.control,
        logger_state->abag_state_controller.ek_bar,
        -1*logger_state->cartvel_controller.GetTwist()(2));

	fclose(params->fpt);
}

void data_logger_running(activity_t *activity){
    data_logger_running_compute(activity);
    data_logger_running_coordinate(activity);
    data_logger_running_configure(activity);
}

// SCHEDULER 
void data_logger_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t) data_logger_config, 
        activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");
    
    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t) data_logger_creation, 
        activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, 
        "creation");

    schedule_t schedule_resource_configuration = {.number_of_functions = 0};
    register_function(&schedule_resource_configuration, (function_ptr_t) data_logger_resource_configuration, 
        activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_configuration, 
        "resource_configuration");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t) data_logger_running, 
        activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, "running");
}


void data_logger_create_lcsm(activity_t* activity, const char* name_activity){
    activity->conf.params = malloc(sizeof(data_logger_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(data_logger_continuous_state_t));
    activity->state.coordination_state = malloc(sizeof(data_logger_coordination_state_t));
}

void data_logger_resource_configure_lcsm(activity_t *activity){
    data_logger_coordination_state_t *coord_state = (data_logger_coordination_state_t *) activity->state.coordination_state;
    resource_configure_lcsm_activity(activity);
    activity->mid = 2;
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;
    coord_state->deinitialisation_request = false;
    coord_state->execution_request = true; 
    
    // Schedule table (adding config() for the first eventloop iteration)
    data_logger_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void data_logger_destroy_lcsm(activity_t* activity){
    destroy_activity(activity);
}

const data_logger_t ec_data_logger ={ 
    .create_lcsm = data_logger_create_lcsm,
    .resource_configure_lcsm = data_logger_resource_configure_lcsm,
    .destroy_lcsm = data_logger_destroy_lcsm,
};

