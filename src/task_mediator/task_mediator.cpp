/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file task_mediator.cpp
 * @date November 09, 2022
 **/

#include "task_mediator/task_mediator.hpp"
#include "task_mediator/task_mediator_petrinet.hpp"

#define FSM_STATE_WAIT 0
#define FSM_STATE_APPROACH 1
#define FSM_STATE_VEL_TRANSITION 2
#define FSM_STATE_CONTACT 3
#define FSM_STATE_FINAL 4

void task_mediator_config(activity_t* activity){
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
void task_mediator_creation_coordinate(activity_t *activity){
    // Coordinating own activity
    if (activity->state.lcsm_flags.creation_complete){
        activity->lcsm.state = RESOURCE_CONFIGURATION;
        update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    }
}

void task_mediator_creation_configure(activity_t *activity){
    if (activity->lcsm.state != CREATION){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "creation");
    }
}

void task_mediator_creation_compute(activity_t *activity){
    activity->petrinet = (petrinet_t *) malloc(sizeof(petrinet_t));
    activity->state.petrinet_flag_map = (flag_token_conversion_map_t *) malloc(sizeof(flag_token_conversion_map_t));
    activity->fsm = (FSM_t *) malloc(sizeof(FSM_t));
    activity->state.lcsm_flags.creation_complete = true;
}

void task_mediator_creation(activity_t *activity){
    task_mediator_creation_compute(activity);
    task_mediator_creation_coordinate(activity);
    task_mediator_creation_configure(activity);
}

// Resource configuration
void task_mediator_resource_configuration_coordinate(activity_t *activity){
     // Internal coordination
    if (activity->state.lcsm_flags.resource_configuration_complete){
        switch (activity->state.lcsm_protocol){ 
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

void task_mediator_resource_configuration_configure(activity_t *activity){
    if (activity->lcsm.state != RESOURCE_CONFIGURATION){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
        // Update flags for next visit to the resource configuration LCS 
        activity->state.lcsm_flags.resource_configuration_complete = false;
    }
}

void task_mediator_resource_configuration_compute(activity_t *activity){
    task_mediator_coordination_state_t * coord_state = (task_mediator_coordination_state_t *) activity->state.coordination_state;

    // Petrinet for coodinatiing the "bringup" of sensors and actuactors in the task
    activity->petrinet[0] = task_mediator_create_bringup_petrinet("bringup_petrinet");
    activity->state.petrinet_flag_map[0] = task_mediator_bringup_petrinet_flag_map;
    // Linking the flag array of the petrinet to coordination flags written by coordinated activities
    activity->state.petrinet_flag_map[0].tracking_sources.flags[BOARD_IN_RANGE] = coord_state->board_in_range;
    activity->state.petrinet_flag_map[0].tracking_sources.flags[BOARD_DIRTY] = coord_state->board_dirty;
    activity->state.petrinet_flag_map[0].tracking_sources.flags[START_VEL_TRANSITION] = coord_state->start_vel_transition;
    activity->state.petrinet_flag_map[0].tracking_sources.flags[END_VEL_TRANSITION] = coord_state->end_vel_transition;
    activity->state.petrinet_flag_map[0].tracking_sources.flags[CONTACT_DETECTED] = coord_state->contact_detected;

    activity->state.petrinet_flag_map[0].tracking_sinks.flags[IDENTIFY_DIRTY_PATCH_READY] = &coord_state->identify_dirty_patch_ready;
    activity->state.petrinet_flag_map[0].tracking_sinks.flags[START_APPROACH] = &coord_state->start_approach;
    activity->state.petrinet_flag_map[0].tracking_sinks.flags[ENTER_BLEND_MODEL] = &coord_state->enter_blend_model;
    activity->state.petrinet_flag_map[0].tracking_sinks.flags[ENTER_SLOW_MOTION] = &coord_state->enter_slow_motion;
    activity->state.petrinet_flag_map[0].tracking_sinks.flags[TERMINATE_NET] = &coord_state->terminate_net;

    activity->state.lcsm_flags.resource_configuration_complete  = true;
}

void task_mediator_resource_configuration(activity_t *activity){
    task_mediator_resource_configuration_compute(activity);
    task_mediator_resource_configuration_coordinate(activity);
    task_mediator_resource_configuration_configure(activity);
}

// Running
void task_mediator_running_coordinate(activity_t *activity){
    task_mediator_coordination_state_t *coord_state = (task_mediator_coordination_state_t *) activity->state.coordination_state;
    // Internal coordination
    // Based on the FSM
    switch (activity->fsm[0].state){
        case (FSM_STATE_WAIT):
            printf("Value of board in range : %d \n", *coord_state->board_in_range);
            printf("Value of board dirty : %d \n\n", *coord_state->board_dirty);
            // Evaluating a flag associated to a token of a petrinet
            communicate_token_flags_flag_map(&activity->petrinet[0], 
                &activity->state.petrinet_flag_map[0]);
            if (coord_state->start_approach)
                activity->fsm[0].state = FSM_STATE_APPROACH;
            break;
        case (FSM_STATE_APPROACH):
            printf("TASK FSM APPROACH \n");
            printf("Value of start vel trans : %d \n", *coord_state->start_vel_transition);
            communicate_token_flags_flag_map(&activity->petrinet[0], 
                &activity->state.petrinet_flag_map[0]);
            if (coord_state->enter_blend_model)
                activity->fsm[0].state = FSM_STATE_VEL_TRANSITION;
            break;
        case (FSM_STATE_VEL_TRANSITION):
            printf("TASK FSM VEL TRANSITION \n");
            communicate_token_flags_flag_map(&activity->petrinet[0], 
                &activity->state.petrinet_flag_map[0]);
            if (coord_state->enter_slow_motion)
                activity->fsm[0].state = FSM_STATE_CONTACT;
            break;
        case (FSM_STATE_CONTACT):
            printf("TASK FSM CONTACT \n");
            communicate_token_flags_flag_map(&activity->petrinet[0], 
                &activity->state.petrinet_flag_map[0]);
            if (coord_state->terminate_net)
                activity->fsm[0].state = FSM_STATE_FINAL;
            break;
        case (FSM_STATE_FINAL):
            printf("TASK FSM FINAL \n");
            activity->state.lcsm_flags.running_complete = true;
            break;
    }

    if (activity->state.lcsm_flags.running_complete){
        activity->lcsm.state = RESOURCE_CONFIGURATION;
        activity->state.lcsm_protocol = DEINITIALISATION;
        update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    }
}

void task_mediator_running_configure(activity_t *activity){
    if (activity->lcsm.state != RUNNING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "running");
        activity->state.lcsm_flags.running_complete = false;
    }
}

void task_mediator_running(activity_t *activity){
    task_mediator_running_coordinate(activity);
    task_mediator_running_configure(activity);
}

// SCHEDULER 
void task_mediator_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t) task_mediator_config, 
        activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");
    
    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t) task_mediator_creation, 
        activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, 
        "creation");

    schedule_t schedule_resource_configuration = {.number_of_functions = 0};
    register_function(&schedule_resource_configuration, (function_ptr_t) task_mediator_resource_configuration, 
        activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_configuration, 
        "resource_configuration");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t) task_mediator_running, 
        activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, "running");
}


void task_mediator_create_lcsm(activity_t* activity, const char* name_activity){
    activity->conf.params = malloc(sizeof(task_mediator_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(task_mediator_continuous_state_t));
    activity->state.computational_state.discrete = malloc(sizeof(task_mediator_discrete_state_t));
    activity->state.coordination_state = malloc(sizeof(task_mediator_coordination_state_t));
}

void task_mediator_resource_configure_lcsm(activity_t *activity){
    resource_configure_lcsm_activity(activity);
    activity->mid = 2;
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = EXECUTION;

    // Schedule table (adding config() for the first eventloop iteration)
    task_mediator_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void task_mediator_destroy_lcsm(activity_t* activity){
    destroy_activity(activity);
}

const task_mediator_t ec_task_mediator ={ 
    .create_lcsm = task_mediator_create_lcsm,
    .resource_configure_lcsm = task_mediator_resource_configure_lcsm,
    .destroy_lcsm = task_mediator_destroy_lcsm,
};
