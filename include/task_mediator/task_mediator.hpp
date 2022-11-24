/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file task_mediator.hpp
 * @date November 09, 2022
 **/

#ifndef TASK_MEDIATOR_HPP
#define TASK_MEDIATOR_HPP

#include <stdio.h>
#include <stdlib.h>

#include <five_c/activity/activity.h>
#include <coordination_libraries/fsm/FSM.h> //if we use this, we need to add it in the CMake
#include <five_c/scheduler/eventloop_composition_and_execution/petrinet_scheduler.h>


typedef struct task_mediator_alg_s{
    void (*create_lcsm)(activity_t*, const char* name_algorithm);
    void (*resource_configure_lcsm)(activity_t*);
    void (*destroy_lcsm)(activity_t*);
}task_mediator_t;

// Parameters
typedef struct task_mediator_params_s{
}task_mediator_params_t;

// Continuous state
typedef struct task_mediator_continuous_state_s{
}task_mediator_continuous_state_t;

// Discrete state
typedef struct task_mediator_discrete_state_s{
}task_mediator_discrete_state_t;

typedef struct task_mediator_coordination_state_s {
    // From external activities   
    bool *board_in_range;
    bool *board_dirty;

    bool *start_vel_transition;
    bool *end_vel_transition;
    bool *contact_detected;

    // To external activities
    bool *initiate_motion;
    bool enter_blend_model;
    bool enter_slow_motion;

    // Internal
    bool identify_dirty_patch_ready;
    bool terminate_net;

} task_mediator_coordination_state_t;

extern const task_mediator_t ec_task_mediator;
#endif // TASK_MEDIATOR_HPP
