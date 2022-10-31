/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file iiwa_controller_activity.hpp
 * @date September 15, 2022
 **/

#ifndef IIWA_CONTROLLER_ACTIVITY_HPP
#define IIWA_CONTROLLER_ACTIVITY_HPP

#include <stdio.h>
#include <pthread.h>

// ACCAL
#include <five_c/activity/activity.h>

#include "iiwa_interface.hpp"

using namespace std;

typedef struct iiwa_Controller_activity_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}iiwa_controller_activity_t; 

// Parameters
typedef struct iiwa_controller_activity_params_s{
    iiwa_state_t *iiwa_controller_params;
    double	goal_jnt_pos[LBRState::NUMBER_OF_JOINTS], local_goal_jnt_pos[LBRState::NUMBER_OF_JOINTS];
    struct iiwa_sensors_s local_sensors;
}iiwa_controller_activity_params_t;

// Continuous state
typedef struct iiwa_controller_activity_continuous_state_s{
    iiwa_params_t *iiwa_controller_state;
}iiwa_controller_activity_continuous_state_t;

//! (computational) discrete state
typedef struct iiwa_controller_activity_discrete_state_s{
}iiwa_controller_activity_discrete_state_t;

//! Coordination state
typedef struct iiwa_controller_activity_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool deinitialisation_request;
    bool commanding_not_active;
    // Mutex
    pthread_mutex_t *sensor_lock, *actuation_lock, goal_lock; //pointers because they will point to the same as the iiwa_activity
} iiwa_controller_activity_coordination_state_t;

extern const iiwa_controller_activity_t ec_iiwa_controller_activity;
#endif //IIWA_CONTROLLER_ACTIVITY_HPP