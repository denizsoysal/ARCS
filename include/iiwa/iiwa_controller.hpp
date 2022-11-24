/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Louis Hanut and Brendan Pousett
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file iiwa_controller.hpp
 * @date September 15, 2022
 **/

#ifndef iiwa_controller_HPP
#define iiwa_controller_HPP

#include <stdio.h>
#include <pthread.h>

// ACCAL
#include <five_c/activity/activity.h>

#include "iiwa_client.hpp"
#include "iiwa_interface.hpp"

using namespace std;

typedef struct iiwa_controller_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}iiwa_controller_t; 

// Parameters
typedef struct iiwa_controller_params_s{
    iiwa_state_t *iiwa_controller_params;
    double	goal_jnt_pos[LBRState::NUMBER_OF_JOINTS], local_goal_jnt_pos[LBRState::NUMBER_OF_JOINTS];

    double  max_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    double  approach_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    double  slow_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    double  jnt_accel[LBRState::NUMBER_OF_JOINTS];
    double  approach_buffer[LBRState::NUMBER_OF_JOINTS];
    double  slow_buffer[LBRState::NUMBER_OF_JOINTS];
    double  goal_buffer[LBRState::NUMBER_OF_JOINTS];

    struct iiwa_sensors_s{
		double 			meas_jnt_pos[LBRState::NUMBER_OF_JOINTS];
		double 			meas_torques[LBRState::NUMBER_OF_JOINTS];
		double 			meas_ext_torques[LBRState::NUMBER_OF_JOINTS];
	}local_sensors;

}iiwa_controller_params_t;

// Continuous state
typedef struct iiwa_controller_continuous_state_s{
    // TODO we should write our own data structures here because:
    //  - we make it more independent of iiwa_interface and general to different robots
    //  - we remove extra fiels from this struct that we don't use, like fri_port, etc...
    iiwa_params_t *iiwa_controller_state;

    double local_cmd_jnt_vel[LBRState::NUMBER_OF_JOINTS];

    struct timespec prev_timespec;
    struct timespec current_timespec;
}iiwa_controller_continuous_state_t;

//! (computational) discrete state
typedef struct iiwa_controller_discrete_state_s{
    bool first_run_compute_cycle = TRUE;
    // flags 
    bool in_wait = FALSE;
    bool in_approach = FALSE;
    bool in_blend = FALSE;
    bool in_slow = FALSE;
    bool in_contact = FALSE;
}iiwa_controller_discrete_state_t;

//! Coordination state
typedef struct iiwa_controller_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool deinitialisation_request;
    bool commanding_not_active;
    // Mutex
    pthread_mutex_t *sensor_lock, *actuation_lock, goal_lock; //pointers because they will point to the same as the iiwa_activity
} iiwa_controller_coordination_state_t;

extern const iiwa_controller_t ec_iiwa_controller;
#endif //iiwa_controller_HPP