/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file iiwa_virtual.hpp
 * @date September 15, 2022
 **/

#ifndef IIWA_VIRTUAL_HPP
#define IIWA_VIRTUAL_HPP

#include <stdio.h>
#include <pthread.h>
#include "iiwa_interface.hpp"

// ACCAL
#include <five_c/activity/activity.h>

using namespace std;

typedef struct iiwa_virtual_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}iiwa_virtual_t; 

// Parameters
typedef struct iiwa_virtual_params_s{
    iiwa_params_t               iiwa_params;
    int                         thread_time;
}iiwa_virtual_params_t;

// Continuous state
typedef struct iiwa_virtual_continuous_state_s{
    iiwa_state_t              iiwa_state;
}iiwa_virtual_continuous_state_t;

//! Coordination state
typedef struct iiwa_virtual_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool deinitialisation_request;
    bool commanding_not_active;
    // Mutex
    pthread_mutex_t sensor_lock, actuation_lock;
} iiwa_virtual_coordination_state_t;

extern const iiwa_virtual_t ec_iiwa_virtual;
#endif //IIWA_VIRTUAL_HPP