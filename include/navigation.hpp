/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Louis Hanut and Brendan Pousett
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file navigation.hpp
 * @date September 15, 2022
 **/

#ifndef iiwa_navigation_HPP
#define iiwa_navigation_HPP

#include <stdio.h>
#include <pthread.h>

// ACCAL
#include "five_c/activity/activity.h"

// KDL
#include <frames.hpp>

// SPDLOG
#include "spdlog/spdlog.h"

using namespace std;

typedef struct navigation_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}navigation_t; 

typedef struct navigation_params_s{
    // Parameters governing the motion spec
    double slowdown_threshold;
    double stop_threshold;
    double fast_vel;
    double slow_vel;

    std::shared_ptr<spdlog::logger> logger;
}navigation_params_t;

typedef struct navigation_continuous_state_s{
    // shared variables to be read from other activity
    KDL::Frame *end_effector_pos;
    // KDL::Vector *set_pos;

    KDL::Vector heading;
    double velocity_magnitude;
}navigation_continuous_state_t;

//! (computational) discrete state
typedef struct navigation_discrete_state_s{
}navigation_discrete_state_t;

//! Coordination state
typedef struct navigation_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool deinitialisation_request;

    // Mutex for navigation
    pthread_mutex_t navigation_lock, *perception_lock, *estimation_lock;

} navigation_coordination_state_t;

extern const navigation_t ec_navigation;

#endif //navigation_HPP