/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Louis Hanut and Brendan Pousett
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file iiwa_state_estimation.hpp
 * @date September 15, 2022
 **/

#ifndef iiwa_state_estimation_HPP
#define iiwa_state_estimation_HPP

#include <stdio.h>
#include <pthread.h>

// ACCAL
#include <five_c/activity/activity.h>

#include "iiwa_client.hpp"
#include "iiwa_interface.hpp"

// KDL
#include <chain.hpp>
#include <frames_io.hpp>
#include <jntarrayvel.hpp>

// SPDLOG
#include "spdlog/spdlog.h"

using namespace std;

typedef struct iiwa_state_estimation_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}iiwa_state_estimation_t; 

typedef struct iiwa_state_estimation_params_s{
    std::shared_ptr<spdlog::logger> logger;
}iiwa_state_estimation_params_t;

// Continuous state which is the state of the state_estimation system, including input and output signals
typedef struct iiwa_state_estimation_continuous_state_s{
    // Input signals from arm sensors (pointers to first elements in array)
    // These could probably be moved in the params data struct.
    double 	*meas_jnt_pos;
    double 	*meas_torques;
	double 	*meas_ext_torques;

    // Local copies of inputs and outputs will be deep copied with memcpy()
    double  local_meas_jnt_pos[LBRState::NUMBER_OF_JOINTS];
    double  local_meas_torques[LBRState::NUMBER_OF_JOINTS];
	double  local_meas_ext_torques[LBRState::NUMBER_OF_JOINTS];

    KDL::JntArray local_q;
    KDL::JntArrayVel local_qd;
    KDL::FrameVel local_cart_vel;
    KDL::Frame local_cart_pos;

    // "State" Parameters which are computed in the activity
    double jnt_pos_prev[LBRState::NUMBER_OF_JOINTS];
    double meas_jnt_vel[LBRState::NUMBER_OF_JOINTS];

    // Data structures for time
    struct timespec prev_timespec;
    struct timespec current_timespec;
    long cycle_time_us; //cycle time in microseconds

    //Data structures communicated to other activities
    KDL::FrameVel cart_vel;
    KDL::Frame cart_pos;
    double estimated_jnt_vel[LBRState::NUMBER_OF_JOINTS];

    // Variables for the averaging of the position measurements
    double jnt_vel_buffer[5][LBRState::NUMBER_OF_JOINTS]; //5 is the size of the averaging window, it might be modified
    int avg_buffer_ind;
    double jnt_vel_avg[LBRState::NUMBER_OF_JOINTS];
    double prev_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    double low_pass_a;
}iiwa_state_estimation_continuous_state_t;

//! (computational) discrete state
typedef struct iiwa_state_estimation_discrete_state_s{
}iiwa_state_estimation_discrete_state_t;

//! Coordination state
typedef struct iiwa_state_estimation_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool deinitialisation_request;

    // Mutex
    pthread_mutex_t *sensor_lock, estimate_lock;

    // First run compute cycle
    bool first_run_compute_cycle;
} iiwa_state_estimation_coordination_state_t;

extern const iiwa_state_estimation_t ec_iiwa_state_estimation;
    
/**
 * Compute the difference in microseconds between two timespecs.
 * 
 * @param *current_timespec from timespec_get()
 * @param *previous_timespec from timespec_get()
 * @return the time difference in microseconds from current-previous. 
*/   
long difftimespec_us_estimate(struct timespec *current_timespec, struct timespec *prev_timespec);

double estimate_velocity(double meas_jnt_pos, double prev_jnt_pos, double cycle_time);

#endif //iiwa_state_estimation_HPP