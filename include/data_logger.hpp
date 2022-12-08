/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file data_logger.hpp
 * @date November 09, 2022
 **/

#ifndef data_logger_HPP
#define data_logger_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <five_c/activity/activity.h>
#include "iiwa/iiwa_controller.hpp"
#include <iiwa_activity.hpp>

typedef struct data_logger_alg_s{
    void (*create_lcsm)(activity_t*, const char* name_algorithm);
    void (*resource_configure_lcsm)(activity_t*);
    void (*destroy_lcsm)(activity_t*);
}data_logger_t;

// Parameters
typedef struct data_logger_params_s{
    FILE *fpt;
    string fname;
}data_logger_params_t;

// Continuous state
typedef struct data_logger_continuous_state_s{
    iiwa_controller_continuous_state_t *controller_continuous_state;
    iiwa_activity_continuous_state_t *iiwa_continuous_state;

    struct timespec initial_timespec;
    struct timespec current_timespec;
    unsigned long time_us;
    
    // local variables for memcpy and fprint
    // state of abag
    abag_state_t abag_state_controller;
    // outputs of controller
    double cmd_jnt_vel_controller[LBRState::NUMBER_OF_JOINTS];
    double cmd_jnt_torque_controller[LBRState::NUMBER_OF_JOINTS];
    double cmd_wrench_controller[CART_VECTOR_DIM];
    // commands received by iiwa
    double cmd_jnt_vel_iiwa[LBRState::NUMBER_OF_JOINTS];
    double cmd_jnt_torque_iiwa[LBRState::NUMBER_OF_JOINTS];
    double cmd_wrench_iiwa[CART_VECTOR_DIM];
    // state of the iiwa
    double meas_jnt_pos_iiwa[LBRState::NUMBER_OF_JOINTS];
	double meas_torques_iiwa[LBRState::NUMBER_OF_JOINTS];
	double meas_ext_torques_iiwa[LBRState::NUMBER_OF_JOINTS];

}data_logger_continuous_state_t;

typedef struct data_logger_coordination_state_s {
    // Activity LCSM
    bool execution_request;
    bool deinitialisation_request;
    bool commanding_not_active;

    // Mutex
    pthread_mutex_t *sensor_lock, *actuation_lock, *goal_lock;
} data_logger_coordination_state_t;

extern const data_logger_t ec_data_logger;
#endif // data_logger_HPP
