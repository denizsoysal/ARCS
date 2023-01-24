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

// KDL
#include <chain.hpp>
#include <frames_io.hpp>
#include <jntarrayvel.hpp>

// SPDLOG
#include "spdlog/spdlog.h"

using namespace std;

typedef struct iiwa_controller_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}iiwa_controller_t; 

// Implement ABAG controller
typedef struct abag_params_s{
    double alpha; // "memory" parameter [0,1], the larger, the slower the exp. avg. responds.

    // bias params
    double delta_bias;
    double bias_thresh;

    // gain params
    double delta_gain;
    double gain_thresh;

    double sat_low;
    double sat_high;
}abag_params_t;

typedef struct abag_state_s{
    double ek_bar;
    // Note bias and gain are states which can be initialized as params to some value
    double bias; //bk
    double gain; //gk
    double control; //uk
}abag_state_t;

// Parameters that configure the behaviour of the controller, like gains, motion spec, etc
typedef struct iiwa_controller_params_s{
    EClientCommandMode cmd_mode;
    // parameter for bounding the capability of the controller
    double max_torque;
    double max_force;

    abag_params_t abag_params_cartesian;

    // logger implemented by spdlog
    std::shared_ptr<spdlog::logger> logger;
}iiwa_controller_params_t;

// Continuous state which is the state of the controller system, including input and output signals
typedef struct iiwa_controller_continuous_state_s{
    // Input signals from the estimation activity
    KDL::FrameVel *cart_vel;
    KDL::Frame *cart_pos;
    double (*jnt_vel)[LBRState::NUMBER_OF_JOINTS]; //this syntax is to create a pointer that points not only to the 0th element of an array but to the complete array

    // Input from the navigation activity
    KDL::Vector *heading;
    double *velocity_magnitude;

    // Output signals to iiwa 
	double	*cmd_jnt_vel;
	double	*cmd_torques;
	double	*cmd_wrench;

    // Local copies of inputs and outputs will be deep copied with memcpy()
    KDL::FrameVel local_cart_vel;
    KDL::Frame local_cart_pos;
    double  local_jnt_vel[LBRState::NUMBER_OF_JOINTS];
	double	local_cmd_jnt_vel[LBRState::NUMBER_OF_JOINTS];
	double	local_cmd_torques[LBRState::NUMBER_OF_JOINTS];
	double	local_cmd_wrench[CART_VECTOR_DIM];
    KDL::Vector local_heading;
    double local_velocity_magnitude;

    // Data structures for control algorithms
    abag_state_t abag_state_x;
    abag_state_t abag_state_y;
    abag_state_t abag_state_z;
}iiwa_controller_continuous_state_t;

//! (computational) discrete state
typedef struct iiwa_controller_discrete_state_s{
    // flags
    bool in_contact;
}iiwa_controller_discrete_state_t;

//! Coordination state
typedef struct iiwa_controller_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool deinitialisation_request;
    bool commanding_not_active;

    // Mutex
    pthread_mutex_t *estimate_lock, *actuation_lock, *navigation_lock;

    // First run compute cycle
    bool first_run_compute_cycle; //still need to check whether it is required
} iiwa_controller_coordination_state_t;

extern const iiwa_controller_t ec_iiwa_controller;

// sgn function {+1, -1}
template <typename T> int sgn(T val);

// heaviside function
template <typename T> int hside(T val);

// saturation function [sat_low, sat+high]
template <typename T> T saturate(T val, T sat_low, T sat_high);
    
/**
 * ABAG Controller, implementation similar to:
 * 
 * "Adaptive Closed-loop Speed Control of BLDC Motors with Applications to Multi-rotor Aerial Vehicles"
 * by Franchi and Mallet. 
 * 
 * Note: the sign of the error has been reversed. Instead of val-setpoint, I use setpoint-val. 
 * This gives a positive control signal uk when the value is less than the setpoint.
*/
void abag(abag_params_t *params, abag_state_t *state, double val, double setpoint);

#endif //iiwa_controller_HPP