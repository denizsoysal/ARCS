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
    // Note bias and gain are both params and states
    double bias;
    double gain;
    
    double control;
}abag_state_t;

// Parameters
typedef struct iiwa_controller_params_s{
    iiwa_state_t *iiwa_controller_params;
    double	goal_jnt_pos[LBRState::NUMBER_OF_JOINTS];
    // TODO where do you use global vs local?
    double  local_goal_jnt_pos[LBRState::NUMBER_OF_JOINTS];

    double	goal_wrench[CART_VECTOR_DIM];
    // TODO where do you use global vs local?
    double  local_goal_wrench[CART_VECTOR_DIM];

    double  max_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    double  slow_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    double  jnt_accel[LBRState::NUMBER_OF_JOINTS];
    double  jnt_jerk[LBRState::NUMBER_OF_JOINTS];
    double  max_jnt_accel[LBRState::NUMBER_OF_JOINTS];
    double  approach_buffer[LBRState::NUMBER_OF_JOINTS];
    double  slow_buffer[LBRState::NUMBER_OF_JOINTS];
    double  goal_buffer[LBRState::NUMBER_OF_JOINTS];

    double max_wrench_step;
    
    double max_torque;

    double torque_gain;

    struct iiwa_sensors_s{
		double 			meas_jnt_pos[LBRState::NUMBER_OF_JOINTS];
		double 			meas_torques[LBRState::NUMBER_OF_JOINTS];
		double 			meas_ext_torques[LBRState::NUMBER_OF_JOINTS];
	}local_sensors;

    abag_params_t abag_params;
}iiwa_controller_params_t;

// Continuous state
typedef struct iiwa_controller_continuous_state_s{
    // TODO we should write our own data structures here because:
    //  - we make it more independent of iiwa_interface and general to different robots
    //  - we remove extra fiels from this struct that we don't use, like fri_port, etc...
    iiwa_params_t *iiwa_controller_state;

    double local_cmd_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    double jnt_pos_error[LBRState::NUMBER_OF_JOINTS];

    double jnt_pos_prev[LBRState::NUMBER_OF_JOINTS];

    double approach_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    double approach_jnt_acc[LBRState::NUMBER_OF_JOINTS];
    double approach_coeffs[4]; // [a0, a1, a2, a3]; a0 + a1*s + a2*s^2 + a3*s^3;
    double local_cmd_wrench[CART_VECTOR_DIM];

    double local_cmd_torques[LBRState::NUMBER_OF_JOINTS];
    struct timespec prev_timespec;
    struct timespec current_timespec;

    abag_state_t abag_state;
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
    pthread_mutex_t *sensor_lock, *actuation_lock, goal_lock; //pointers because they will point to the same as the iiwa_activity

    // First run compute cycle
    bool first_run_compute_cycle;
} iiwa_controller_coordination_state_t;

extern const iiwa_controller_t ec_iiwa_controller;

// Useful Functions
template <typename T> int sgn(T val);
template <typename T> int hside(T val);
template <typename T> T saturate(T val, T sat_low, T sat_high);
    
/**
 * compute the current timespec, and cycle time. copy current timespec to previous timespec.
 * todo move this time tracking into activity.h
*/   
double get_cycle_time(struct timespec *prev_timespec, struct timespec *current_timespec);

double *cubic_vel_traj_1d(double vel1, double acc1, double vel2, double acc2, double duration);

/**
 * Compute the acceleration setpoint to converge smoothly to desired velocity in a specified distance,
 * given the current velocity and acceleration. 
 * 
 * @param veld desired velocity
 * @param converge_distance "radius" from current position travelled before velk=veld
 * @param cycle_time of the controller, s
 * @return accd desired acceleration setpoint
*/
double acc_setpoint(double velk, double acck, double veld, double converge_distance, double cycle_time);

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