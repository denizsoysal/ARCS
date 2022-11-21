/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file task_mediator_petrinet.hpp
 * @date November 09, 2022
 **/

#ifndef TASK_MEDIATOR_PETRINET_HPP
#define TASK_MEDIATOR_PETRINET_HPP

#include <coordination_libraries/petrinet/petrinet.h> 
#include <five_c/scheduler/eventloop_composition_and_execution/petrinet_scheduler.h>  

// Tracking sources
#define NUMBER_OF_TRACKING_SOURCES 5
#define BOARD_IN_RANGE 0
#define BOARD_DIRTY 0
#define START_VEL_TRANSITION 0
#define END_VEL_TRANSITION 0
#define CONTACT_DETECTED 0

// Tracking sinks
#define NUMBER_OF_TRACKING_SINKS 5
#define IDENTIFY_DIRTY_PATCH_READY 0
#define START_APPROACH 0
#define ENTER_BLEND_MODEL 0
#define ENTER_SLOW_MOTION 0
#define TERMINATE_NET 0

//const char
petrinet_t task_mediator_create_bringup_petrinet(char *name); //the name bringup can be changed as it was created for the mobile platform specifically

void task_mediator_reset_bringup_petrinet(petrinet_t *p);

extern flag_token_conversion_map_t task_mediator_bringup_petrinet_flag_map;

#endif // TASK_MEDIATOR_PETRINET_HPP
