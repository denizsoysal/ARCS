/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_controller_petrinet.hpp
 * @date November 24, 2022
 **/

#ifndef IIWA_CONTROLLER_PETRINET_HPP
#define IIWA_CONTROLLER_PETRINET_HPP

#include <coordination_libraries/petrinet/petrinet.h> 
#include <five_c/scheduler/eventloop_composition_and_execution/petrinet_scheduler.h>  

// Tracking sources
#define NUMBER_OF_TRACKING_SOURCES 4
#define INITIATE_MOTION 0
#define START_VEL_TRANSITION 1
#define END_VEL_TRANSITION 2
#define CONTACT_DETECTED 3

// Tracking sinks
#define NUMBER_OF_TRACKING_SINKS 4
#define START_APPROACH 0
#define ENTER_BLEND_MODEL 1
#define ENTER_SLOW_MOTION 2
#define TERMINATE_NET 3

//const char
petrinet_t iiwa_controller_create_petrinet(char *name); 
void iiwa_controller_reset_petrinet(petrinet_t *p);

extern flag_token_conversion_map_t iiwa_controller_petrinet_flag_map;

#endif // IIWA_CONTROLLER_PETRINET_HPP
