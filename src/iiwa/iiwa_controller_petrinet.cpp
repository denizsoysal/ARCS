/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_controller_petrinet.cpp
 * @date November 24, 2022
 **/

#include "iiwa/iiwa_controller_petrinet.hpp"

// Tracking sources
char* controller_tracking_source_names[] = {//const
    "initiate_motion",
    "start_vel_transition",
    "end_vel_transition",
    "contact_detected"
};
bool* cotroller_tracking_source_flags[NUMBER_OF_TRACKING_SOURCES];

// Tracking sinks
char* controller_tracking_sink_names[] = {//const
    "start_approach",
    "enter_blend_model",
    "enter_slow_motion",
    "terminate_net"
};
bool* controller_tracking_sink_flags[NUMBER_OF_TRACKING_SINKS];

flag_token_conversion_map_t iiwa_controller_petrinet_flag_map = {
        .converting_sources = {
                .names = NULL,
                .flags = NULL,
                .number_of_flags = 0
        },
        .tracking_sources = {
                .names = controller_tracking_source_names,
                .flags = controller_tracking_source_flags,
                .number_of_flags = NUMBER_OF_TRACKING_SOURCES
        },
        .converting_sinks = {
                .names = NULL,
                .flags = NULL,
                .number_of_flags = 0
        },
        .tracking_sinks = {
                .names = controller_tracking_sink_names,
                .flags = controller_tracking_sink_flags,
                .number_of_flags = NUMBER_OF_TRACKING_SINKS
        }
};

petrinet_t iiwa_controller_create_petrinet(char *name) {
    petrinet_t *p = init_petrinet(name);

    place_t *p_initiate_motion = create_place(p, controller_tracking_source_names[INITIATE_MOTION]);
    place_t *p_start_vel_transition = create_place(p, controller_tracking_source_names[START_VEL_TRANSITION]);
    place_t *p_end_vel_transition = create_place(p, controller_tracking_source_names[END_VEL_TRANSITION]);
    place_t *p_contact_detected = create_place(p, controller_tracking_source_names[CONTACT_DETECTED]);

    place_t *p_start_approach = create_place(p, controller_tracking_sink_names[START_APPROACH]);
    place_t *p_enter_blend_model = create_place(p, controller_tracking_sink_names[ENTER_BLEND_MODEL]);
    place_t *p_enter_slow_motion = create_place(p, controller_tracking_sink_names[ENTER_SLOW_MOTION]);
    place_t *p_terminate_net = create_place(p, controller_tracking_sink_names[TERMINATE_NET]);

    transition_behaviour_t b1 = {
            .condition = cond_Black1,
            .consumption_behaviour=consume_Black1,
            .production_behaviour=produce_Black1
    }; 

    // TRANSITIONS
    transition_t *t1 = create_transition(p, "t1");
    add_behaviour(t1, &b1);

    agedge(p, p_initiate_motion, t1, "1", TRUE);
    agedge(p, t1, p_start_approach, "1", TRUE);

    transition_t *t2 = create_transition(p, "t2");
    add_behaviour(t2, &b1);

    agedge(p, p_start_approach, t2, "1", TRUE);
    agedge(p, p_start_vel_transition, t2, "1", TRUE);
    agedge(p, t2, p_enter_blend_model, "1", TRUE);

    transition_t *t3 = create_transition(p, "t3");
    add_behaviour(t3, &b1);
    
    agedge(p, p_enter_blend_model, t3, "1", TRUE);
    agedge(p, p_end_vel_transition, t3, "1", TRUE);
    agedge(p, t3, p_enter_slow_motion, "1", TRUE);

    transition_t *t4 = create_transition(p, "t4");
    add_behaviour(t4, &b1);
    
    agedge(p, p_enter_slow_motion, t4, "1", TRUE);
    agedge(p, p_contact_detected, t4, "1", TRUE);
    agedge(p, t4, p_terminate_net, "1", TRUE);
 
    return *p;
}

void iiwa_controller_reset_petrinet(petrinet_t *p) {
        ;
}
