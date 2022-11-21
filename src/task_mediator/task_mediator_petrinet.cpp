/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file task_mediator_petrinet.cpp
 * @date November, 2022
 **/

#include "task_mediator/task_mediator_petrinet.hpp"

// Tracking sources
char* bringup_tracking_source_names[] = {//const
    "board_in_range",
    "board_dirty",
    "start_vel_transition",
    "end_vel_transition",
    "contact_detected"
};
bool* bringup_tracking_source_flags[NUMBER_OF_TRACKING_SOURCES];

// Tracking sinks
char* bringup_tracking_sink_names[] = {//const
    "identify_dirty_patch_ready",
    "start_approach",
    "enter_blend_model",
    "enter_slow_motion",
    "terminate_net"
};
bool* bringup_tracking_sink_flags[NUMBER_OF_TRACKING_SINKS];

flag_token_conversion_map_t task_mediator_bringup_petrinet_flag_map = {
        .converting_sources = {
                .names = NULL,
                .flags = NULL,
                .number_of_flags = 0
        },
        .tracking_sources = {
                .names = bringup_tracking_source_names,
                .flags = bringup_tracking_source_flags,
                .number_of_flags = NUMBER_OF_TRACKING_SOURCES
        },
        .converting_sinks = {
                .names = NULL,
                .flags = NULL,
                .number_of_flags = 0
        },
        .tracking_sinks = {
                .names = bringup_tracking_sink_names,
                .flags = bringup_tracking_sink_flags,
                .number_of_flags = NUMBER_OF_TRACKING_SINKS
        }
};

petrinet_t task_mediator_create_bringup_petrinet(char *name) {//const
    petrinet_t *p = init_petrinet(name);

    place_t *p_board_in_range= create_place(p, bringup_tracking_source_names[BOARD_IN_RANGE]);
    place_t *p_board_dirty = create_place(p, bringup_tracking_source_names[BOARD_DIRTY]);
    place_t *p_start_vel_transition = create_place(p, bringup_tracking_source_names[START_VEL_TRANSITION]);

    place_t *p_identify_dirty_patch_ready = create_place(p, bringup_tracking_sink_names[IDENTIFY_DIRTY_PATCH_READY]);
    place_t *p_start_approach = create_place(p, bringup_tracking_sink_names[START_APPROACH]);
    place_t *p_enter_blend_model = create_place(p, bringup_tracking_sink_names[ENTER_BLEND_MODEL]);
    
    transition_behaviour_t b1 = {
            .condition = cond_Black1,
            .consumption_behaviour=consume_Black1,
            .production_behaviour=produce_Black1
    }; //this behavior is when all places before transition needs to be filled to fire the tr. After, they are all consumed and all output places are filled

    // TRANSITIONS
    transition_t *t1 = create_transition(p, "t1");
    add_behaviour(t1, &b1);

    transition_t *t2 = create_transition(p, "t2");
    add_behaviour(t2, &b1);

    agedge(p, p_board_in_range, t1, "1", TRUE);
    agedge(p, p_board_dirty, t1, "1", TRUE);
    agedge(p, t1, p_identify_dirty_patch_ready, "1", TRUE);
    agedge(p, t1, p_start_approach, "1", TRUE);

    agedge(p, p_start_approach, t2, "1", TRUE);
    agedge(p, p_start_vel_transition, t2, "1", TRUE);
    agedge(p, t2, p_enter_blend_model, "1", TRUE);
 
    return *p;
}

void task_mediator_reset_bringup_petrinet(petrinet_t *p) {
        ;
}
