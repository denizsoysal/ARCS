/*
 * @file top-level_4_state_lcsm.c
 * @brief Instantiation of Top Level 4 state Life-Cycle State Machine
 *
 * Top Level 4 state Life-Cycle State Machine:
 *  - instantiation of configuration structure to create a 4 state FSM to coordinate the overall life-cycle
 *
 * (c) Filip Reniers (KU Leuven) 18.10.20
 *
 */


#include <coordination-libs/top-level_4_state_lcsm/top-level_4_state_lcsm.h>

const char *lcsm_4state_names[] =
        {
                "Idle",
                "Initialization",
                "Execution",
                "Deinitialization"
        };

const char *lcsm_4event_names[] = {
        "start",
        "start_execution",
        "stop_execution",
        "stop"
};

/* transition table:
 * IDLE    -> (start)           -> INIT
 * INIT    -> (start_execution) -> EXEC
 * EXEC    -> (stop_execution)  -> DEINIT
 * DEINIT  -> (stop)            -> IDLE
 */
transition_table_entry_t entries[] = {
        {
                .state1 = 0,
                .transition_event = 0,
                .state2 = 1
        },
        {
                .state1 = 1,
                .transition_event = 1,
                .state2 = 2
        },
        {
                .state1 = 2,
                .transition_event = 2,
                .state2 = 3
        },
        {
                .state1 = 3,
                .transition_event = 3,
                .state2 = 0
        }
};

transition_table_t transition_table = {
        .transition_table_entries = entries,
        .number_of_transitions = 4
};

FSM_configuration_t top_level_lcsm = {
        .transition_table = &transition_table,
        .state_names = lcsm_4state_names,
        .number_of_states = 4,
        .default_state = 0,
        .event_names = lcsm_4event_names,
        .number_of_events = 4
};
