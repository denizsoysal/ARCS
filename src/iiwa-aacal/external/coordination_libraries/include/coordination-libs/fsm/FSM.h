/*
 * @file FSM.h
 * @brief header of optimized Finite-State Machine implementation
 *
 * Finite-State Machine implementation:
 *  - Requires the specification of the transition table, state names and event names
 *  - Makes a state transition look-up table for efficient access and
 *  - Firing events happens with string name.
 *
 * (c) Filip Reniers (KU Leuven) 18.10.20
 *
 */

#ifndef COORDINATION_LIBS__FSM_H
#define COORDINATION_LIBS__FSM_H

#define MAX_LENGTH_FSM_EVENT_QUEUE 10
#define MAX_LENGTH_INDEX_MAP_NAME 32

#include "uthash.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct index_map_s {
    char name[MAX_LENGTH_INDEX_MAP_NAME];             /* key (string is WITHIN the structure) */
    unsigned int index;
    UT_hash_handle hh;                               /* makes this structure hashable */
} index_map_t;

//Prerequisite: states and events are monotonically increasing without gaps starting from zero
typedef unsigned int state_t;
typedef unsigned int transition_event_t;

typedef struct transition_table_entry_s {
    state_t state1;
    transition_event_t transition_event;
    state_t state2;
} transition_table_entry_t;

typedef struct transition_table_s {
    transition_table_entry_t *transition_table_entries;
    int number_of_transitions;
} transition_table_t;

typedef struct transition_index_table_s {
    transition_table_entry_t **transition_table_entries;
    int number_of_transitions;
} transition_index_table_t;

typedef struct FSM_configuration_s {
    transition_table_t *transition_table;

    const char **state_names;
    int number_of_states;
    state_t default_state;

    const char **event_names;
    int number_of_events;
} FSM_configuration_t;

typedef struct FSM_transition_lookup_s {
    transition_index_table_t *state_entries;
    int number_of_states;
} FSM_transition_lookup_t;

typedef struct FSM_s {
    state_t state;
    transition_event_t event_queue[MAX_LENGTH_FSM_EVENT_QUEUE];
    int number_of_events;

    FSM_configuration_t *fsm_configuration;

    // Optimization: state look-up table -> transition
    FSM_transition_lookup_t transition_lookup;
    // Optimization: hash table event + state names
    index_map_t *state_names;
    index_map_t *event_names;
} FSM_t;

int add_event_fsm(FSM_t *fsm, const char *event_name);

void init_fsm(FSM_t *fsm, FSM_configuration_t *fsm_configuration);

void update_fsm(FSM_t *fsm);

void delete_fsm(FSM_t *fsm);

#ifdef __cplusplus
}
#endif

#endif //COORDINATION_LIBS__FSM_H
