/*
 * @file FSM.c
 * @brief Optimized Finite-State Machine implementation
 *
 * Finite-State Machine implementation:
 *  - Requires the specification of the transition table, state names and event names
 *  - Makes a state transition look-up table for efficient access and
 *  - Firing events happens with string name.
 *
 * (c) Filip Reniers (KU Leuven) 18.10.20
 *
 */
#include <coordination-libs/fsm/FSM.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

unsigned int index_lookup(index_map_t **hash_table, const char *name) {

    index_map_t *s;
    HASH_FIND_STR(*hash_table, name, s);

    return s->index;
}

void new_string_index_entry(index_map_t **hash_table, const char *name, int index){
    index_map_t *s = (index_map_t *)malloc(sizeof(index_map_t));

    snprintf(s->name, MAX_LENGTH_INDEX_MAP_NAME, "%s", name);
    s->index = index;
    HASH_ADD_STR( *hash_table, name, s );

}

void delete_index_hash_table(index_map_t **hash_table){
    index_map_t *current_entry, *tmp;
    HASH_ITER(hh, *hash_table, current_entry, tmp) {
        HASH_DEL(*hash_table, current_entry);
        free(current_entry);
    }
}

void create_index_hash_table(index_map_t **hash_table, const char **string_names, int number_of_strings) {
    for (int i = 0; i < number_of_strings; i++) {
        new_string_index_entry(hash_table, string_names[i], i);
    }
}


int add_event_fsm_int(FSM_t *fsm, transition_event_t event) { // todo char *
    int ret = -1;
    if (fsm->number_of_events < MAX_LENGTH_FSM_EVENT_QUEUE) {
        fsm->event_queue[fsm->number_of_events] = event; // since setting an int is atomic, this should be thread safe
        fsm->number_of_events++;
        ret = 0;
    }
    return ret;
}

int add_event_fsm(FSM_t *fsm, const char *event_name) {

    transition_event_t event_int = index_lookup(&fsm->event_names, event_name);

    return add_event_fsm_int(fsm, event_int);
}

void init_fsm(FSM_t *fsm, FSM_configuration_t *fsm_configuration) {
    fsm->state = fsm_configuration->default_state;

    for (int i = 0; i < MAX_LENGTH_FSM_EVENT_QUEUE; i++) {
        fsm->event_queue[i] = 0;
    }
    fsm->number_of_events = 0;

    fsm->fsm_configuration = fsm_configuration;

    /* Create for every state an index table */
    fsm->transition_lookup.state_entries = (transition_index_table_t *) malloc(
            fsm_configuration->number_of_states * sizeof(transition_index_table_t));
    fsm->transition_lookup.number_of_states = fsm_configuration->number_of_states;

    transition_table_t *all_transitions = fsm->fsm_configuration->transition_table; // transition table
    transition_index_table_t *array_of_indices = fsm->transition_lookup.state_entries; // array of transition index tables

    // Initialize count of transition entries
    for (int i = 0; i < fsm->fsm_configuration->number_of_states; i++) {
        array_of_indices[i].number_of_transitions = 0;
    }

    // Count occurences of transition entries for every state1
    for (int i = 0; i < all_transitions->number_of_transitions; i++) {
        array_of_indices[all_transitions->transition_table_entries[i].state1].number_of_transitions++;
    }

    // Allocate memory for every state (reset number of state_entries for last iteration)
    for (int i = 0; i < fsm->fsm_configuration->number_of_states; i++) {
        array_of_indices[i].transition_table_entries = (transition_table_entry_t **) malloc(
                array_of_indices[i].number_of_transitions * sizeof(transition_table_entry_t *));
        array_of_indices[i].number_of_transitions = 0;
    }

    // Put pointer to state_entries in index
    for (int i = 0; i < all_transitions->number_of_transitions; i++) {
        state_t curr_state = all_transitions->transition_table_entries[i].state1;
        transition_index_table_t *index_table_i = &array_of_indices[curr_state]; // transition index table

        index_table_i->transition_table_entries[array_of_indices[curr_state].number_of_transitions] = &all_transitions->transition_table_entries[i];
        array_of_indices[curr_state].number_of_transitions++;
    }

    fsm->state_names = NULL;
    fsm->event_names = NULL;

    // HASH TABLES STRING NAMES
    create_index_hash_table(&fsm->state_names, fsm->fsm_configuration->state_names, fsm->fsm_configuration->number_of_states);
    create_index_hash_table(&fsm->event_names, fsm->fsm_configuration->event_names, fsm->fsm_configuration->number_of_events);

}

void update_fsm(FSM_t *fsm) {
    for (int i = 0; i < fsm->number_of_events; i++) {
        transition_index_table_t transition_index_table = fsm->transition_lookup.state_entries[fsm->state];
        for (int j = 0; j < transition_index_table.number_of_transitions; j++) {
            transition_table_entry_t *transition_table_entry = transition_index_table.transition_table_entries[j];
            if (fsm->event_queue[i] == transition_table_entry->transition_event) {
                fsm->state = transition_table_entry->state2;
            }
        }
    }
    fsm->number_of_events = 0;
}

void delete_fsm(FSM_t *fsm) {

    for (int i = 0; i < fsm->transition_lookup.number_of_states; i++) {
        free(fsm->transition_lookup.state_entries[i].transition_table_entries);
    }

    free(fsm->transition_lookup.state_entries);

    delete_index_hash_table(&fsm->state_names);
    delete_index_hash_table(&fsm->event_names);

}