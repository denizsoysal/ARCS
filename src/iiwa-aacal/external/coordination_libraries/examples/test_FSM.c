/*
 * @file test_FSM.c
 * @brief Test for FSM functionality
 *
 * Test for FSM functionality
 *  - makes use of optimized fsm library and the instantiation of the 4 state FSM configuration structure.
 *
 * (c) Filip Reniers (KU Leuven) 18.10.20
 *
 */

#include <coordination-libs/fsm/FSM.h>
#include <coordination-libs/top-level_4_state_lcsm/top-level_4_state_lcsm.h>

int main(int argc, char **argv){

    FSM_t fsm;

    init_fsm(&fsm, &top_level_lcsm);

    update_fsm(&fsm);
    add_event_fsm(&fsm, "start");
    update_fsm(&fsm);
    add_event_fsm(&fsm, "start_execution");
    update_fsm(&fsm);
    add_event_fsm(&fsm, "stop_execution");
    update_fsm(&fsm);
    add_event_fsm(&fsm, "stop");
    update_fsm(&fsm);

    delete_fsm(&fsm);


    return 0;
}