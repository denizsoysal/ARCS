/*
 * @file top-level_4_state_lcsm.h
 * @brief Instantiation of Top Level 4 state Life-Cycle State Machine
 *
 * Top Level 4 state Life-Cycle State Machine:
 *  - instantiation of configuration structure to create a 4 state FSM to coordinate the overall life-cycle
 *
 * (c) Filip Reniers (KU Leuven) 18.10.20
 *
 */


#ifndef COORDINATION_LIBS_TOP_LEVEL_4_STATE_LCSM_H
#define COORDINATION_LIBS_TOP_LEVEL_4_STATE_LCSM_H

#include <coordination-libs/fsm/FSM.h>

#ifdef __cplusplus
extern "C" {
#endif

    extern FSM_configuration_t top_level_lcsm;

#ifdef __cplusplus
}
#endif

#endif //COORDINATION_LIBS_TOP_LEVEL_4_STATE_LCSM_H
