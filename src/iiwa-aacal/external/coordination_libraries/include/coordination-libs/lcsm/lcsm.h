/*
 *
 * @file lcsm.h
 * @brief LCSM example
 *
 * Life-cycle state machine implementation
 *
 * (c) Filip Reniers (KU Leuven) 2019
 * (c) Nico Huebel (KU Leuven) 2019
 * (c) Sergio Portoles Diez (KU Leuven) 2019
 *
 */

#ifndef COORDINATION_LIBS_LCSM_
#define COORDINATION_LIBS_LCSM_

# define MAX_EVENT_QUEUE 5
#define NUM_NEW_LCSM_STATES 7

#ifdef __cplusplus
extern "C" {
#endif

// TODO: Fix the names of different states,
// TODO: onExit(), onEnter(), doo()
typedef enum {
    CREATION, RESOURCE_CONFIGURATION, CAPABILITY_CONFIGURATION, PAUSING, RUNNING, CLEANING, DONE
} LCSM_state_t;
typedef enum {
    CREATE,
    CREATED,
    CONFIGURE_RESOURCES,
    RESOURCES_CONFIGURED,
    CONFIGURE_CAPABILITIES,
    CAPABILITIES_CONFIGURED,
    START,
    STOP,
    CLEANUP_RESOURCES,
    CLEANED,
    EMPTY
} LCSM_event_t;
/* transition table:
 * creation -(created)-> configuring
 * creation -(cleanup)-> cleaning
 * configuring -(configured)-> active
 * active -(start)-> running
 * running -(stop)-> active
 * (NOT), active -(disable)-> creation (need to cleanup)
 * cleaning -(cleaned)-> creation
 */

static const char *lcsm_state_names[] =
        {
                "Creation",
                "Resource configuration",
                "Capability configuration",
                "Pausing",
                "Running",
                "Deletion",
                "Done"
        };

typedef
struct LCSM_s {
    LCSM_state_t state;
    LCSM_event_t event_queue[MAX_EVENT_QUEUE]; //TODO: replace with event stream/ring buffer
    int number_of_events;
} LCSM_t;

int add_event(LCSM_t *sm, LCSM_event_t event);
void init_lcsm(LCSM_t *sm);
void update_lcsm(LCSM_t *sm);

#ifdef __cplusplus
}
#endif

#endif // End of COORDINATION_LIBS_LCSM_
