
/*
 * @file lcsm.c
 * @brief LCSM example
 *
 * Life-cycle state machine implementation
 *
 * (c) Nico Huebel (KU Leuven) 2019
 * (c) Sergio Portoles Diez (KU Leuven) 2019
 * (c) Filip Reniers (KU Leuven) 2019
 *
 */


#include <coordination-libs/lcsm/lcsm.h>
// #include <stdio.h>

//could be replaced with constructure or default initialization above in C++
void init_lcsm(LCSM_t * sm)
{
    sm->state = CREATION;
    for (int i = 0; i<MAX_EVENT_QUEUE; i++)
    {
        sm->event_queue[i] = EMPTY;
    }
    sm->number_of_events = 0;
}


void update_lcsm(LCSM_t *sm)
{
    for (int i=0;i<sm->number_of_events; i++)
    {
        switch(sm->state) {
            case CREATION:
                if (sm->event_queue[i] == CREATED)
                {
                    // printf("LCSM: I was creation. Received: created. Moving to: configuring.\n");
                    sm->state = RESOURCE_CONFIGURATION;
                }
                break;
            case RESOURCE_CONFIGURATION:
                if (sm->event_queue[i] == RESOURCES_CONFIGURED)
                {
                    sm->state = CAPABILITY_CONFIGURATION;
                }
                else if (sm->event_queue[i] == CLEANUP_RESOURCES)
                {
                    sm->state = CLEANING;
                }
                break;
            case CAPABILITY_CONFIGURATION:
                if (sm->event_queue[i] == CAPABILITIES_CONFIGURED)
                {
                    sm->state = PAUSING;
                }
                else if (sm->event_queue[i] == CONFIGURE_RESOURCES)
                {
                    sm->state = RESOURCE_CONFIGURATION;
                }
                break;
            case PAUSING:
                if (sm->event_queue[i] == START)
                {
                    sm->state = RUNNING;
                }
                else if (sm->event_queue[i] == CONFIGURE_CAPABILITIES)
                {
                    sm->state = CAPABILITY_CONFIGURATION;
                }
                break;
            case RUNNING:
                if (sm->event_queue[i] == STOP)
                {
                    sm->state = PAUSING;
                }
                break;
            case CLEANING:
                if (sm->event_queue[i] == CLEANED)
                {
                    sm->state = DONE;
                }
                break;
            case DONE:
                if (sm->event_queue[i] == CREATE)
                {
                    sm->state = CREATION;
                }
                break;
        }
        // Assumption: local copy of event queue -> so can delete all events after consumption; TODO: replace with ring buffer
        sm->event_queue[i] = EMPTY;
    }
    sm->number_of_events = 0;
}

int add_event(LCSM_t *sm, LCSM_event_t event) {
    int ret = -1;
    if (sm->number_of_events < MAX_EVENT_QUEUE)
    {
        sm->event_queue[sm->number_of_events] = event; // since setting an int is atomic, this should be thread safe
        sm->number_of_events++;
        ret = 0;
    }
    return ret;
}
