#ifndef __FOC_CTRL_NEW_H__
#define __FOC_CTRL_NEW_H__

#include "foc_drv.h"
#include "observer.h"
#include "startup.h"

typedef enum
{
    ANGLE_SRC_ENCODER = 0,
    ANGLE_SRC_OBSERVER = 1,
    ANGLE_SRC_HYBRID = 2
} angle_src_t;

typedef struct
{
    angle_src_t current_src;
    angle_src_t target_src;
    float transition_weight;
    uint8_t transition_active;
} angle_switch_t;

extern observer_t g_observer;
extern startup_t g_startup;
extern angle_switch_t g_angle_switch;

void foc_ctrl_init(void);
void foc_ctrl_update(void);
void angle_source_switch(angle_src_t target_src);
void angle_fusion_update(void);

#endif
