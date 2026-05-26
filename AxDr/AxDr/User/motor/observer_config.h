#ifndef __OBSERVER_CONFIG_H__
#define __OBSERVER_CONFIG_H__

#define OBSERVER_TYPE_SMO       1
#define OBSERVER_TYPE_FLUX      2
#define OBSERVER_TYPE_EMF       3

#ifndef OBSERVER_TYPE
#define OBSERVER_TYPE           OBSERVER_TYPE_SMO
#endif

#define OBS_MOTOR_RS            0.5f
#define OBS_MOTOR_LD            0.001f
#define OBS_MOTOR_LQ            0.001f
#define OBS_MOTOR_FLUX          0.01f
#define OBS_MOTOR_TS            0.00005f

#define OBS_SMO_H1_BASE         100.0f
#define OBS_SMO_PLL_KP          500.0f
#define OBS_SMO_PLL_KI          1000.0f

#define OBS_FLUX_GAMMA_BASE     200.0f
#define OBS_FLUX_PLL_KP         500.0f
#define OBS_FLUX_PLL_KI         1000.0f

#define STARTUP_ALIGN_CURRENT   0.5f
#define STARTUP_ALIGN_TIME_MS   500.0f
#define STARTUP_START_FREQ      5.0f
#define STARTUP_END_FREQ        50.0f
#define STARTUP_RAMP_RATE       10.0f
#define STARTUP_SWITCH_TIME_MS  100.0f

#endif
