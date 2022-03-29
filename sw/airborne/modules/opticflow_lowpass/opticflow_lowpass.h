
/**
 * @file "modules/opticflow_lowpass/opticflow_lowpass.h"
 * @author Group 5
 * Avoid Obstacles with optic flow
 */

#ifndef OPTICFLOW_LOWPASS_H
#define OPTICFLOW_LOWPASS_H

// settings
extern float div_thr;

// functions
extern void opticflow_lowpass_init(void);
extern void opticflow_lowpass_periodic(void);

#endif
