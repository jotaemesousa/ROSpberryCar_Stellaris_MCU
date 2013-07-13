/*
 * Encoder.h
 *
 *  Created on: Jul 1, 2013
 *      Author: bgouveia
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

void encoder_read(int *left_c, int *right_c);
void encoder_get_velocity(int16_t *left_vel, int16_t *right_vel, uint32_t time);
void encoder_init();



#endif /* ENCODER_H_ */
