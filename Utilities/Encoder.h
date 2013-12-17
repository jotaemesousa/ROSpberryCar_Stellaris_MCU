/*
 * Encoder.h
 *
 *  Created on: Jul 1, 2013
 *      Author: bgouveia
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

void encoder_read(int32_t *left_c, int32_t *right_c);
void encoder_read_reset(int32_t *left_c, int32_t *right_c);
void encoder_get_velocity(int32_t *left_vel, int32_t *right_vel, uint32_t time);
void encoder_init();



#endif /* ENCODER_H_ */
