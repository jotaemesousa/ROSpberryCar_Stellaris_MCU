/*
 * remote_defines.h
 *
 *  Created on: May 9, 2013
 *      Author: joao
 */

#ifndef REMOTE_DEFINES_H_
#define REMOTE_DEFINES_H_

#define COM_DRV_ZERO	694
#define COM_DRV_FRONT	779
#define COM_DRV_REAR	586
#define COM_SERVO_ZERO	645
#define COM_SERVO_RIGHT	737
#define COM_SERVO_LEFT	536

#define SERVO_ZERO		80
#define SERVO_LEFT		116
#define SERVO_RIGHT		40

#define DRV_ZERO		0
#define DRV_REAR		-70
#define DRV_FRONT		70

#define SEND_MSG_TIME	500


typedef struct ROSpberryRemote
{
	int16_t linear;
	int16_t steer;
	uint8_t buttons;

}RC_remote;

#endif /* REMOTE_DEFINES_H_ */
