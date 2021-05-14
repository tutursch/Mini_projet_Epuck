/*
 * detect_obstacle.h
 *
 *  Created on: 26 avr. 2021
 *      Author: baudo
 */

#ifndef DETECT_OBSTACLE_H_
#define DETECT_OBSTACLE_H_

#define NB_LEDS 8

#include <msgbus/messagebus.h>
#include <sensors/proximity.h>



void show_obstacle(proximity_msg_t *prox_values);
void detect_obstacle_start(void);



#endif /* DETECT_OBSTACLE_H_ */
