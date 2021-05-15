
#ifndef DETECT_OBSTACLE_H_
#define DETECT_OBSTACLE_H_

#define NB_LEDS 2

#include <msgbus/messagebus.h>
#include <sensors/proximity.h>



void show_obstacle(proximity_msg_t *prox_values);
void detect_obstacle_start(void);



#endif /* DETECT_OBSTACLE_H_ */
