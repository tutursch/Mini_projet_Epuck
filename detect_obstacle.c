/*
 * detect_obstacle.c
 *
 *  Created on: 26 avr. 2021
 *      Author: baudo
 */

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <i2c_bus.h>

#include <main.h>
#include <detect_obstacle.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <leds.h>
#include <motors.h>
#include <spi_comm.h>
#include <msgbus/messagebus.h>

static BSEMAPHORE_DECL(obstacle_detected_sem, TRUE);
static uint8_t presence_obstacle = 0;
static uint8_t threshold = 100;

void show_obstacle(proximity_msg_t *prox_values){

	uint8_t led_val[NB_LEDS] = {0};
	uint8_t nb_obstacles = 0;
	for (uint8_t i = 0; i < PROXIMITY_NB_CHANNELS ; i++){

	    if (prox_values->ambient[i] - prox_values->reflected[i] > threshold){
	    	led_val[i] = 1;
	        nb_obstacles++;
	    }else{
	    	led_val[i]=0;
	    }
	 }

	 set_led (LED1, led_val[0]);
	 set_rgb_led (LED8, led_val[7]*255,0,0);

	 //arrêter les moteurs si IR1 ou IR8 détectent un obstacle
	 if ((led_val[0] == 1) || (led_val[7]==1)){
		presence_obstacle = 1;
	 }else{
		 presence_obstacle = 0;
	 }

}



static THD_WORKING_AREA(waDetectObstacle, 256);
static THD_FUNCTION(DetectObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

    systime_t time_det;

    calibrate_ir();

    while(1){

    	time_det = chVTGetSystemTime();

    	messagebus_topic_wait(proximity_topic, &prox_values, sizeof(prox_values));
    	//chprintf((BaseSequentialStream *)&SD3, "delta: %d ", prox_values.delta[0]);
    	//chprintf((BaseSequentialStream *)&SD3, "init: %d ", prox_values.initValue[0]);

    	show_obstacle(&prox_values);

    	if (presence_obstacle==1){
    	    left_motor_set_speed(0);
    	    right_motor_set_speed(0);
    	  	chBSemSignal(&obstacle_detected_sem);
    	}else{
    		__asm__ volatile ("nop");
    	}


    	chThdSleepUntilWindowed(time_det, time_det + MS2ST(10));

    }
}

static THD_WORKING_AREA(waTreatObstacle, 256);
static THD_FUNCTION(TreatObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	chBSemWait(&obstacle_detected_sem);

    	if (presence_obstacle == 1){//on remet la condition, à cause de la priorité supérieure de cette thd.
    		playMelody(RUSSIA,ML_SIMPLE_PLAY, NULL);
    		set_led (LED7, 1);
    		set_led (LED3, 1);
      		chThdSleepMilliseconds(500);
 		   	set_led (LED7, 0);
 		   	set_led (LED3, 0);
    		chThdSleepMilliseconds(500);


    	}else{
    		stopCurrentMelody();
    	  //  __asm__ volatile ("nop");
    	}

   	}

    //chThdSleepUntilWindowed(time_obst, time_obst + MS2ST(10));
    	//chThdSleep(MS2ST(30));
}


void detect_obstacle_start(){
	chThdCreateStatic(waDetectObstacle, sizeof(waDetectObstacle), NORMALPRIO, DetectObstacle, NULL );
	chThdCreateStatic(waTreatObstacle, sizeof(waTreatObstacle), NORMALPRIO+1, TreatObstacle, NULL );
}








