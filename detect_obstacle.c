
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
static bool presence_obstacle = 0;
static uint8_t threshold = 100;

void show_obstacle(proximity_msg_t *prox_values){

	uint8_t led_val[NB_LEDS] = {0};
	if (prox_values->ambient[0] - prox_values->reflected[0] > threshold){
	    led_val[0] = 1;
	}else{
		led_val[0]=0;
	 }

	if (prox_values->ambient[7] - prox_values->reflected[7] > threshold){
	    led_val[1] = 1;
	 }else{
	    led_val[1]=0;
	 }

	 //arrêter les moteurs si IR1 ou IR8 détectent un obstacle
	 if ((led_val[0] == 1) & (led_val[1]==1)){
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
    		//animation clignotant lorsqu'un piéton est détecté
    		playNote(NOTE_D4,10);
    		set_led (LED7, 1);
    		set_led (LED3, 1);
      		chThdSleepMilliseconds(500);
      		playNote(NOTE_A3,10);
 		   	set_led (LED7, 0);
 		   	set_led (LED3, 0);
    		chThdSleepMilliseconds(500);


    	}else{//éviter les cas non traités
    	    __asm__ volatile ("nop");
    	}
   	}
}


void detect_obstacle_start(){
	chThdCreateStatic(waDetectObstacle, sizeof(waDetectObstacle), NORMALPRIO, DetectObstacle, NULL );
	chThdCreateStatic(waTreatObstacle, sizeof(waTreatObstacle), NORMALPRIO+1, TreatObstacle, NULL );
}








