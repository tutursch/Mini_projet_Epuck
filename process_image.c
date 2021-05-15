#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>
#include <leds.h>

#include <process_image.h>


static int width = 0;
static uint16_t line_position = 0;
static uint8_t status = 1; // 0 : à l'arret, 1 : vitesse normale, 2 : vitesse passage piétons, 3 : arrêt warning
static uint8_t count = 0;
static uint8_t light = 0;
static uint32_t intensity_mean = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(image_light_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

static THD_WORKING_AREA(waCaptureImage, 1024);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){

    	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 to check if there is a red light
    	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
   		dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
   		dcmi_prepare();
   		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured

		light = red_light();

		//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 470 + 471 to "read" the road
		po8030_advanced_config(FORMAT_RGB565, 0, 470, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
		dcmi_enable_double_buffering();
		dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
		dcmi_prepare();
		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImageRoad, 1024);
static THD_FUNCTION(ProcessImageRoad, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		uint16_t i = 0, begin = 0, end = 0, width = 0;
		uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
		uint32_t mean = 0;

		static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

		//performs an average
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			image[i] = ((img_buff_ptr[i*2] & 0b11111000));
			mean += image[i];
		}
		mean /= IMAGE_BUFFER_SIZE;
		//calculation of the mean of 3 pixels around the middle
		intensity_mean = (image[250] + image[320] + image[390])/ 3;
		if (count == 50){
			intensity_mean /= count;
		}
		do{
			wrong_line = 0;
			//search for a begin
			while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
				//the slope must at least be WIDTH_SLOPE wide and is compared
			    //to the mean of the image
			    if(image[i] > mean && image[i+WIDTH_SLOPE] < mean){
			    	begin = i;
				    stop = 1;
			    }
			    i++;
			}
			//if a begin was found, search for an end
			if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin){
				stop = 0;

			    while(stop == 0 && i < IMAGE_BUFFER_SIZE){
			    	if(image[i] > mean && image[i-WIDTH_SLOPE] < mean){
			    		end = i;
				        stop = 1;
				    }
			        i++;
			    }
			    //if an end was not found
			    if (i > IMAGE_BUFFER_SIZE || !end){
				    line_not_found = 1;
				}
			}
			else{//if no begin was found
				line_not_found = 1;
			}
			//if a line too small has been detected, continues the search
			if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
				i = end;
				begin = 0;
				end = 0;
				stop = 0;
				wrong_line = 1;
			}
		}while(wrong_line);
		if(line_not_found){
			begin = 0;
			end = 0;
			width = last_width;
		}else{
			last_width = width = (end - begin);
			line_position = (begin + end)/2; //gives the line position.
		}


		//sets a maximum width or returns the measured width
		if((PXTOCM/width) > MAX_DISTANCE){
			width = PXTOCM/MAX_DISTANCE;
		}
		//if the width is big enough, then we are on the road
		if ((width > 250) & (light == 0)){
			status = 1;
		//condition to stop stop when meeting a red light
		}else if ((width > 350) & (light == 1)){
			status = 0;
		//if there is an intensity higher in the 3 central pixels, then we are on a pedestrian crossing
		}else if(ERROR_COEFF * intensity_mean > mean){
			line_position = IMAGE_BUFFER_SIZE/2;
			status = 2;
			set_led (LED5,1);
		}
		count ++;
    }
}

uint8_t red_light(){

	uint8_t *img_buff_ptr;
	uint8_t image_R[100] = {0};
	uint8_t image_G[100] = {0};
	uint8_t image_B[100] = {0};

 	//gets the pointer to the array filled with the last image in RGB565
	img_buff_ptr = dcmi_get_last_image_ptr();

	uint32_t mean_R = 0, mean_G = 0, mean_B = 0;

	//performs an average on each color only within the 100 first pixels that are where the robot
	//may see a light
	for(uint16_t i = 0 ; i < 100 ; i++){
		image_R[i] = (img_buff_ptr[i*2] & 0b11111000);
		mean_R += image_R[i];
		image_G[i] = ((img_buff_ptr[i*2] & 0b00000111)<<5) + ((img_buff_ptr[i*2+1] & 0b11100000)>>3);
		mean_G += image_G[i];
		image_B[i] = ((img_buff_ptr[i*2+1] & 0b00011111)<<3);
		mean_B += image_B[i];
	}
	//
	chBSemSignal(&image_light_ready_sem);
	mean_R /= 100;
	mean_G /= 100;
	mean_B /= 100;

	//if the intensity of red is higher than the intensity of green and blue, then there is a red light
	if ((mean_R > ERROR_COEFF * mean_G) & (mean_R > ERROR_COEFF * mean_B)){
		return 1;
	}else{
		return 0;
	}
}

float get_distance_cm(void){
	float dist = 0;
	dist = (400-width)/21.8;
	return dist;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImageRoad, sizeof(waProcessImageRoad), NORMALPRIO, ProcessImageRoad, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint8_t get_status(void){
	return status;
}
