#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#define ERROR_COLOR_COEFF 1.5f //coefficient de compensation de l'erreur de la camera
#define MEAN_SECURITY 3 //coefficient pour être sûr de bien être dans la zone noire


static int width = 0;
static uint16_t line_position = 0;
static int status = 0; // 0 : à l'arret, 1 : vitesse normale, 2 : vitesse passage piétons
static uint32_t previous_mean = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(image_light_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */

static THD_WORKING_AREA(waCaptureImageRoad, 256);
static THD_FUNCTION(CaptureImageRoad, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 470, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
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

	//systime_t tt;

	bool send_to_computer = true;

    while(1){


    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);

       // tt = chVTGetSystemTime();
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		uint16_t i = 0, begin = 0, end = 0, width = 0;
		uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
		uint32_t mean = 0;

		static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

		//performs an average
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			image[i] = (img_buff_ptr[i*2+1] & 0b00011111);
			mean += image[i];
		}
		mean /= IMAGE_BUFFER_SIZE;
		if (previous_mean == 0){
//			chprintf((BaseSequentialStream *)&SDU1, "DONE ");
			previous_mean = mean;
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
//		chprintf((BaseSequentialStream *)&SDU1, "width = %d\r\n", width);

		if (width > 100){ //si la bande noire est aussi grande que d'habitude, l'e-puck est sur une route classique
			status = 1;
			previous_mean = mean;
		}else if (previous_mean * ERROR_COLOR_COEFF < mean){ //si le robot perçoit une couleur bien plus grande que d'habitude, l'e-puck croise un passage piéton
			line_position = IMAGE_BUFFER_SIZE/2;
			status = 2;
		}
//		chprintf((BaseSequentialStream *)&SDU1, "status = %d\r\n", status);
		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}

		//invert the bool
		send_to_computer = !send_to_computer;
<<<<<<< HEAD
		chprintf((BaseSequentialStream *)&SDU1, "mean = %d\r\n", mean);
//		chprintf((BaseSequentialStream *)&SDU1, "previous_mean = %d\r\n", previous_mean);
//		chprintf((BaseSequentialStream *)&SDU1, "line_position = %d\r\n", line_position);
//		chprintf((BaseSequentialStream *)&SDU1, "status = %d\r\n", status);
    }
}

static THD_WORKING_AREA(waCaptureImageLight, 256);
static THD_FUNCTION(CaptureImageLight, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 100, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_light_ready_sem);
    }
}

static THD_WORKING_AREA(waProcessImageLight, 1024);
static THD_FUNCTION(ProcessImageLight, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_R[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_G[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_B[IMAGE_BUFFER_SIZE] = {0};

	//bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_light_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		uint8_t mean_R = 0, mean_G = 0, mean_B = 0;

		//performs an average
		for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
			image_R[i] = (img_buff_ptr[i*2] & 0b11111000);
			mean_R += image_R[i];
			image_G[i] = (img_buff_ptr[i*2] & 0b00000111) & (img_buff_ptr[i*2+1] & 0b11100000);
			mean_G += image_G[i];
			image_B[i] = (img_buff_ptr[i*2+1] & 0b00011111);
			mean_B += image_B[i];
		}
		mean_R /= IMAGE_BUFFER_SIZE;
		mean_G /= IMAGE_BUFFER_SIZE;
		mean_B /= IMAGE_BUFFER_SIZE;
		if ((mean_R > ERROR_COLOR_COEFF * mean_G) & (mean_R > ERROR_COLOR_COEFF * mean_B)){
			status = 0;
		}
//		if(send_to_computer){
//			//sends to the computer the image
//			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
//		}
//
//		//invert the bool
//		send_to_computer = !send_to_computer;
=======
		//chprintf((BaseSequentialStream *)&SD3, "delta: %d ", 127);

		//chThdSleepUntilWindowed(tt, tt + MS2ST(10));
>>>>>>> detect_obstacle
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
<<<<<<< HEAD
	chThdCreateStatic(waProcessImageRoad, sizeof(waProcessImageRoad), NORMALPRIO, ProcessImageRoad, NULL);
	chThdCreateStatic(waCaptureImageRoad, sizeof(waCaptureImageRoad), NORMALPRIO, CaptureImageRoad, NULL);
	chThdCreateStatic(waProcessImageLight, sizeof(waProcessImageLight), NORMALPRIO, ProcessImageLight, NULL);
	chThdCreateStatic(waCaptureImageLight, sizeof(waCaptureImageLight), NORMALPRIO, CaptureImageLight, NULL);
}

int get_status(void){
	return status;
=======
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
>>>>>>> detect_obstacle
}
