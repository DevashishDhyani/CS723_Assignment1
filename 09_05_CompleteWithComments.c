// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "system.h"                     // to use the symbolic names
#include "io.h"
#include "altera_avalon_pio_regs.h" 	// to use PIO functions
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types
#include "sys/alt_irq.h"              	// to register interrupts

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "FreeRTOS/timers.h"

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Task Priorities
#define SEVEN_SEG_TASK_PRIORITY 1
#define PRINT_LCD_TASK_PRIORITY 2
#define PRINT_CONSOLE_TASK_PRIORITY 3
#define STATE_MANAGEMENT_TASK_PRIORITY 4
#define STABILITY_TASK_PRIORITY 6

//LCD Macros
FILE *lcd;
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// Definition of Frequency Queue
#define FREQ_QUEUE_SIZE  10
QueueHandle_t freqQueue;

// Handlers
TaskHandle_t state_management_handle;

// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
SemaphoreHandle_t shared_lcd_sem;
SemaphoreHandle_t shared_console_sem;
SemaphoreHandle_t shared_sevenseg_sem;
SemaphoreHandle_t shared_freq_sem; //protects freqArray, rocArray
SemaphoreHandle_t shared_state_toggle_sem;
SemaphoreHandle_t shared_stability_control_sem;

//Definition of Timer
TimerHandle_t systemTimer;
TimerHandle_t loadsTimer;
TimerHandle_t sheddingTimer;

// globals variables

//threshold via KB
int thresholdFreq = 50;
char tempThreshold[4] = "";
int thresholdROC = 10;

//state toggle
int stateToggle = 0; //0 = maintenance mode, 1 = load management mode

//frequency related
double freqArray[FREQ_QUEUE_SIZE] = {40, 40, 40, 40, 40, 40, 40, 40, 40, 40};
double rocArray[FREQ_QUEUE_SIZE] = {40, 40, 40, 40, 40, 40, 40, 40, 40, 40};
int freqIndex = 0;
int stabilityControlNew = 0; //0= stable, 1 = unstable
int stabilityControlOld = 0; //0= stable, 1 = unstable

//time related
int secondsCounter = 0;
int timerExpired = 0;  //0 = not timed out, 1 = timed out

unsigned int sheddingTimeCounter = 0;
unsigned int minTimeShed = 999999; //min 1st load shed time
unsigned int maxTimeShed = 0; //max 1st load shed time
unsigned int sheddingTotalCounter = 0; //for calculating average time
unsigned int sheddingSumTimeCounter = 0; //for calculating average time
double sheddingAverageTime = 0.0;

//switch related
unsigned int uiSwitchValue;
unsigned int uiSwitchValueOld;

//roc related
double rocSum = 0.0;
double rocAverage = 0.0;
unsigned int rocCounter = 0;

//load related
int describeBehaviour = 0; // 1 = connect, 2 = disconnect

//debug variables
unsigned int debugCounter =0;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

//Helper function prototypes
void describeLEDS(int describeBehaviour);
int checkFullConnection();
void manualSwitchIntervention();

//This function gets called by an ISR to generate a frequency value. The frequency is sent to freqQueue.
void freq_relay(){
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0);
	double tempFreq = 16000/(double) temp;
	xQueueSendToFrontFromISR(freqQueue, &tempFreq, pdFALSE);
	return;
}

//Timer for calculating overall system time.
void vSystemRunTimer(xTimerHandle t_timer){
	secondsCounter = secondsCounter + 1;
}

//Timer which is used to check for continuous stability or instability of frequency values for 500ms.
void vLoadsTimer(xTimerHandle t_timer){
	timerExpired = 1;
}

//Timer used to find how many milliseconds it takes for the 1st load to be shed after system is stable.
void vSheddingTimer(xTimerHandle t_timer){
	sheddingTimeCounter = sheddingTimeCounter + 1;
}

//ISR to account for keyboard inputs.
void ps2_isr (void* context, alt_u32 id)
{
  char ascii;
  int status = 0;
  unsigned char key = 0;
  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii) ;
  if ( status == 0 ) //success
  {
    switch ( decode_mode )
    {
      case KB_ASCII_MAKE_CODE :
        //printf ( "ASCII   : %x\n", key ) ;
        break ;
      case KB_LONG_BINARY_MAKE_CODE :
        // do nothing
      case KB_BINARY_MAKE_CODE :
        //printf ( "MAKE CODE : %x\n", key ) ;
        break ;
      case KB_BREAK_CODE :
        // do nothing
      default :
    	printf("\n");
    	int z = (int)(key); // Using type-casting

        if(z == 112){
        	strcat(tempThreshold, "0");
        }
        else if(z == 105){
            strcat(tempThreshold, "1");
        }
        else if(z == 114){
        	strcat(tempThreshold, "2");
        }
        else if(z == 122){
        	strcat(tempThreshold, "3");
		}
		else if(z == 107){
			strcat(tempThreshold, "4");
		}
		else if(z == 115){
			strcat(tempThreshold, "5");
		}
		else if(z == 116){
			strcat(tempThreshold, "6");
		}
		else if(z == 108){
			strcat(tempThreshold, "7");
		}
		else if(z == 117){
			strcat(tempThreshold, "8");
		}
		else if(z == 125){
			strcat(tempThreshold, "9");
		}
		else if(z == 43){ //f
			thresholdFreq = atoi(tempThreshold);  //atoi converts a char array to int
			tempThreshold[0] = '\0';
		}
		else if(z == 45) { //r
			thresholdROC = atoi(tempThreshold);
			tempThreshold[0] = '\0';
		}
		else {
			//do nothing
			tempThreshold[0] = '\0'; //for any random keystroke, reset temp variable
		}

        break ;
    }
  }
}

//ISR to account for push button presses.
void button_interrupts_function(void* context, alt_u32 id)
{
  // need to cast the context first before using it
  int* temp = (int*) context;
  (*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

  //for key3
  if(*temp==4) {
	  stateToggle = !stateToggle;
  }

  // clears the edge capture register
  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

//Task used to find RoC, system stability and notifying next task to be triggered.
void stability_task(void *pvParameters)
{
	while (1)
	{
		xQueueReceive(freqQueue, freqArray+freqIndex, portMAX_DELAY); // Pops queue, blocks indefinitely if empty

		xSemaphoreTake(shared_freq_sem, portMAX_DELAY);

		//find ROC
		if(freqIndex == 0){
			double oldF = freqArray[9];
			double newF = freqArray[0];
			rocArray[0] = fabs(((newF - oldF) * newF * oldF * 2.0) / (newF + oldF));

			if(stateToggle == 1) {
				rocSum = rocSum + rocArray[0];
				rocCounter++;
				rocAverage = rocSum/rocCounter;
			}

		} else {
			double oldF = freqArray[freqIndex-1];
			double newF = freqArray[freqIndex];
			rocArray[freqIndex] = fabs(((newF - oldF) * newF * oldF * 2.0) / (newF + oldF));

			if(stateToggle == 1) {
				rocSum = rocSum + rocArray[freqIndex];
				rocCounter++;
				rocAverage = rocSum/rocCounter;
			}
		}

		//check if stability conditions are violated
		if((freqArray[freqIndex] < thresholdFreq) || (rocArray[freqIndex] > thresholdROC)) {
			stabilityControlOld = stabilityControlNew;
			stabilityControlNew = 1;
		}
		else {
			stabilityControlOld = stabilityControlNew;
			stabilityControlNew = 0;
		}

		freqIndex = freqIndex+1;  //next element to be written to
		if(freqIndex >= FREQ_QUEUE_SIZE) {
			freqIndex = 0;
		}

		xSemaphoreGive(shared_freq_sem);

		xTaskNotifyGive(state_management_handle);
	}
}

//Task used to find current switch value and manipulate loads to match current system status.
//This task also find times associated to the 1st load sheds.
void state_management_task(void *pvParameters)
{
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY ); /* Clear the notification value before exiting. & Block indefinitely. */

		uiSwitchValueOld = uiSwitchValue;
		uiSwitchValue = 0;
		// read the value of the switch and store to uiSwitchValue
		uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

		xSemaphoreTake(shared_state_toggle_sem, portMAX_DELAY);
		xSemaphoreTake(shared_freq_sem, portMAX_DELAY);

		//checking if any switch has been manually turned off from previous iteration
		if(uiSwitchValue < uiSwitchValueOld) {
			manualSwitchIntervention();
		}

		//maintenance mode
		if(stateToggle == 0) {
			// write the value of the switches to the  LEDs
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);

			//reset stabiltyControlNew
			stabilityControlNew = 0;
			stabilityControlOld = 0;
		}
		//load management mode
		else {
			//stable
			if(stabilityControlOld == 0 && stabilityControlNew == 0){
				//timed out
				if(timerExpired == 1) {
					describeBehaviour = 1; //asking for Connect
					describeLEDS(describeBehaviour);

					//check if all loads connected or not and start timer accordingly.
					if(checkFullConnection()) {
						//
					}
					else {
						xTimerStart(loadsTimer, 0);
					}

					timerExpired = 0;
				}
				//timer is running
				else if(xTimerIsTimerActive(loadsTimer) != pdFALSE ) {
					//do Nothing
				}
				//timer hasn't started
				else {
					// write the value of the switches to the  LEDs
					IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue);
					IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
				}
			}

			//unstable
			else if(stabilityControlOld == 1 && stabilityControlNew == 1){
				//timed out
				if(timerExpired == 1) {
					describeBehaviour = 2; //asking for Disconnect
					describeLEDS(describeBehaviour);
					xTimerStart(loadsTimer, 0);
					timerExpired = 0;
				}
				//timer is running
				else if(xTimerIsTimerActive(loadsTimer) != pdFALSE ) {
					//do Nothing
				}
				//timer hasn't started
				else {
					printf("Empty condition. Never triggered.\n \n");
				}
			}

			//going from stable to unstable
			else if(stabilityControlOld == 0 && stabilityControlNew == 1){
				//timed out
				if(timerExpired == 1) {
					describeBehaviour = 1; //asking for Connect
					describeLEDS(describeBehaviour);

					//check if all loads connected and choose describeBehaviour accordingly. Start timer.
					if(checkFullConnection()) {
						describeBehaviour = 2; //asking for Disconnect
						describeLEDS(describeBehaviour);
					}
					else {
						//
					}

					xTimerStart(loadsTimer, 0);
					timerExpired = 0;
				}
				//timer is running
				else if(xTimerIsTimerActive(loadsTimer) != pdFALSE ) {
					//reset the timer
					xTimerReset(loadsTimer,0);
				}
				//timer hasn't started
				else {
					describeBehaviour = 2;
					describeLEDS(describeBehaviour);

					//shed timer stop
					xTimerStop(sheddingTimer,0);

					//max and min times related to 1st load sheds being modified
					if(sheddingTimeCounter > maxTimeShed) {
						maxTimeShed = sheddingTimeCounter;
					}
					if(sheddingTimeCounter < minTimeShed) {
						minTimeShed = sheddingTimeCounter;
					}

					//average of 1st load shed times
					sheddingTotalCounter++;
					sheddingSumTimeCounter += sheddingTimeCounter;
					sheddingAverageTime = sheddingSumTimeCounter/(double)sheddingTotalCounter;

					xTimerStart(loadsTimer, 0);
					timerExpired = 0;
				}
			}

			//going from unstable to stable
			else if(stabilityControlOld == 1 && stabilityControlNew == 0){
				//timed out
				if(timerExpired == 1) {
					describeBehaviour = 2; //asking for Disconnect
					describeLEDS(describeBehaviour);
					xTimerStart(loadsTimer, 0);
					timerExpired = 0;
				}
				//timer is running
				else if(xTimerIsTimerActive(loadsTimer) != pdFALSE ) {
					//reset the timer
					xTimerReset(loadsTimer,0);
				}
				//timer hasn't started
				else {
					printf("Empty condition. Never triggered. \n \n ");
				}
			}
		}

		//stops the shed timer if already running, or else starts it if it was stopped due to 1st load shed
		if(xTimerIsTimerActive(sheddingTimer) != pdFALSE ) {
			xTimerStop(sheddingTimer,0);
			sheddingTimeCounter = 0;
			if (xTimerStart(sheddingTimer, 0) != pdPASS){
				printf("Cannot start timer");
			}
		}
		else {
			sheddingTimeCounter = 0;
			if (xTimerStart(sheddingTimer, 0) != pdPASS){
				printf("Cannot start timer");
			}
		}

		xSemaphoreGive(shared_freq_sem);
		xSemaphoreGive(shared_state_toggle_sem);
	}
}

//Task used for printing to the seven-seg display
void seven_seg_task(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(shared_sevenseg_sem, portMAX_DELAY);
		IOWR_ALTERA_AVALON_PIO_DATA(SEVEN_SEG_BASE, 318818083);
		xSemaphoreGive(shared_sevenseg_sem);
		vTaskDelay(10000);
	}
}

//Task used to print to the lcd.
void print_lcd_task(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(shared_lcd_sem, portMAX_DELAY);

		// if the lcd is open successfully
		if(lcd != NULL)
		{
			// clearing the LCD
			fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
			fprintf(lcd, "Threshold = %d \n", thresholdFreq);
			fprintf(lcd, "RoC = %d, S = %d \n", thresholdROC, stateToggle);
		}

		xSemaphoreGive(shared_lcd_sem);
		vTaskDelay(500);
	}
}

//Task used to print to the console.
void print_console_task(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(shared_console_sem, portMAX_DELAY);
		xSemaphoreTake(shared_freq_sem, portMAX_DELAY);

		//Printing last 5 frequencies in queue. Allows for rolling over to index 9 from index 0.
		printf("Last 5 Frequencies =   ");
		if(freqIndex >= 5) {
			for(int i=0; i<=4; i++) {
				printf("%f   ", freqArray[freqIndex-1-i]);
			}
		}
		else {
			for(int i=0; i<=4; i++) {
				if((freqIndex-1-i) >= 0) {
					printf("%f   ", freqArray[freqIndex-1-i]);
				}
				else {
					if((freqIndex-1-i) == -1) {
						printf("%f   ", freqArray[9]);
					}
					else if((freqIndex-1-i) == -2) {
						printf("%f   ", freqArray[8]);
					}
					else if((freqIndex-1-i) == -3) {
						printf("%f   ", freqArray[7]);
					}
					else if((freqIndex-1-i) == -4) {
						printf("%f   ", freqArray[6]);
					}
					else if((freqIndex-1-i) == -5) {
						printf("%f ", freqArray[5]);
					}
				}
			}
		}


		printf("\n");
		printf("Total time system has been active = %d seconds \n", secondsCounter);

		if(rocAverage == 0) {
			printf("Running Average of ROC is Undefined as Load Management State hasn't been reached.\n");
		}
		else {
			printf("Running Average of ROC = %f \n", rocAverage);
		}

		if(maxTimeShed == 0) {
			printf("Maximum Shed Time, Minimum Shed Time and Average Shed Time are all Undefined as No Load has been Shed \n \n");
		}
		else {
			printf("Maximum Shed Time = %d, Minimum Shed Time = %d, Average Shed Time = %f \n \n", maxTimeShed, minTimeShed, sheddingAverageTime);
		}

		xSemaphoreGive(shared_freq_sem);
		xSemaphoreGive(shared_console_sem);

		vTaskDelay(1000);
	}
}

//Main function that is run. Sets up timers, ISRs, the scheduler and the data structures.
int main(int argc, char* argv[], char* envp[])
{
	//Timer ISR for system
	systemTimer = xTimerCreate("System Timer", 1000, pdTRUE, NULL, vSystemRunTimer);
	if (xTimerStart(systemTimer, 0) != pdPASS){
		printf("Cannot start timer");
	}

	//Timer ISR for loads in freqRelay
	loadsTimer = xTimerCreate("Loads Timer", 500, pdFALSE, NULL, vLoadsTimer);

	//Timer ISR for 1st load shed
	sheddingTimer = xTimerCreate("Shedding Timer", 1, pdTRUE, NULL, vSheddingTimer);

	//Frequency ISR
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	//Keyboard ISR
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

    if(ps2_device == NULL){
    	printf("can't find PS/2 device\n");
    	return 1;
    }

    alt_up_ps2_clear_fifo (ps2_device) ;

    alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);
    // register the PS/2 interrupt
    IOWR_8DIRECT(PS2_BASE,4,1);

	//Button ISR
	int buttonValue = 0;
	// clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	// enable interrupts for all buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	// register the ISR
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, button_interrupts_function);

	//LCD
	lcd = fopen(CHARACTER_LCD_NAME, "w");

	//Initializations
	initOSDataStructs();
	initCreateTasks();
	vTaskStartScheduler();

	while(1){

	}

	fclose(lcd);
	return 0;
}

//This function creates a queue and semaphores
int initOSDataStructs(void)
{
	freqQueue = xQueueCreate(FREQ_QUEUE_SIZE, sizeof(double));
	shared_lcd_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_console_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_sevenseg_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_freq_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_state_toggle_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_stability_control_sem = xSemaphoreCreateCounting( 9999, 1 );
	return 0;
}

//This function creates the tasks used.
int initCreateTasks(void)
{
	xTaskCreate(print_lcd_task, "print_lcd_task", TASK_STACKSIZE, NULL, PRINT_LCD_TASK_PRIORITY, NULL);
	xTaskCreate(print_console_task, "print_console_task", TASK_STACKSIZE, NULL, PRINT_CONSOLE_TASK_PRIORITY, NULL);
	xTaskCreate(seven_seg_task, "seven_seg_task", TASK_STACKSIZE, NULL, SEVEN_SEG_TASK_PRIORITY, NULL);
	xTaskCreate(state_management_task, "state_management_task", TASK_STACKSIZE, NULL, STATE_MANAGEMENT_TASK_PRIORITY, &state_management_handle);
	xTaskCreate(stability_task, "stability_task", TASK_STACKSIZE, NULL, STABILITY_TASK_PRIORITY, NULL);
	return 0;
}


//Helper Functions

//This function finds the highest bit of a binary number. This is used to identify the highest priority
//load to revive.
//Reference: https://stackoverflow.com/questions/53161/find-the-highest-order-bit-in-c
int hob (int num)
{
    if (!num)
        return 0;

    int ret = 1;

    while (num >>= 1)
        ret <<= 1;

    return ret;
}

//This function is used to manipulate red and green leds based on current state.
void describeLEDS(int describeBehaviour) {
    unsigned int red_led_value = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
    unsigned int green_led_value = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);

	//connect highest priority
	if(describeBehaviour == 1) {
		//cond 1 : if switches == off, then no connection
        if (uiSwitchValue == 0){
            //no connection available - return
            return;
        }
		//cond 2 : if switches == red leds (edge case), no connection possible
        else if (uiSwitchValue == red_led_value){
            //all loads connected, no other connection possible
            return;
        }
		//cond 3 : if switches != red leds, find highest load that can be turned on
		//here you check if your switch is on, check if red led is off and compare with associated green led
        else{
            //returns the decimal value of the red LED you need to connect
			int ledAdded = hob(uiSwitchValue - red_led_value);
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red_led_value + ledAdded);

			//turns off green led if needed for given load
			if(((int)green_led_value - ledAdded) < 0) {
				//do nothing
			}
			else {
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green_led_value - ledAdded);
			}

            return;
        }
	}
	//disconnect lowest priority
	else if (describeBehaviour == 2) {
		//cond 1 : if switches == off, then no disconnection possible
        if (uiSwitchValue == 0){
            //there are no loads to shed
            return;
        }
		//cond 2 : if switches == no red led (i.e. already been shed), then no disconnection possible
        else if (red_led_value == 0){
            //all loads have already been shed - there are no more loads to shed
            return;
        }
		//cond 3 : if switches != red leds, find lowest load that can be turned off
		//here you check if switch is on, check if the red led is on
        else {
            int number = red_led_value;
            int ledMinus = (number & -number); //finding lowest bit that needs to be disconnected
            IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red_led_value - ledMinus);
            IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green_led_value + ledMinus);
            return;
        }
	}
	return;
}

//This function checks if all loads have been connected or not.
int checkFullConnection() {
	unsigned int red_led_value = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
	if(red_led_value == uiSwitchValue) {
		return 1;
	}
	else {
		return 0;
	}

	return 0;
}

//This function turns off associated red and green leds for manually turned off switches.
void manualSwitchIntervention() {
	unsigned int red_led_value = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
	unsigned int green_led_value = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);

	//finding which leds need to be kept switched on based on current switch status
	red_led_value &= uiSwitchValue;
	green_led_value &= uiSwitchValue;

	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red_led_value);
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green_led_value);
}
