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
#define SEVEN_SEG_TASK_PRIORITY 1  //seven seg related
#define PRINT_LCD_TASK_PRIORITY 2
#define PRINT_CONSOLE_TASK_PRIORITY 3

#define SWITCHES_TASK_PRIORITY 4

#define UPDATE_LOAD_TASK_PRIORITY 5
#define STABILITY_TASK_PRIORITY 6


//LCD Macros
FILE *lcd;
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// Definition of Message Queue
#define   MSG_QUEUE_SIZE  30
QueueHandle_t msgqueue;

#define FREQ_QUEUE_SIZE  10
QueueHandle_t freqQueue;


// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
SemaphoreHandle_t shared_lcd_sem;
SemaphoreHandle_t shared_console_sem;
SemaphoreHandle_t shared_sevenseg_sem;
SemaphoreHandle_t shared_freq_sem; //protects freqArray, rocArray

//Definition of Timer
TimerHandle_t systemTimer;

// globals variables

//threshold via KB
int thresholdFreq = 50;
char tempThreshold[4] = "";
int thresholdROC = 10;

//state toggle
int stateToggle = 0; //0 = maintenance mode, 1 = freq relay mode

//frequency related
double freqArray[FREQ_QUEUE_SIZE] = {40, 40, 40, 40, 40, 40, 40, 40, 40, 40};
double rocArray[FREQ_QUEUE_SIZE] = {40, 40, 40, 40, 40, 40, 40, 40, 40, 40};
int freqIndex = 0;
int stabilityControl = 0; //0= stable, 1 = unstable

//time related
int secondsCounter = 0;



//debug variables
unsigned int debugCounter =0;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);


void freq_relay(){
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0);
	double tempFreq = 16000/(double) temp;
	//printf("%d\n", debugCounter);
	//debugCounter++;
	//printf("%f Hz in being SENT \n", 16000/(double)temp);
	xQueueSendToFrontFromISR(freqQueue, &tempFreq, pdFALSE);
	return;
}


//for system time
void vSystemRunTimer(xTimerHandle t_timer){
	secondsCounter = secondsCounter + 1;
}


void ps2_isr (void* context, alt_u32 id)
{
  char ascii;
  int status = 0;
  unsigned char key = 0;
  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii) ;
  if ( status == 0 ) //success
  {
    // print out the result
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
    	int z = (int)(key); // Using typecasting

        if(z == 112){
        	strcat(tempThreshold, "0");
        	//printf ( "DEFAULT   : %s\n", tempThreshold);
        }
        else if(z == 105){
            strcat(tempThreshold, "1");
            //printf ( "DEFAULT   : %s\n", tempThreshold);
        }
        else if(z == 114){
        	strcat(tempThreshold, "2");
        	//printf ( "DEFAULT   : %s\n", tempThreshold);
        }
        else if(z == 122){
        	strcat(tempThreshold, "3");
        	//printf ( "DEFAULT   : %s\n", tempThreshold);
		}
		else if(z == 107){
			strcat(tempThreshold, "4");
			//printf ( "DEFAULT   : %s\n", tempThreshold);
		}
		else if(z == 115){
			strcat(tempThreshold, "5");
			//printf ( "DEFAULT   : %s\n", tempThreshold);
		}
		else if(z == 116){
			strcat(tempThreshold, "6");
			//printf ( "DEFAULT   : %s\n", tempThreshold);
		}
		else if(z == 108){
			strcat(tempThreshold, "7");
			//printf ( "DEFAULT   : %s\n", tempThreshold);
		}
		else if(z == 117){
			strcat(tempThreshold, "8");
			//printf ( "DEFAULT   : %s\n", tempThreshold);
		}
		else if(z == 125){
			strcat(tempThreshold, "9");
			//printf ( "DEFAULT   : %s\n", tempThreshold);
		}
		else if(z == 43){ //f
			thresholdFreq = atoi(tempThreshold);
			tempThreshold[0] = '\0';
			//printf("\nThe value of thresholdFreq : %d", thresholdFreq);
			//printf ("\nThe value of tempThreshold : %s\n", tempThreshold);
		}
		else if(z == 45) { //r
			thresholdROC = atoi(tempThreshold);
			tempThreshold[0] = '\0';
			//printf("\nThe value of thresholdROC : %d", thresholdROC);
		}
		else {
			//do nothing
			tempThreshold[0] = '\0'; //for any random keystroke, reset temp variable
		}

        break ;
    }
  }
}

void button_interrupts_function(void* context, alt_u32 id)
{
  // need to cast the context first before using it
  int* temp = (int*) context;
  (*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

  //for key3
  if(*temp==4) {
	  printf("State change button pressed \n \n");
	  stateToggle = !stateToggle;
  }

  // clears the edge capture register
  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

//need to enable writing to a queue for this
void switches_task(void *pvParameters)
{
	while (1)
	{
		unsigned int uiSwitchValue = 0;
		// read the value of the switch and store to uiSwitchValue
		uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		// write the value of the switches to the red LEDs
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue);

		vTaskDelay(100);
	}
}

//need to follow a pattern to calc RoC, threshold check and overwriting old values
void stability_task(void *pvParameters)
{
	while (1)
	{
		xQueueReceive(freqQueue, freqArray+freqIndex, portMAX_DELAY); // Pops queue, blocks indefinitely if empty

		xSemaphoreTake(shared_freq_sem, portMAX_DELAY);

		//printf("%f Hz in being RECEIVED\n", freqArray[freqIndex]);

		//find ROC
		if(freqIndex == 0){
			double oldF = freqArray[9];
			double newF = freqArray[0];
			rocArray[0] = fabs(((newF - oldF) * newF * oldF * 2.0) / (newF + oldF));
		} else {
			double oldF = freqArray[freqIndex-1];
			double newF = freqArray[freqIndex];
			rocArray[freqIndex] = fabs(((newF - oldF) * newF * oldF * 2.0) / (newF + oldF));
		}


		//check if stability conditions are violated
		if((freqArray[freqIndex] < thresholdFreq) || (rocArray[freqIndex] > thresholdROC)) {
			stabilityControl = 1;
		}
		else {
			stabilityControl = 0;
		}

		/*
		printf("%f Instantaneous Frequency\n", freqArray[freqIndex]);
		printf("%d Frequency Threshold\n", thresholdFreq);
		printf("%f ROC calculated\n", rocArray[freqIndex]);
		printf("%d ROC Threshold\n", thresholdROC);
		printf("%d Stability \n", stabilityControl);
		*/

		freqIndex = freqIndex+1;  //next element to be written to
		if(freqIndex >= FREQ_QUEUE_SIZE) {
			freqIndex = 0;
		}

		xSemaphoreGive(shared_freq_sem);

		//maybe some sort of ulTaskNotifiy for LoadManager might be needed
	}
}

//empty atm
void update_load_task(void *pvParameters)
{
	while (1)
	{
		//maybe some sort of ulTaskNotifiy for LoadManager might be needed
	}
}

//printing group name for now
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


// The following prints lcd status information every 0.5 seconds.
void print_lcd_task(void *pvParameters)
{
	//gets triggered on a Queue for RoC
	while (1)
	{
		xSemaphoreTake(shared_lcd_sem, portMAX_DELAY);

		// if the lcd is open successfully
		if(lcd != NULL)
		{
			// clearing the LCD
			fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
			fprintf(lcd, "Threshold = %d \n", thresholdFreq);
			fprintf(lcd, "RoC = %d \n", thresholdROC);
		}

		//insert some code to release the semophore
		xSemaphoreGive(shared_lcd_sem);

		vTaskDelay(500);
	}
}

// The following prints lcd status information every 0.5 seconds.
void print_console_task(void *pvParameters)
{
	//gets triggered on a Queue for RoC
	while (1)
	{
		xSemaphoreTake(shared_console_sem, portMAX_DELAY);

		xSemaphoreTake(shared_freq_sem, portMAX_DELAY);

		printf("Frequency = ");

		if(freqIndex >= 5) {
			for(int i=0; i<=4; i++) {
				printf("%f = %d ", freqArray[freqIndex-1-i], freqIndex-1-i);
			}
		}
		else {
			for(int i=0; i<=4; i++) {
				if((freqIndex-1-i) >= 0) {
					printf("%f = %d ", freqArray[freqIndex-1-i], freqIndex-1-i);
				}
				else {
					if((freqIndex-1-i) == -1) {
						printf("%f = %d ", freqArray[9], 9);
					}
					else if((freqIndex-1-i) == -2) {
						printf("%f = %d ", freqArray[8], 8);
					}
					else if((freqIndex-1-i) == -3) {
						printf("%f = %d ", freqArray[7], 7);
					}
					else if((freqIndex-1-i) == -4) {
						printf("%f = %d ", freqArray[6], 6);
					}
				}
			}
		}


		printf("\n");

		printf("Total time system has been active = %d seconds \n", secondsCounter);

		xSemaphoreGive(shared_freq_sem);

		xSemaphoreGive(shared_console_sem);

		vTaskDelay(1000);
	}
}


int main(int argc, char* argv[], char* envp[])
{
	//TIMER ISR for System
	systemTimer = xTimerCreate("System Timer", 1000, pdTRUE, NULL, vSystemRunTimer);
	if (xTimerStart(systemTimer, 0) != pdPASS){
		printf("Cannot start timer");
	}


	//FREQUENCY ISR
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);


	//KEYBOARD ISR
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

    if(ps2_device == NULL){
    	printf("can't find PS/2 device\n");
    	return 1;
    }

    alt_up_ps2_clear_fifo (ps2_device) ;

    alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);
    // register the PS/2 interrupt
    IOWR_8DIRECT(PS2_BASE,4,1);


	//BUTTON ISR
	int buttonValue = 0;
	// clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	// enable interrupts for all buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	// register the ISR
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, button_interrupts_function);


	//LCD
	lcd = fopen(CHARACTER_LCD_NAME, "w");


	initOSDataStructs();
	initCreateTasks();
	vTaskStartScheduler();

	while(1){

	}

	fclose(lcd);
	return 0;
}

// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	freqQueue = xQueueCreate(FREQ_QUEUE_SIZE, sizeof(double));
	msgqueue = xQueueCreate( MSG_QUEUE_SIZE, sizeof( void* ) );

	shared_lcd_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_console_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_sevenseg_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_freq_sem = xSemaphoreCreateCounting( 9999, 1 );
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(print_lcd_task, "print_lcd_task", TASK_STACKSIZE, NULL, PRINT_LCD_TASK_PRIORITY, NULL);
	xTaskCreate(print_console_task, "print_console_task", TASK_STACKSIZE, NULL, PRINT_CONSOLE_TASK_PRIORITY, NULL);

	xTaskCreate(seven_seg_task, "seven_seg_task", TASK_STACKSIZE, NULL, SEVEN_SEG_TASK_PRIORITY, NULL);

	xTaskCreate(switches_task, "switches_task", TASK_STACKSIZE, NULL, SWITCHES_TASK_PRIORITY, NULL);

	xTaskCreate(stability_task, "stability_task", TASK_STACKSIZE, NULL, STABILITY_TASK_PRIORITY, NULL);

	//xTaskCreate(update_load_task, "update_load_task", TASK_STACKSIZE, NULL, UPDATE_LOAD_TASK_PRIORITY, NULL);

	return 0;
}


//Helper Functions
// function to convert decimal to hexadecimal
int decToHexa(int n)
{
    // char array to store hexadecimal number
    char hexaDeciNum[100];
    char correctOrder[100];
    int tempPos = 0;

    // counter for hexadecimal number array
    int i = 0;
    while(n!=0)
    {
        // temporary variable to store remainder
        int temp  = 0;

        // storing remainder in temp variable.
        temp = n % 16;

        // check if temp < 10
        if(temp < 10)
        {
            hexaDeciNum[i] = temp + 48;
            i++;
        }
        else
        {
            hexaDeciNum[i] = temp + 55;
            i++;
        }

        n = n/16;
    }

    // printing hexadecimal number array in reverse order
    for(int j=i-1; j>=0; j--) {
        //printf("%c",hexaDeciNum[j]);
        //strcat(correctOrder,hexaDeciNum[j]);
    	correctOrder[tempPos] = hexaDeciNum[j];
    	tempPos++;
    }


    printf("%s \n", correctOrder);
    printf("%d", atoi(correctOrder));
    return(atoi(correctOrder));
}
