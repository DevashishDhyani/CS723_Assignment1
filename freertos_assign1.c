// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>

#include "system.h"                     // to use the symbolic names
#include "altera_avalon_pio_regs.h" 	// to use PIO functions
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
#define PRINT_STATUS_TASK_PRIORITY 14
#define GETSEM_TASK1_PRIORITY      13
#define GETSEM_TASK2_PRIORITY      12
#define RECEIVE_TASK1_PRIORITY    11
#define RECEIVE_TASK2_PRIORITY    10
#define SEND_TASK_PRIORITY        9

#define COUNTER_TASK_PRIORITY 6
#define LCD_TASK1_PRIORITY  17
#define LCD_TASK2_PRIORITY  16

#define SWITCHES_TASK_PRIORITY 1

//LCD Macros
FILE *lcd;
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// Definition of Message Queue
#define   MSG_QUEUE_SIZE  30
QueueHandle_t msgqueue;

// used to delete a task
TaskHandle_t xHandle;

// Definition of Semaphore
SemaphoreHandle_t shared_resource_sem;
SemaphoreHandle_t shared_lcd_sem;

//Definition of Timer
TimerHandle_t timer_switches;


// globals variables
unsigned int number_of_messages_sent = 0;
unsigned int number_of_messages_received_task1 = 0;
unsigned int number_of_messages_received_task2 = 0;
unsigned int getsem_task1_got_sem = 0;
unsigned int getsem_task2_got_sem = 0;
char sem_owner_task_name[20];

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);


// first we write our interrupt function
void button_interrupts_function(void* context, alt_u32 id)
{
  // need to cast the context first before using it
  int* temp = (int*) context;
  (*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

  //for key3
  if(*temp==4) {
	  printf("State change button pressed \n \n");
  }

  // clears the edge capture register
  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

void vTimerSwitches(xTimerHandle t_timer){ //Timer flashes green LEDs
	//IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0xFF^IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE));

	unsigned int uiSwitchValue = 0;
    // read the value of the switch and store to uiSwitchValue
    uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
    // write the value of the switches to the red LEDs
    IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue);
}

void switches_task(void *pvParameters)
{
	//TIMER Switches
	timer_switches = xTimerCreate("Timer Switches", 1000, pdTRUE, NULL, vTimerSwitches);

	if (xTimerStart(timer_switches, 0) != pdPASS){
		printf("Cannot start timer");
	}

	while (1)
	{
		//vTaskDelay(100);
	}
}

// For exercise in lab
void counter_task(void *pvParameters)
{
	int counter = 0;
	while (1)
	{
		counter++;
		IOWR_ALTERA_AVALON_PIO_DATA(SEVEN_SEG_BASE, counter);
		vTaskDelay(1000);
	}
}

void lcd_task1(void *pvParameters)
{
	while (1)
	{
		//insert some code here to request semophore
		xSemaphoreTake(shared_lcd_sem, portMAX_DELAY);

		// if the lcd is open successfully
		if(lcd != NULL)
		{
			// clearing the LCD
			fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
			fprintf(lcd, "LCD task 1\n");
		}

		//insert some code to release the semophore
		xSemaphoreGive(shared_lcd_sem);

		vTaskDelay(5000);
	}
}

void lcd_task2(void *pvParameters)
{
	while (1)
	{
		//insert some code here to request semophore
		xSemaphoreTake(shared_lcd_sem, portMAX_DELAY);

		// if the lcd is open successfully
		if(lcd != NULL)
		{
			// clearing the LCD
			fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
			fprintf(lcd, "LCD task 2\n");
		}

		//insert some code to release the semophore
		xSemaphoreGive(shared_lcd_sem);

		vTaskDelay(3000);
	}
}

// The following test prints out status information every 3 seconds.
void print_status_task(void *pvParameters)
{
	while (1)
	{
		vTaskDelay(3000);
		/*
		printf("****************************************************************\n");
		printf("Hello From FreeRTOS Running on NIOS II.  Here is the status:\n");
		printf("\n");
		printf("The number of messages sent by the send_task:         %d\n", number_of_messages_sent);
		printf("\n");
		printf("The number of messages received by the receive_task1: %d\n", number_of_messages_received_task1);
		printf("\n");
		printf("The number of messages received by the receive_task2: %d\n", number_of_messages_received_task2);
		printf("\n");
		printf("The shared resource is owned by: %s\n", &sem_owner_task_name[0]);
		printf("\n");
		printf("The Number of times getsem_task1 acquired the semaphore %d\n", getsem_task1_got_sem);
		printf("\n");
		printf("The Number of times getsem_task2 acquired the semaphore %d\n", getsem_task2_got_sem);
		printf("\n");
		printf("****************************************************************\n");
		printf("\n");
		*/
		printf("****************************************************************\n");
	}
}

// The next two task compete for a shared resource via a semaphore.  The name of
// the task that owns the semaphore is copied into the global variable
// sem_owner_task_name[].

void getsem_task1(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(shared_resource_sem, portMAX_DELAY);
		// block forever until receive the mutex
		strcpy(&sem_owner_task_name[0], "getsem_task1");
		getsem_task1_got_sem++;
		xSemaphoreGive(shared_resource_sem);
		vTaskDelay(100);
	}
}

void getsem_task2(void *pvParameters)
{
	while (1)
	{
		xSemaphoreTake(shared_resource_sem, portMAX_DELAY);
		// block forever until receive the mutex
		strcpy(&sem_owner_task_name[0], "getsem_task2");
		getsem_task2_got_sem++;
		xSemaphoreGive(shared_resource_sem);
		vTaskDelay(130);
	}
}

// The following task fills up a message queue with incrementing data.  The data
// is not actually used by the application.  If the queue is full the task is
// suspended for 1 second.
void send_task(void *pvParameters)
{
	unsigned int msg = 0;
	while (1)
	{
		if (xQueueSend(msgqueue, (void *)&msg, 0) == pdPASS)
		{
			// in the message queue
			msg++;
			number_of_messages_sent++;
		}
		else
		{
			vTaskDelay(1000);
		}
	}
}

// The next two task pull messages in the queue at different rates.  The number
// of messages received by the task is incremented when a new message is received
void receive_task1(void *pvParameters)
{
	unsigned int *msg;
	while (1)
	{
		xQueueReceive(msgqueue, &msg, portMAX_DELAY);
		number_of_messages_received_task1++;
		vTaskDelay(333);
	}
}

void receive_task2(void *pvParameters)
{
	unsigned int *msg;
	while (1)
	{
		xQueueReceive(msgqueue, &msg, portMAX_DELAY);
		number_of_messages_received_task2++;
		vTaskDelay(1000);
	}
}


int main(int argc, char* argv[], char* envp[])
{
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
	msgqueue = xQueueCreate( MSG_QUEUE_SIZE, sizeof( void* ) );
	shared_resource_sem = xSemaphoreCreateCounting( 9999, 1 );
	shared_lcd_sem = xSemaphoreCreateCounting( 9999, 1 );
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(getsem_task1, "getsem_task1", TASK_STACKSIZE, NULL, GETSEM_TASK1_PRIORITY, NULL);
	xTaskCreate(getsem_task2, "getsem_task2", TASK_STACKSIZE, NULL, GETSEM_TASK2_PRIORITY, NULL);
	xTaskCreate(receive_task1, "receive_task1", TASK_STACKSIZE, NULL, RECEIVE_TASK1_PRIORITY, NULL);
	xTaskCreate(receive_task2, "receive_task2", TASK_STACKSIZE, NULL, RECEIVE_TASK2_PRIORITY, NULL);
	xTaskCreate(send_task, "send_task", TASK_STACKSIZE, NULL, SEND_TASK_PRIORITY, NULL);
	xTaskCreate(print_status_task, "print_status_task", TASK_STACKSIZE, NULL, PRINT_STATUS_TASK_PRIORITY, NULL);
	xTaskCreate(counter_task, "counter_task", TASK_STACKSIZE, NULL, COUNTER_TASK_PRIORITY, NULL);

	xTaskCreate(lcd_task1, "lcd_task1", TASK_STACKSIZE, NULL, LCD_TASK1_PRIORITY, NULL);
	xTaskCreate(lcd_task2, "lcd_task2", TASK_STACKSIZE, NULL, LCD_TASK2_PRIORITY, NULL);

	xTaskCreate(switches_task, "switches_task", TASK_STACKSIZE, NULL, SWITCHES_TASK_PRIORITY, NULL);

	return 0;
}


