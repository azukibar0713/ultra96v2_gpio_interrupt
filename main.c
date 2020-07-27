/*
    Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
    Copyright (C) 2012 - 2018 Xilinx, Inc. All Rights Reserved.

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in
    the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software. If you wish to use our Amazon
    FreeRTOS name, please do so in a fair use way that does not cause confusion.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
    FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
    COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    http://www.FreeRTOS.org
    http://aws.amazon.com/freertos


    1 tab == 4 spaces!
*/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"

#include "pmon.h"

#include "xgpiops.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xplatform_info.h"


#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9
/*-----------------------------------------------------------*/

/* The Tx and Rx tasks as described at the top of this file. */
static void prvTxTask( void *pvParameters );
static void prvRxTask( void *pvParameters );
static void vTimerCallback( TimerHandle_t pxTimer );
/*-----------------------------------------------------------*/

/* The queue used by the Tx and Rx tasks, as described at the top of this
file. */
static TaskHandle_t xTxTask;
static TaskHandle_t xRxTask;
static QueueHandle_t xQueue = NULL;
static TimerHandle_t xTimer = NULL;
char HWstring[15] = "Hello World";
long RxtaskCntr = 0;

#define PRI_NUM (2)
#define TASK_NUM_BY_PRI (1)
static TaskHandle_t tskhdl[PRI_NUM][TASK_NUM_BY_PRI];



void GetTaskStatus( const TaskHandle_t xHandle, TaskStatus_t* xTaskDetails )
{
    //TaskHandle_t xHandle;
    //TaskStatus_t xTaskDetails;

    /* Obtain the handle of a task from its name. */
    //xHandle = xTaskGetHandle( "Task_Name" );

    /* Check the handle is not NULL. */
    configASSERT( xHandle );

    /* Use the handle to obtain further information about the task. */
    vTaskGetTaskInfo( /* The handle of the task being queried. */
                      xHandle,
                      /* The TaskStatus_t structure to complete with information on xHandle. */
                      xTaskDetails,
                      /* Include the stack high water mark value in the TaskStatus_t
                      structure. */
                      pdTRUE,
                      /* Include the task state in the TaskStatus_t structure. */
                      eInvalid );
}
const char* GetTaskName( const TaskHandle_t xHandle)
{
	TaskStatus_t xTaskDetails;
	GetTaskStatus(xHandle, &xTaskDetails);
	return xTaskDetails.pcTaskName;
}

static int button_push = 0;
static unsigned long start, end;
static unsigned long task_steady, int_start, int_end, task_start, task_start;
static TaskHandle_t TaskHdlPri00;
static TaskHandle_t TaskHdlPri01;
static void TaskPri00( void *pvParameters ) {
	while(1) {
		//xil_printf("%s\r\n", GetTaskName(xTaskGetCurrentTaskHandle()));
		//vTaskDelay(pdMS_TO_TICKS(1000));

		//vTaskResume(TaskHdlPri01);

	}
}
static void TaskPri01( void *pvParameters ) {
	xil_printf("%s\r\n", GetTaskName(xTaskGetCurrentTaskHandle()));
	start = pmon_start_cycle_counter();
	int_start = 0;
	int_end = 0;
	while(1) {
		xil_printf("%s\r\n", GetTaskName(xTaskGetCurrentTaskHandle()));
		vTaskDelay(pdMS_TO_TICKS(1000));

		//vTaskSuspend(NULL);
		task_steady = pmon_read_cycle_counter();
		//end = pmon_read_cycle_counter();
		//xil_printf("time = %ld\r\n", end - start);
		//if (int_start != 0 && int_end != 0) {
		if(button_push) {
			xil_printf("%ld %ld\r\n", int_start - task_steady, int_end - int_start);
			int_start = 0;
			int_end = 0;
		}
	}
}

// https://github.com/Xilinx/embeddedsw/blob/master/XilinxProcessorIPLib/drivers/gpiops/examples/xgpiops_intr_example.c
/************************** Constant Definitions *****************************/

/*
 * The following constants map to the names of the hardware instances that
 * were created in the EDK XPS system.  They are defined here such that
 * the user can easily change all the needed device IDs in one place.
 */
#define GPIO_DEVICE_ID		XPAR_XGPIOPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define GPIO_INTERRUPT_ID	XPAR_XGPIOPS_0_INTR

/* The following constants define the GPIO banks that are used. */
#define GPIO_BANK	XGPIOPS_BANK0  /* Bank 0 of the GPIO Device */

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

static int GpioIntrExample(XScuGic *Intc, XGpioPs *Gpio, u16 DeviceId,
			   u16 GpioIntrId);
static void IntrHandler(void *CallBackRef, u32 Bank, u32 Status);
static int SetupInterruptSystem(XScuGic *Intc, XGpioPs *Gpio, u16 GpioIntrId);

/************************** Variable Definitions *****************************/

/*
 * The following are declared globally so they are zeroed and so they are
 * easily accessible from a debugger.
 */
static XGpioPs Gpio; /* The Instance of the GPIO Driver */

static XScuGic Intc; /* The Instance of the Interrupt Controller Driver */

static u32 AllButtonsPressed; /* Intr status of the bank */
static u32 Input_Pin; /* Switch button */
static u32 Output_Pin; /* LED button */

int main( void )
{
	xil_printf("main\r\n");


#if 1
	//xTaskCreate(TaskPri00,"TaskPri00", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &TaskHdlPri00 );
	xTaskCreate(TaskPri01,"TaskPri01", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &TaskHdlPri01 );



	/*
	 * Run the GPIO interrupt example, specify the parameters that
	 * are generated in xparameters.h.
	 */
	int Status = GpioIntrExample(&Intc, &Gpio, GPIO_DEVICE_ID,
				 GPIO_INTERRUPT_ID);

	if (Status != XST_SUCCESS) {
		xil_printf("GPIO Interrupt Example Test Failed\r\n");
		return XST_FAILURE;
	}



	xil_printf( "Hello from Freertos example main\r\n" );



	/* Start the tasks and timer running. */
	vTaskStartScheduler();
	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );

#else
	const TickType_t x10seconds = pdMS_TO_TICKS( DELAY_10_SECONDS );
	/* Create the two tasks.  The Tx task is given a lower priority than the
	Rx task, so the Rx task will leave the Blocked state and pre-empt the Tx
	task as soon as the Tx task places an item in the queue. */
	xTaskCreate( 	prvTxTask, 					/* The function that implements the task. */
					( const char * ) "Tx", 		/* Text name for the task, provided to assist debugging only. */
					configMINIMAL_STACK_SIZE, 	/* The stack allocated to the task. */
					NULL, 						/* The task parameter is not used, so set to NULL. */
					tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
					&xTxTask );

	xTaskCreate( prvRxTask,
				 ( const char * ) "GB",
				 configMINIMAL_STACK_SIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &xRxTask );

	/* Create the queue used by the tasks.  The Rx task has a higher priority
	than the Tx task, so will preempt the Tx task and remove values from the
	queue as soon as the Tx task writes to the queue - therefore the queue can
	never have more than one item in it. */
	xQueue = xQueueCreate( 	1,						/* There is only one space in the queue. */
							sizeof( HWstring ) );	/* Each space in the queue is large enough to hold a uint32_t. */

	/* Check the queue was created. */
	configASSERT( xQueue );

	/* Create a timer with a timer expiry of 10 seconds. The timer would expire
	 after 10 seconds and the timer call back would get called. In the timer call back
	 checks are done to ensure that the tasks have been running properly till then.
	 The tasks are deleted in the timer call back and a message is printed to convey that
	 the example has run successfully.
	 The timer expiry is set to 10 seconds and the timer set to not auto reload. */
	xTimer = xTimerCreate( (const char *) "Timer",
							x10seconds,
							pdFALSE,
							(void *) TIMER_ID,
							vTimerCallback);
	/* Check the timer was created. */
	configASSERT( xTimer );

	/* start the timer with a block time of 0 ticks. This means as soon
	   as the schedule starts the timer will start running and will expire after
	   10 seconds */
	xTimerStart( xTimer, 0 );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );
#endif
}


/*-----------------------------------------------------------*/
static void prvTxTask( void *pvParameters )
{
const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );

	for( ;; )
	{
		/* Delay for 1 second. */
		vTaskDelay( x1second );

		/* Send the next value on the queue.  The queue should always be
		empty at this point so a block time of 0 is used. */
		xQueueSend( xQueue,			/* The queue being written to. */
					HWstring, /* The address of the data being sent. */
					0UL );			/* The block time. */
	}
}

/*-----------------------------------------------------------*/
static void prvRxTask( void *pvParameters )
{
char Recdstring[15] = "";

	for( ;; )
	{
		/* Block to wait for data arriving on the queue. */
		xQueueReceive( 	xQueue,				/* The queue being read. */
						Recdstring,	/* Data is read into this address. */
						portMAX_DELAY );	/* Wait without a timeout for data. */

		/* Print the received data. */
		xil_printf( "Rx task received string from Tx task: %s\r\n", Recdstring );
		RxtaskCntr++;
	}
}

/*-----------------------------------------------------------*/
static void vTimerCallback( TimerHandle_t pxTimer )
{
	long lTimerId;
	configASSERT( pxTimer );

	lTimerId = ( long ) pvTimerGetTimerID( pxTimer );

	if (lTimerId != TIMER_ID) {
		xil_printf("FreeRTOS Hello World Example FAILED");
	}

	/* If the RxtaskCntr is updated every time the Rx task is called. The
	 Rx task is called every time the Tx task sends a message. The Tx task
	 sends a message every 1 second.
	 The timer expires after 10 seconds. We expect the RxtaskCntr to at least
	 have a value of 9 (TIMER_CHECK_THRESHOLD) when the timer expires. */
	if (RxtaskCntr >= TIMER_CHECK_THRESHOLD) {
		xil_printf("FreeRTOS Hello World Example PASSED");
	} else {
		xil_printf("FreeRTOS Hello World Example FAILED");
	}

	vTaskDelete( xRxTask );
	vTaskDelete( xTxTask );
}






/****************************************************************************/
/**
* This function shows the usage of interrupt fucntionality of the GPIO device.
* It is responsible for initializing the GPIO device, setting up interrupts and
* providing a foreground loop such that interrupts can occur in the background.
*
* @param	Intc is a pointer to the XScuGic driver Instance.
* @param	Gpio is a pointer to the XGpioPs driver Instance.
* @param	DeviceId is the XPAR_<Gpio_Instance>_PS_DEVICE_ID value
*		from xparameters.h.
* @param	GpioIntrId is XPAR_<GIC>_<GPIO_Instance>_VEC_ID value
*		from xparameters.h
*
* @return
*		- XST_SUCCESS if the example has completed successfully.
*		- XST_FAILURE if the example has failed.
*
* @note		None
*
*****************************************************************************/
int GpioIntrExample(XScuGic *Intc, XGpioPs *Gpio, u16 DeviceId, u16 GpioIntrId)
{
	XGpioPs_Config *ConfigPtr;
	int Status;
	int Type_of_board;

	/* Initialize the Gpio driver. */
	ConfigPtr = XGpioPs_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}
	Type_of_board = XGetPlatform_Info();
	switch (Type_of_board) {
		case XPLAT_ZYNQ_ULTRA_MP:
			Input_Pin = 23;
			Output_Pin = 20;
			break;

		case XPLAT_ZYNQ:
			Input_Pin = 14;
			Output_Pin = 10;
			break;
	}
	XGpioPs_CfgInitialize(Gpio, ConfigPtr, ConfigPtr->BaseAddr);

	/* Run a self-test on the GPIO device. */
	Status = XGpioPs_SelfTest(Gpio);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Set the direction for the specified pin to be input */
	XGpioPs_SetDirectionPin(Gpio, Input_Pin, 0x0);

	/* Set the direction for the specified pin to be output. */
	XGpioPs_SetDirectionPin(Gpio, Output_Pin, 1);
	XGpioPs_SetOutputEnablePin(Gpio, Output_Pin, 1);
	XGpioPs_WritePin(Gpio, Output_Pin, 0x0);

	/*
	 * Setup the interrupts such that interrupt processing can occur. If
	 * an error occurs then exit.
	 */
	Status = SetupInterruptSystem(Intc, Gpio, GPIO_INTERRUPT_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	xil_printf("\n\rPush Switch button to exit\n\r");
	AllButtonsPressed = FALSE;

	/*
	 * Loop forever while the button changes are handled by the interrupt
	 * level processing.
	 */
	//while(AllButtonsPressed == FALSE);

	return XST_SUCCESS;
}

/****************************************************************************/
/**
* This function is the user layer callback function for the bank 0 interrupts of
* the GPIO device. It checks if all the switches have been pressed to stop the
* interrupt processing and exit from the example.
*
* @param	CallBackRef is a pointer to the upper layer callback reference.
* @param	Status is the Interrupt status of the GPIO bank.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void IntrHandler(void *CallBackRef, u32 Bank, u32 Status)
{
	//int_start = pmon_read_cycle_counter();

	XGpioPs *Gpio = (XGpioPs *)CallBackRef;
	u32 DataRead;

	xil_printf("InitrHandler\n\r");
	button_push = 1;
#if 0
	/* Push the switch button */
	DataRead = XGpioPs_ReadPin(Gpio, Input_Pin);
	if (DataRead != 0) {
		XGpioPs_SetDirectionPin(Gpio, Output_Pin, 1);
		XGpioPs_SetOutputEnablePin(Gpio, Output_Pin, 1);
		XGpioPs_WritePin(Gpio, Output_Pin, DataRead);
		AllButtonsPressed = TRUE;
	}
#endif
	//int_end= pmon_read_cycle_counter();
}


/*****************************************************************************/
/**
*
* This function sets up the interrupt system for the example. It enables falling
* edge interrupts for all the pins of bank 0 in the GPIO device.
*
* @param	GicInstancePtr is a pointer to the XScuGic driver Instance.
* @param	GpioInstancePtr contains a pointer to the instance of the GPIO
*		component which is going to be connected to the interrupt
*		controller.
* @param	GpioIntrId is the interrupt Id and is typically
*		XPAR_<GICPS>_<GPIOPS_instance>_VEC_ID value from
*		xparameters.h.
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None.
*
****************************************************************************/
static int SetupInterruptSystem(XScuGic *GicInstancePtr, XGpioPs *Gpio,
				u16 GpioIntrId)
{
	int Status;

	XScuGic_Config *IntcConfig; /* Instance of the interrupt controller */

	Xil_ExceptionInit();

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(GicInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler)XScuGic_InterruptHandler,
				GicInstancePtr);

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(GicInstancePtr, GpioIntrId,
				(Xil_ExceptionHandler)XGpioPs_IntrHandler,
				(void *)Gpio);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/****************************************************************************/
	/**
	*
	* This function is used for setting the Interrupt Type, Interrupt Polarity and
	* Interrupt On Any for the specified GPIO Bank pins.
	*
	* @param	InstancePtr is a pointer to an XGpioPs instance.
	* @param	Bank is the bank number of the GPIO to operate on.
	*		Valid values are 0-3 in Zynq and 0-5 in Zynq Ultrascale+ MP.
	* @param	IntrType is the 32 bit mask of the interrupt type.
	*		0 means Level Sensitive and 1 means Edge Sensitive.
	* @param	IntrPolarity is the 32 bit mask of the interrupt polarity.
	*		0 means Active Low or Falling Edge and 1 means Active High or
	*		Rising Edge.
	* @param	IntrOnAny is the 32 bit mask of the interrupt trigger for
	*		edge triggered interrupts. 0 means trigger on single edge using
	*		the configured interrupt polarity and 1 means  trigger on both
	*		edges.
	*
	* @return	None.
	*
	* @note		This function is used for setting the interrupt related
	*		properties of all the pins in the specified bank. The previous
	*		state of the pins is not maintained.
	*		To change the Interrupt properties of a single GPIO pin, use the
	*		function XGpioPs_SetPinIntrType().
	*
	*****************************************************************************/
	/* Enable falling edge interrupts for all the pins in bank 0. */
	//XGpioPs_SetIntrType(Gpio, GPIO_BANK, 0x00, 0xFFFFFFFF, 0x00);
	//XGpioPs_SetIntrType(Gpio, GPIO_BANK, 1, 0, 0x00);  // これだとSW1を押している時だけ割り込みが入った.
	//XGpioPs_SetIntrType(Gpio, GPIO_BANK, 1, 1, 0x00);  // これでも↑と同じ. Falling Edge -> Rising　Edgeにかえたはずだが。そもそもedgeでなくlevelになってる.
	XGpioPs_SetIntrType(Gpio, GPIO_BANK, 0xFFFFFFFF, 0, 0x00);  // bitごとのマスクだった。これで押したときだけ割り込みはい９った！.

	/* Set the handler for gpio interrupts. */
	XGpioPs_SetCallbackHandler(Gpio, (void *)Gpio, IntrHandler);


	/* Enable the GPIO interrupts of Bank 0. */
	XGpioPs_IntrEnable(Gpio, GPIO_BANK, (1 << Input_Pin));


	/* Enable the interrupt for the GPIO device. */
	XScuGic_Enable(GicInstancePtr, GpioIntrId);


	/* Enable interrupts in the Processor. */
	Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);

	return XST_SUCCESS;
}

