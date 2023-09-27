#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* Pino ECHO */
#define ECHO_PIO PIOC
#define ECHO_PIO_ID ID_PIOC
#define ECHO_PIO_IDX 13
#define ECHO_PIO_IDX_MASK (1 << ECHO_PIO_IDX)

/* Pino TRIGGER */
#define TRIGGER_PIO PIOD
#define TRIGGER_PIO_ID ID_PIOD
#define TRIGGER_PIO_IDX 30
#define TRIGGER_PIO_IDX_MASK (1 << TRIGGER_PIO_IDX)



/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


/** prototypes */
void but_callback(void);
void echo_callback(void);
static void BUT_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void configure_console(void);


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}


/************************************************************************/
/* Semaphore                                                              */
/************************************************************************/

SemaphoreHandle_t xSemaphore = NULL;

/************************************************************************/
/* Queue                                                              */
/************************************************************************/

QueueHandle_t xQueue = NULL;

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
}

void echo_callback(void){
	if(pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK) == 1){
		RTT_init(8200, 8200, 0);
	}
	else{
		uint32_t ticks;
		ticks = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueue, &ticks, 0);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();

	float distance;
	uint32_t ticks;
	
	char str[32];

	for (;;)  {
		if (xSemaphoreTake(xSemaphore, ( TickType_t ) 500) == pdTRUE) {
			pio_set(TRIGGER_PIO, TRIGGER_PIO_IDX_MASK);
			delay_us(10);
			pio_clear(TRIGGER_PIO, TRIGGER_PIO_IDX_MASK);
			
			if (xQueueReceive(xQueue, &ticks, ( TickType_t ) 500) == pdTRUE) {
				distance = (((float) ticks) * 340.0) / (2.0 * 8200.0);
				distance = distance * 100;
				printf("Distancia: %f cm\n", distance);
				sprintf(str, "%6.3f", distance);
				gfx_mono_draw_string(str, 25, 12, &sysfont);
				gfx_mono_draw_string(" cm  ", 65, 12, &sysfont);
			}
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	pmc_enable_periph_clk(BUT_PIO_ID);
	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);

	
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_get_interrupt_status(BUT_PIO);

	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

}

static void ECHO_init(void) {

	pmc_enable_periph_clk(ECHO_PIO_ID);

	// configure callback
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK, PIO_DEFAULT);

	// debounce server para retirar o ruido do botão e evitar que o mesmo seja pressionado varias vezes
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_IDX_MASK, PIO_IT_EDGE, echo_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_IDX_MASK);

	pio_get_interrupt_status(ECHO_PIO);
	
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);
}

static void TRIGGER_init(void) {
	pmc_enable_periph_clk(TRIGGER_PIO_ID);
	pio_set_output(TRIGGER_PIO, TRIGGER_PIO_IDX_MASK, 0, 0, 0);
}

/**
* Configura RTT
*
* arg0 pllPreScale  : Frequência na qual o contador irá incrementar
* arg1 IrqNPulses   : Valor do alarme
* arg2 rttIRQSource : Pode ser uma
*     - 0:
*     - RTT_MR_RTTINCIEN: Interrupção por incremento (pllPreScale)
*     - RTT_MR_ALMIEN : Interrupção por alarme
*/

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}


/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	
	/* Create semaphore */
	xSemaphore = xSemaphoreCreateBinary();

	/* Create queue */
	xQueue = xQueueCreate(32, sizeof(uint32_t));

	if (xQueue == NULL) {
		printf("Failed to create queue\r\n");
	}

	if (xSemaphore == NULL) {
		printf("Failed to create semaphore\r\n");
	}

	/* Initialize the button */
	BUT_init();

	/* Initialize the echo */
	ECHO_init();

	/* Initialize the trigger */
	TRIGGER_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){
	}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
