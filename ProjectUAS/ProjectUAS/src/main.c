/**
 * \file
 *
 * \brief Proyek UAS Sinambos
 *
 */

#include <asf.h>
#include <stdio.h>
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/timers.h"
#include "FreeRTOS/include/semphr.h"

/* Define FreeRTOS tasks */
static portTASK_FUNCTION_PROTO(vUltrasonicSensor, p_);
static portTASK_FUNCTION_PROTO(vBlinkLed1, q_);
static portTASK_FUNCTION_PROTO(vCounter, r_);
static portTASK_FUNCTION_PROTO(vPushButton1, s_);

/* Define semaphore */
SemaphoreHandle_t xSemaphore;
uint16_t counter = 0;


/* Define functions */
void setup_timer(void);
void print_message(void);

/* Soil Moisture Sensor Variables */


/* Ultrasonic Sensor Variables */
uint16_t score = 0;
uint16_t incremental = 0;
uint16_t distance = 0;		// Needed for UART

/*

// OPTION 1: THIS IS THE TIMER USING FREERTOS TIMER

uint32_t previousMicros = 0;
uint32_t incrementInterval = 29; // 29 microseconds for each increment

// Timer handle
TimerHandle_t xIncrementTimer;

void vIncrementCallback(TimerHandle_t xTimer) {
	if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
		incremental = incremental + 1;  // Increment the value every 29 microseconds
		xSemaphoreGive(xSemaphore);
	}
}

// Function to setup the software timer
void setup_timer(void) {
	// Create a timer that will call the vIncrementCallback function every 29 microseconds
	xIncrementTimer = xTimerCreate("IncrementTimer", pdMS_TO_TICKS(0.029), pdTRUE, (void *)0, vIncrementCallback);

	if (xIncrementTimer == NULL) {
		// Timer creation failed
		gfx_mono_draw_string("FAILED", 0, 16, &sysfont);
	} else {
		// Start the timer with a period of 29 microseconds
		xTimerStart(xIncrementTimer, 0);
	}
}

*/

// OPTION 2: THIS IS THE TIMER USING TC LIBRARY FROM ATMEL STUDIO
//Fungsi setup timer
void setup_timer(void){
	tc_enable(&TCE1);
	tc_set_overflow_interrupt_callback(&TCE1,print_message);
	tc_set_wgm(&TCE1, TC_WG_NORMAL);
	tc_write_period(&TCE1, 58);		// 29 microseconds
	tc_set_overflow_interrupt_level(&TCE1, TC_INT_LVL_HI);
	// tc_write_clock_source(&TCE1, TC_CLKSEL_DIV1_gc);
	tc_write_clock_source(&TCE1, TC_CLKSEL_OFF_gc); // Stop the timer
	//tc_disable(&TCE1);	// Stop the timer (fails bruh)
}

//Fungsi ini bukan utk print message, tapi increment nilai variabel "increment" setiap 29us
void print_message(void){
	char strbuf[128];
	
	/**
	 * Semaphore di bawah ini dikomen alias salah!! karena JANGAN PERNAH AMBIL SEMAPHORE DI DALAM TASK YG LAGI PAKE SEMAPHORE
	 * KARENA GABAKAL BISA DIAMBIL KAN MASIH DIPAKE TASK YG MANGGIL CALLBACK FUNCTION INI (si task ultrasonic)
	 * ENDINGNYA MALAH STUCK NGELOOP 1000 TAHUN NUNGGUIN SEMAPHORENYA BISA DIAMBIL
	 */
	
	//if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {		
		incremental = incremental + 1;
		//xSemaphoreGive(xSemaphore);
	//}
}


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	// sysclk_init();
	board_init();
	pmic_init();
	gfx_mono_init();
	
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
	gfx_mono_draw_string("RTOS v10.2.1", 0, 0, &sysfont);
	
	// Workaround for known issue: Enable RTC32 sysclk
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	while (RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm) {
		// Wait for RTC32 sysclk to become stable
	}
	
	delay_ms(1000);

	setup_timer();		// used when using timer from freertos, because should run before vtaskstartscheduler
	/* Create the task */
	
	xTaskCreate(vUltrasonicSensor, "", 1000, NULL, tskIDLE_PRIORITY + 0, NULL);	// higher priority
	xTaskCreate(vBlinkLed1, "", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);	// higher priority
	xTaskCreate(vPushButton1, "", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);	// higher priority
	xTaskCreate(vCounter, "", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);			// low priority
	
	/* Semaphore */
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);
	
	/* Start the task */
	vTaskStartScheduler();
}


static portTASK_FUNCTION(vUltrasonicSensor, p_) {
	char strbuf[128];	
	//setup_timer();
	while(1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			//xTimerStart(xIncrementTimer, 0);		// Start the timer (doesnt work bruh)
			//tc_enable(&TCE1);		// Start the timer (doesnt work bruh)
			tc_write_clock_source(&TCE1, TC_CLKSEL_DIV1_gc);	// Start the timer
			
			PORTB.DIR = 0b11111111; //Set output
			PORTB.OUT = 0b00000000; //Set low
			PORTB.OUT = 0b11111111; //Set high selama 5us
			delay_us(5);
			PORTB.OUT = 0b00000000; //Kembali menjadi low
			PORTB.DIR = 0b00000000; //Set menjadi input
			delay_us(400); //Delay holdoff selama 750us
			int oldinc = incremental;
			delay_us(115); //Delay lagi, kali ini seharusnya pin menjadi high
			
			// taskENTER_CRITICAL();	// freertos substitute for cpu_irq_enable (fails)
			cpu_irq_enable(); //Mulai interrupt
			
			while(PORTB.IN & PIN0_bm){
				//Tidak ada apa-apa di sini. Loop ini berfungsi untuk mendeteksi pin 0 PORT B yang berubah menjadi low
			}
			int newinc = incremental; //Catat selisih waktu antara suara dikirim hingga diterima
			
			// xTimerStop(xIncrementTimer, 0);		// Stop the timer  (doesnt work bruh)
			// tc_disable(&TCE1);		// Stop the timer  (doesnt work bruh)
			tc_write_clock_source(&TCE1, TC_CLKSEL_OFF_gc); // Stop the timer

			//taskEXIT_CRITICAL();		// freertos substitute for cpu_irq_disable (fails)
			cpu_irq_disable(); //Interrupt dimatikan
			
			if (incremental > 300){ //Jika hasil lebih dari 300 cm, dibulatkan menjadi 300 cm
				distance = 300;
				snprintf(strbuf, sizeof(strbuf), "Panjang: %d cm   ", distance);
				gfx_mono_draw_string(strbuf, 0, 0, &sysfont);
				incremental = 0;
			} else {
				int inc = newinc - oldinc;
				distance = inc/2; //Dibagi 2 seperti rumus sonar
				snprintf(strbuf, sizeof(strbuf), "Panjang: %d cm   ", distance);
				gfx_mono_draw_string(strbuf, 0, 0, &sysfont);
				incremental = 0; //reset nilai variable incremental
			}
			xSemaphoreGive(xSemaphore);
		}
		vTaskDelay(150 / portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vBlinkLed1, q_) {	
	char strbuf[128];
	int flagLed1 = 0;
	
	while(1) {
		flagLed1 = !flagLed1;
		ioport_set_pin_level(LED1_GPIO, flagLed1);
		snprintf(strbuf, sizeof(strbuf), "LED 1 : %d", !flagLed1);
		gfx_mono_draw_string(strbuf,0, 24, &sysfont);
		vTaskDelay(375/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vPushButton1, s_) {
	char strbuf[128];
	
	while(1) {
		
		if(ioport_get_pin_level(GPIO_PUSH_BUTTON_1)==0){
			if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
				counter++;
				snprintf(strbuf, sizeof(strbuf), "Counter : %d", counter);
				gfx_mono_draw_string(strbuf,0, 8, &sysfont);
				xSemaphoreGive(xSemaphore);
			}
		}
		
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vCounter, r_) {
	char strbuf[128];
	
	while(1) {
		
		if(xSemaphoreTake(xSemaphore, (TickType_t) 10) == pdTRUE) {
			counter++;
			snprintf(strbuf, sizeof(strbuf), "Counter : %d", counter);
			gfx_mono_draw_string(strbuf,0, 8, &sysfont);
			xSemaphoreGive(xSemaphore);	
		}
		
		vTaskDelay(100/portTICK_PERIOD_MS);
	}	
}