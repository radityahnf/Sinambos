/**
 * \file
 *
 * \brief Proyek UAS Sinambos
 *
 */

#include <asf.h>
#include <stdio.h>
#include <string.h> // Include this for strcpy
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/timers.h"
#include "FreeRTOS/include/semphr.h"

/* Define FreeRTOS tasks */
static portTASK_FUNCTION_PROTO(vUltrasonicSensor, p_);
static portTASK_FUNCTION_PROTO(vSoilSensor, q_);
static portTASK_FUNCTION_PROTO(vCounter, r_);
static portTASK_FUNCTION_PROTO(vPushButton1, s_);

/* Define semaphore */
SemaphoreHandle_t xSemaphore;
uint16_t counter = 0;

/* Define all functions */
static void adc_init_soil(void);
static uint16_t adc_read_soil(void);
void setup_timer(void);
void increment_distance(void);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

/* Global functions */
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	return ((uint32_t)(x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
}

/* Soil Moisture Sensor Variables */
#define MY_ADC    ADCA
#define MY_ADC_CH ADC_CH0 // Using Channel 0 for PA4 (ADC4)
static char soilMoistureResult[128];
#define RELAY_PIN IOPORT_CREATE_PIN(PORTC, 0) // Pin configuration for the relay (PC0)
const uint16_t soilMoistureThreshold = 3800;		// Soil moisture thresholds as a border between dry soil and damp

/* Functions for Soil Moisture Sensor */
// Function to initialize ADC for the capacitive soil moisture sensor
static void adc_init_soil(void) {
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	// Read current ADC configuration
	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);

	// 12-bit resolution conversion parameters
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);

	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL); // Clock rate for ADC

	// PA4 (ADC4) as the input channel
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN4, ADCCH_NEG_NONE, 1); // ADC4 is PA4 (J2_PIN0)
	
	adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
}

// Read value from the ADC
static uint16_t adc_read_soil(void) {
	uint16_t result;

	// Enable ADC
	adc_enable(&MY_ADC);
	
	// Start ADC conversion on the selected channel
	adc_start_conversion(&MY_ADC, MY_ADC_CH);

	// Wait for the conversion to complete using interrupt
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);

	// Get the result from the ADC
	result = adc_get_result(&MY_ADC, MY_ADC_CH);

	// Disable the ADC
	adc_disable(&MY_ADC);

	return result;
}


/* Ultrasonic Sensor Variables */
uint16_t score = 0;
uint16_t incremental = 0;
uint16_t distance = 0;
uint16_t distance_percentage = 0;		// Needed for UART

/* Functions for Soil Moisture Sensor */
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
	tc_set_overflow_interrupt_callback(&TCE1,increment_distance);
	tc_set_wgm(&TCE1, TC_WG_NORMAL);
	tc_write_period(&TCE1, 58);		// 29 microseconds
	tc_set_overflow_interrupt_level(&TCE1, TC_INT_LVL_HI);
	// tc_write_clock_source(&TCE1, TC_CLKSEL_DIV1_gc);
	tc_write_clock_source(&TCE1, TC_CLKSEL_OFF_gc); // Stop the timer
	//tc_disable(&TCE1);	// Stop the timer (fails bruh)
}

//Fungsi ini untuk increment nilai variabel "increment" setiap 29us
void increment_distance(void){
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




/* MAIN FUNCTION */
int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();
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

	// Setup
	setup_timer();
	adc_init_soil();
	
	/* Create the task */
	xTaskCreate(vUltrasonicSensor, "", 1000, NULL, tskIDLE_PRIORITY + 0, NULL);	// higher priority
	xTaskCreate(vSoilSensor, "", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);	// higher priority
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
			
			PORTB.DIRSET = PIN0_bm;    // Set PIN0 direction as output
			PORTB.OUTCLR = PIN0_bm;    // Set PIN0 low
			PORTB.OUTSET = PIN0_bm;    // Set PIN0 high
			delay_us(5);               // Keep it high for 5us
			PORTB.OUTCLR = PIN0_bm;    // Set PIN0 back to low
			PORTB.DIRCLR = PIN0_bm;    // Set PIN0 direction back to input			
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

static portTASK_FUNCTION(vSoilSensor, q_) {	
	char strbuf[128];
	uint16_t soilMoistureValue = 0;
	
	// Initialize relay pin as output and set it to inactive (HIGH since the relay is active low, IMPORTANT!!!)
	ioport_set_pin_dir(RELAY_PIN, IOPORT_DIR_OUTPUT);
	gpio_set_pin_high(RELAY_PIN); // Set the relay to inactive state (HIGH)
	
	while(1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			// Read soil moisture value from the ADC
			soilMoistureValue = adc_read_soil();

			// Display the soil moisture value on the LCD
			snprintf(strbuf, sizeof(strbuf), "Soil: %u", soilMoistureValue);
			gfx_mono_draw_string(strbuf, 0, 16, &sysfont);

			// Control the relay based on soil moisture levels
			if (soilMoistureValue > soilMoistureThreshold) {
				// Soil is dry, turn on the water pump (active low relay)
				strcpy(soilMoistureResult, "Kering");
				gpio_set_pin_low(RELAY_PIN); // Activate relay
				snprintf(strbuf, sizeof(strbuf), "Tanah %s, Pump ON", soilMoistureResult);
			} else if (soilMoistureValue <= soilMoistureThreshold) {
				// Soil is damp, turn off the water pump
				strcpy(soilMoistureResult, "Lembab");
				gpio_set_pin_high(RELAY_PIN); // Deactivate relay
				snprintf(strbuf, sizeof(strbuf), "Tanah %s, Pump OFF", soilMoistureResult);
			}
			
			gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
						
			xSemaphoreGive(xSemaphore);
		}
		vTaskDelay(100/portTICK_PERIOD_MS);
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
