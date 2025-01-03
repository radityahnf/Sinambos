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
static portTASK_FUNCTION_PROTO(vPhotosensitiveSensor, r_);
static portTASK_FUNCTION_PROTO(vSendUART, s_);
static portTASK_FUNCTION_PROTO(vReceiveUART, t_);

/* Define semaphore */
SemaphoreHandle_t xSemaphore;
uint16_t counter = 0;

/* Define all functions */
static void adc_init_soil(void);
static uint16_t adc_read_soil(void);
void setup_timer(void);
void increment_distance(void);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
void setUpSerial(void);
void setup_photosensitive_sensor(void);

/* Global functions */
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	return ((uint32_t)(x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
}


/* UART Variables */
#define USART_SERIAL_EXAMPLE             &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE    9600
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            false
static char reads[200];

/*Functions for UART */
void setUpSerial(void)
{
	// Baud rate selection
	// BSEL = (2000000 / (2^0 * 16*9600) -1 = 12.0208... ~ 12 -> BSCALE = 0
	// FBAUD = ( (2000000)/(2^0*16(12+1)) = 9615.384 -> mendekati lah ya
	
	USARTC0_BAUDCTRLB = 0; //memastikan BSCALE = 0
	USARTC0_BAUDCTRLA = 0x0C; // 12
	
	//USARTC0_BAUDCTRLB = 0; //Just to be sure that BSCALE is 0
	//USARTC0_BAUDCTRLA = 0xCF; // 207
	
	//Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	//8 data bits, no parity and 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	
	//Enable receive and transmit
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}


/* Photosensitive Sensor Variables */
static uint16_t lightResult = 0;
static uint16_t lightUART = 1;
#define LIGHT_PIN IOPORT_CREATE_PIN(PORTC, 1) // Pin configuration for the light (PC1)


/* Functions for Photosensitive Sensor*/
// Setup PB3 as input
void setup_photosensitive_sensor(void) {
	PORTB.DIRCLR = PIN3_bm; // Set PB0 (connected to D0 pin of the sensor) as input
}


/* Soil Moisture Sensor Variables */
#define MY_ADC    ADCA
#define MY_ADC_CH ADC_CH0 // Using Channel 0 for PA4 (ADC4)
static uint16_t soilMoistureResult = 0;
static uint16_t pumpStatus = 0;
static uint16_t pumpUART = 1;
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
uint16_t distancePercentage = 0;		// Needed for UART

//Fungsi setup timer
void setup_timer(void){
	tc_enable(&TCE1);
	tc_set_overflow_interrupt_callback(&TCE1,increment_distance);
	tc_set_wgm(&TCE1, TC_WG_NORMAL);
	tc_write_period(&TCE1, 58);		// 29 microseconds
	tc_set_overflow_interrupt_level(&TCE1, TC_INT_LVL_HI);
	tc_write_clock_source(&TCE1, TC_CLKSEL_OFF_gc); // Stop the timer
}

//Fungsi ini untuk increment nilai variabel "increment" setiap 29us
void increment_distance(void){
	incremental = incremental + 1;
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
	gfx_mono_draw_string("Proyek UAS Sinambos", 0, 0, &sysfont);
	
	// Workaround for known issue: Enable RTC32 sysclk
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	while (RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm) {
		// Wait for RTC32 sysclk to become stable
	}
	
	delay_ms(1000);

	// Setup
	setUpSerial();
	setup_timer();
	adc_init_soil();
	setup_photosensitive_sensor();
	
	/* Create the task */
	xTaskCreate(vUltrasonicSensor, "", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vSoilSensor, "", 1000, NULL, tskIDLE_PRIORITY + 5, NULL);
	xTaskCreate(vPhotosensitiveSensor, "", 1000, NULL, tskIDLE_PRIORITY + 4, NULL);	
	xTaskCreate(vSendUART, "", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vReceiveUART, "", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
		
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
			tc_write_clock_source(&TCE1, TC_CLKSEL_DIV1_gc);	// Start timer
			
			PORTB.DIRSET = PIN0_bm;    // Set arah PortB PIN0 sebagai output
			PORTB.OUTCLR = PIN0_bm;    // Set PortB PIN0 menjadi low
			PORTB.OUTSET = PIN0_bm;    // Set PortB PIN0 menjadi high
			delay_us(5);               // Tahan high selama 5us
			PORTB.OUTCLR = PIN0_bm;    // Set PortB Pin0 menjadi low kembali
			PORTB.DIRCLR = PIN0_bm;    // Set arah PortB PIN0 kembali menjadi input			
			delay_us(400); //Delay holdoff selama 400us
			int oldinc = incremental;
			delay_us(115); //Delay lagi, kali ini seharusnya pin menjadi high
			
			cpu_irq_enable(); //Mulai interrupt
			
			while(PORTB.IN & PIN0_bm){
				//Tidak ada apa-apa di sini. Loop ini berfungsi untuk mendeteksi pin 0 PORT B yang berubah menjadi low
			}
			int newinc = incremental; //Catat selisih waktu antara suara dikirim hingga diterima
			
			tc_write_clock_source(&TCE1, TC_CLKSEL_OFF_gc); // Stop the timer

			cpu_irq_disable(); //Interrupt dimatikan
			
			if (incremental > 300){ //Jika hasil lebih dari 300 cm, dibulatkan menjadi 300 cm
				distance = 300;				
			} else {
				int inc = newinc - oldinc;
				distance = inc/2; //Dibagi 2 seperti rumus sonar
			}
			
			// Map jaraknya menjadi persentase dan dikurangi 100 agar terbalik (jarak tertinggi berarti 0 persen, jarak terpanjang berarti 100 persen)
			distancePercentage = 100 - map(distance, 0, 11, 0, 100);
			if (distancePercentage > 100){
				distancePercentage = 0;
			}
			
			snprintf(strbuf, sizeof(strbuf), "Panjang: %d cm %d%%  ", distance, distancePercentage);
			gfx_mono_draw_string(strbuf, 0, 0, &sysfont);
			incremental = 0;	// reset incremental menjadi 0
			
			xSemaphoreGive(xSemaphore);
		}
		vTaskDelay(150 / portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vSoilSensor, q_) {	
	char strbuf[128];
	uint16_t soilMoistureValue = 0;
	
	// Inisialisasi pin relay sebagai output dan set menjadi inactive (HIGH karena relay active low, PENTING!!!)
	ioport_set_pin_dir(RELAY_PIN, IOPORT_DIR_OUTPUT);
	gpio_set_pin_high(RELAY_PIN);
	
	while(1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			// Baca value sensor
			soilMoistureValue = adc_read_soil();
			
			// Display value di LCD
			snprintf(strbuf, sizeof(strbuf), "Soil: %u         ", soilMoistureValue);
			gfx_mono_draw_string(strbuf, 0, 16, &sysfont);
			
			// Kontrol relay berdasarkan value dan threshold
			if (soilMoistureValue > soilMoistureThreshold) {
				// Soil kering, nyalakan water pump (active low relay)
				soilMoistureResult = 0;		// Untuk UART
				
				// Cek dahulu apakah pengguna menyalakan penanganan water pump secara otomatis atau tidak
				// Jika tidak, maka hanya display tanah kering, tidak perlu menyalakan pump
				// Jika iya, maka display tanah kering beserta nyalakan pump secara otomatis
				if (pumpUART == 0) {
					pumpStatus = 0;
					gpio_set_pin_high(RELAY_PIN); // Deactivate relay
					snprintf(strbuf, sizeof(strbuf), "Tanah kering         ");
				} else {
					pumpStatus = 1;
					gpio_set_pin_low(RELAY_PIN); // Activate relay
					snprintf(strbuf, sizeof(strbuf), "Tanah kering Pump ON ");
				}
			} else if (soilMoistureValue <= soilMoistureThreshold) {
				// Soil basah, matikan water pump
				soilMoistureResult = 1;		// Untuk UART
				
				// Cek dahulu apakah pengguna menyalakan penanganan water pump secara otomatis atau tidak
				// Jika tidak, maka hanya display tanah lembab. Jika iya, maka display tanah lembab Pump Off
				if (pumpUART == 0) {
					pumpStatus = 0;
					gpio_set_pin_high(RELAY_PIN); // Deactivate relay
					snprintf(strbuf, sizeof(strbuf), "Tanah lembab         ");
				} else {
					pumpStatus = 0;
					gpio_set_pin_high(RELAY_PIN); // Deactivate relay
					snprintf(strbuf, sizeof(strbuf), "Tanah lembab Pump OFF");
				}
			}
			
			gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
			
			xSemaphoreGive(xSemaphore);
		}
		vTaskDelay(80/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vPhotosensitiveSensor, r_) {
	char strbuf[128];
	
	while(1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			// Inisialisasi PORTC pin 1 (LED) sebagai output
			PORTC.DIRSET = PIN1_bm;
			
			// Cek jika Port B Pin 3 (sensor cahaya)
			if (!(PORTB.IN & PIN3_bm)) {
				// Sensor mengembalikan HIGH, maka intensitas cahaya di atas threshold
				snprintf(strbuf, sizeof(strbuf), "Light detected   ");
				
				// Cek dahulu apakah pengguna menyalakan penanganan lampu secara otomatis atau tidak
				// Jika iya, matikan lampu karena intensitas cahaya di sekitar tanaman sudah terang
				if (lightUART == 1) {
					lightResult = 1;		// Terang
					gpio_set_pin_high(LIGHT_PIN); // Deactivate light
					PORTC.OUTCLR = PIN1_bm;  // Turn off LED (set PA1 low)
				}
				
			} else {
				// Sensor mengembalikan LOW, maka intensitas cahaya di bawah threshold
				snprintf(strbuf, sizeof(strbuf), "No light detected");
				
				// Cek dahulu apakah pengguna menyalakan penanganan lampu secara otomatis atau tidak
				// Jika iya, nyalakan lampu karena intensitas cahaya di sekitar tanaman masih gelap
				if (lightUART == 1) {
					lightResult = 0;		// Gelap
					gpio_set_pin_low(LIGHT_PIN); // Deactivate light
					PORTC.OUTSET = PIN1_bm;  // Turn on LED (set PA1 high)
				}
			}
			
			gfx_mono_draw_string(strbuf, 0, 8, &sysfont);
			
			xSemaphoreGive(xSemaphore);
		}
		
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vSendUART, s_) {
	char buffer[50]; // String buffer untuk menyimpan data yang akan dikirimkan
	
	while(1)
	{
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			// Format datanya menjadi string
			snprintf(buffer, sizeof(buffer), "WL:%u H:%u L:%u P:%u LED:%u",
			distancePercentage, soilMoistureResult, lightResult, pumpStatus, lightResult);	
			// Kirimkan string secara karakter satu per satu
			char *text = buffer;
			while (*text) {
				usart_putchar(USART_SERIAL_EXAMPLE, *text++);
				delay_ms(20);
			}
			gfx_mono_draw_string(buffer, 0, 16, &sysfont);	// Display data yang dikirimkan
			xSemaphoreGive(xSemaphore);
		}
		
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vReceiveUART, t_) {
	char strbuf[128];

	while(1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			int i = 0;
			bool validInput = true; // Flag untuk mengindikasikan bahwa input valid (ada pesan yang diterima)

			while(1) {
				char inp = usart_getchar(USART_SERIAL_EXAMPLE);
				if (inp == 0xFF) { // Timeout occurred
					validInput = false;
					break; // Exit the loop and allow other tasks to run
				}
				
				if(inp=='\n') {
					break;
				} else {
					reads[i++] = inp;
				}
			}
			
			// char reads[] = "P:0 LED:0";		// Example
			// Parse hanya jika validInput true
			if (validInput) {
				// Parse Pump
				char *pump_temp = strchr(reads, 'P');
				if (pump_temp) {
					sscanf(pump_temp, "P:%d", &pumpUART);
				}

				// Parse LED
				char *led_temp = strchr(reads, 'L');
				if (led_temp) {
					sscanf(led_temp, "LED:%d", &lightUART);
				}
				
				snprintf(strbuf, sizeof(strbuf), "Receive P:%d LED:%d", pumpUART, lightUART);
				gfx_mono_draw_string(strbuf, 0, 16, &sysfont);
			} else {
				// Jika tidak ada yang diterima, display bahwa Receive UART false
				snprintf(strbuf, sizeof(strbuf), "Receive UART: False");
				gfx_mono_draw_string(strbuf, 0, 16, &sysfont);
			}
			
			xSemaphoreGive(xSemaphore);
		}
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}