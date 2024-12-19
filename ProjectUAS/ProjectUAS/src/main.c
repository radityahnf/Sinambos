#include <asf.h>
#include <stdio.h>
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/timers.h"
#include "FreeRTOS/include/semphr.h"

int score = 0;
int phase = 0;
int incremental = 0;
int distance = 0;
static char buffarray[200];


/* Define a task */
static portTASK_FUNCTION_PROTO(vSoilMoistureSensor, p_);
static portTASK_FUNCTION_PROTO(vServo, q_);
static portTASK_FUNCTION_PROTO(vDistanceSensor, r_);
static portTASK_FUNCTION_PROTO(vPushButton1, s_);

/* Define semaphore */
SemaphoreHandle_t xSemaphore;
uint16_t counter = 0;

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
	return ((uint32_t)(x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
}


// Function to initialize ADC for potentiometer reading
void adc_init() {
	ADCB.CTRLA = 0x01;
	ADCB.CTRLB = 0x00;
	ADCB.REFCTRL = 0x02;
	ADCB.PRESCALER = 0b0000111;
}

// Read value from ADC
uint16_t read_adc_ch0() {
	// Start a conversion on ADC channel 0
	ADCB.CH0.MUXCTRL = 0x00; // Select ADC pin 0
	ADCB.CH0.CTRL |= 0x81;

	// Wait for conversion to complete
	while (!(ADCB.INTFLAGS & 0x01)); // Wait for interrupt flag

	// Read the conversion result
	uint16_t result = ADCB.CH0RES;
	
	return result;
}

/* PWM configurations for servo */
void PWM_Init(void)
{
	/* Set output */
	PORTC.DIR |= PIN0_bm;	// PC0
	/* Set Register */
	TCC0.CTRLA = (PIN2_bm) | (PIN1_bm);		// Berdasarkan datasheet, 0110 adalah prescaler 256
	TCC0.CTRLB = (PIN4_bm) | (PIN2_bm) | (PIN1_bm);		// Berdasarkan datasheet, pin4 dan 110 adalah enable cca dan double-slope pwm
	/* Set Period */
	TCC0.PER = 156;		// didapat dari rumus PER = clock / (prescaler x frekuensi) dmn clocknya 2MHz (default system clock), prescaler 256, dan frekuensi 50hz (dari 20ms pulse cycle servo)
	/* Set Compare Register value*/
	TCC0.CCA = 1;
}

int main(void) {
	/* System clock initialization */
	sysclk_init();
	board_init();
	gfx_mono_init();

	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
	gfx_mono_draw_string("RaihanRadityaRafinal", 0, 0, &sysfont);

	/* Create the task */
	xTaskCreate(vSoilMoistureSensor, "", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vServo, "", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(vPushButton1, "", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vCounter, "", 1000, NULL, tskIDLE_PRIORITY, NULL);

	/* Semaphore */
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);

	/* Start the task */
	vTaskStartScheduler();
}

static portTASK_FUNCTION(vSoilMoistureSensor, p_) {
	char strbuf[128];
	/* Initialize ADC */
	adc_init();

	while (1) {
		// Read ADC value from potentiometer
		uint16_t adc_result = read_adc_ch0();
		
		// Map ADC value (0-4095) to PWM duty cycle (50 to 84)
		duty_cycle = map(adc_result, 0, 4095, 50, 84);
		
		// Display the ADC and Duty Cycle value on LCD
		snprintf(strbuf, sizeof(strbuf), "Soil: %d", adc_result);
		gfx_mono_draw_string(strbuf, 0, 16, &sysfont);
		
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

static portTASK_FUNCTION(vServo, q_) {
	char strbuf[128];
	/* Initialize PWM for servo control */
	PWM_Init();
	// variables for controlling servo
	uint16_t duty_cycle = 1;
	uint16_t direction = 1;
	
	while (1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			TCC0.CCA = duty_cycle;
			
			// Update the duty cycle
			duty_cycle += direction;
			
			// Change direction if reach the ends
			// 1 for 0 Degress and 10 for 180 degrees
			if (duty_cycle >= 10) {
				direction = -1; // Change direction to decrease
				} else if (duty_cycle <= 1) {
				direction = 1; // Change direction to increase
			}

			// Display the duty cycle value to keep track
			snprintf(strbuf, sizeof(strbuf), "Servo Duty: %u", duty_cycle);
			gfx_mono_draw_string(strbuf, 0, 24, &sysfont);

			xSemaphoreGive(xSemaphore);
		}
		vTaskDelay(20 / portTICK_PERIOD_MS); // Adjust delay for smooth movement
	}
}

//Fungsi setup timer
void setup_timer(void){
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0,print_message);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, 58);
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_HI);
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);
}

//Fungsi ini bukan utk print message, tapi increment nilai variabel "increment" setiap 29us
void print_message(void){
	incremental = incremental + 1;
}

static portTASK_FUNCTION(vUltrasonicSensor, s_) {
	char strbuf[128];

	// Workaround for known issue: Enable RTC32 sysclk
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_RTC);
	while (RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm) {
		// Wait for RTC32 sysclk to become stable
	}
	
	delay_ms(1000);
	setup_timer();
	
	// Insert application code here, after the board has been initialized.
	while(1){
		PORTB.DIR = 0b11111111; //Set output
		PORTB.OUT = 0b00000000; //Set low
		PORTB.OUT = 0b11111111; //Set high selama 5us
		delay_us(5);
		PORTB.OUT = 0b00000000; //Kembali menjadi low
		PORTB.DIR = 0b00000000; //Set menjadi input
		delay_us(750); //Delay holdoff selama 750us
		int oldinc = incremental;
		delay_us(115); //Delay lagi, kali ini seharusnya pin menjadi high
		cpu_irq_enable(); //Mulai interrupt
		while(PORTB.IN & PIN0_bm){
			//Tidak ada apa-apa di sini. Loop ini berfungsi untuk mendeteksi pin 0 PORT B yang berubah menjadi low
		}
		int newinc = incremental; //Catat selisih waktu antara suara dikirim hingga diterima
		cpu_irq_disable(); //Interrupt dimatikan
		if (incremental > 300){ //Jika hasil lebih dari 300 cm, dibulatkan menjadi 300 cm
			score = 300;
			snprintf(buffarray, sizeof(buffarray), "Panjang: %d cm  ", score);
			gfx_mono_draw_string(buffarray, 0, 0, &sysfont);
			delay_ms(100);
			incremental = 0;
			} else {
			int inc = newinc - oldinc;
			int newscore = inc/2; //Dibagi 2 seperti rumus sonar
			snprintf(buffarray, sizeof(buffarray), "Panjang: %d cm  ", newscore);
			gfx_mono_draw_string(buffarray, 0, 0, &sysfont);
			delay_ms(100);
			incremental = 0; //reset nilai variable incremental
		}
	}
}

static portTASK_FUNCTION(vCounter, r_) {
	char strbuf[128];

	while (1) {
		if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
			counter++;
			snprintf(strbuf, sizeof(strbuf), "Counter : %d", counter);
			gfx_mono_draw_string(strbuf, 0, 8, &sysfont);
			xSemaphoreGive(xSemaphore);
		}

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
