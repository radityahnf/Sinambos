/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <stdio.h>

void setup_timer(void);
void print_message(void);

int score = 0;
int phase = 0;
int incremental = 0;
int distance = 0;
static char buffarray[200];

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

int main (void)
{
	// Insert system clock initialization code here (sysclk_init()).
	board_init();
	sysclk_init();
	pmic_init();
	gfx_mono_init();
	
	gpio_set_pin_high(NHD_C12832A1Z_BACKLIGHT);

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

/*
*Keterangan: Kecepatan suara yang digunakan adalah 1/29 cm/us.
*Hasil sedikit tidak akurat, kemungkinan karena salah pengimplementasian rumus. Mohon dikoreksi jika menemukan kesalahan pada kode ini
*/