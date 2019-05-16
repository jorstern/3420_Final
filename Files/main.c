#include "board_accelerometer.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "Driver_I2C.h"
#include <stdio.h> 
#include "utils.h"

#define I2C_ADDR 0x70 //default address for the driver
#define I2C_PORT 0

#define turnon 0x21 //default data that turns on the LED board, some kind of mode

/* I2C Driver */
extern ARM_DRIVER_I2C Driver_I2C0;
static ARM_DRIVER_I2C * I2Cdrv = &Driver_I2C0;

//From Adafruit
#ifndef _BV
#define _BV(bit) (1<<(bit))
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif
//////------------------------------------//////
//values for the LED display driver
#define LED_ON 1
#define LED_OFF 0

#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0

//buffer to hold display values for LED matrix
uint8_t displaybuffer[17]; //array of unsigned chars

//for realtime clock
typedef struct {
	unsigned int sec;
	unsigned int msec;
} realtime_t;

realtime_t current_time;

int outside_clock; //second clock
int stop; //in case we want to use interrupts


//setup stuff for the LED matrix
void set_brightness(uint8_t b){
	if (b > 15) b = 15;
	uint8_t wr_buff[1];
	wr_buff[0] = (HT16K33_CMD_BRIGHTNESS | b);
  I2Cdrv->MasterTransmit(I2C_ADDR, wr_buff, 1, false);
	while (I2Cdrv->GetStatus().busy);
}

void blink_rate(uint8_t b) {
  if (b > 3) b = 0; // turn off if not sure
  uint8_t buffer[1];
	buffer[0] = (HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));	
  I2Cdrv->MasterTransmit(I2C_ADDR, buffer, 1, false);
	while (I2Cdrv->GetStatus().busy);
}

//I2C setup
void LED_init (void) {
  I2Cdrv->Initialize   (NULL);
  I2Cdrv->PowerControl (ARM_POWER_FULL);
  I2Cdrv->Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  I2Cdrv->Control      (ARM_I2C_BUS_CLEAR, 0);
}

//PIT Timer setup
static void setup_PIT() {
	
	int speed = DEFAULT_SYSTEM_CLOCK*0.0008;
	
	NVIC_EnableIRQ(PIT0_IRQn);
	SIM->SCGC6 = SIM_SCGC6_PIT_MASK; // Enable clock to PIT module
	PIT->MCR = 0;
	PIT->CHANNEL[0].LDVAL = speed;
	PIT->CHANNEL[0].TCTRL |= 1;      // Enable the PIT timer - set bit 0
	PIT->CHANNEL[0].TCTRL |= ( 1<<1 );	// Enable timer interrupt - set bit 1
	
	current_time.sec = 0;
	current_time.msec = 0;
	
	outside_clock = 0;
}
void begin(uint8_t addr) {
	uint8_t buffer[1];
	buffer[0] = 0x21;
	I2Cdrv->MasterTransmit(addr, buffer, 1, false);
	while (I2Cdrv->GetStatus().busy);
	blink_rate(HT16K33_BLINK_OFF);
	set_brightness(5);

}
//clear the display buffer
void clear_display() {
	for (uint8_t i=0; i<17; i++) {
    displaybuffer[i] = 0x00;
  }
}

void write_to_display() {
	uint8_t buffer[17];
	buffer[0] = (uint8_t)0x00; //starts this shit at address 00
	
	buffer[1] = 0x01;
	buffer[2] = 0x02;
	buffer[3] = 0x03;
	buffer[4] = 0x04;
	buffer[5] = 0x05;
	buffer[6] = 0x06;
	buffer[7] = 0x07;
	buffer[8] = 0x08;
	buffer[9] = 0x09;
	buffer[10] = 0x10;
	buffer[11] = 0x11;
	buffer[12] = 0x12;
	buffer[13] = 0x13;
	buffer[14] = 0x14;
	buffer[15] = 0x15;
	buffer[16] = 0x16;
	
	I2Cdrv->MasterTransmit(I2C_ADDR, buffer, 17, false); //send her
	while (I2Cdrv->GetStatus().busy);
}

void update_buffer() {
	int curr_seconds = outside_clock;

	
}
//==========HARDCODED NUMBERS==========
void one() {
	displaybuffer[0]  = 0x00;
	displaybuffer[16]  = 0x01; displaybuffer[15]  = 0x00;
	displaybuffer[14]  = 0x03; displaybuffer[13]  = 0x00;
	displaybuffer[12]  = 0x05; displaybuffer[11]  = 0x00;
	displaybuffer[10]  = 0x01; displaybuffer[9]  = 0x00;
	displaybuffer[8]  = 0x01; displaybuffer[7]  = 0x00;
	displaybuffer[6]  = 0x01; displaybuffer[5]  = 0x00;
	displaybuffer[4]  = 0x07; displaybuffer[3]  = 0xC0;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}

void two() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x03; displaybuffer[15]  = 0xC0;
	displaybuffer[14]  = 0x00; displaybuffer[13]  = 0x40;
	displaybuffer[12]  = 0x00; displaybuffer[11]  = 0x40;
	displaybuffer[10]  = 0x03; displaybuffer[9]  = 0xC0;
	displaybuffer[8]  = 0x02; displaybuffer[7]  = 0x00;
	displaybuffer[6]  = 0x02; displaybuffer[5]  = 0x00;
	displaybuffer[4]  = 0x03; displaybuffer[3]  = 0xC0;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}	

void three() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x03; displaybuffer[15]  = 0xC0;
	displaybuffer[14]  = 0x00; displaybuffer[13]  = 0x40;
	displaybuffer[12]  = 0x00; displaybuffer[11]  = 0x40;
	displaybuffer[10]  = 0x03; displaybuffer[9]  = 0xC0;
	displaybuffer[8]  = 0x00; displaybuffer[7]  = 0x40;
	displaybuffer[6]  = 0x00; displaybuffer[5]  = 0x40;
	displaybuffer[4]  = 0x03; displaybuffer[3]  = 0xC0;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}
//=======================================

int main() {
	hardware_init();
  LED_Initialize();//from utils
  LED_init();
	begin(I2C_ADDR);
	clear_display();
	setup_PIT();
	while(!stop){};
}