#include "board_accelerometer.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "Driver_I2C.h"
#include <stdio.h> 
#include "utils.h"
#include <string.h>

#define I2C_ADDR 0x70 //default address for the driver
#define I2C_PORT 0

#define turnon 0x21 //default data that turns on the LED board, some kind of mode

/* I2C Driver */
extern ARM_DRIVER_I2C Driver_I2C0;
static ARM_DRIVER_I2C * I2Cdrv = &Driver_I2C0;

//copied from adafruit
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
///////////-----------------------------///////////////
//global display values written to LED matrix
uint8_t displaybuffer[17];

//global temporary variable to store hex values for rythyms
uint8_t pixels[2];

//rythms for each round in the game
unsigned char music[12] = {0xED, 0x60, 0xDA, 0xD0, 0xB5, 0xB0, 0x6B, 0x70, 0xD6, 0xE0, 0xAD, 0xD0};

//# rythym we're on
int beat_num;

//your final_score accumulator
int final_score;

///////////-----------------------------///////////////
//Realtime clock stuff
typedef struct {
	unsigned int sec;
	unsigned int msec;
} realtime_t;

// The current time relative to process_start
realtime_t current_time;
int counter_to_twelve;

int outside_clock;

//unsigned int second = 0x10F0000;
int stop_game;
int outside_clock;

void blinkRate(uint8_t b) {
  if (b > 3) b = 0; // turn off if not sure
  uint8_t wr_buff[1];
	wr_buff[0] = (HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1));	
  I2Cdrv->MasterTransmit(I2C_ADDR, wr_buff, 1, false);
	while (I2Cdrv->GetStatus().busy);
}

void setBrightness(uint8_t b){
	if (b > 15) b = 15;
	uint8_t wr_buff[1];
	wr_buff[0] = (HT16K33_CMD_BRIGHTNESS | b);
  I2Cdrv->MasterTransmit(I2C_ADDR, wr_buff, 1, false);
	while (I2Cdrv->GetStatus().busy);
}


//I2C setup
void LED_init (void) {
  I2Cdrv->Initialize   (NULL);
  I2Cdrv->PowerControl (ARM_POWER_FULL);
  I2Cdrv->Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
  I2Cdrv->Control      (ARM_I2C_BUS_CLEAR, 0);
}


void clear() {
  for (uint8_t i=0; i<17; i++) {
    displaybuffer[i] = 0x00;
  }
}

void begin(uint8_t addr) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; //Enable clock to Port B
	PORTB_PCR9 |= (PORT_PCR_MUX(1));  // Set 
	PTB->PDDR &= ~(1 << 9); // Configuring Port B as input
	PORTB_PCR9 |= (1<<1); //enables pull up
	PORTB_PCR9 |= (1001 << 16); //triggers interrupt on rising edge
	NVIC_EnableIRQ(PORTB_IRQn) ;  //Setting IRQ handler for PortB
	uint8_t wr_buff[1];
	wr_buff[0] = 0x21;

	I2Cdrv->MasterTransmit(addr, wr_buff, 1, false);
	while (I2Cdrv->GetStatus().busy);
	blinkRate(HT16K33_BLINK_OFF);
	setBrightness(5);
	clear();
	
}

uint8_t * copy_to_wr(uint8_t *arr){
	for (int i = 0; i<17; i++) {
		arr[i] = displaybuffer[i];
	}
	return arr;
}

//weather
void sunny() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x08; displaybuffer[15]  = 0x10;
	displaybuffer[14]  = 0x04; displaybuffer[13]  = 0x20;
	displaybuffer[12]  = 0x01; displaybuffer[11]  = 0x80;
	displaybuffer[10]  = 0x02; displaybuffer[9]  = 0x58;
	displaybuffer[8]  = 0x32; displaybuffer[7]  = 0x40;
	displaybuffer[6]  = 0x01; displaybuffer[5]  = 0x80;
	displaybuffer[4]  = 0x04; displaybuffer[3]  = 0x20;
	displaybuffer[2]  = 0x08; displaybuffer[1]  = 0x10;
}

void cloudy() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x06	; displaybuffer[15]  = 0x00;
	displaybuffer[14]  = 0x09; displaybuffer[13]  = 0x00;
	displaybuffer[12]  = 0x38; displaybuffer[11]  = 0xb8;
	displaybuffer[10]  = 0x40; displaybuffer[9]  = 0x44;
	displaybuffer[8]  = 0x40; displaybuffer[7]  = 0x04;
	displaybuffer[6]  = 0x40; displaybuffer[5]  = 0x04;
	displaybuffer[4]  = 0x30; displaybuffer[3]  = 0x08;
	displaybuffer[2]  = 0xf; displaybuffer[1]  = 0xf0;
}
void drizzle() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x30; displaybuffer[15]  = 0x21;
	displaybuffer[14]  = 0xcf; displaybuffer[13]  = 0xde;
	displaybuffer[12]  = 0x00; displaybuffer[11]  = 0x00;
	displaybuffer[10]  = 0x10; displaybuffer[9]  = 0x04;
	displaybuffer[8]  = 0x11; displaybuffer[7]  = 0x04;
	displaybuffer[6]  = 0x01; displaybuffer[5]  = 0x20;
	displaybuffer[4]  = 0x00; displaybuffer[3]  = 0x20;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}
void rain() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x30; displaybuffer[15]  = 0x21;
	displaybuffer[14]  = 0xcf; displaybuffer[13]  = 0xde;
	displaybuffer[12]  = 0x00; displaybuffer[11]  = 0x04;
	displaybuffer[10]  = 0x10; displaybuffer[9]  = 0x04;
	displaybuffer[8]  = 0x51; displaybuffer[7]  = 0x04;
	displaybuffer[6]  = 0x51; displaybuffer[5]  = 0x25;
	displaybuffer[4]  = 0x01; displaybuffer[3]  = 0x21;
	displaybuffer[2]  = 0x41; displaybuffer[1]  = 0x21;
}
void thunder() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x02; displaybuffer[15]  = 0x00;
	displaybuffer[14]  = 0x03; displaybuffer[13]  = 0xc0;
	displaybuffer[12]  = 0x00; displaybuffer[11]  = 0x80;
	displaybuffer[10]  = 0x01; displaybuffer[9]  = 0x00;
	displaybuffer[8]  = 0x03; displaybuffer[7]  = 0x80;
	displaybuffer[6]  = 0x01; displaybuffer[5]  = 0x00;
	displaybuffer[4]  = 0x01; displaybuffer[3]  = 0x00;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}
void snow() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x00; displaybuffer[15]  = 0x00;
	displaybuffer[14]  = 0x10; displaybuffer[13]  = 0x08;
	displaybuffer[12]  = 0x54; displaybuffer[11]  = 0x2a;
	displaybuffer[10]  = 0x38; displaybuffer[9]  = 0x1c;
	displaybuffer[8]  = 0x38; displaybuffer[7]  = 0x1c;
	displaybuffer[6]  = 0x54; displaybuffer[5]  = 0x2a;
	displaybuffer[4]  = 0x10; displaybuffer[3]  = 0x08;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}
void mist() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x00; displaybuffer[15]  = 0x00;
	displaybuffer[14]  = 0x00; displaybuffer[13]  = 0x60;
	displaybuffer[12]  = 0x00; displaybuffer[11]  = 0x92;
	displaybuffer[10]  = 0x01; displaybuffer[9]  = 0x0c;
	displaybuffer[8]  = 0x18; displaybuffer[7]  = 0x00;
	displaybuffer[6]  = 0x24; displaybuffer[5]  = 0x80;
	displaybuffer[4]  = 0x43; displaybuffer[3]  = 0x00;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}
//hardcoded display for the number 1
void one() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x08; 
	displaybuffer[14]  = 0x18;
	displaybuffer[12]  = 0x28;
	displaybuffer[10]  = 0x08;
	displaybuffer[8]  = 0x08; 
	displaybuffer[6]  = 0x08; 
	displaybuffer[4]  = 0x08; 
	displaybuffer[2]  = 0x00; 
}

void oneR() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x08; 
	displaybuffer[13]  = 0x18;
	displaybuffer[11]  = 0x28;
	displaybuffer[9]  = 0x08;
	displaybuffer[7]  = 0x08; 
	displaybuffer[5]  = 0x08; 
	displaybuffer[3]  = 0x08; 
	displaybuffer[1]  = 0x00; 
}

void two()
{
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x3C;
	displaybuffer[14]  = 0x04;
	displaybuffer[12]  = 0x04;
	displaybuffer[10]  = 0x3C;
	displaybuffer[8]  = 0x20;
	displaybuffer[6]  = 0x20;
  displaybuffer[4]  = 0x3C;
	displaybuffer[2]  = 0x00;
}

void twoR()
{
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x3C;
	displaybuffer[13]  = 0x04;
	displaybuffer[11]  = 0x04;
	displaybuffer[9]  = 0x3C;
	displaybuffer[7]  = 0x20;
	displaybuffer[5]  = 0x20;
  displaybuffer[3]  = 0x3C;
	displaybuffer[1]  = 0x00;
}

void delayer() {
	for (int i = 0; i < 10; i++) {delay();}
}

void display_temp(int temp) {
	int first_dig = temp/10;
	int second_dig = temp % 10;
	
	switch(first_dig) {
		case 1 : one(); break;
		case 2 : two(); break;
//		case 3 : three(); break;
//		case 4 : four(); break;
//		case 5 : five(); break;
//		case 6 : six(); break;
//		case 7 : seven(); break;
//		case 8 : eight(); break;
//		case 9 : nine(); break;
//		default : zero(); break;
	}
	switch(second_dig) {
		case 1 : oneR(); break;
		case 2 : twoR(); break;
//		case 3 : threeR(); break;
//		case 4 : fourR(); break;
//		case 5 : fiveR(); break;
//		case 6 : sixR(); break;
//		case 7 : sevenR(); break;
//		case 8 : eightR(); break;
//		case 9 : nineR(); break;
//		default : zeroR(); break;
	}
}

void display_weather(int weather){
	switch(weather) {
		case 0 : sunny(); break;
		case 1 : cloudy(); break;
		case 2 : drizzle(); break;
		case 3 : rain(); break;
		case 4 : thunder(); break;
		case 5 : snow(); break;
		case 6 : mist(); break;
	}
	
}




int syscall() {
	char command[50];

  strcpy( command, "dir" );
  system(command);

  return(0);
}

//main function. Runs all the setup functions and waits in while loop for game to execute
int main(){

	hardware_init();	

	LED_Initialize();//from utils
	//setup_button();
	LED_init();
	begin(I2C_ADDR);
	
	syscall();
	
	clear();
	I2Cdrv->MasterTransmit(I2C_ADDR, displaybuffer, 17, false);		
	while (I2Cdrv->GetStatus().busy);
	
	while(1) {
		display_temp(21);
	
		I2Cdrv->MasterTransmit(I2C_ADDR, displaybuffer, 17, false);		
		while (I2Cdrv->GetStatus().busy);
		
		delayer();
		
		clear(); 
		display_weather(6); 
	
		I2Cdrv->MasterTransmit(I2C_ADDR, displaybuffer, 17, false);		
		while (I2Cdrv->GetStatus().busy);
		delayer();
	}
}
