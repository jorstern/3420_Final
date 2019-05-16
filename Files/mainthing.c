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
  I2Cdrv->Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  I2Cdrv->Control      (ARM_I2C_BUS_CLEAR, 0);
}

void begin(uint8_t addr) {
	//i2c_addr = _addr;
	uint8_t wr_buff[1];
	wr_buff[0] = 0x21;
//	wr_buf[0] = (uint8_t)(addr >> 8);
//  wr_buf[1] = (uint8_t)(addr & 0xFF);	
	//pass in the address, the buffer of data, 1 for the size of data (array of size 1), false bc steph said so
	I2Cdrv->MasterTransmit(addr, wr_buff, 1, false);
	while (I2Cdrv->GetStatus().busy);
	blinkRate(HT16K33_BLINK_OFF);
	setBrightness(5);
}

//really not sure how to lay out this loop
void writeDisplay() {
	uint8_t wr_buff[17];
	wr_buff[0] = ((uint8_t)0x00); // start at address $00
//  I2Cdrv->MasterTransmit(I2C_ADDR, wr_buff, 1, false);
//	while (I2Cdrv->GetStatus().busy);
	wr_buff[1] = 0x01;
	wr_buff[2] = 0x02;
	wr_buff[3] = 0x03;
	wr_buff[4] = 0x04;
	wr_buff[5] = 0x05;
	wr_buff[6] = 0x06;
	wr_buff[7] = 0x07;
	wr_buff[8] = 0x08;
	wr_buff[9] = 0x09;
	wr_buff[10] = 0x10;
	wr_buff[11] = 0x11;
	wr_buff[12] = 0x12;
	wr_buff[13] = 0x13;
	wr_buff[14] = 0x14;
	wr_buff[15] = 0x15;
	wr_buff[16] = 0x16;
	I2Cdrv->MasterTransmit(I2C_ADDR, wr_buff, 17, false);		
	while (I2Cdrv->GetStatus().busy);
  //Wire.endTransmission();  
	//while (I2Cdrv->GetStatus().busy);
}

void clear() {
  for (uint8_t i=0; i<17; i++) {
    displaybuffer[i] = 0x00;
  }
}

//sample function to generate rythyms
void return_rhythm(){
	//rhythm 1
	//[111011010110]
	//ED6
	//backwards: 60ED
	
	//rhythm 2
	//[110110101101]	
	//DAD
	//backwards: D0DA
	
	//uint8_t wr_buff[17];
//	displaybuffer[0] = ((uint8_t)0x00); // start at address $00
//	displaybuffer[1] = 0x01;
//	displaybuffer[2] = 0x02;
//	displaybuffer[3] = 0x03;
//	displaybuffer[4] = 0x04;
//	displaybuffer[5] = 0x05;
//	displaybuffer[6] = 0x06;
//	displaybuffer[7] = 0x07;
//	displaybuffer[8] = 0x08;
//	displaybuffer[9] = 0x09;
//	displaybuffer[10] = 0x10;
//	displaybuffer[11] = 0x11;
//	displaybuffer[12] = 0x12;
//	displaybuffer[13] = 0x13;
//	displaybuffer[14] = 0x14;
//	displaybuffer[15] = 0x15;
//	displaybuffer[16] = 0x16;

	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[1]  = 0x60;
	displaybuffer[2]  = 0xED;
	displaybuffer[3]  = 0x00;
	displaybuffer[4]  = 0x00;
	displaybuffer[5]  = 0x00;
	displaybuffer[6]  = 0x00;
	displaybuffer[7]  = 0xD0;
	displaybuffer[8]  = 0xDA;
	displaybuffer[9]  = 0x00;
	displaybuffer[10] = 0x00;
	displaybuffer[11] = 0x00;
	displaybuffer[12] = 0x00;
	displaybuffer[13] = 0x00;
	displaybuffer[14] = 0x00;
	displaybuffer[15] = 0x60;
	displaybuffer[16] = 0xED;
}

// Sets up PIT Timer.
static void setUpPit() {
	
		int tempo = DEFAULT_SYSTEM_CLOCK*0.8;
	
		NVIC_EnableIRQ(PIT0_IRQn);
		SIM->SCGC6 = SIM_SCGC6_PIT_MASK; // Enable clock to PIT module
		PIT->MCR = 0;
//		PIT->CHANNEL[0].LDVAL = second * 9 / 10; 	 // Load some value to the PIT timer
		PIT->CHANNEL[0].LDVAL = tempo/1000;
		PIT->CHANNEL[0].TCTRL |= 1;      // Enable the PIT timer - set bit 0
		PIT->CHANNEL[0].TCTRL |= ( 1<<1 );	// Enable timer interrupt - set bit 1
		
//not using this anymore
		//	//SET UP PIT1
//		NVIC_EnableIRQ(PIT1_IRQn); 
//		PIT->CHANNEL[1].TCTRL |= 11; // Enable timer and interrupts
//		PIT->CHANNEL[1].LDVAL = tempo / 1000; // 1 ms	
//	
		// init current time
		current_time.sec = 0;
		current_time.msec = 0;
		outside_clock = 0;
	  beat_num = 0;
	
		counter_to_twelve = 12;		
		final_score = 0;
		
		stop_game = 1;
	}

//hardcoded display for the number 1
void one() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0x01; displaybuffer[15]  = 0x00;
	displaybuffer[14]  = 0x03; displaybuffer[13]  = 0x00;
	displaybuffer[12]  = 0x05; displaybuffer[11]  = 0x00;
	displaybuffer[10]  = 0x01; displaybuffer[9]  = 0x00;
	displaybuffer[8]  = 0x01; displaybuffer[7]  = 0x00;
	displaybuffer[6]  = 0x01; displaybuffer[5]  = 0x00;
	displaybuffer[4]  = 0x07; displaybuffer[3]  = 0xC0;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}

//hardcoded display for the number 2
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
	
//hardcoded display for the number 3
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

//hardcoded display for the word "SCORE"
void Score_Screen() {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[16]  = 0xDB; displaybuffer[15]  = 0xBB;
	displaybuffer[14]  = 0x92; displaybuffer[13]  = 0xAA;
	displaybuffer[12]  = 0xD2; displaybuffer[11]  = 0xBB;
	displaybuffer[10]  = 0x52; displaybuffer[9]  = 0xB2;
	displaybuffer[8]  = 0xDB; displaybuffer[7]  = 0xAB;
	displaybuffer[6]  = 0x00; displaybuffer[5]  = 0x00;
	displaybuffer[4]  = 0x00; displaybuffer[3]  = 0x00;
	displaybuffer[2]  = 0x00; displaybuffer[1]  = 0x00;
}

//hardcoded display for the number 0 at the end
void zero_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x7E;
	displaybuffer[13]  = 0x42;
	displaybuffer[11]  = 0x42;
	displaybuffer[9]  = 0x42;
	displaybuffer[7]  = 0x42;
	displaybuffer[5]  = 0x42;
	displaybuffer[3]  = 0x7E;
	displaybuffer[1]  = 0x00;
}

//hardcoded display for the number 2
void two_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x7E;
	displaybuffer[13]  = 0x02;
	displaybuffer[11]  = 0x02;
	displaybuffer[9]  = 0x7E;
	displaybuffer[7]  = 0x40;
	displaybuffer[5]  = 0x40;
	displaybuffer[3]  = 0x7E;
	displaybuffer[1]  = 0x00;
}

//hardcoded display for the number 3
void three_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x7E;
	displaybuffer[13]  = 0x02;
	displaybuffer[11]  = 0x02;
	displaybuffer[9]  = 0x7E;
	displaybuffer[7]  = 0x02;
	displaybuffer[5]  = 0x02;
	displaybuffer[3]  = 0x7E;
	displaybuffer[1]  = 0x00;
}	

//hardcoded display for the number 4
void four_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x42;
	displaybuffer[13]  = 0x42;
	displaybuffer[11]  = 0x42;
	displaybuffer[9]  = 0x7E;
	displaybuffer[7]  = 0x02;
	displaybuffer[5]  = 0x02;
	displaybuffer[3]  = 0x02;
	displaybuffer[1]  = 0x00;
}	

//hardcoded display for the number 5
void five_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x7E;
	displaybuffer[13]  = 0x40;
	displaybuffer[11]  = 0x40;
	displaybuffer[9]  = 0x7E;
	displaybuffer[7]  = 0x02;
	displaybuffer[5]  = 0x02;
	displaybuffer[3]  = 0x7E;
	displaybuffer[1]  = 0x00;
}	

//hardcoded display for the number 6
void six_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x7E;
	displaybuffer[13]  = 0x40;
	displaybuffer[11]  = 0x40;
	displaybuffer[9]  = 0x7E;
	displaybuffer[7]  = 0x42;
	displaybuffer[5]  = 0x42;
	displaybuffer[3]  = 0x7E;
	displaybuffer[1]  = 0x00;
}	

//hardcoded display for the number 7
void seven_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x7E;
	displaybuffer[13]  = 0x02;
	displaybuffer[11]  = 0x02;
	displaybuffer[9]  = 0x02;
	displaybuffer[7]  = 0x02;
	displaybuffer[5]  = 0x02;
	displaybuffer[3]  = 0x02;
	displaybuffer[1]  = 0x00;
}	

//hardcoded display for the number 8
void eight_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x7E;
	displaybuffer[13]  = 0x42;
	displaybuffer[11]  = 0x42;
	displaybuffer[9]  = 0x7E;
	displaybuffer[7]  = 0x42;
	displaybuffer[5]  = 0x42;
	displaybuffer[3]  = 0x7E;
	displaybuffer[1]  = 0x00;
}	

//hardcoded display for the number 9
void nine_end () {
	displaybuffer[0]  = 0x00; // start at address $00
	displaybuffer[15]  = 0x7E;
	displaybuffer[13]  = 0x42;
	displaybuffer[11]  = 0x42;
	displaybuffer[9]  = 0x7E;
	displaybuffer[7]  = 0x02;
	displaybuffer[5]  = 0x02;
	displaybuffer[3]  = 0x02;
	displaybuffer[1]  = 0x00;
}	

//clears the right half of the matrix
void clear_right_half () {
	displaybuffer[15]  = 0x00;
	displaybuffer[13]  = 0x00;
	displaybuffer[11]  = 0x00;
	displaybuffer[9]  = 0x00;
	displaybuffer[7]  = 0x00;
	displaybuffer[5]  = 0x00;
	displaybuffer[3]  = 0x00;
	displaybuffer[1]  = 0x00;
}

//displays the correct digit based on the final_score
void score_number(int front, int end) {
	clear();
	if (front == 1) {
		for (int i =3; i <= 15; i=i+2){displaybuffer[i] = 0x10;}
	} else if (front == 0) {
		zero_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	} else if (front == 2) {
		two_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	} else if (front == 3) {
		three_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	} else if (front == 4) {
		four_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	} else if (front == 5) {
		five_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	} else if (front == 6) {
		six_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	} else if (front == 7) {
		seven_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	} else if (front == 8) {
		eight_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	} else if (front == 9) {
		nine_end();
		for (int i = 2; i <= 16; i=i+2) {displaybuffer[i] = displaybuffer[i-1];}
	}
	
	clear_right_half();
	if (end == 1) {
		for (int i =3; i <= 15; i=i+2){displaybuffer[i] = 0x10;}
	} else if (end == 0) {
		zero_end();
	} else if (end == 2) {
		two_end();
	} else if (end == 3) {
		three_end();
	} else if (end == 4) {
		four_end();
	} else if (end == 5) {
		five_end();
	} else if (end == 6) {
		six_end();
	} else if (end == 7) {
		seven_end();
	} else if (end == 8) {
		eight_end();
	} else if (end == 9) {
		nine_end();
	}
	
}

//void setup_button(){
//	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; //Enable clock to Port C
//	PORTC_PCR6=PORT_PCR_MUX(001);  //Set up PTC6 as GPIO
//	PTC->PDDR |= (0 << 22); //Configuring Port C as input
//	NVIC_EnableIRQ(PORTC_IRQn) ;  //Setting IRQ handler for SW2
//}

//setup for GPIO external pins
void setup_button(){
	//PTB9
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; //Enable clock to Port B
	PORTB_PCR9 |= (PORT_PCR_MUX(1));  // Set 
	PTB->PDDR &= ~(1 << 9); // Configuring Port B as input
	PORTB_PCR9 |= (1<<1); //enables pull up
	PORTB_PCR9 |= (1001 << 16); //triggers interrupt on rising edge
	NVIC_EnableIRQ(PORTB_IRQn) ;  //Setting IRQ handler for PortB
	
	//PTA1
//	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; //Enable clock to Port B
//	PORTA_PCR1 |= (PORT_PCR_MUX(1));  // Set up ptc16 as output
//	PTA->PDDR |= (1 << 1); // Configuring Port B as output
//	NVIC_EnableIRQ(PORTA_IRQn) ;  //Setting IRQ handler for PortB
}

// if(GPIOB_PDIR & (1 << 9))  
//    {  
//         //Input pin is in high state.  
//    }  
//    else  
//    {  
//         //Input pin is in low state.   
//    }
//	
//ut33
//ps675


//#define DAC0_ADDR 0;

//void DAC_setup(){
//	SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK;
//	dac_converter_config_t dac_config_t;
//	DAC_DRV_StructInitUserConfigNormal( &dac_config_t );
//	DAC_DRV_Init( DAC0_ADDR, &dac_config_t );
//	
//	//test port
//	DAC_DRV_Output( DAC0_ADDR, 4095 );

//}

//setup for PWM output
void setupPWM(){
//PTD0
SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK; /*Enable the FTM0 clock*/
SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; /*Enable the PORTD clock*/
//PORTD_PCR5 = PORT_PCR_MUX(4); /*MUX = ALT 4*/
PORTD_PCR0 = PORT_PCR_MUX(4); /*MUX = ALT 4*/

	NVIC_EnableIRQ(FTM0_IRQn); /*Enable the FTM Interrupt*/
	FTM0_SC |= 0x004F; /*Setting TOIE = 1,CLKS = 01, PS = 111*/
	FTM0_MOD = 32000;
	FTM0_C5SC |= 0x0028; /*Setting MSB = 1, ELSnB = 1*/
	FTM0_C5V = 500;
}


//main function. Runs all the setup functions and waits in while loop for game to execute
int main(){

	hardware_init();	

	LED_Initialize();//from utils
	setup_button();
	LED_init();
	begin(I2C_ADDR);
	
  clear();
	//return_rhythm();
	setUpPit();

	
	while(stop_game){
		//do nothing, play game through interrupts
	};
}

/* scoring updates the current score if a button 
 * press is detected within a range of 500 ms from
 * time the button press is expected */
void scoring(){
	int index;
	int curr_sec = current_time.sec % 12;
	if (current_time.msec < 500) {
		index = curr_sec;
	} else {
		index = curr_sec+1;
	}
	
	
	if(index<8){
		if(music[beat_num] & (1<<(7-index))){ //not zero -> button pressed on correct beat
			final_score++;
		}
	}
	else{
		index = index - 8;
		if(music[beat_num+1] & (1<<(7-index))){ //not zero -> button pressed on correct beat
			final_score++;
		}
	}
		
}

/* PortB_IRQHandler is the interrupt handler for
 * button presses. Each time the button is pressed, 
 * the handler calculates the current score */
void PORTB_IRQHandler(void) {
	 //GPIOA_PSOR |= (1 << 1);
	__disable_irq(); // sets PM flag
	
	if(PTB->PDIR & GPIO_PDIR_PDI(1<<9) ){
  	//LEDGreen_On();
		//scoring logic
		scoring();
	} 
	else {		
		//LED_Off();
	}
	
	LED_Off();
	PORTB -> ISFR = (1 << 9); //clears it?
	//NVIC_ClearPendingIRQ(PORTB_IRQn);
		__enable_irq(); // clears PM flag
}


//11101101011
void sub_rhythm1(){
	//uint8_t pixels[2];
	pixels[1] = 0xED; pixels[0] = 0x60; 
}

//110110101101
void sub_rhythm2(){
	//uint8_t pixels[2];
	pixels[1] = 0xDA; pixels[0] = 0xD0; 
}

//101101011011
void sub_rhythm3(){
	//uint8_t pixels[2];
	pixels[1] = 0xB5; pixels[0] = 0xB0; 
}

//011010110111
void sub_rhythm4(){
	//uint8_t pixels[2];
	pixels[1] = 0x6B; pixels[0] = 0x70; 
}

//110101101110
void sub_rhythm5(){
	//uint8_t pixels[2];
	pixels[1] = 0xD6; pixels[0] = 0xE0; 
}


//101011011101
void sub_rhythm6(){
	//uint8_t pixels[2];
	pixels[1] = 0xAD; pixels[0] = 0xD0; 
}

void sub_rhythm0(){
	//uint8_t pixels[2];
	pixels[1] = 0x00; pixels[0] = 0x00; 
}

/* update_buffer() updates display_buffer[] with the current
 * LED configuration to be displayed on the led matrix.  */
void update_buffer(){
	int curr_sec = outside_clock;
	if (curr_sec < 4) {
		three();
	} else if (curr_sec < 8) {
		two();
	} else if (curr_sec < 12) {
		one();
	} else {
		//clear();
		//in_countdown = 0;
	}
	
	if(outside_clock>=12){
			displaybuffer[16] = 0xFF; displaybuffer[15] = 0xF0;
			displaybuffer[14] = 0x00; displaybuffer[13] = 0x00;
	}
	
	if(outside_clock>=12 && outside_clock<24){
		//display first frame
		sub_rhythm3();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm2();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm2();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm1();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm1();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=24 && outside_clock<36){
		//display second frame
		sub_rhythm3();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm3();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm2();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm2();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm1();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=36 && outside_clock<48){
		beat_num = 2;
		//display second frame
		sub_rhythm4();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm3();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm3();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm2();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm2();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=48 && outside_clock<60){
		//display second frame
		sub_rhythm4();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm4();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm3();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm3();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm2();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
		else if(outside_clock>=60 && outside_clock<(12*6)){
		beat_num = 4;
		//display second frame
		sub_rhythm5();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm4();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm4();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm3();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm3();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=(12*6) && outside_clock<12*7){
		//display second frame
		sub_rhythm5();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm5();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm4();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm4();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm3();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=12*7 && outside_clock<12*8){
		beat_num = 6;
		//display second frame
		sub_rhythm6();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm5();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm5();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm4();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm4();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=12*8 && outside_clock<12*9){
		//display second frame
		sub_rhythm6();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm6();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm5();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm5();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm4();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=12*9 && outside_clock<12*10){
		beat_num = 8;
		//display second frame
		sub_rhythm1();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm6();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm6();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm5();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm5();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=12*10 && outside_clock<12*11){
		//display second frame
		sub_rhythm1();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm1();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm6();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm6();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm5();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=12*11 && outside_clock<12*12){
		beat_num = 10;
		//display second frame
		sub_rhythm0();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm1();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm1();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm6();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm6();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
		else if(outside_clock>=12*12 && outside_clock<12*13){
		//display second frame
		sub_rhythm0();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm0();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm1();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm1();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm6();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
		else if(outside_clock>=12*13 && outside_clock<12*14){
		beat_num = 0;
		//display second frame
		sub_rhythm0();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm0();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm0();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm1();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm1();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
		else if(outside_clock>=12*14 && outside_clock<12*15){
		//display second frame
		sub_rhythm0();
		displaybuffer[12] = pixels[1]; displaybuffer[11] = pixels[0];
		sub_rhythm0();
		displaybuffer[10] = pixels[1]; displaybuffer[9]  = pixels[0];
		sub_rhythm0();
		displaybuffer[8]  = pixels[1]; displaybuffer[7]  = pixels[0];
		sub_rhythm0();
		displaybuffer[6]  = pixels[1]; displaybuffer[5]  = pixels[0];
		sub_rhythm1();
		displaybuffer[4]  = pixels[1]; displaybuffer[3]  = pixels[0];
	}
	else if(outside_clock>=12*15 && outside_clock<12*16){
		Score_Screen();
	}
	else if(outside_clock>=12*16 && outside_clock<12*18){
		score_number((int)(final_score/10), (int)(final_score%10));
		sub_rhythm0();
		displaybuffer[2]  = pixels[1]; displaybuffer[1]  = pixels[0];
	}
	else if(outside_clock>=12*18){
			stop_game = 0;;
	}
	
}

//PIT0 interrupt handler updates tempo beats
void PIT0_IRQHandler(void) {
	PIT->CHANNEL[0].TCTRL &= 0xFFFFFFFC; //Disable timer and interrupts
	if (current_time.msec == 999) {
		current_time.msec = 0;
		current_time.sec++;
		
		//update_buffer();
	
	if (counter_to_twelve == 0) {
		counter_to_twelve = 11;
	} else {
		counter_to_twelve--;
	}
	if(counter_to_twelve > 3){
		displaybuffer[2] = (1<<(counter_to_twelve-4));
		displaybuffer[1] = (0x00);
	} else{
		displaybuffer[2] = (0x00);
		displaybuffer[1] = (1<<(counter_to_twelve+4));	
	}	
	update_buffer();
	I2Cdrv->MasterTransmit(I2C_ADDR, displaybuffer, 17, false);
	outside_clock++;
	
	if(final_score>55){
		LEDBlue_On();
	}
	
		} else {
		current_time.msec++;
	}
//PIT->CHANNEL[0].LDVAL = something; 
	PIT->CHANNEL[0].TFLG |= 1; 
	PIT->CHANNEL[0].TCTRL = 3;//|= 11; // Enable timer and interrupts
	PIT->CHANNEL[0].TFLG |= (1 << 1);
}

//PIT1 interrupt handler updates current time
//void PIT1_IRQHandler(void) {
//	//__disable_irq();
//	PIT->CHANNEL[1].TCTRL &= 0xFFFFFFFC; //Disable timer and interrupts
//	if (current_time.msec == 999) {
//		current_time.msec = 0;
//		current_time.sec++;
//	} else {
//		current_time.msec++;
//	}
//	//PIT->CHANNEL[1].LDVAL = second / 1000; 
//	PIT->CHANNEL[1].TFLG |= 1; 
//	PIT->CHANNEL[1].TCTRL = 3;//|= 11; // Enable timer and interrupts
//	//__enable_irq();
//}

/* FTM0_IRQHandler is the handler for the speaker */
void FTM0_IRQHandler (void)
{
unsigned long ChannelValue = FTM0_C5V; /*Take the value of the Channel to
	compare it*/
	(void)FTM0_SC;
	FTM0_SC |= 0x0080; /*FTM counter has overflow*/
	if(ChannelValue < 32000) /*Channel Value > Modulo Value*/
	{
	FTM0_C5V += 500; /*Add 500 to Channel*/
	}
	else
	{
	FTM0_C5V = 0; /*Set Channel in 0*/
	}
}
