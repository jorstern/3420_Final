#include "board_accelerometer.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "board_accelerometer.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "Driver_I2C.h"

#define I2C_ADDR 0x70 //default address for the driver
#define I2C_PORT 0
#define _I2C_Driver_(n)  Driver_I2C##n
#define  I2C_Driver_(n) _I2C_Driver_(n)
extern ARM_DRIVER_I2C    I2C_Driver_(I2C_PORT);
#define ptrI2C         (&I2C_Driver_(I2C_PORT))

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

//not too sure how to pass in the array as buf, but I'll leave the function here for reference
void writeBuffer(const uint8_t *buf, uint32_t len){
	//uint8_t wr_buff[1];
	//wr_buff[0] = 0x21;
	ptrI2C->MasterTransmit(I2C_ADDR, buf, len, false);
	while (I2Cdrv->GetStatus().busy);
	
}

void blinkRate(uint8_t b) {
  if (b > 3) b = 0; // turn off if not sure
  
  Wire.write(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1)); 
	
	
  ptrI2C->MasterTransmit(_addr, buf, len, false);
	while (I2Cdrv->GetStatus().busy);
}

int32_t EEPROM_Initialize (void) {
  uint8_t val;
 
  I2Cdrv->Initialize   (NULL);
  I2Cdrv->PowerControl (ARM_POWER_FULL);
  I2Cdrv->Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  I2Cdrv->Control      (ARM_I2C_BUS_CLEAR, 0);
 
  /* Init 24LC128 EEPROM device */
  //DeviceAddr = EEPROM_I2C_ADDR;
 
  /* Read min and max address */
  if (EEPROM_ReadBuf (0x00, &val, 1) == 0) {
    return (EEPROM_ReadBuf (EEPROM_MAX_ADDR-1, &val, 1));
  }
  return -1;
}

void begin(uint8_t _addr) {
	//i2c_addr = _addr;
	uint8_t wr_buff[1];
	wr_buff[0] = 0x21;
//	wr_buf[0] = (uint8_t)(addr >> 8);
//  wr_buf[1] = (uint8_t)(addr & 0xFF);
 
//memory copy -> idk what this does might just be for the sample function
//  memcpy (&wr_buf[2], &buf[0], len);
	
	//pass in the address, the buffer of data, 1 for the size of data (array of size 1), false bc steph said so
	ptrI2C->MasterTransmit(_addr, wr_buff, 1, false);
	while (I2Cdrv->GetStatus().busy);
	
//  i2c_addr = _addr;

//  Wire.begin();

//  Wire.beginTransmission(i2c_addr);
//  Wire.write(0x21);  // turn on oscillator
//  Wire.endTransmission();
//  blinkRate(HT16K33_BLINK_OFF);
//  
//  setBrightness(15); // max brightness
}

void setBrightness(uint8_t b){
	if (b > 15) b = 15;
  Wire.beginTransmission(i2c_addr);
  Wire.write(HT16K33_CMD_BRIGHTNESS | b);
  Wire.endTransmission();
}

int main(){
	uint8_t reg;
	uint16_t reg2;
	
	hardware_init();
	reg = turnon;
	ptrI2C->MasterTransmit(I2C_ADDR, &reg, 1, true);
	ptrI2C->MasterTransmit(I2C_ADDR, &reg, 1, true);
}
	
//ACCELEROMETER_STATE state;
//int main(){
//	hardware_init();
//	Accelerometer_Initialize();
//	while(1){
//		Accelerometer_GetState(&state);
//		debug_printf("%5d %5d %5d\r\n", state.x, state.y, state.z);
//		
//	}
//	
//}