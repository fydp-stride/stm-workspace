
/**
* @Library for ADXL345 3-axis accelometer 
* @Hardware dependencies: Could be changed very easily.
						STM32L152R uC
						SPI2 
						Some GPIOs
* @Author Iman Hosseinzadeh iman[dot]hosseinzadeh AT gmail
  https://github.com/ImanHz
						
*/
 /**
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/

#ifndef ADXL_H
#define ADXL_H

// Change the library according to your uC
#include "stm32l4xx_hal.h"

// SPI handler. Change it if needed.
// NOTE: SPI must be set in High Polarity and 2 Edges Phase mode.

// If you are using libraries other than HAL, i.e. STDPeriph, change writeRegister and readRegister functions.
#define SPIhandler hspi1
#define I2CHandler hi2c1
extern SPI_HandleTypeDef SPIhandler;
extern I2C_HandleTypeDef hi2c1;

// GPIO definition
#define ADXLCS_Pin GPIO_PIN_4
#define ADXLCS_GPIO_Port GPIOA

#define ADXL_ADDRESS 0x53 << 1


// Registers' Address 
#define DEVID 					0x0
#define BW_RATE					0x2C 
#define DATA_FORMAT 			0x31
#define FIFO_CTL 				0x38
#define DATA0					0x32
#define POWER_CTL 				0x2D
#define THRESH_TAP				0x1D
#define DUR						0x21
#define TAP_AXES				0x2A
#define INT_ENABLE				0x2E
#define INT_MAP					0x2F
#define LATENT					0x22
#define WINDOW					0x23
#define THRESH_ACT				0x24
#define THRESH_INACT			0x25
#define TIME_INAT				0x26
#define ACT_INACT_CTL			0x27
#define THRESH_FF 				0x28
#define TIME_FF					0x29
#define OFFX					0x1E
#define OFFY					0x1F
#define OFFZ					0x20
#define INT_SOURCE				0x30


// Init return values
typedef enum {ADXL_OK,ADXL_ERR} adxlStatus;

// ON/OFF enum
typedef enum {ON,OFF} Switch;

// Init. Definitions
#define SPIMODE_3WIRE 1
#define SPIMODE_4WIRE 0

#define LPMODE_NORMAL 0
#define LPMODE_LOWPOWER 1

#define BWRATE_6_25 	6
#define BWRATE_12_5 	7
#define BWRATE_25 		8
#define BWRATE_50 		9
#define BWRATE_100		10
#define BWRATE_200		11
#define BWRATE_400		12
#define BWRATE_800		13
#define BWRATE_1600   	14
#define BWRATE_3200   	15

#define BWRATE_12_5 	7
#define BWRATE_25 		8
#define BWRATE_50 		9
#define BWRATE_100		10
#define BWRATE_200		11
#define BWRATE_400		12

#define INT_ACTIVEHIGH 0
#define INT_ACTIVELOW  1

#define RESOLUTION_FULL  1
#define RESOLUTION_10BIT 0

#define JUSTIFY_MSB 	1
#define JUSTIFY_SIGNED  0
 
 


#define	SLEEP_RATE_1HZ 3
#define SLEEP_RATE_2HZ 2
#define SLEEP_RATE_4HZ 1
#define SLEEP_RATE_8HZ 0

#define RANGE_2G  0
#define RANGE_4G  1
#define RANGE_8G  2
#define RANGE_16G 3

#define AUTOSLEEPON  1
#define AUTOSLEEPOFF 0

#define LINKMODEON  1
#define LINKMODEOFF 0

// Init Type Def
typedef struct {
	uint8_t SPIMode;
	uint8_t IntMode;
	uint8_t LPMode;
	uint8_t Rate;
	uint8_t Range;
	uint8_t Resolution;
	uint8_t Justify;
	uint8_t AutoSleep;
	uint8_t LinkMode;
}ADXL_InitTypeDef;

	
// Private functions

/** Writing ADXL Registers. 
* @address: 8-bit address of register
* @value  : 8-bit value of corresponding register
* Since the register values to be written are 8-bit, there is no need to multiple writing
*/
// static void writeRegister(uint8_t address,uint8_t value);


/** Reading ADXL Registers. 
* @address: 8-bit address of register
* @value  : array of 8-bit values of corresponding register
* @num		: number of bytes to be written
*/

// static void readRegister(uint8_t address,uint8_t * value, uint8_t num);


/**
Bandwidth Settings:
 Setting BW_RATE register
 BWRATE[4] = LOW POWER SETTING
 BWRATE[0-3] = DATA RATE i.e. 0110 for 6.25 Hz // See Table 6,7
 @param LPMode = 0 // Normal mode, Default
							 = 1 // Low Power Mode
 @param BW : Badwidth; See Tables 6 and 7
				
								NORMAL MODE
				BW value 	|  Output Data Rate (Hz)
				---------------------------------
						6 		|  				6.25 // Default		
						7 		|  				12.5		
						8 		|  				25
						9 		|  				50		
						10 		|  				100
						11 		|  				200
						12 		|  				400
						13 		|  				800
						14 		|  				1600
						15 		|  				3200
								
								
								LOWPOWER MODE
				BW value 	|  Output Data Rate (Hz)
				---------------------------------
						7 		|  				12.5	// Default
						8 		|  				25
						9 		|  				50		
						10 		|  				100
						11 		|  				200
						12 		|  				400
						
		*/

// static void adxlBW(ADXL_InitTypeDef * adxl);
	
	
/**
	Data Format Settings
	DATA_FORMAT[7-0] = SELF_TEST  SPI  INT_INVERT  0  FULL_RES  Justify  Range[2]
	@TODO : SELF_TEST is not implemented yet!
	
	SPI bit: 			0 = 4-wire (Default) 
								1 = 3-wire
	INT_Invert:   0 = Active High (Default) 
								1 = Active Low
	Full Res:			0 = 10-bit (Default) 
								1 = Full Resolution
	Justify:			0 = Signed (Default) 
								1 = MSB
	Range:
								
					 

				Range value 	|  Output Data Rate (Hz)
				---------------------------------
						0 		|  				+-2g	// Default
						1 		|  				+-4g
						2 		|  				+-8g		
						3 		|  				+-16g
	 									
		*/

// static void adxlFormat(ADXL_InitTypeDef * adxl);
	
	
	
	
// Public Functions

/** Initializes the ADXL unit
	* @param adxl, structure of ADXL_InitTypeDef: 
*/
adxlStatus ADXL_Init(ADXL_InitTypeDef * adxl);



 // ADXL_getAccel function definitions 
#define OUTPUT_FLOAT  0
#define OUTPUT_SIGNED 1

/** Reading Data
* @retval : data				: array of accel. 
* @param	:	outputType	: OUTPUT_SIGNED: signed int
						      OUTPUT_FLOAT: float
						if output is float, the GAIN(X-Y-Z) should be defined in definitions.
*/
void ADXL_getAccel(void *Data,uint8_t outputType);

/** Starts Measure Mode
* @param: s = ON or OFF				

*/
void ADXL_Measure(Switch s);


void ADXL_reset();

/** Reading Main Registers
regs[0] = BW_RATE
regs[1] = DATA_FORMAT
regs[2] = POWER_CTL
*/
void ADXL_test(uint8_t * regs);

/**
 Setting the offsets for calibration
* @param 	user-set offset adjustments in twos complement format
					with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g).
				
*/				
void ADXL_SetOffset(int8_t off_x,int8_t off_y,int8_t off_z);

#endif
