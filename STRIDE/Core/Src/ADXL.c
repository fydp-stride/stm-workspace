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




#include "ADXL.h"


	float GAINX = 0.0f;
	float GAINY = 0.0f;
	float GAINZ = 0.0f;
	
/** Writing ADXL Registers. 
* @address: 8-bit address of register
* @value  : 8-bit value of corresponding register
* Since the register values to be written are 8-bit, there is no need to multiple writing
*/
static void writeRegister(uint8_t address,uint8_t value)
{
	if (address > 63)
	address = 63;

	// Setting R/W = 0, i.e.: Write Mode
	address &= ~(0x80);

	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&SPIhandler,&address,1,10);
	HAL_SPI_Transmit(&SPIhandler,&value,1,10);
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
}


/** Reading ADXL Registers. 
* @address: 8-bit address of register
* @retval value  : array of 8-bit values of corresponding register
* @num		: number of bytes to be written
*/
static void readRegister(uint8_t address,uint8_t * value, uint8_t num)
{
		if (address > 63)
		address = 63;

		// Multiple Byte Read Settings
		if (num > 1)
		address |= 0x40;
		else
		address &= ~(0x40);

		// Setting R/W = 1, i.e.: Read Mode
    address |= (0x80);

	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&SPIhandler,&address,1,10);
	HAL_SPI_Receive(&SPIhandler,value,num,10);
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
}


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
static void adxlBW(ADXL_InitTypeDef * adxl)
		{
		uint8_t bwreg=0;
		writeRegister(BW_RATE,bwreg);
		if (adxl->LPMode == LPMODE_LOWPOWER) 
						{
						// Low power mode
						bwreg |= (1 << 4);
						if ( ((adxl->Rate) <7) && ((adxl->Rate)>12) ) bwreg += 7;
								else bwreg +=(adxl->Rate);
						writeRegister(BW_RATE,bwreg);	
						} 
		else
				{
				// Normal Mode
	
				if ( ((adxl->Rate) <6) && ((adxl->Rate)>15) ) bwreg += 6;
						else bwreg +=(adxl->Rate);
				writeRegister(BW_RATE,bwreg);	
				}
		}

	
/**
	Data Format Settings
	DATA_FORMAT[7-0] = SELF_TEST  SPI  INT_INVERT  0  FULL_RES  Justify  Range[2]
	
	SPI bit: 			0 = 4-wire (Default) 
								1 = 3-wire
	INT_Invert:   		0 = Active High (Default) 
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

static void adxlFormat(ADXL_InitTypeDef * adxl)
			{
			uint8_t formatreg=0;
			writeRegister(DATA_FORMAT,formatreg);
			formatreg = (adxl->SPIMode << 6) | (adxl->IntMode << 5) | (adxl->Justify << 2) | (adxl->Resolution << 3);
			formatreg += (adxl -> Range);
			writeRegister(DATA_FORMAT,formatreg);
			}

// Public Functions

// Initializes the ADXL unit
adxlStatus ADXL_Init(ADXL_InitTypeDef * adxl)
{
	// CS is active low. Here we deselect the chip. In each function the CS signal is asserted individually
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
	// Unknown delay should apply
	HAL_Delay(5);
	ADXL_reset();

	uint8_t testval = 0;
	// The Device Address register is constant, i.e. = 0xE5
	readRegister(DEVID,&testval,1);
	if (testval != 0xE5) return ADXL_ERR;
	// Init. of BW_RATE and DATAFORMAT registers
	adxlBW(adxl);
	adxlFormat(adxl);
	
	// Settings gains 
	if (adxl->Resolution == RESOLUTION_10BIT)
			{
			switch (adxl->Range) {
							case RANGE_2G:
								GAINX = GAINY = GAINZ = 1/255.0f;
								break;
							case RANGE_4G:
								GAINX = GAINY = GAINZ = 1/127.0f;
								break;
							case RANGE_8G:
								GAINX = GAINY = GAINZ = 1/63.0f;
								break;
							case RANGE_16G:
								GAINX = GAINY = GAINZ = 1/31.0f;
								break;
								}
			} else 
			{
			GAINX = GAINY = GAINZ = 1/255.0f;
			}
	// Setting AutoSleep and Link bits
			uint8_t reg;
			readRegister(POWER_CTL,&reg,1);
			if ( (adxl->AutoSleep) == AUTOSLEEPON) reg |= (1 << 4); else reg &= ~(1 << 4);
			if ( (adxl->LinkMode) == LINKMODEON) reg |= (1 << 5); else reg &= ~(1 << 5);
			writeRegister(POWER_CTL,reg);
			
	return ADXL_OK;
	
}


/** Reading Data
* @retval : data				: array of accel. 
						outputType	: OUTPUT_SIGNED: signed int
													OUTPUT_FLOAT: float
						if output is float, the GAIN(X-Y-Z) should be defined in definitions.
* @usage :	Depending on your desired output, define an array of type uint16_t or float with 3 cells:
						uint16_t acc[3];
						ADXL_getAccel(acc,OUTPUT_SIGNED);
						and so on...
*/
void ADXL_getAccel(void *Data , uint8_t outputType) {
	uint8_t data[6]={0,0,0,0,0,0};	
	readRegister(DATA0,data,6);
	
	if (outputType == OUTPUT_SIGNED) {
		int16_t * acc = Data;	
		// Two's Complement
		acc[0] = (int16_t) ((data[1]*256+data[0]));
		acc[1] = (int16_t) ((data[3]*256+data[2]));
		acc[2] = (int16_t) ((data[5]*256+data[4]));
    } else if (outputType == OUTPUT_FLOAT) {
		float * fdata = Data;
		fdata[0] = ( (int16_t) ((data[1]*256+data[0])))*GAINX;
		fdata[1] = ( (int16_t) ((data[3]*256+data[2])))*GAINY;
		fdata[2] = ( (int16_t) ((data[5]*256+data[4])))*GAINZ;
	}
}
	
	
/** Starts Measure Mode
* @param: s = ON or OFF				

*/
void ADXL_Measure(Switch s)
		{
			uint8_t reg;
			readRegister(POWER_CTL,&reg,1);
			switch (s) {
				case ON: 
				reg &= ~(1<<2);
				reg |= (1<<3);
				writeRegister(POWER_CTL,reg);
				break;
				case OFF:
				reg &= ~(1<<3);
				writeRegister(POWER_CTL,reg);
				break;				
				}
		}

void ADXL_reset() {
	writeRegister(POWER_CTL,0x08);
}
	
/** Reading Main Registers
regs[0] = BW_RATE
regs[1] = DATA_FORMAT
regs[2] = POWER_CTL
*/
void ADXL_test(uint8_t * regs)
		{
			readRegister(BW_RATE,&regs[0],1);
			readRegister(DATA_FORMAT,&regs[1],1);
			readRegister(POWER_CTL,&regs[2],1);
			
		}

/**
 Setting the offsets for calibration
* @param 	user-set offset adjustments in twos complement format
					with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g).

*/
void ADXL_SetOffset(int8_t off_x,int8_t off_y,int8_t off_z)
			{
			writeRegister(OFFX,(uint8_t) off_x );
			writeRegister(OFFY,(uint8_t) off_y );
			writeRegister(OFFZ,(uint8_t) off_z );
			}
