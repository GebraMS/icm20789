/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
#include	"GebraBit_ICM20789.h"

extern SPI_HandleTypeDef hspi1;
	
/*=========================================================================================================================================
 * @brief     Read data from spacial register.
 * @param     regAddr Register Address of ICM20789
 * @param     data    Pointer to Variable that register value is saved .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t	GB_ICM20789_Read_Reg_Data ( uint8_t regAddr, uint8_t* data)
{	
	uint8_t txBuf[2] = {regAddr|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	//GB_ICM20789_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		*data = rxBuf[1];
	}
	return stat;
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of ICM20789 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20789_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (GB_ICM20789_Read_Reg_Data( regAddr, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
		tempData &= mask; // zero all non-important bits in data
		tempData >>= (start_bit - len + 1); //shift data to zero position
		*data = tempData;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
		*data = 0;
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of ICM20789 that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20789_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
//	uint8_t pTxBuf[513];
//	uint8_t pRxBuf[513];
	uint8_t status = HAL_ERROR;
	pTxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1)); // reason of "+1" is for register address that comes in first byte
	pRxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, (byteQuantity + 1)*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t)); //here we dont have "+1" beacause we don't need first byte that was register data , we just need DATA itself
	}
	free(pTxBuf);
	free(pRxBuf);
	return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of ICM20789
 * @param     data    Value that will be writen to register .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20789_Write_Reg_Data(uint8_t regAddr, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr|0x00 , data}; //Write operation: set the 8th-bit to 0
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	return status;	
}

/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of ICM20789 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20789_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}
	if (GB_ICM20789_Read_Reg_Data( regAddr,  &tempData) == HAL_OK)	
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= data; // combine data with existing byte

		txBuf[0] = regAddr;
		txBuf[1] = tempData;
	
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Write value to Multiple register address.
 * @param     regAddr First Register Address of ICM20789 that writing multiple data start from this address
 * @param     data    Pointer to Variable that multiple data are writen from .
 * @param     byteQuantity Quantity of data that we want to write .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_ICM20789_Burst_Write		( uint8_t regAddr, uint8_t *data, 	uint16_t byteQuantity)
{
	uint8_t txBuf[byteQuantity + 1]; // +1 is for register address that is 1 byte
	uint8_t rxBuf[byteQuantity + 1];
	uint8_t status = HAL_ERROR;
	txBuf[0] = regAddr | 0x00; //Write operation: set the 8th-bit to 0.
	memcpy(txBuf+1, data, byteQuantity); // +1 is for set the address of data from [1]th position of array

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return status;
}

/*=========================================================================================================================================
 * @brief     Reset ICM20789
 * @param     ICM20789   ICM20789 Struct RESET  variable
 * @param     ICM20789   ICM20789 Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20789_Soft_Reset ( GebraBit_ICM20789 * ICM20789 )
{
	GB_ICM20789_Write_Reg_Bits(ICM20789_USER_CTRL, START_MSB_BIT_AT_0,BIT_LENGTH_1 , 1);
	do 
	 {
		GB_ICM20789_Write_Reg_Bits(ICM20789_PWR_MGMT_1, START_MSB_BIT_AT_7,BIT_LENGTH_1 , 1);
		HAL_Delay(100);
		GB_ICM20789_Read_Reg_Bits (ICM20789_PWR_MGMT_1, START_MSB_BIT_AT_7,BIT_LENGTH_1, &ICM20789->RESET);
		if ( ICM20789->RESET == DONE )
			break;
	 }while(1);
}
/*=========================================================================================================================================
 * @brief     Get Who am I Register Value From Sensor
 * @param     ICM20789     ICM20789 Struct WHO_AM_I variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_ICM20789_Who_am_I(GebraBit_ICM20789 * ICM20789)
{
	GB_ICM20789_Read_Reg_Data( ICM20789_WHO_AM_I,&ICM20789->WHO_AM_I);
}	
/*=========================================================================================================================================
 * @brief     Enable Or Disable DMP and Set it to Low Power mode if needed.
 * @param     ICM20789   ICM20789 Struct DMP  variable
 * @param     dmp    Enable Or Disable DMP
 * @param     dmp_lp Determines DMP in 	DMP_LOW_POWER or NOT_DMP_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/

void GB_ICM20789_DMP(GebraBit_ICM20789* ICM20789 ,ICM20789_Ability dmp,ICM20789_DMP_LP dmp_lp)
{
	GB_ICM20789_Write_Reg_Bits (ICM20789_USER_CTRL , START_MSB_BIT_AT_7, BIT_LENGTH_1 , dmp);
	GB_ICM20789_Write_Reg_Bits (ICM20789_PWR_MGMT_2 , START_MSB_BIT_AT_6, BIT_LENGTH_1 , dmp_lp);
	ICM20789->DMP = dmp ;
}

/*=========================================================================================================================================
 * @brief     Reset DMP
 * @param     rst     Determines reset DMP or not( Enable Or Disable)
 * @return    Nothing
 ========================================================================================================================================*/ 

void GB_ICM20789_DMP_Reset(GebraBit_ICM20789* ICM20789 ,ICM20789_Ability rst)
{
  GB_ICM20789_Write_Reg_Bits (ICM20789_USER_CTRL , START_MSB_BIT_AT_3, BIT_LENGTH_1 , rst);
}

/*=========================================================================================================================================
 * @brief     Set DMP interrupt 
 * @param     interrupt Enable Or Disable interrupt
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_DMP_Interrupt_Pin(ICM20789_Ability interrupt)///DMP_INTERRUPT---->STRUCT
{
	GB_ICM20789_Write_Reg_Bits (ICM20789_INT_ENABLE , START_MSB_BIT_AT_1, BIT_LENGTH_1 , interrupt);
}
/*=========================================================================================================================================
 * @brief     SET ICM20789 Sleep or Awake
 * @param     ICM20789   ICM20789 Struct IS_ICM20789_Sleep  variable
 * @param     working   Determines ICM20789_AWAKE or ICM20789_SLEEP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Sleep_Awake (GebraBit_ICM20789 * ICM20789, ICM20789_Sleep  working  ) 
{
  GB_ICM20789_Write_Reg_Bits (ICM20789_PWR_MGMT_1 , START_MSB_BIT_AT_6, BIT_LENGTH_1 , working);
  ICM20789->IS_ICM20789_SLEEP = working ;
}

/*=========================================================================================================================================
 * @brief     Set ICM20789 Accelerometer Power Mode
 * @param     ICM20789   ICM20789 Struct ACCEL_POWER_MODE  variable
 * @param     pmode        Determines ICM20789 Accelerometer Power Mode in ICM20789_LOW_NOISE or ICM20789_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/ 
 void GB_ICM20789_ACCEL_Power_Mode(GebraBit_ICM20789* ICM20789 ,ICM20789_Power_Mode pmode)
{
	GB_ICM20789_Write_Reg_Bits (ICM20789_PWR_MGMT_1 , START_MSB_BIT_AT_5, BIT_LENGTH_1 , pmode);
	ICM20789->ACCEL_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     Set ICM20789 Gyroscope Power Mode
 * @param     ICM20789   ICM20789 Struct GYRO_POWER_MODE  variable
 * @param     pmode        Determines ICM20789 Gyroscope Power Mode in ICM20789_LOW_NOISE or ICM20789_LOW_POWER
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_GYRO_Power_Mode(GebraBit_ICM20789* ICM20789 ,ICM20789_Power_Mode pmode)
{
	GB_ICM20789_Write_Reg_Bits (ICM20789_LP_MODE_CTRL , START_MSB_BIT_AT_7, BIT_LENGTH_1 , pmode);
	ICM20789->GYRO_POWER_MODE = pmode ;
}
/*=========================================================================================================================================
 * @brief     Set ICM20789 Clock Source
 * @param     ICM20789   ICM20789 Struct CLOCK_SOURCE  variable
 * @param     clk    Determines between INTERNAL_20MHZ_OSCILLATOR , AUTO_SELECT and CLOCK_STOP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Set_Clock_Source(GebraBit_ICM20789 * ICM20789 , ICM20789_CLK clk)
{ 
 GB_ICM20789_Write_Reg_Bits( ICM20789_PWR_MGMT_1, START_MSB_BIT_AT_2, BIT_LENGTH_3 , clk);
 ICM20789->CLOCK_SOURCE = clk ;
} 
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature Sensor
 * @param     ICM20789   ICM20789 Struct TEMPERATURE  variable
 * @param     temp     Determines DISABLE or ENABLE Temperature Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Temperature(GebraBit_ICM20789* ICM20789 ,ICM20789_Ability temp)
{
	GB_ICM20789_Write_Reg_Bits (ICM20789_PWR_MGMT_1 , START_MSB_BIT_AT_3, BIT_LENGTH_1 , !temp);
  ICM20789->TEMPERATURE = temp ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Accelerometer Sensor
 * @param     ICM20789   ICM20789 Struct ACCEL  variable
 * @param     accel     Determines SENSOR_DISABLE or SENSOR_ENABLE Accelerometer Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20789_Accelerometer(GebraBit_ICM20789 * ICM20789 , ICM20789_Sensor accel)
{
	GB_ICM20789_Write_Reg_Bits (ICM20789_PWR_MGMT_2 , START_MSB_BIT_AT_5, BIT_LENGTH_3 , accel);
  ICM20789->ACCEL = accel ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Gyroscope Sensor
 * @param     ICM20789   ICM20789 Struct GYRO  variable
 * @param     gyro     Determines SENSOR_DISABLE or SENSOR_ENABLE Gyroscope Sensor
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Gyroscope(GebraBit_ICM20789 * ICM20789 , ICM20789_Sensor gyro)
{
	GB_ICM20789_Write_Reg_Bits (ICM20789_PWR_MGMT_2 , START_MSB_BIT_AT_2, BIT_LENGTH_3 , gyro);
  ICM20789->GYRO = gyro ; 
}
/*=========================================================================================================================================
 * @brief     Configure hardware interrupt pin (INT) 
 * @param     ICM20789  ICM20789 struct INT_PIN_LEVEL , INT_PIN_TYPE and INT_PIN_LATCH  variables
 * @param     level   ACTIVE_HIGH or  ACTIVE_LOW 
 * @param     type    PUSH_PULL   or  OPEN_DRAIN
 * @param     latch   _50_US      or  HELD_STATUS_CLEAR
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20789_Set_INT_Pin(GebraBit_ICM20789 * ICM20789 , ICM20789_INT_Level level ,ICM20789_INT_Type type , ICM20789_Latch_Type latch )
{
  GB_ICM20789_Write_Reg_Bits( ICM20789_INT_PIN_CFG, START_MSB_BIT_AT_7, BIT_LENGTH_1 , level);
	GB_ICM20789_Write_Reg_Bits( ICM20789_INT_PIN_CFG, START_MSB_BIT_AT_6, BIT_LENGTH_1 , type);
	GB_ICM20789_Write_Reg_Bits( ICM20789_INT_PIN_CFG, START_MSB_BIT_AT_5, BIT_LENGTH_1 , latch);
	ICM20789->INT_PIN_LEVEL = level ; 
	ICM20789->INT_PIN_TYPE  = type  ;
	ICM20789->INT_PIN_LATCH = latch ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE FIFO Overflow Interrupt Pin
 * @param     ICM20789   ICM20789 Struct FIFO_OVERFLOW_INT  variable
 * @param     data_ovf_int    Determines  FIFO Overflow Interrupt Disable or Enable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_FIFO_Overflow_Interrupt_Pin(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability data_ovf_int)
{
	GB_ICM20789_Write_Reg_Bits(ICM20789_INT_ENABLE, START_MSB_BIT_AT_4, BIT_LENGTH_1 , data_ovf_int);
	ICM20789->FIFO_OVERFLOW_INT = data_ovf_int  ; 
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Data Ready Interrupt Pin
 * @param     ICM20789   ICM20789 Struct DATA_READY_INT  variable
 * @param     data_ready_int    Determines Data Ready Interrupt Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Data_Ready_Interrupt_Pin(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability data_ready_int)
{
	GB_ICM20789_Write_Reg_Bits(ICM20789_INT_ENABLE, START_MSB_BIT_AT_0, BIT_LENGTH_1 , data_ready_int);
	ICM20789->DATA_READY_INT = data_ready_int  ; 
}
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     Check if FIFO Overflow
 * @param     ICM20789   Store FIFO OVERFLOW status onICM20789 Struct FIFO_OVERFLOW  variable
 * @return    FIFO_IS_NOT_OVERFLOW or FIFO_IS_OVERFLOW
 ========================================================================================================================================*/ 
ICM20789_FIFO_Overflow GB_ICM20789_Check_FIFO_Overflow(GebraBit_ICM20789 * ICM20789)
{
  GB_ICM20789_Read_Reg_Bits(ICM20789_INT_STATUS, START_MSB_BIT_AT_4, BIT_LENGTH_1 , &ICM20789->FIFO_OVERFLOW);
	return ICM20789->FIFO_OVERFLOW;
}
/*=========================================================================================================================================
 * @brief     Check if Data is ready
 * @param     ICM20789    Store data ready status on ICM20789 Struct DATA_STATUS variable
 * @return    IS_Ready or IS_NOT_Ready
 ========================================================================================================================================*/ 
ICM20789_Preparation GB_ICM20789_Check_Data_Preparation(GebraBit_ICM20789 * ICM20789)
{
  GB_ICM20789_Read_Reg_Bits(ICM20789_INT_STATUS, START_MSB_BIT_AT_0, BIT_LENGTH_1 , &ICM20789->DATA_STATUS); 
	return ICM20789->DATA_STATUS;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Access Serial Interface To FIFO
 * @param     ICM20789  ICM20789 struct INTERFACE_ACCESS_FIFO  variable
 * @param     interface_access_fifo    DISABLE or ENABLE
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Access_Serial_Interface_To_FIFO(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability interface_access_fifo) 
{ 
	GB_ICM20789_Write_Reg_Bits (ICM20789_USER_CTRL , START_MSB_BIT_AT_6, BIT_LENGTH_1,  interface_access_fifo);
  ICM20789->INTERFACE_ACCESS_FIFO = interface_access_fifo ;  
}

/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE accelerometer to be written on FIFO
 * @param     ICM20789  ICM20789 struct ACCEL_TO_FIFO  variable  
 * @param     accel_fifo Determines accelerometer write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Write_ACCEL_FIFO(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability accel_fifo )
{
   GB_ICM20789_Write_Reg_Bits (ICM20789_FIFO_EN, START_MSB_BIT_AT_3, BIT_LENGTH_1,accel_fifo); 
	 ICM20789->ACCEL_TO_FIFO = accel_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Gyroscope to be written on FIFO
 * @param     ICM20789  ICM20789 struct GYRO_TO_FIFO  variable  
 * @param     gyro_fifo  Determines Gyroscope write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Write_GYRO_FIFO(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability gyro_fifo )
{
   GB_ICM20789_Write_Reg_Bits (ICM20789_FIFO_EN, START_MSB_BIT_AT_6, BIT_LENGTH_3,(uint8_t)(gyro_fifo*7)); ///*7 to make b111 to enable xyz
	 ICM20789->GYRO_TO_FIFO = gyro_fifo ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Temperature to be written on FIFO
 * @param     ICM20789  ICM20789 struct TEMP_TO_FIFO  variable 
 * @param     temp_fifo  Determines Temperature write on fifo
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20789_Write_TEMP_FIFO(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability temp_fifo )
{
   GB_ICM20789_Write_Reg_Bits (ICM20789_FIFO_EN, START_MSB_BIT_AT_7, BIT_LENGTH_1,temp_fifo); 
	 ICM20789->TEMP_TO_FIFO = temp_fifo ;
}
/*=========================================================================================================================================
 * @brief     Set FIFO Size
 * @param     ICM20789  ICM20789 struct FIFO_SIZE  variable 
* @param      fifo_size     Determines FIFO Size among : 512bytes , 1 KB , 2 KB or 4 KB
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_FIFO_Size(GebraBit_ICM20789 * ICM20789 , ICM20789_FIFO_Size fifo_size )
{
  GB_ICM20789_Write_Reg_Bits (ICM20789_ACCEL_CONFIG2,START_MSB_BIT_AT_7, BIT_LENGTH_2, fifo_size); 
  GB_ICM20789_FIFO_Reset();
	ICM20789->FIFO_SIZE = fifo_size ; 
}
/*=========================================================================================================================================
 * @brief     Set FIFO MODE
 * @param     ICM20789  ICM20789 struct FIFO_MODE  variable 
 * @param     fifo_mode     Determines FIFO MODE BYPASS ,  STREAM_TO_FIFO , STOP_ON_FULL_FIFO_SNAPSHOT
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_FIFO_Mode(GebraBit_ICM20789 * ICM20789 , ICM20789_FIFO_Mode fifo_mode )
{
  GB_ICM20789_Write_Reg_Bits (ICM20789_CONFIG,START_MSB_BIT_AT_6, BIT_LENGTH_1, fifo_mode); 
  ICM20789->FIFO_MODE = fifo_mode;
}
/*=========================================================================================================================================
 * @brief     Set FIFO reset.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_FIFO_Reset(void ) 
{
  GB_ICM20789_Write_Reg_Bits (ICM20789_USER_CTRL, START_MSB_BIT_AT_2, BIT_LENGTH_1, 1); 
}
/*=========================================================================================================================================
 * @brief     Get FIFO Count  
 * @param     ICM20789   ICM20789 struct  FIFO_COUNT variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_GET_FIFO_Count (GebraBit_ICM20789 * ICM20789 ) 
{
	uint8_t count_h , count_l;
  GB_ICM20789_Read_Reg_Data( ICM20789_FIFO_COUNTH, &count_h); 
	GB_ICM20789_Read_Reg_Data( ICM20789_FIFO_COUNTL, &count_l );
	ICM20789->FIFO_COUNT = (uint16_t)((count_h << 8) | count_l);////13_Bit
}
/*=========================================================================================================================================
 * @brief     Read Data Directly from FIFO
 * @param     ICM20789  ICM20789 struct FIFO_DATA variable
 * @param     qty    Determine hoe many Data Byte to read
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Read_FIFO(GebraBit_ICM20789 * ICM20789 , uint16_t qty)  
{
  GB_ICM20789_Burst_Read( ICM20789_FIFO_R_W,ICM20789->FIFO_DATA, qty);
}
/*=========================================================================================================================================
 * @brief     Set Gyroscope Full Scale Range and select Gyroscope SCALE FACTOR
 * @param     ICM20789   ICM20789 Struct GYRO_FULL_SCALE and GYRO_SCALE_FACTOR variable
 * @param     fs         Determines Full Scale Range among FS_250_DPS , FS_500_DPS , FS_1000_DPS , FS_2000_DPS
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20789_GYRO_Full_Scale ( GebraBit_ICM20789 * ICM20789 , ICM20789_Gyro_Fs_Sel fs ) 
{
  GB_ICM20789_Write_Reg_Bits (ICM20789_GYRO_CONFIG , START_MSB_BIT_AT_4, BIT_LENGTH_2, fs);
	ICM20789->GYRO_FULL_SCALE = fs ; 
	switch(fs)
	 {
	  case FS_250_DPS:
		ICM20789->GYRO_SCALE_FACTOR = SCALE_FACTOR_131_LSB_DPS ;
		ICM20789->PRECISE_GYRO_SF   =  131 ;
    break;
		case FS_500_DPS:
		ICM20789->GYRO_SCALE_FACTOR = SCALE_FACTOR_65p5_LSB_DPS ;
		ICM20789->PRECISE_GYRO_SF   =  65.5 ;
    break;	
		case FS_1000_DPS:
		ICM20789->GYRO_SCALE_FACTOR = SCALE_FACTOR_32p8_LSB_DPS ;
		ICM20789->PRECISE_GYRO_SF   =  32.8 ;
    break;	
		case FS_2000_DPS:
		ICM20789->GYRO_SCALE_FACTOR = SCALE_FACTOR_16p4_LSB_DPS ;
		ICM20789->PRECISE_GYRO_SF   =  16.4 ;
    break;			
		default:
		ICM20789->GYRO_SCALE_FACTOR = SCALE_FACTOR_131_LSB_DPS ;
    ICM20789->PRECISE_GYRO_SF   =  131 ;		
	 }
}
/*=========================================================================================================================================
 * @brief     Enable Or Bypass GYRO and Temperature Low Pass Filter
 * @param     ICM20789     ICM20789 Struct GYRO_FCHOICEB variable
 * @param     bypass       Determines BYPASS_DLPF_FCHOICEB or ENABLE_DLPF_FCHOICEB
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20789_GYRO_TEMP_Low_Pass_Filter  (GebraBit_ICM20789 * ICM20789 ,  ICM20789_FCHOICEB bypass )
{
//	if ( bypass == BYPASS_DLPF_FCHOICEB  )
//		ICM20789->GYRO_SAMPLE_RATE = _32_KHz ;
	GB_ICM20789_Write_Reg_Bits(ICM20789_GYRO_CONFIG, START_MSB_BIT_AT_1, BIT_LENGTH_2, (uint8_t) (bypass*2));//2 Bits : 11
	ICM20789->GYRO_FCHOICEB =bypass ;   
}
/*=========================================================================================================================================
 * @brief     Set GYRO and Temperature Low Pass Filter value
 * @param     ICM20789     ICM20789 Struct GYRO_TEMP_DLPF variable
 * @param     dlpf         Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 		
void GB_ICM20789_GYRO_TEMP_Low_Pass_Filter_Value  (GebraBit_ICM20789 * ICM20789 , ICM20789_GYRO_TEMP_DLPF dlpf )
{
	  GB_ICM20789_Write_Reg_Bits(ICM20789_CONFIG , START_MSB_BIT_AT_2, BIT_LENGTH_3,  dlpf);
	  ICM20789->GYRO_TEMP_DLPF =  dlpf ;
}
/*=========================================================================================================================================
 * @brief     Set  GYRO Averaging Filter
 * @param     ICM20789  ICM20789 Struct GYRO_AVERAGING_FILTER variable
 * @param     avg       Averaging value
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_ICM20789_GYRO_LP_Averaging_Filter  (GebraBit_ICM20789 * ICM20789 , ICM20789_GYRO_Averaging_Filter avg )
{
	  GB_ICM20789_GYRO_TEMP_Low_Pass_Filter(ICM20789,ENABLE_DLPF_FCHOICEB);
	  GB_ICM20789_Write_Reg_Bits(ICM20789_LP_MODE_CTRL , START_MSB_BIT_AT_6, BIT_LENGTH_3,  avg);
	  ICM20789->GYRO_AVERAGING_FILTER =  avg ;
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Full Scale Range and select sensor SCALE FACTOR
 * @param     ICM20789   ICM20789 struct ACCEL_FULL_SCALE and ACCEL_SCALE_FACTOR variable
 * @param     fs         Determines Full Scale Range among 2g , 4g , 8g , 16g , 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_ICM20789_ACCEL_Full_Scale ( GebraBit_ICM20789 * ICM20789 , ICM20789_Accel_Fs_Sel fs ) 
{
  GB_ICM20789_Write_Reg_Bits( ICM20789_ACCEL_CONFIG, START_MSB_BIT_AT_4, BIT_LENGTH_2 , fs);
	ICM20789->ACCEL_FULL_SCALE =  fs ;
	switch(fs)
	 {
	  case FULL_SCALE_2g:
		ICM20789->ACCEL_SCALE_FACTOR = SCALE_FACTOR_16384_LSB_g ;
    break;
		case FULL_SCALE_4g:
		ICM20789->ACCEL_SCALE_FACTOR = SCALE_FACTOR_8192_LSB_g ;
    break;	
		case FULL_SCALE_8g:
		ICM20789->ACCEL_SCALE_FACTOR = SCALE_FACTOR_4096_LSB_g ;
    break;	
		case FULL_SCALE_16g: 
		ICM20789->ACCEL_SCALE_FACTOR = SCALE_FACTOR_2048_LSB_g ;
    break;			
		default:
		ICM20789->ACCEL_SCALE_FACTOR = SCALE_FACTOR_16384_LSB_g ;		
	 }
}

/*=========================================================================================================================================
 * @brief     Enable Or Bypass Accelerometer Low Pass Filter
 * @param     ICM20789     ICM20789 Struct ACCEL_FCHOICEB variable
 * @param     bypass       Determines ENABLE_DLPF_FCHOICEB or BYPASS_DLPF_FCHOICEB
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20789_ACCEL_Low_Pass_Filter  (GebraBit_ICM20789 * ICM20789 ,  ICM20789_FCHOICEB bypass )
{
//	if ( bypass == BYPASS_DLPF_FCHOICEB  )
//		ICM20789->ACCEL_SAMPLE_RATE = _4_KHz ;
	GB_ICM20789_Write_Reg_Bits(ICM20789_ACCEL_CONFIG2 , START_MSB_BIT_AT_3, BIT_LENGTH_1,  bypass);
	ICM20789->ACCEL_FCHOICEB =bypass ;   
}
/*=========================================================================================================================================
 * @brief     Set Accelerometer Low Pass Filter value
 * @param     ICM20789     ICM20789 Struct ACCEL_DLPF variable
 * @param     dlpf     Low Pass Filter value 
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_ICM20789_ACCEL_Low_Pass_Filter_Value  (GebraBit_ICM20789 * ICM20789 , ICM20789_ACCEL_DLPF dlpf )
{
	  GB_ICM20789_Write_Reg_Bits(ICM20789_ACCEL_CONFIG2, START_MSB_BIT_AT_2, BIT_LENGTH_3,  dlpf);
	  ICM20789->ACCEL_DLPF =  dlpf ;
}
/*=========================================================================================================================================
 * @brief     Set  Accelerometer Averaging Filter
 * @param     ICM20789  ICM20789 Struct ACCEL_AVERAGING_FILTER variable
 * @param     avg       Averaging value
 * @return    Nothing 
 ========================================================================================================================================*/ 		
void GB_ICM20789_ACCEL_LP_Averaging_Filter  (GebraBit_ICM20789 * ICM20789 , ICM20789_ACCEL_Averaging_Filter avg )
{   
	  GB_ICM20789_ACCEL_Low_Pass_Filter(ICM20789,ENABLE_DLPF_FCHOICEB);
	  GB_ICM20789_ACCEL_Low_Pass_Filter_Value(ICM20789,ICM20789_ACCEL_DLPF_420);
	  GB_ICM20789_Write_Reg_Bits(ICM20789_ACCEL_CONFIG2 , START_MSB_BIT_AT_5, BIT_LENGTH_2,  avg);
	  ICM20789->ACCEL_AVERAGING_FILTER =  avg ;
}
/*=========================================================================================================================================
 * @brief     Set Sensor Output Sample Rate that controls  data output rate, FIFO sample rate
 * @param     ICM20789   ICM20789 struct ACCEL_SAMPLE_RATE and ACCEL_SAMPLE_DEVIDE variable
 * @param     rate_hz    Sample Rate in Hz
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Output_Sample_Rate (GebraBit_ICM20789 * ICM20789 , uint16_t rate_hz)
{
	uint8_t  gfchoice , gdlpf ;
	GB_ICM20789_Read_Reg_Bits(ICM20789_GYRO_CONFIG, START_MSB_BIT_AT_1, BIT_LENGTH_2,&gfchoice );
	GB_ICM20789_Read_Reg_Bits(ICM20789_CONFIG , START_MSB_BIT_AT_2, BIT_LENGTH_3,  &gdlpf);
	if((gfchoice==0)&&(0<gdlpf)&&(gdlpf<7))
	{
		ICM20789->INTERNAL_SAMPLE_RATE = _1_KHz ;
		ICM20789->SAMPLE_RATE = rate_hz ; 
    ICM20789->SAMPLE_DEVIDE=(ICM20789->INTERNAL_SAMPLE_RATE/rate_hz)-1;  
		GB_ICM20789_Write_Reg_Data( ICM20789_SMPLRT_DIV ,ICM20789->SAMPLE_DEVIDE ); 
	}
//	else if((gfchoice==0)&&(1>gdlpf)&&(gdlpf>6))
//	{
//	  ICM20789->GYRO_SAMPLE_RATE = _8_KHz  ;
//	}
}
/*=========================================================================================================================================
 * @brief     Configure FIFO
 * @param     ICM20789       ICM20789 Struct FIFO variable
 * @param     fifo           Configure ICM20789 FIFO according it is FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_FIFO_Configuration ( GebraBit_ICM20789 * ICM20789 , ICM20789_FIFO_Ability fifo  )
{
	ICM20789->FIFO_PACKET_QTY = PACKET_QTY_IN_FULL_FIFO ;  
	if( fifo==Enable )  
	{
		ICM20789->FIFO = FIFO_ENABLE  ;
		GB_ICM20789_FIFO_Reset();
		GB_ICM20789_Access_Serial_Interface_To_FIFO( ICM20789 , Enable );
		GB_ICM20789_FIFO_Size ( ICM20789 , _1_KBYTE );
		GB_ICM20789_FIFO_Mode ( ICM20789 , STOP_ON_FULL_FIFO_SNAPSHOT );
		GB_ICM20789_Write_GYRO_FIFO ( ICM20789 , Enable );
	  GB_ICM20789_Write_ACCEL_FIFO( ICM20789 , Enable );
		GB_ICM20789_Write_TEMP_FIFO ( ICM20789 , Enable );
		GB_ICM20789_FIFO_Overflow_Interrupt_Pin( ICM20789 ,Enable ) ;
	}
	else if ( fifo == Disable )
	{
		ICM20789->FIFO = FIFO_DISABLE  ;
		GB_ICM20789_FIFO_Overflow_Interrupt_Pin( ICM20789 ,Disable ) ;
		GB_ICM20789_Write_GYRO_FIFO ( ICM20789 , Disable );
	  GB_ICM20789_Write_ACCEL_FIFO( ICM20789 , Disable );
		GB_ICM20789_Write_TEMP_FIFO ( ICM20789 , Disable );
		GB_ICM20789_Access_Serial_Interface_To_FIFO( ICM20789 , Disable );
		GB_ICM20789_FIFO_Reset( );
	}
}
/*=========================================================================================================================================
 * @brief     Set ICM20789 Power Management
 * @param     ICM20789   ICM20789 Struct ACCEL_POWER_MODE and GYRO_POWER_MODE  variable
 * @param     pmode        Determines ICM20789 Accelerometer Power Mode in ICM20789_LOW_NOISE or ICM20789_LOW_POWER or ICM20789_SLEEP_OFF
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Set_Power_Management(GebraBit_ICM20789 * ICM20789 , ICM20789_Power_Mode pmode) 
{	
	
 GB_ICM20789_Temperature(ICM20789 , Enable );
 GB_ICM20789_Accelerometer(ICM20789 , SENSOR_ENABLE );
 GB_ICM20789_Gyroscope(ICM20789 , SENSOR_ENABLE );
 if(pmode==ICM20789_LOW_POWER)
 {
	GB_ICM20789_Sleep_Awake(ICM20789 , ICM20789_AWAKE );
  GB_ICM20789_GYRO_Power_Mode (ICM20789 , ICM20789_LOW_POWER );
  GB_ICM20789_ACCEL_Power_Mode(ICM20789 , ICM20789_LOW_POWER );		 
 }
  else if(pmode==ICM20789_LOW_NOISE)
 {
	GB_ICM20789_Sleep_Awake(ICM20789 , ICM20789_AWAKE );
  GB_ICM20789_GYRO_Power_Mode (ICM20789 , ICM20789_LOW_NOISE );
  GB_ICM20789_ACCEL_Power_Mode(ICM20789 , ICM20789_LOW_NOISE );	  
 }
 else if (pmode==ICM20789_SLEEP_OFF)
 {
	ICM20789->ACCEL_POWER_MODE = ICM20789_SLEEP_OFF ;
  ICM20789->GYRO_POWER_MODE = ICM20789_SLEEP_OFF ;
	GB_ICM20789_Temperature(ICM20789 , Disable );
	GB_ICM20789_Accelerometer(ICM20789 , SENSOR_DISABLE );
	GB_ICM20789_Gyroscope(ICM20789 , SENSOR_DISABLE );
	GB_ICM20789_Sleep_Awake(ICM20789 , ICM20789_SLEEP );
 }
 HAL_Delay(1);
}

/*=========================================================================================================================================
 * @brief     initialize ICM20789
 * @param     ICM20789     initialize ICM20789 according  
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_initialize( GebraBit_ICM20789 * ICM20789 )
{
  HAL_Delay(3);
  GB_ICM20789_Who_am_I(ICM20789);
	GB_ICM20789_Soft_Reset(ICM20789);
	GB_ICM20789_DMP( ICM20789 ,Disable ,DMP_LOW_POWER );
	//GB_ICM20789_Select_SPI4_Interface(ICM20789 , IS_SPI);
	GB_ICM20789_Set_Power_Management( ICM20789 , ICM20789_LOW_NOISE );
	GB_ICM20789_Set_Clock_Source( ICM20789 , AUTO_SELECT );
	GB_ICM20789_GYRO_TEMP_Low_Pass_Filter (ICM20789,ENABLE_DLPF_FCHOICEB);
	GB_ICM20789_ACCEL_Low_Pass_Filter(ICM20789,ENABLE_DLPF_FCHOICEB);
	GB_ICM20789_GYRO_TEMP_Low_Pass_Filter_Value (ICM20789,ICM20789_GYRO_TEMP_DLPF_92);
	GB_ICM20789_ACCEL_Low_Pass_Filter_Value(ICM20789,ICM20789_ACCEL_DLPF_99);
	GB_ICM20789_FIFO_Configuration ( ICM20789 ,FIFO_DISABLE ) ;
	GB_ICM20789_Set_INT_Pin( ICM20789 , ACTIVE_LOW  , OPEN_DRAIN  ,  HELD_STATUS_CLEAR );
	GB_ICM20789_Data_Ready_Interrupt_Pin( ICM20789 ,Enable ) ;
}
/*=========================================================================================================================================
 * @brief     Configure ICM20789
 * @param     ICM20789  Configure ICM20789 according 
 * @param     fifo           Configure ICM20789 FIFO according it is FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Configuration(GebraBit_ICM20789 * ICM20789, ICM20789_FIFO_Ability fifo)
{
	GB_ICM20789_Output_Sample_Rate(ICM20789,SAMPLE_RATE_ODR_HZ );
	GB_ICM20789_GYRO_Full_Scale ( ICM20789 ,FS_1000_DPS ) ;
	GB_ICM20789_ACCEL_Full_Scale( ICM20789 ,FULL_SCALE_4g ) ; 
	GB_ICM20789_FIFO_Configuration ( ICM20789 ,fifo ) ;
	HAL_Delay(20);	
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature from Register 
 * @param     ICM20789  store Raw Data Of Temprature in GebraBit_ICM20789 Staruct REGISTER_RAW_TEMP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_Temp_Register_Raw_Data(GebraBit_ICM20789 * ICM20789)
{
	uint8_t temp_msb , temp_lsb;
  GB_ICM20789_Read_Reg_Data(ICM20789_TEMP_OUT_H , &temp_msb);
	GB_ICM20789_Read_Reg_Data(ICM20789_TEMP_OUT_L, &temp_lsb);
	ICM20789->REGISTER_RAW_TEMP = (int16_t)((temp_msb << 8) | temp_lsb);
}

/*=========================================================================================================================================
 * @brief     Get Valid Data Of Temprature Base on Datasheet Formula 
 * @param     ICM20789  store Valid Data Of Temprature in GebraBit_ICM20789 Staruct VALID_TEMP_DATA
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_Temp_Valid_Data(GebraBit_ICM20789 * ICM20789)
{ 
  ICM20789->VALID_TEMP_DATA =(ICM20789->REGISTER_RAW_TEMP / 333.333 )+ 25-ROOM_TEMPERATURE_OFFSET ;///25 - 8 PCS OFSET!!!
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis GYRO from Register 
 * @param     ICM20789  store Raw Data Of X Axis GYRO DATA in GebraBit_ICM20789 Staruct REGISTER_RAW_GYRO_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_GYRO_X_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789)
{
	uint8_t gyrox_msb , gyrox_lsb;
  GB_ICM20789_Read_Reg_Data( ICM20789_GYRO_XOUT_H, &gyrox_msb);
	GB_ICM20789_Read_Reg_Data( ICM20789_GYRO_XOUT_L, &gyrox_lsb );
	ICM20789->REGISTER_RAW_GYRO_X = (int16_t)((gyrox_msb << 8) | gyrox_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis GYRO from Register 
 * @param     ICM20789  store Raw Data Of Y Axis GYRO DATA in GebraBit_ICM20789 Staruct REGISTER_RAW_GYRO_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_GYRO_Y_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789)
{
	uint8_t gyroy_msb , gyroy_lsb;
  GB_ICM20789_Read_Reg_Data( ICM20789_GYRO_YOUT_H, &gyroy_msb);
	GB_ICM20789_Read_Reg_Data( ICM20789_GYRO_YOUT_L, &gyroy_lsb );
	ICM20789->REGISTER_RAW_GYRO_Y = (int16_t)((gyroy_msb << 8) | gyroy_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis GYRO from Register 
 * @param     ICM20789  store Raw Data Of Z Axis GYRO DATA in GebraBit_ICM20789 Staruct REGISTER_RAW_GYRO_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_GYRO_Z_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789)
{
	uint8_t gyroz_msb , gyroz_lsb;
  GB_ICM20789_Read_Reg_Data( ICM20789_GYRO_ZOUT_H, &gyroz_msb);
	GB_ICM20789_Read_Reg_Data( ICM20789_GYRO_ZOUT_L, &gyroz_lsb );
	ICM20789->REGISTER_RAW_GYRO_Z = (int16_t)((gyroz_msb << 8) | gyroz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis GYRO Base on GebraBit_ICM20789 Staruct SCALE_FACTOR 
 * @param     ICM20789  store Valid Data Of X Axis GYRO in GebraBit_ICM20789 Staruct VALID_GYRO_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_GYRO_DATA_X_Valid_Data(GebraBit_ICM20789 * ICM20789)
{
  ICM20789->VALID_GYRO_DATA_X =(ICM20789->REGISTER_RAW_GYRO_X /ICM20789->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis GYRO Base on GebraBit_ICM20789 Staruct SCALE_FACTOR 
 * @param     ICM20789  store Valid Data Of Y Axis GYRO in GebraBit_ICM20789 Staruct VALID_GYRO_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_GYRO_DATA_Y_Valid_Data(GebraBit_ICM20789 * ICM20789)
{
  ICM20789->VALID_GYRO_DATA_Y =(ICM20789->REGISTER_RAW_GYRO_Y /ICM20789->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis GYRO Base on GebraBit_ICM20789 Staruct SCALE_FACTOR 
 * @param     ICM20789  store Valid Data Of Z Axis GYRO in GebraBit_ICM20789 Staruct VALID_GYRO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_GYRO_DATA_Z_Valid_Data(GebraBit_ICM20789 * ICM20789)
{
  ICM20789->VALID_GYRO_DATA_Z =(ICM20789->REGISTER_RAW_GYRO_Z /ICM20789->PRECISE_GYRO_SF);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis ACCEL from Register 
 * @param     ICM20789  store Raw Data Of X Axis ACCEL DATA in GebraBit_ICM20789 Staruct REGISTER_RAW_ACCEL_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_ACCEL_X_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789)
{
	uint8_t accelx_msb , acclx_lsb;
  GB_ICM20789_Read_Reg_Data( ICM20789_ACCEL_XOUT_H, &accelx_msb);
	GB_ICM20789_Read_Reg_Data( ICM20789_ACCEL_XOUT_L, &acclx_lsb );
	ICM20789->REGISTER_RAW_ACCEL_X = (int16_t)((accelx_msb << 8) | acclx_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis ACCEL from Register 
 * @param     ICM20789  store Raw Data Of Y Axis ACCEL DATA in GebraBit_ICM20789 Staruct REGISTER_RAW_ACCEL_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_ACCEL_Y_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789)
{
	uint8_t accely_msb , accly_lsb;
  GB_ICM20789_Read_Reg_Data( ICM20789_ACCEL_YOUT_H, &accely_msb);
	GB_ICM20789_Read_Reg_Data( ICM20789_ACCEL_YOUT_L, &accly_lsb );
	ICM20789->REGISTER_RAW_ACCEL_Y = (int16_t)((accely_msb << 8) | accly_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis ACCEL from Register 
 * @param     ICM20789  store Raw Data Of Z Axis ACCEL DATA in GebraBit_ICM20789 Staruct REGISTER_RAW_ACCEL_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_ACCEL_Z_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789)
{
	uint8_t accelz_msb , acclz_lsb;
  GB_ICM20789_Read_Reg_Data( ICM20789_ACCEL_ZOUT_H, &accelz_msb);
	GB_ICM20789_Read_Reg_Data( ICM20789_ACCEL_ZOUT_L, &acclz_lsb );
	ICM20789->REGISTER_RAW_ACCEL_Z = (int16_t)((accelz_msb << 8) | acclz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis ACCEL Base on GebraBit_ICM20789 Staruct SCALE_FACTOR 
 * @param     ICM20789  store Valid Data Of X Axis ACCEL in GebraBit_ICM20789 Staruct VALID_ACCEL_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_ACCEL_DATA_X_Valid_Data(GebraBit_ICM20789 * ICM20789)
{
	float scale_factor = ICM20789->ACCEL_SCALE_FACTOR;
  ICM20789->VALID_ACCEL_DATA_X =(ICM20789->REGISTER_RAW_ACCEL_X /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis ACCEL Base on GebraBit_ICM20789 Staruct SCALE_FACTOR 
 * @param     ICM20789  store Valid Data Of Y Axis ACCEL in GebraBit_ICM20789 Staruct VALID_ACCEL_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_ACCEL_DATA_Y_Valid_Data(GebraBit_ICM20789 * ICM20789)
{
	float scale_factor = ICM20789->ACCEL_SCALE_FACTOR;
  ICM20789->VALID_ACCEL_DATA_Y =(ICM20789->REGISTER_RAW_ACCEL_Y /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis ACCEL Base on GebraBit_ICM20789 Staruct SCALE_FACTOR 
 * @param     ICM20789  store Valid Data Of Z Axis ACCEL in GebraBit_ICM20789 Staruct VALID_ACCEL_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_ACCEL_DATA_Z_Valid_Data(GebraBit_ICM20789 * ICM20789)
{
	float scale_factor = ICM20789->ACCEL_SCALE_FACTOR;
  ICM20789->VALID_ACCEL_DATA_Z =(ICM20789->REGISTER_RAW_ACCEL_Z /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Temprature Directly 
 * @param     ICM20789       GebraBit_ICM20789 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_Temperature(GebraBit_ICM20789 * ICM20789)
{
  GB_ICM20789_Get_Temp_Register_Raw_Data  (ICM20789);
	GB_ICM20789_Get_Temp_Valid_Data(ICM20789);
}
/*=========================================================================================================================================
 * @brief     Get XYZ GYROSCOPE Directly 
 * @param     ICM20789       GebraBit_ICM20789 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_XYZ_GYROSCOPE(GebraBit_ICM20789 * ICM20789)
{
	GB_ICM20789_Get_GYRO_X_Register_Raw_DATA(ICM20789);
	GB_ICM20789_Get_GYRO_DATA_X_Valid_Data(ICM20789);
	GB_ICM20789_Get_GYRO_Y_Register_Raw_DATA(ICM20789);
	GB_ICM20789_Get_GYRO_DATA_Y_Valid_Data(ICM20789);
	GB_ICM20789_Get_GYRO_Z_Register_Raw_DATA(ICM20789);
	GB_ICM20789_Get_GYRO_DATA_Z_Valid_Data(ICM20789);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION Directly 
 * @param     ICM20789       GebraBit_ICM20789 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_XYZ_ACCELERATION(GebraBit_ICM20789 * ICM20789)
{
	GB_ICM20789_Get_ACCEL_X_Register_Raw_DATA(ICM20789);
	GB_ICM20789_Get_ACCEL_DATA_X_Valid_Data(ICM20789);
	GB_ICM20789_Get_ACCEL_Y_Register_Raw_DATA(ICM20789);
	GB_ICM20789_Get_ACCEL_DATA_Y_Valid_Data(ICM20789);
	GB_ICM20789_Get_ACCEL_Z_Register_Raw_DATA(ICM20789);
	GB_ICM20789_Get_ACCEL_DATA_Z_Valid_Data(ICM20789);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION and GYROSCOPE and Temprature Directly From Registers
 * @param     ICM20789       GebraBit_ICM20789 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_ACCEL_GYRO_TEMP_From_Registers(GebraBit_ICM20789 * ICM20789)
{
  if (IS_Ready==GB_ICM20789_Check_Data_Preparation(ICM20789))
	 {
		 ICM20789->GET_DATA =  FROM_REGISTER ; 
	   GB_ICM20789_Get_Temperature( ICM20789 );
	   GB_ICM20789_Get_XYZ_ACCELERATION( ICM20789);
		 GB_ICM20789_Get_XYZ_GYROSCOPE( ICM20789);
	 }
}
/*=========================================================================================================================================
 * @brief     Separate XYZ ACCELERATION , GYROSCOPE and Temprature Data From FIFO and caculate Valid data
 * @param     ICM20789  store Valid Data Of XYZ ACCEL Axis and temp from FIFO TO GebraBit_ICM20789 Staruct VALID_FIFO_DATA_X , VALID_FIFO_DATA_Y ,VALID_FIFO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20649_FIFO_Data_Partition_ACCEL_GYRO_XYZ_TEMP(GebraBit_ICM20789 * ICM20789)
{
	uint16_t i,offset=0;
  float accel_scale_factor = ICM20789->ACCEL_SCALE_FACTOR;
	for ( i = 0 ; i < PACKET_QTY_IN_FULL_FIFO ; i++ )
	{
		if ( (ICM20789->TEMP_TO_FIFO == Enable ) && ( ICM20789->ACCEL_TO_FIFO == Enable ) && ( ICM20789->GYRO_TO_FIFO == Enable ) )
		{
			ICM20789->VALID_FIFO_ACCEL_X[i] = ((int16_t)( (ICM20789->FIFO_DATA[offset] << 8) | ICM20789->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2; 
			ICM20789->VALID_FIFO_ACCEL_Y[i] = ((int16_t)( (ICM20789->FIFO_DATA[offset] << 8) | ICM20789->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20789->VALID_FIFO_ACCEL_Z[i] = ((int16_t)( (ICM20789->FIFO_DATA[offset] << 8) | ICM20789->FIFO_DATA[offset+1]))/accel_scale_factor ;
			offset += 2;
			ICM20789->VALID_FIFO_TEMP[i]    = (((int16_t)( (ICM20789->FIFO_DATA[offset] << 8)| ICM20789->FIFO_DATA[offset+1]))/ 333.333) + 25-ROOM_TEMPERATURE_OFFSET ;
			offset += 2;
			ICM20789->VALID_FIFO_GYRO_X[i]  = ((int16_t)( (ICM20789->FIFO_DATA[offset] << 8) | ICM20789->FIFO_DATA[offset+1]))/ICM20789->PRECISE_GYRO_SF ;
			offset += 2; 
			ICM20789->VALID_FIFO_GYRO_Y[i]  = ((int16_t)( (ICM20789->FIFO_DATA[offset] << 8) | ICM20789->FIFO_DATA[offset+1]))/ICM20789->PRECISE_GYRO_SF ;
			offset += 2;
			ICM20789->VALID_FIFO_GYRO_Z[i]  = ((int16_t)( (ICM20789->FIFO_DATA[offset] << 8) | ICM20789->FIFO_DATA[offset+1]))/ICM20789->PRECISE_GYRO_SF ;
			offset += 2;
		}
	}
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION , GYRO and Temprature From FIFO
 * @param     ICM20789       GebraBit_ICM20789 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_ACCEL_GYRO_TEMP_From_FIFO(GebraBit_ICM20789 * ICM20789)
{
	  

	if (IS_Ready==GB_ICM20789_Check_Data_Preparation(ICM20789))
	{
		  if (FIFO_IS_OVERFLOW==GB_ICM20789_Check_FIFO_Overflow(ICM20789))
		  {
        GB_ICM20789_GET_FIFO_Count(ICM20789);
				GB_ICM20789_Read_FIFO(ICM20789,FIFO_DATA_BUFFER_SIZE);
				GB_ICM20649_FIFO_Data_Partition_ACCEL_GYRO_XYZ_TEMP(ICM20789); 
        //memset(ICM20789->FIFO_DATA , 0, FIFO_DATA_BUFFER_SIZE*sizeof(uint8_t));				
				GB_ICM20789_FIFO_Reset();
				ICM20789->GET_DATA =  FROM_FIFO ;
		  } 
	}	
}
/*=========================================================================================================================================
 * @brief     Get Data From ICM20789
 * @param     ICM20789       GebraBit_ICM20789 Staruct
 * @param     get_data       Determine Method of reading data from sensoe : FROM_REGISTER or FROM_FIFO
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_ICM20789_Get_Data(GebraBit_ICM20789 * ICM20789 , ICM20789_Get_DATA get_data)
{
 if( (get_data == FROM_REGISTER)&&(ICM20789->FIFO == Disable) )
	 GB_ICM20789_Get_ACCEL_GYRO_TEMP_From_Registers(ICM20789);
 else if ((get_data == FROM_FIFO)&&(ICM20789->FIFO == Enable)) 
	GB_ICM20789_Get_ACCEL_GYRO_TEMP_From_FIFO(ICM20789); 
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/