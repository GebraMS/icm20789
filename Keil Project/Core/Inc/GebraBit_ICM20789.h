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
#ifndef	_ICM20789__H_
#define	_ICM20789__H_
#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "spi.h"
#include "string.h"
#include "math.h"

/************************************************
 *         USER BANK 0 REGISTER MAP             *
 ***********************************************/ 
#define ICM20789_SELF_TEST_X_GYRO           (0x00)
#define ICM20789_SELF_TEST_Y_GYRO           (0x01)
#define ICM20789_SELF_TEST_Z_GYRO           (0x02)
#define ICM20789_SELF_TEST_X_ACCEL          (0x0D)
#define ICM20789_SELF_TEST_Y_ACCEL          (0x0E)
#define ICM20789_SELF_TEST_Z_ACCEL          (0x0F)
#define ICM20789_XG_OFFS_USRH               (0x13)
#define ICM20789_XG_OFFS_USRL        				(0x14)
#define ICM20789_YG_OFFS_USRH        				(0x15)
#define ICM20789_YG_OFFS_USRL        				(0x16)
#define ICM20789_ZG_OFFS_USRH      					(0x17)
#define ICM20789_ZG_OFFS_USRL          			(0x18)
#define ICM20789_SMPLRT_DIV        					(0x19)
#define ICM20789_CONFIG        							(0x1A)
#define ICM20789_GYRO_CONFIG        				(0x1B)
#define ICM20789_ACCEL_CONFIG         			(0x1C)
#define ICM20789_ACCEL_CONFIG2         			(0x1D)
#define ICM20789_LP_MODE_CTRL        				(0x1E)
#define ICM20789_ACCEL_WOM_X_THR        		(0x20)
#define ICM20789_ACCEL_WOM_Y_THR        		(0x21)
#define ICM20789_ACCEL_WOM_Z_THR        		(0x22)
#define ICM20789_FIFO_EN        						(0x23)
#define ICM20789_INT_PIN_CFG        				(0x37)
#define ICM20789_INT_ENABLE        					(0x38)
#define ICM20789_DMP_INT_STATUS         		(0x39)
#define ICM20789_INT_STATUS         				(0x3A)
#define ICM20789_ACCEL_XOUT_H         			(0x3B)
#define ICM20789_ACCEL_XOUT_L         			(0x3C)
#define ICM20789_ACCEL_YOUT_H         			(0x3D)
#define ICM20789_ACCEL_YOUT_L         			(0x3E)
#define ICM20789_ACCEL_ZOUT_H          			(0x3F)
#define ICM20789_ACCEL_ZOUT_L          			(0x40)
#define ICM20789_TEMP_OUT_H           			(0x41)
#define ICM20789_TEMP_OUT_L           			(0x42)
#define ICM20789_GYRO_XOUT_H            		(0x43)
#define ICM20789_GYRO_XOUT_L           			(0x44)
#define ICM20789_GYRO_YOUT_H         				(0x45)
#define ICM20789_GYRO_YOUT_L         				(0x46)
#define ICM20789_GYRO_ZOUT_H            		(0x47)
#define ICM20789_GYRO_ZOUT_L     						(0x48)
#define ICM20789_SIGNAL_PATH_RESET          (0x68)
#define ICM20789_ACCEL_INTEL_CTRL        		(0x69)
#define ICM20789_USER_CTRL        					(0x6A)
#define ICM20789_PWR_MGMT_1        					(0x6B)
#define ICM20789_PWR_MGMT_2      						(0x6C)
#define ICM20789_FIFO_COUNTH          			(0x72)
#define ICM20789_FIFO_COUNTL        				(0x73)
#define ICM20789_FIFO_R_W        						(0x74)
#define ICM20789_WHO_AM_I        						(0x75)
#define ICM20789_XA_OFFSET_H         				(0x77)
#define ICM20789_XA_OFFSET_L         				(0x78)
#define ICM20789_YA_OFFSET_H       					(0x7A)
#define ICM20789_YA_OFFSET_L        				(0x7B)
#define ICM20789_ZA_OFFSET_H        				(0x7D)
#define ICM20789_ZA_OFFSET_L        				(0x7E)
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 

/************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
 /************************************************
 *          Register Values Begin                *
 ***********************************************/ 
#define FIFO_ACCEL_DATA_SIZE          6
#define FIFO_GYRO_DATA_SIZE           6
#define FIFO_TEMP_DATA_SIZE           2
#define FIFO_DATA_BUFFER_SIZE         1024
#define BYTE_QTY_IN_ONE_PACKET        14
#define ROOM_TEMPERATURE_OFFSET       4
#define PACKET_QTY_IN_FULL_FIFO       (FIFO_DATA_BUFFER_SIZE/BYTE_QTY_IN_ONE_PACKET)     
//#define INTERNAL_SAMPLE_RATE          1000
#define SAMPLE_RATE_ODR_HZ              500
/*----------------------------------------------*
 *           Register Values End                *
 *----------------------------------------------*/
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/****************************************************
 *    Values For I2C_IF_DIS in USER_CTRL Register   *
 ****************************************************/ 
typedef enum interface
{  
	NOT_SPI = 0     ,                  					 
	IS_SPI                                 					 /* 1: Disable I2C Slave module and put the serial interface in SPI mode only */ 
}ICM20789_Interface;
/******************************************************
 * Values For ACCEL_FS_SEL in ACCEL_CONFIG0  Register *
 ******************************************************/ 
typedef enum accel_fs_sel
{  
	FULL_SCALE_2g = 0  ,                    							  /* 00: ±2g  Full scale select for accelerometer */
	FULL_SCALE_4g      ,                							      /* 01: ±4g  Full scale select for accelerometer */
	FULL_SCALE_8g      ,               								      /* 10: ±8g  Full scale select for accelerometer */
	FULL_SCALE_16g            															/* 11: ±16g Full scale select for accelerometer */
}ICM20789_Accel_Fs_Sel;
/**************************************************
 *         Values For ACCEL Scale_Factor          *
 **************************************************/ 
typedef enum 
{  
	SCALE_FACTOR_16384_LSB_g = 16384    ,                  /* 00: ±2g   scale factor for accelerometer */   
	SCALE_FACTOR_8192_LSB_g  = 8192     ,                  /* 01: ±4g   scale factor for accelerometer */
	SCALE_FACTOR_4096_LSB_g  = 4096     ,                  /* 10: ±8g   scale factor for accelerometer */    
	SCALE_FACTOR_2048_LSB_g  = 2048           						 /* 11: ±16g   scale factor for accelerometer */
}ICM20789_Accel_Scale_Factor;
/******************************************************
 *     Values For FS_SEL in GYRO_CONFIG  Register     *
 ******************************************************/ 
typedef enum gyro_fs_sel
{ 
  FS_250_DPS = 0   ,	                                  /* 00: ±250dps   Full scale select for Gyro */
	FS_500_DPS       ,                    							  /* 01: ±500dps   Full scale select for Gyro */
	FS_1000_DPS      ,                							      /* 10: ±1000dps  Full scale select for Gyro */
	FS_2000_DPS                     								      /* 11: ±2000dps  Full scale select for Gyro */           															
}ICM20789_Gyro_Fs_Sel;
/**************************************************
 *           Values For Gyro Scale_Factor         *
 **************************************************/ 
typedef enum 
{  
	SCALE_FACTOR_131_LSB_DPS   = 131   ,                 /* 00: ±250dps   Full scale factor for Gyro */
	SCALE_FACTOR_65p5_LSB_DPS  = 65    ,                 /* 01: ±500dps   Full scale factor for Gyro */
	SCALE_FACTOR_32p8_LSB_DPS  = 32    ,                 /* 10: ±1000dps  Full scale factor for Gyro */    
	SCALE_FACTOR_16p4_LSB_DPS  = 16             				 /* 11: ±2000dps  Full scale factor for Gyro */
}ICM20789_Gyro_Scale_Factor;
/*************************************************
 *    Values For FIFO_MODE in CONFIG Register    *
 **************************************************/ 
typedef enum FIFO_Size 
{  
	_512_BYTE = 0 ,                                 
	_1_KBYTE  = 1 ,                      
	_2_KBYTE  = 2 ,                       
	_4_KBYTE  = 3 ,                       
}ICM20789_FIFO_Size ;
/*************************************************
 *    Values For FIFO_MODE in CONFIG Register    *
 **************************************************/ 
typedef enum FIFO_Mode 
{  
	STREAM_TO_FIFO = 0 ,                                  /* 0: when the FIFO is full, additional writes will be written to the FIFO,replacing the oldest data. */
	STOP_ON_FULL_FIFO_SNAPSHOT = 1                        /*1: when the FIFO is full, additional writes will not be written to FIFO.    */
}ICM20789_FIFO_Mode ;

/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum 
{  
	Disable = 0     ,                      
	Enable     
}ICM20789_Ability;        
/**************************************************
 *     Values For ACCEL And Gyro POWER_MODE       *
 **************************************************/ 
typedef enum
{
	ICM20789_LOW_NOISE  = 0,        							
	ICM20789_LOW_POWER  = 1, 											                   
	ICM20789_SLEEP_OFF  = 2
} ICM20789_Power_Mode;
/**************************************************************
 *         Values For G_AVGCFG in ALP_MODE_CFG Register       *
 **************************************************************/ 
typedef enum
{
	 GYRO_AVERAGE_1_SAMPLES_FILTER  = 0 ,											/* 000 Average 1   samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_2_SAMPLES_FILTER  = 1 , 										/* 001 Average 2   samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_4_SAMPLES_FILTER  = 2 ,										  /* 010 Average 4   samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_8_SAMPLES_FILTER  = 3 ,                     /* 011 Average 8   samples for Low Power Accelerometer mode           */
	 GYRO_AVERAGE_16_SAMPLES_FILTER = 4 ,											/* 100 Average 16  samples for Low Power Accelerometer mode           */
	 GYRO_AVERAGE_32_SAMPLES_FILTER = 5	,										  /* 101 Average 32  samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_64_SAMPLES_FILTER = 6	,                     /* 110 Average 64  samples for Low Power Accelerometer mode 				  */
	 GYRO_AVERAGE_128_SAMPLES_FILTER= 7	                      /* 111 Average 128 samples for Low Power Accelerometer mode 				  */
} ICM20789_GYRO_Averaging_Filter;
/**************************************************************
 *        Values For DEC2_CFG in ACCEL_CONFIG2 Register       *
 **************************************************************/ 
typedef enum
{
	 ACCEL_AVERAGE_4_SAMPLES_FILTER    = 0 ,                  /* 00 Average 4  samples for Low Power Accelerometer mode 				  */
	 ACCEL_AVERAGE_8_SAMPLES_FILTER    = 1 ,									/* 01 Average 8  samples for Low Power Accelerometer mode 				  */
	 ACCEL_AVERAGE_16_SAMPLES_FILTER   = 2 ,									/* 10 Average 16 samples for Low Power Accelerometer mode 				  */
	 ACCEL_AVERAGE_32_SAMPLES_FILTER   = 3 										/* 11 Average 32 samples for Low Power Accelerometer mode 				  */
} ICM20789_ACCEL_Averaging_Filter;

/*************************************************
 *         Values For Data Preparation           *
 **************************************************/ 
typedef enum 
{  
	IS_NOT_Ready = 0     ,                      
	IS_Ready     
}ICM20789_Preparation;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	DONE     = 0     ,                      
	FAILED   = 1    
}ICM20789_Reset_Status;
/**************************************************
 *       Values For Disable And Enable FIFO       *
 **************************************************/ 
typedef enum FIFO_Ability
{  
	FIFO_DISABLE = 0     ,                      
	FIFO_ENABLE     
} ICM20789_FIFO_Ability;

/**************************************************
 * Values For Methode of getting data from sensor *
 **************************************************/ 
typedef enum Get_DATA
{  
	FROM_REGISTER = 0     ,                      
	FROM_FIFO     
} ICM20789_Get_DATA; 
/**************************************************************
 *        Values For FIFO_LP_EN in PWR_MGMT_2 Register        *
 **************************************************************/ 
typedef enum DMP_LP
{  
	DMP_LOW_POWER = 0     ,                      
	NOT_DMP_LOW_POWER     
} ICM20789_DMP_LP; 
/**************************************************************
 *        Values For SLEEP in  PWR_MGMT_1 Register            *
 **************************************************************/ 
typedef enum sleep
{
	ICM20789_AWAKE   = 0 ,																		
	ICM20789_SLEEP
}ICM20789_Sleep;
/**************************************************************
 *        Values For CLKSEL in PWR_MGMT_1 Register            *
 **************************************************************/ 
typedef enum Clock_Source
{  
	INTERNAL_20MHZ_OSCILLATOR = 0  ,                        /* 000: Internal 20 MHz oscillator. */
	AUTO_SELECT = 1      ,                    							/* 001: Auto selects the best available clock source – PLL if ready, else use the Internal oscillator */
	CLOCK_STOP  = 7                                         /* 110: Stops the clock and keeps timing generator in reset */
}ICM20789_CLK ;
/**************************************************
 *       Values For power off or on a sensor      *
 **************************************************/ 
typedef enum Sensor
{  
	SENSOR_ENABLE   = 0  ,                                      							
	SENSOR_DISABLE  = 7                                     
}ICM20789_Sensor ;
/*******************************************************
 *     Values For INT_LEVEL in INT_PIN_CFG Register    *
 *******************************************************/ 
typedef enum int_level
{  
	ACTIVE_HIGH = 0     ,                      
	ACTIVE_LOW     
} ICM20789_INT_Level; 
/*******************************************************
 *     Values For INT_OPEN in INT_PIN_CFG Register     *
 *******************************************************/ 
typedef enum int_type
{  
	PUSH_PULL = 0     ,                      
	OPEN_DRAIN     
}ICM20789_INT_Type; 
/*******************************************************
 *  Values For LATCH_INT_EN in INT_PIN_CFG Register    *
 *******************************************************/ 
typedef enum latch_type
{  
	_50_US = 0     ,                                         /*0: INT/DRDY pin indicates interrupt pulse’s width is 50us       */
	HELD_STATUS_CLEAR     																	 /*1: INT/DRDY pin level held until interrupt status is cleared    */
} ICM20789_Latch_Type; 
/*******************************************************
 *  Values For FIFO_OFLOW_INT in INT_STATUS Register   *
 *******************************************************/
typedef enum FIFO_Overflow
{  
	FIFO_IS_NOT_OVERFLOW = 0     ,                      
	FIFO_IS_OVERFLOW  = 1     
} ICM20789_FIFO_Overflow; 
/*******************************************************
 *           Values For BYPASS or Enable DLPF          *
 *******************************************************/
typedef enum FCHOICEB
{  
	ENABLE_DLPF_FCHOICEB = 0     ,                      
	BYPASS_DLPF_FCHOICEB = 1     ,                          
}ICM20789_FCHOICEB;
/**************************************************************
 *                 Values For Sensors Sample Rate                 *
 **************************************************************/ 
typedef enum sample_rate
{  
	_1_KHz   = 1000      ,                        /* 000: Internal 20 MHz oscillator. */
	_4_KHz   = 4000      ,                    							/* 001: Auto selects the best available clock source – PLL if ready, else use the Internal oscillator */
	_8_KHz   = 8000      ,                                    /* 110: Stops the clock and keeps timing generator in reset */
	_32_KHz  = 32000
}ICM20789_Sample_Rate ;
/*******************************************************
 *       Values For DLPF_CFG in CONFIG Register        *
 *******************************************************/
typedef enum GYRO_TEMP_DLPF
{
	ICM20789_GYRO_TEMP_DLPF_250     = 0, 									/*!< accel 3-dB BW(Hz) = 250,   Rate(kHz) = 8 */
	ICM20789_GYRO_TEMP_DLPF_176	    = 1,								  /*!< accel 3-dB BW(Hz) = 176,   Rate(kHz) = 1 */
	ICM20789_GYRO_TEMP_DLPF_92		  = 2, 									/*!< accel 3-dB BW(Hz) = 92,    Rate(kHz) = 1 */
	ICM20789_GYRO_TEMP_DLPF_41	  	= 3, 									/*!< accel 3-dB BW(Hz) = 41,    Rate(kHz) = 1 */
	ICM20789_GYRO_TEMP_DLPF_20	  	= 4, 									/*!< accel 3-dB BW(Hz) = 20,    Rate(kHz) = 1 */
	ICM20789_GYRO_TEMP_DLPF_10			= 5, 									/*!< accel 3-dB BW(Hz) = 10,    Rate(kHz) = 1 */
	ICM20789_GYRO_TEMP_DLPF_5	      = 6, 									/*!< accel 3-dB BW(Hz) = 5,     Rate(kHz) = 1 */
	ICM20789_GYRO_TEMP_DLPF_3281		= 7 									/*!< accel 3-dB BW(Hz) = 3281,  Rate(kHz) = 8 */
}ICM20789_GYRO_TEMP_DLPF ;
/*******************************************************
 *   Values For A_DLPF_CFG in ACCEL_CONFIG2 Register   *
 *******************************************************/
typedef enum Accel_DLPF_CFG
{
	ICM20789_ACCEL_DLPF_218	    = 1 ,								  		/*!< accel 3-dB BW(Hz) = 218.1,   Rate(kHz) = 1 */
	ICM20789_ACCEL_DLPF_99	  	= 2, 											/*!< accel 3-dB BW(Hz) = 99,      Rate(kHz) = 1 */
	ICM20789_ACCEL_DLPF_45	  	= 3, 											/*!< accel 3-dB BW(Hz) = 44.8,    Rate(kHz) = 1 */
	ICM20789_ACCEL_DLPF_21	  	= 4, 											/*!< accel 3-dB BW(Hz) = 21.2,    Rate(kHz) = 1 */
	ICM20789_ACCEL_DLPF_10			= 5, 											/*!< accel 3-dB BW(Hz) = 10.2,    Rate(kHz) = 1 */
	ICM20789_ACCEL_DLPF_5		    = 6, 											/*!< accel 3-dB BW(Hz) = 5.1,     Rate(kHz) = 1 */
	ICM20789_ACCEL_DLPF_420 	  = 7 											/*!< accel 3-dB BW(Hz) = 420.0,   Rate(kHz) = 1 */
}ICM20789_ACCEL_DLPF ;

 /*************************************************
 *  Defining ICM20789 Register & Data As Struct   *
 **************************************************/
typedef	struct ICM20789
{
	  uint8_t                       		Register_Cache1;
		uint8_t                       		Register_Cache2;
	  ICM20789_Get_DATA             		GET_DATA;
	  ICM20789_Reset_Status         		RESET;
	  uint8_t                       		WHO_AM_I;
	  ICM20789_Sleep 							  		IS_ICM20789_SLEEP;
	  ICM20789_CLK     									CLOCK_SOURCE;
	  ICM20789_Ability              		DMP;
	  ICM20789_Ability                  TEMPERATURE;
	  ICM20789_Sensor                   GYRO;
	  ICM20789_Gyro_Fs_Sel				      GYRO_FULL_SCALE;
		ICM20789_Gyro_Scale_Factor        GYRO_SCALE_FACTOR;
		float            							    PRECISE_GYRO_SF;
	  ICM20789_Power_Mode 				  	  GYRO_POWER_MODE;
		ICM20789_FCHOICEB              		GYRO_FCHOICEB;
    ICM20789_GYRO_TEMP_DLPF  					GYRO_TEMP_DLPF;		
	  ICM20789_GYRO_Averaging_Filter    GYRO_AVERAGING_FILTER;
	  ICM20789_Sensor                   ACCEL;
		ICM20789_Accel_Fs_Sel				      ACCEL_FULL_SCALE;
		ICM20789_Accel_Scale_Factor       ACCEL_SCALE_FACTOR;
		ICM20789_Power_Mode 				  	  ACCEL_POWER_MODE;
		ICM20789_FCHOICEB                 ACCEL_FCHOICEB; 
		ICM20789_ACCEL_DLPF  					    ACCEL_DLPF;
		ICM20789_ACCEL_Averaging_Filter   ACCEL_AVERAGING_FILTER;	
		ICM20789_Sample_Rate              INTERNAL_SAMPLE_RATE;
		uint16_t                      		SAMPLE_RATE;
	  uint8_t                       		SAMPLE_DEVIDE;
    ICM20789_Ability 							    FIFO_OVERFLOW_INT;
    ICM20789_Ability 							    DATA_READY_INT;	  
		ICM20789_INT_Level                INT_PIN_LEVEL;
  	ICM20789_INT_Type                 INT_PIN_TYPE;
	  ICM20789_Latch_Type               INT_PIN_LATCH;		
		ICM20789_Preparation            	DATA_STATUS;
	  ICM20789_FIFO_Ability        			FIFO;
		ICM20789_Ability              		INTERFACE_ACCESS_FIFO;
		ICM20789_Ability              		TEMP_TO_FIFO;
		ICM20789_Ability									GYRO_TO_FIFO;
		ICM20789_Ability              		ACCEL_TO_FIFO;
		ICM20789_FIFO_Size                FIFO_SIZE;
		ICM20789_FIFO_Mode					  		FIFO_MODE;
		uint16_t                      		FIFO_PACKET_QTY;
		ICM20789_FIFO_Overflow          	FIFO_OVERFLOW;
		uint16_t                      		FIFO_COUNT ;
		int16_t 													REGISTER_RAW_ACCEL_X;
		int16_t 													REGISTER_RAW_ACCEL_Y;
		int16_t														REGISTER_RAW_ACCEL_Z;
		int16_t 													REGISTER_RAW_GYRO_X;
		int16_t 													REGISTER_RAW_GYRO_Y;
		int16_t 													REGISTER_RAW_GYRO_Z;
		int16_t 													REGISTER_RAW_TEMP;
		float 														VALID_ACCEL_DATA_X;
		float 														VALID_ACCEL_DATA_Y;
		float 														VALID_ACCEL_DATA_Z;
		float 														VALID_TEMP_DATA;
		float 														VALID_GYRO_DATA_X;
		float 														VALID_GYRO_DATA_Y;
		float 														VALID_GYRO_DATA_Z;
		uint8_t 													FIFO_DATA[FIFO_DATA_BUFFER_SIZE];
		float														  VALID_FIFO_ACCEL_X[PACKET_QTY_IN_FULL_FIFO];
		float														  VALID_FIFO_ACCEL_Y[PACKET_QTY_IN_FULL_FIFO];
		float 														VALID_FIFO_ACCEL_Z[PACKET_QTY_IN_FULL_FIFO];
		float 														VALID_FIFO_TEMP[PACKET_QTY_IN_FULL_FIFO];
		float														  VALID_FIFO_GYRO_X[PACKET_QTY_IN_FULL_FIFO];
		float														  VALID_FIFO_GYRO_Y[PACKET_QTY_IN_FULL_FIFO];
		float 														VALID_FIFO_GYRO_Z[PACKET_QTY_IN_FULL_FIFO];
}GebraBit_ICM20789;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *Declare Read&Write ICM20789 Register Values Functions *
 ********************************************************/
extern	uint8_t	GB_ICM20789_Read_Reg_Data ( uint8_t regAddr,uint8_t* data);
extern	uint8_t GB_ICM20789_Read_Reg_Bits (uint8_t regAddr,uint8_t start_bit, uint8_t len, uint8_t* data);
extern	uint8_t GB_ICM20789_Burst_Read(uint8_t regAddr,uint8_t *data, uint16_t byteQuantity);
extern	uint8_t GB_ICM20789_Write_Reg_Data(uint8_t regAddr, uint8_t data);
extern	uint8_t	GB_ICM20789_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data);
extern	uint8_t GB_ICM20789_Burst_Write		( uint8_t regAddr,uint8_t *data, 	uint16_t byteQuantity);
/********************************************************
 *       Declare ICM20789 Configuration Functions       *
 ********************************************************/
extern void GB_ICM20789_Soft_Reset ( GebraBit_ICM20789 * ICM20789 );
extern void GB_ICM20789_Who_am_I(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_DMP(GebraBit_ICM20789* ICM20789 ,ICM20789_Ability dmp,ICM20789_DMP_LP dmp_lp);
extern void GB_ICM20789_DMP_Reset(GebraBit_ICM20789* ICM20789 ,ICM20789_Ability rst);
extern void GB_ICM20789_DMP_Interrupt(ICM20789_Ability interrupt);
extern void GB_ICM20789_Sleep_Awake (GebraBit_ICM20789 * ICM20789, ICM20789_Sleep  working  ) ;
extern void GB_ICM20789_ACCEL_Power_Mode(GebraBit_ICM20789* ICM20789 ,ICM20789_Power_Mode pmode);
extern void GB_ICM20789_GYRO_Power_Mode(GebraBit_ICM20789* ICM20789 ,ICM20789_Power_Mode pmode);
extern void GB_ICM20789_Set_Clock_Source(GebraBit_ICM20789 * ICM20789 , ICM20789_CLK clk) ;
extern void GB_ICM20789_Temperature(GebraBit_ICM20789* ICM20789 ,ICM20789_Ability temp);
extern void GB_ICM20789_Accelerometer(GebraBit_ICM20789 * ICM20789 , ICM20789_Sensor accel);
extern void GB_ICM20789_Gyroscope(GebraBit_ICM20789 * ICM20789 , ICM20789_Sensor gyro) ;
extern void GB_ICM20789_Set_INT_Pin(GebraBit_ICM20789 * ICM20789 , ICM20789_INT_Level level ,ICM20789_INT_Type type , ICM20789_Latch_Type latch );
extern ICM20789_Preparation GB_ICM20789_Check_Data_Preparation(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_GYRO_Full_Scale ( GebraBit_ICM20789 * ICM20789 , ICM20789_Gyro_Fs_Sel fs ) ;
extern void GB_ICM20789_GYRO_Low_Pass_Filter  (GebraBit_ICM20789 * ICM20789 ,  ICM20789_FCHOICEB bypass ) ;
extern void GB_ICM20789_GYRO_TEMP_Low_Pass_Filter_Value  (GebraBit_ICM20789 * ICM20789 , ICM20789_GYRO_TEMP_DLPF dlpf );
extern void GB_ICM20789_GYRO_LP_Averaging_Filter  (GebraBit_ICM20789 * ICM20789 , ICM20789_GYRO_Averaging_Filter avg );
extern void GB_ICM20789_GYRO_Output_Sample_Rate (GebraBit_ICM20789 * ICM20789 , uint16_t rate_hz);
extern void GB_ICM20789_ACCEL_Full_Scale ( GebraBit_ICM20789 * ICM20789 , ICM20789_Accel_Fs_Sel fs );
extern void GB_ICM20789_ACCEL_Low_Pass_Filter  (GebraBit_ICM20789 * ICM20789 ,  ICM20789_FCHOICEB bypass );
extern void GB_ICM20789_ACCEL_Low_Pass_Filter_Value  (GebraBit_ICM20789 * ICM20789 , ICM20789_ACCEL_DLPF dlpf );
extern void GB_ICM20789_ACCEL_LP_Averaging_Filter  (GebraBit_ICM20789 * ICM20789 , ICM20789_ACCEL_Averaging_Filter avg );
extern void GB_ICM20789_Output_Sample_Rate (GebraBit_ICM20789 * ICM20789 , uint16_t rate_hz);
extern void GB_ICM20789_FIFO_Overflow_Interrupt_Pin(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability data_ovf_int);
extern void GB_ICM20789_Data_Ready_Interrupt_Pin(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability data_ready_int);
/********************************************************
 *          Declare ICM20789 FIFO Functions             *
 ********************************************************/
extern void GB_ICM20789_Access_Serial_Interface_To_FIFO(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability interface_access_fifo);
extern ICM20789_FIFO_Overflow GB_ICM20789_Check_FIFO_Overflow(GebraBit_ICM20789 * ICM20789) ;
extern void GB_ICM20789_Write_ACCEL_FIFO(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability accel_fifo ) ;
extern void GB_ICM20789_Write_GYRO_FIFO(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability gyro_fifo ) ;
extern void GB_ICM20789_Write_TEMP_FIFO(GebraBit_ICM20789 * ICM20789 , ICM20789_Ability temp_fifo );
extern void GB_ICM20789_FIFO_Size(GebraBit_ICM20789 * ICM20789 , ICM20789_FIFO_Size fifo_size );
extern void GB_ICM20789_FIFO_Mode(GebraBit_ICM20789 * ICM20789 , ICM20789_FIFO_Mode fifo_mode );
extern void GB_ICM20789_FIFO_Reset(void) ;
extern void GB_ICM20789_GET_FIFO_Count (GebraBit_ICM20789 * ICM20789 ) ;
extern void GB_ICM20789_Read_FIFO(GebraBit_ICM20789 * ICM20789 , uint16_t qty);
extern void GB_ICM20789_Get_ACCEL_GYRO_TEMP_From_FIFO(GebraBit_ICM20789 * ICM20789);
/********************************************************
 *          Declare ICM20789 DATA Functions             *
 ********************************************************/
extern void GB_ICM20789_Get_Temp_Register_Raw_Data(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_Temp_Valid_Data(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_GYRO_X_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_GYRO_Y_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_GYRO_Z_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_GYRO_DATA_X_Valid_Data(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_GYRO_DATA_Y_Valid_Data(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_GYRO_DATA_Z_Valid_Data(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_ACCEL_X_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_ACCEL_Y_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_ACCEL_Z_Register_Raw_DATA(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_ACCEL_DATA_X_Valid_Data(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_ACCEL_DATA_Y_Valid_Data(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_ACCEL_DATA_Z_Valid_Data(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_Temperature(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_XYZ_GYROSCOPE(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_XYZ_ACCELERATION(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_ACCEL_GYRO_TEMP_From_Registers(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20649_FIFO_Data_Partition_ACCEL_GYRO_XYZ_TEMP(GebraBit_ICM20789 * ICM20789);
extern void GB_ICM20789_Get_Data(GebraBit_ICM20789 * ICM20789 , ICM20789_Get_DATA get_data);
/********************************************************
 *          Declare ICM20789 HIGH LEVEL Functions       *
 ********************************************************/
extern void GB_ICM20789_FIFO_Configuration ( GebraBit_ICM20789 * ICM20789 , ICM20789_FIFO_Ability fifo );
extern void GB_ICM20789_Set_Power_Management(GebraBit_ICM20789 * ICM20789 , ICM20789_Power_Mode pmode) ;
extern void GB_ICM20789_initialize( GebraBit_ICM20789 * ICM20789 );
extern void GB_ICM20789_Configuration(GebraBit_ICM20789 * ICM20789, ICM20789_FIFO_Ability fifo);

#endif
