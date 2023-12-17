/*	Purpose : AER(ARM Embedded RTOS) Project 
	Target Name : Cortex-M0 NUC130 RTOS-T37
	Compiler : MDK-ARM Compiler V5
	CMSIS : NUC130_CMSIS302_LIBRARY
	Document : 
	- PCA9685 modul 정의
	Information : https://cafe.naver.com/fws
	책이름 : ARM 임베디드 RTOS 소스코드
	보드이름 : Cortex-M0 NUC130 RTOS
	---------------------------------------------------------------
	Copyright by FirmwareBank Inc,. All Rights Reserved. */

#include "AER.h"
#include "PCA9685modul.h"

// I2C
extern void I2C_MasterRx(uint32_t u32Status);
extern void I2C_MasterTx(uint32_t u32Status);
extern int32_t I2C1_PCARead_Write_SLAVE(uint8_t slvaddr);	
extern int32_t I2C1_PCARead_(uint8_t reg_addr); // PCA9685 read
extern int32_t I2C1_PCAWrite_(uint8_t reg_addr, uint8_t write_data); // PCA9685 Write
extern int32_t I2C1_PCA3B_Write_(uint8_t reg_addr, unsigned int write_2BYTE);

void HCPCA9685_Init(unsigned char Mode){  
	HCPCA9685_Sleep(0);
	HCPCA9685_AutoIncrement(true); 
	if(Mode == SERVO_MODE){
		HCPCA9685_OutputOnTime(0, 0);
		HCPCA9685_OutputOffTime(0, 0);
		HCPCA9685_SetPeriodFreq(50);//50Hz(20ms) for servo motor
	}
}

void HCPCA9685_Sleep(unsigned char Mode){
  unsigned char Data = I2C1_PCARead_(MODE1);
		
  Data = (Data & MODE1_SLEEP_MASK) | (Mode << MODE1_SLEEP_BIT);
  (void) I2C1_PCAWrite_(MODE1, Data);

  if(Data & ~MODE1_RESTART_MASK)  {
	CLK_SysTickDelay(500);
    Data |= (1 << MODE1_RESTART_BIT);
  }
}

void HCPCA9685_SetPeriodFreq(unsigned int Freq){ // 24 (24Hz) to 1526 (1.526KHz)
	
	unsigned char Data = (OSC_FREQ / (4096 * Freq)) - 1;
	(void) I2C1_PCAWrite_(PRE_SCALE, Data);
}

void HCPCA9685_SetPreScaller(unsigned char Period){
	(void) I2C1_PCAWrite_(PRE_SCALE, Period);
}
  
void HCPCA9685_Servo(unsigned char Chan, unsigned int Pos){
  Pos += SERVO_TRIM_MIN; // +110
  if (Pos > SERVO_TRIM_MAX) // >590
    Pos = SERVO_TRIM_MAX;
  HCPCA9685_Output3(Chan, 0 /*Offset*/, Pos /*+ Offset*/);
}

void HCPCA9685_Output3(unsigned char Chan, unsigned int On_Time, unsigned int Off_Time){
  if(Chan > 15)
    Chan = 15;

  Chan = LED0_ON_L + (Chan << 2); 
  (void)I2C1_PCA3B_Write_(Chan, On_Time); // PCA9685 Write	
  (void)I2C1_PCA3B_Write_(Chan+2, Off_Time); // PCA9685 Write	    
}

void HCPCA9685_dgree(unsigned char Ch, unsigned long dgree){
	if(Ch > 15)
		Ch = 15;

	Ch = LED0_ON_L + (Ch << 2);
	dgree = ((2.16 * (float)dgree)) + 122.88;
	
	  if (dgree > 2050) // 
    dgree = 2050; 
	(void)I2C1_PCA3B_Write_(Ch+2, dgree); // PCA9685 Write	 
}	

void HCPCA9685_Output2(unsigned int On_Time, unsigned int Off_Time){	
	(void)I2C1_PCA3B_Write_(ALL_LED_ON_L, On_Time); // PCA9685 Write	  	
	(void)I2C1_PCA3B_Write_(ALL_LED_ON_L + 2, Off_Time); // PCA9685 Write	
}

void HCPCA9685_OutputOnTime(unsigned char Chan, unsigned int Time){
  if(Chan > 15)
    Chan = 15;
  Chan = LED0_ON_L + (Chan << 2);
  
	(void)I2C1_PCA3B_Write_(Chan, Time); // PCA9685 Write	    
}

void HCPCA9685_OutputOffTime(unsigned char Chan, unsigned int Time){
  if(Chan > 15)
    Chan = 15;
  Chan = LED0_OFF_L + (Chan << 2);  
  	(void)I2C1_PCA3B_Write_(Chan, Time); // PCA9685 Write	 
}

void HCPCA9685_OutputNotEnableState(unsigned char State){
	unsigned char Data = I2C1_PCARead_(MODE2);
	Data = (Data & MODE2_OUTNE_MASK) | (State << MODE2_OUTNE_BIT);
	(void)I2C1_PCAWrite_(MODE2, Data); // PCA9685 Write
}

void HCPCA9685_OutputDrivers(unsigned char Mode){
	unsigned char Data = I2C1_PCARead_(MODE2);
	Data = (Data & MODE2_OUTDRV_MASK) | (Mode << MODE2_OUTDRV_BIT);
	(void)I2C1_PCAWrite_(MODE2, Data); // PCA9685 Write	
}

void HCPCA9685_OCH(unsigned char Mode){
	unsigned char Data = I2C1_PCARead_(MODE2);
	Data = (Data & MODE2_OCH_MASK) | (Mode << MODE2_OCH_BIT);
	(void)I2C1_PCAWrite_(MODE2, Data); // PCA9685 Write
}

void HCPCA9685_Invert(unsigned char Mode){
	unsigned char Data = I2C1_PCARead_(MODE2);
	Data = (Data & MODE2_INVRT_MASK) | (Mode << MODE2_INVRT_BIT);
	(void)I2C1_PCAWrite_(MODE2, Data); // PCA9685 Write
}

void HCPCA9685_Enable_Sub1(unsigned char Mode){
	unsigned char Data = I2C1_PCARead_(MODE1);
	Data = (Data & MODE1_SUB1_MASK) | (Mode << MODE1_SUB1_BIT);
	(void)I2C1_PCAWrite_(MODE1, Data); // PCA9685 Write
}

void HCPCA9685_Enable_Sub2(unsigned char Mode){
	unsigned char Data = I2C1_PCARead_(MODE1);
	Data = (Data & MODE1_SUB2_MASK) | (Mode << MODE1_SUB2_BIT);
	(void)I2C1_PCAWrite_(MODE1, Data); // PCA9685 Write	
}

void HCPCA9685_Enable_Sub3(unsigned char Mode){
	unsigned char Data = I2C1_PCARead_(MODE1);
	Data = (Data & MODE1_SUB3_MASK) | (Mode << MODE1_SUB3_BIT);
	(void)I2C1_PCAWrite_(MODE1, Data); // PCA9685 Write
}

void HCPCA9685_Enable_AllCall(unsigned char Mode){
	unsigned char Data = I2C1_PCARead_(MODE1);
	Data = (Data & MODE1_ALLCALL_MASK) | (Mode << MODE1_ALLCALL_BIT);
	(void)I2C1_PCAWrite_(MODE1, Data); // PCA9685 Write	
}

void HCPCA9685_SetSubAddress(unsigned char SubAddress, unsigned char Address){
	(void)I2C1_PCAWrite_(SubAddress, ((0x7F & Address) << 1)); // PCA9685 Write
}

void HCPCA9685_SetAllCallAddress(unsigned char Address){
	(void)I2C1_PCAWrite_(ALLCALLADR, ((0x7F & Address) << 1)); // PCA9685 Write
}

void HCPCA9685_ExtClk(void){
	unsigned char Data;	
	HCPCA9685_Sleep(1);
  
  Data = I2C1_PCARead_(MODE1);
  Data = Data | (1 << MODE1_EXTCLK_BIT) | (1 << MODE1_SLEEP_BIT);
  (void)I2C1_PCAWrite_(MODE1, Data); // PCA9685 Write	
}

void HCPCA9685_AutoIncrement(unsigned char Mode){
  unsigned char Data = I2C1_PCARead_(MODE1);
  Data = (Data & MODE1_AI_MASK) | (Mode << MODE1_AI_BIT);
	(void)I2C1_PCAWrite_(MODE1, Data); // PCA9685 Write
}
