#ifndef __PCA9685_H__
#define	__PCA9685_H__

//PCA9685_I2CADD
#define PCA9685_I2CADD	0x40	// no extended Pin
  
#define SERVO_TRIM_MIN 110
#define SERVO_TRIM_MAX 590

#define OSC_FREQ 25000000UL // 내부 클럭 사용

#define MODE1 0x00
#define MODE1_ALLCALL_BIT 0
#define MODE1_ALLCALL_MASK ~(1 << MODE1_ALLCALL_BIT)
#define MODE1_SUB3_BIT 1
#define MODE1_SUB3_MASK ~(1 << MODE1_SUB3_BIT)
#define MODE1_SUB2_BIT 2
#define MODE1_SUB2_MASK ~(1 << MODE1_SUB2_BIT)
#define MODE1_SUB1_BIT 3
#define MODE1_SUB1_MASK ~(1 << MODE1_SUB1_BIT)
#define MODE1_SLEEP_BIT 4
#define MODE1_SLEEP_MASK ~(1 << MODE1_SLEEP_BIT)
#define MODE1_AI_BIT 5
#define MODE1_AI_MASK ~(1 << MODE1_AI_BIT)
#define MODE1_EXTCLK_BIT 6
#define MODE1_EXTCLK_MASK ~(1 << MODE1_EXTCLK_BIT)
#define MODE1_RESTART_BIT 7
#define MODE1_RESTART_MASK ~(1 << MODE1_RESTART_BIT)

#define MODE2 0x01
#define MODE2_OUTNE_BIT 0
#define MODE2_OUTNE_MASK ~(3 << MODE2_OUTNE_BIT)
#define MODE2_OUTDRV_BIT 2
#define MODE2_OUTDRV_MASK ~(1 << MODE2_OUTDRV_BIT)
#define MODE2_OCH_BIT 3
#define MODE2_OCH_MASK ~(1 << MODE2_OCH_BIT)
#define MODE2_INVRT_BIT 4
#define MODE2_INVRT_MASK ~(1 << MODE2_INVRT_BIT)
#define OCH_ACK 1
#define OCH_STOP 0
#define OUTDRV_TOTEM_POLE 1
#define OUTDRV_OPEN_DRAIN 1
#define OUTNE_LOW 0
#define OUTNE_HIGH 1
#define OUTNE_HIGH_IMPEDANCE 2

#define SUBADR1 0x02
#define SUBADR2 0x03
#define SUBADR3 0x04
#define ALLCALLADR 0x05
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define ALL_LED_ON_L 0xFA
#define ALL_LED_ON_H 0xFB
#define ALL_LED_OFF_L 0xFC
#define ALL_LED_OFF_H 0xFD
#define PRE_SCALE 0xFE

#define SERVO 50
#define DEFAULT_MODE 0
#define SERVO_MODE 1

// PCA9685
void HCPCA9685_Init(unsigned char Mode); // 50hz servo only
void HCPCA9685_Sleep(unsigned char Mode);
void HCPCA9685_SetPeriodFreq(unsigned int Freq); 
void HCPCA9685_SetPreScaller(unsigned char Period);
void HCPCA9685_Servo(unsigned char Chan, unsigned int Pos);
void HCPCA9685_Output3(unsigned char Chan, unsigned int On_Time, unsigned int Off_Time);
void HCPCA9685_Output2(unsigned int On_Time, unsigned int Off_Time);
void HCPCA9685_OutputOnTime(unsigned char Chan, unsigned int Time);
void HCPCA9685_OutputOffTime(unsigned char Chan, unsigned int Time);
void HCPCA9685_OutputNotEnableState(unsigned char State);
void HCPCA9685_OutputDrivers(unsigned char Mode);
void HCPCA9685_OCH(unsigned char Mode);
void HCPCA9685_Invert(unsigned char Mode);
void HCPCA9685_Enable_Sub1(unsigned char Mode);
void HCPCA9685_Enable_Sub2(unsigned char Mode);
void HCPCA9685_Enable_Sub3(unsigned char Mode);
void HCPCA9685_Enable_AllCall(unsigned char Mode);
void HCPCA9685_SetSubAddress(unsigned char SubAddress, unsigned char Address);
void HCPCA9685_SetAllCallAddress(unsigned char Address);
void HCPCA9685_ExtClk(void);
unsigned char HCPCA9685_I2CReadReg(unsigned char Register);
void HCPCA9685_AutoIncrement(unsigned char Mode);

void HCPCA9685_dgree(unsigned char Ch, unsigned long dgree);

#endif

