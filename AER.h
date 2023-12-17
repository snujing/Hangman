/*	Purpose : AER(ARM Embedded RTOS) Project
	Target Name : Cortex-M0 NUC130 RTOS-T37
	Compiler : MDK-ARM Compiler V5
	CMSIS : NUC130_CMSIS302_LIBRARY
	Document : ARM_Embedded_RTOS_읽어두기.txt
	Information : https://cafe.naver.com/fws
	---------------------------------------------------------------
	Copyright by FirmwareBank Inc,. All Rights Reserved. */
	
#ifndef __AER_H__
#define __AER_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "NUC100Series.h"

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif	

#define QSIZE 120	

// runflag define
#define F_IN8_BUT 		0x80000000
#define F_IN15_BUT 		0x40000000

#define F_STEP_MOT_ON	0x08000000
#define F_STEP_MOT_CW	0x04000000
#define F_STEP_MOT_CCW	0x02000000

// Running 
#define F_TIME_UART0	0x00008000
#define F_TIME_UART1	0x00004000
#define F_OK_PROTOCOL	0x00002000

// timer
#define TM_1SEC		1
#define TM_2SEC		2
#define TM_3SEC		3	
#define TM_4SEC		4
#define TM_5SEC		5
#define TM_10SEC	10
#define TM_20SEC	20
#define TM_30SEC	30
#define TM_40SEC	40

#define TM_10mS		1
#define TM_20mS		2
#define TM_50mS		5
#define TM_100mS	10
#define TM_200mS	20	
#define TM_500mS	50
#define TM_1000mS	100
#define TM_2000mS	200
#define TM_3000mS	300

// Hardware, OUT
#define LEDR(bon)		(PA6 = (bon)? 0 : 1)	
#define LEDG(bon)		(PA7 = (bon)? 0 : 1)	
#define	BUZ(OOn)		(PA5 = (OOn)? 0 : 1)	

#define	EX2_PORT4(OOn)		(PC7 = (OOn)? 1 : 0)	
#define	EX2_PORT3(OOn)		(PA13 = (OOn)? 1 : 0)	

#define	SMOT_RESET(OOn)		(PC6 = (OOn)? 1 : 0)	
#define	SMOT_Sleep(OOn)		(PB3 = (OOn)? 1 : 0)	
#define	SMOT_STEP_PWM(OOn)	(PA15 = (OOn)? 1 : 0)	
#define	SMOT_DIR(OOn)		(PB2 = (OOn)? 1 : 0)	

#define	SE16_OE(OOn)	(PB12 = (OOn)? 0 : 1)	// Low Active
//#define	PWM_OEF(OOn)	(PB12 = (OOn)? 0 : 1)	// Low Active

#define SMOT_STEP_PWM_PORT	(PA15)
#define BUZZER		(PA5)

// Hardware, IN
#define IN8 	(PB8)	// 버튼 8 
#define IN15 	(PB15)	// 버튼 15 

#endif
