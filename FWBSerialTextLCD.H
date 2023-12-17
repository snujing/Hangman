/* 	Purpose : FWB Serial TextLCD Module Utility
	Target Name : FWB Serial TextLCD 
	Compiler : MDK ARM Compiler
	CMSIS : Mini51DE_CMSIS_V301002
	Document : FWB Serial TextLCD 메뉴얼 
---------------------------------------------
Copyright by FirmwareBank Inc,. All Rights Reserved. */

#ifndef __FWBLCD_H__
#define __FWBLCD_H__

#include <stdint.h>
#include <stdio.h>

#include "NUC100Series.h"

// Module LCD function
void Module_LCD_goto_xy(unsigned char x, unsigned char y);
void textLCD_string(char *lcdstring16);
void uart_LCDsend(unsigned char send1byte);

// FWB Serial TextLCD Protocol
#define LCD_FIRST 		0xFB
#define LCD_Clear 		0xC0
#define LCD_HOME 		0xC2
#define LCD_CON_MINUS	0xD0
#define LCD_CON_PLUS 	0xD1
#define LCD_BACK_ON 	0xD3
#define LCD_BACK_OFF	0xD2

/*
“FWB Serial TextLCD” 처음(전원공급 후 최초) 명령  : 
- Format : 1111 1011(고정), 0xFB
- 모듈의 최초 혼선 방지를 위한 제어코드
- 예) 0xFB : “FWB Serial TextLCD” 활성화 됨  
- 예) 0xFB 0x90 : 활성화 되면서 커서는 (0,1)로 이동  
- 예) 0xFB 0x90 0x41 0x42 ‘C’ : 활성화 되면서 커서 (0,1) 이동, 그리고 ABC 출력   

LCD 커서 위치 명령 : 
- Format : 10yy xxxx(6비트 커서 위치 x, y 정보)   
- 출력 커서 위치 이동 (x, y), 구간 값, x=0~15, y=1~2 
- 예) 0x90 : 1001 0000, 커서는 (0,1)로 이동  
- 예) 0xAF : 1010 1111, 커서는 (15,2)로 이동  

ASCII 문자 출력 전송 :
- Format : 0qwe rtyu(7비트 ASCII 문자)
- ASCII Code 범위 : 0x01 ~ 0x7F
- 예) 0x31 : 0011 0001, LCD에 숫자 ‘1’ 출력
- 예) 0x41 : 0100 0001, LCD에 대문자 ‘A’ 출력

LCD 제어 명령 :
- Format : 11bc defg(6비트 LCD 제어)
- 0xC0 : 1100 0000, LCD 화면 Clear, 화면을 모두 지우고 커서를 홈(0,1)에 둔다.
- 0xC2 : 1100 0010, 커서를 홈(0,1)으로 이동한다.
- 0xC6 : 1100 0110, 커서 자동증가
- 0xC4 : 1100 0100, 커서 자동감소
- 0xC7 : 1100 0111, 커서 자동증가, 커서는 현 위치에 정지하고 화면만 이동
- 0xC5 : 1100 0101, 커서 자동감소, 커서는 현 위치에 정지하고 화면만 이동
- 0xC8 : 1100 1000, 커서 기능 모두 해지
- 0xC9 : 1100 1001, 커서 놓인 문자 깜빡이는 점렬
- 0xCA : 1100 1010, 커서 보임
- 0xCB : 1100 1011, 커서 보임, 커서 점렬
- 0xCC : 1100 1100, LCD 전체 보임
- 0xCD : 1100 1101, LCD 전체 보임, 커서 점렬
- 0xCE : 1100 1110, LCD 전체 보임, 커서 보임 
- 0xCF : 1100 1111, LCD 전체 보임, 커서 보임, 커서 점렬
- 0xD0 : 1101 0000, LCD Contrast ?1, 감소, 문자 더 어두워짐
- 0xD1 : 1101 0001, LCD Contrast +1, 증가, 문자 더 밝아짐 
- 0xD2 : 1101 0010, Back Light OFF
- 0xD3 : 1101 0011, Back Light ON
- 0xDF : LCD 모듈 System Reset, 처음 전원 공급때와 동일하게 시작한다. 모듈에 있는 MCU는 개별적으로 동작되기 때문에 전원 공급을 다시 하기 전에는 과거의 정보를 가지고 있다.  
*/

#endif
