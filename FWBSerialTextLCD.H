/* 	Purpose : FWB Serial TextLCD Module Utility
	Target Name : FWB Serial TextLCD 
	Compiler : MDK ARM Compiler
	CMSIS : Mini51DE_CMSIS_V301002
	Document : FWB Serial TextLCD �޴��� 
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
��FWB Serial TextLCD�� ó��(�������� �� ����) ����  : 
- Format : 1111 1011(����), 0xFB
- ����� ���� ȥ�� ������ ���� �����ڵ�
- ��) 0xFB : ��FWB Serial TextLCD�� Ȱ��ȭ ��  
- ��) 0xFB 0x90 : Ȱ��ȭ �Ǹ鼭 Ŀ���� (0,1)�� �̵�  
- ��) 0xFB 0x90 0x41 0x42 ��C�� : Ȱ��ȭ �Ǹ鼭 Ŀ�� (0,1) �̵�, �׸��� ABC ���   

LCD Ŀ�� ��ġ ���� : 
- Format : 10yy xxxx(6��Ʈ Ŀ�� ��ġ x, y ����)   
- ��� Ŀ�� ��ġ �̵� (x, y), ���� ��, x=0~15, y=1~2 
- ��) 0x90 : 1001 0000, Ŀ���� (0,1)�� �̵�  
- ��) 0xAF : 1010 1111, Ŀ���� (15,2)�� �̵�  

ASCII ���� ��� ���� :
- Format : 0qwe rtyu(7��Ʈ ASCII ����)
- ASCII Code ���� : 0x01 ~ 0x7F
- ��) 0x31 : 0011 0001, LCD�� ���� ��1�� ���
- ��) 0x41 : 0100 0001, LCD�� �빮�� ��A�� ���

LCD ���� ���� :
- Format : 11bc defg(6��Ʈ LCD ����)
- 0xC0 : 1100 0000, LCD ȭ�� Clear, ȭ���� ��� ����� Ŀ���� Ȩ(0,1)�� �д�.
- 0xC2 : 1100 0010, Ŀ���� Ȩ(0,1)���� �̵��Ѵ�.
- 0xC6 : 1100 0110, Ŀ�� �ڵ�����
- 0xC4 : 1100 0100, Ŀ�� �ڵ�����
- 0xC7 : 1100 0111, Ŀ�� �ڵ�����, Ŀ���� �� ��ġ�� �����ϰ� ȭ�鸸 �̵�
- 0xC5 : 1100 0101, Ŀ�� �ڵ�����, Ŀ���� �� ��ġ�� �����ϰ� ȭ�鸸 �̵�
- 0xC8 : 1100 1000, Ŀ�� ��� ��� ����
- 0xC9 : 1100 1001, Ŀ�� ���� ���� �����̴� ����
- 0xCA : 1100 1010, Ŀ�� ����
- 0xCB : 1100 1011, Ŀ�� ����, Ŀ�� ����
- 0xCC : 1100 1100, LCD ��ü ����
- 0xCD : 1100 1101, LCD ��ü ����, Ŀ�� ����
- 0xCE : 1100 1110, LCD ��ü ����, Ŀ�� ���� 
- 0xCF : 1100 1111, LCD ��ü ����, Ŀ�� ����, Ŀ�� ����
- 0xD0 : 1101 0000, LCD Contrast ?1, ����, ���� �� ��ο���
- 0xD1 : 1101 0001, LCD Contrast +1, ����, ���� �� ����� 
- 0xD2 : 1101 0010, Back Light OFF
- 0xD3 : 1101 0011, Back Light ON
- 0xDF : LCD ��� System Reset, ó�� ���� ���޶��� �����ϰ� �����Ѵ�. ��⿡ �ִ� MCU�� ���������� ���۵Ǳ� ������ ���� ������ �ٽ� �ϱ� ������ ������ ������ ������ �ִ�.  
*/

#endif