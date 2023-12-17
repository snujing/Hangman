/* 		Purpose : FWB Serial TextLCD Module Utility
		Target Name : FWB Serial TextLCD 
		Compiler : MDK ARM Compiler
		Ver : V3.4
		CMSIS : Mini51DE_CMSIS_V301002
		Document : FWB Serial TextLCD �޴��� 
---------------------------------------------
Copyright by FirmwareBank Inc,. All Rights Reserved. */

#include "FWBSerialTextLCD.H"

void Module_LCD_goto_xy(unsigned char x, unsigned char y);
void textLCD_string(char *lcdstring16);
void uart_LCDsend(unsigned char send1byte);

#define UART_LCD	UART1

// Module LCD
void Module_LCD_goto_xy(unsigned char x, unsigned char y){
	uart_LCDsend((0x80) | (y << 4) |(x));// Format : 10yy xxxx(6��Ʈ Ŀ�� ��ġ x, y ����)
}

void uart_LCDsend(unsigned char send1byte){
unsigned short delay_lcd;// do not clear
	UART_WRITE(UART_LCD, send1byte); 
	for(delay_lcd=0; delay_lcd < 0x800; delay_lcd++);	
}

// LCD
void textLCD_string(char *lcdstring16){
	unsigned char i;
	for(i=0; lcdstring16[i]!='\0'; i++) { 	// Null('\0') ���ڸ� ���� ������
		uart_LCDsend(lcdstring16[i]); 	
	}
}