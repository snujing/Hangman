// Target 보드와 소자 연결 회로
// NUC130		  	Interface
// PB12					U2(5) PCA9685 모듈, OE
// PA11/I2C1SCL			U2(4) PCA9685 모듈, SCL_I2C
// PA10/I2C1SDA			U2(3) PCA9685 모듈, SDA_I2C

// PA6					Red LED
// PA7					Green LED
// PB15					IN15 Button
// PB8					IN8 Button
// PA5					Buzzer

// Target BD 모듈	  		모터 연결
// PCA9685 PWM(노랑)		Servo Motor, SG90일 경우 주황색, PWM신호
// PCA9685 VCC(빨강)		Servo Motor, SG90일 경우 빨강색, 5V
// PCA9685 GND(검정)		Servo Motor, SG90일 경우 검정색, 그라운드 

#include "AER.h"
#include "PCA9685modul.h"
#include "FWBSerialTextLCD.H"


struct ring{   
	volatile unsigned long		wp;
	volatile unsigned long 		rp;
	volatile unsigned char		buffer[QSIZE];
}; 

struct ring q0;
struct ring q1;			// for ring buffer for UART1
unsigned char get_char0, get_char1;
unsigned char swtich_uart0, swtich_uart1;
volatile unsigned long uart_timeout;
unsigned char protocol_cm; // procol
volatile unsigned long timer_uart0;

// Geaneral Value
unsigned long ulpass;
unsigned short uspass;
unsigned char ucpass;
unsigned long runflag; 

// timer
volatile unsigned long timer0tick, timer1tick; // Timer Value.
volatile unsigned long timer_buzzer;// buzzer on time
volatile unsigned long timer_redled;// LED Red Count time

// I2C
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static volatile I2C_FUNC s_I2C1HandlerFn = NULL;

// Proto Type
// timer
void time0_delay(unsigned long T0);//10m
void time1_delay(unsigned long T1);//1S

// Utility
unsigned long isflag(unsigned long flag);
void setflag(unsigned long flag);
void clrflag(unsigned long flag);
void togflag(unsigned long flag);

// system
void setup(void);

// I2C
void I2C_MasterRx(uint32_t u32Status);
void I2C_MasterTx(uint32_t u32Status);
int32_t I2C1_PCARead_Write_SLAVE(uint8_t slvaddr);	
int32_t I2C1_PCARead_(uint8_t reg_addr); // PCA9685 read
int32_t I2C1_PCAWrite_(uint8_t reg_addr, uint8_t write_data); // PCA9685 Write
int32_t I2C1_PCA3B_Write_(uint8_t reg_addr, unsigned int write_2BYTE); // PCA9685 Write

// uart
unsigned char check_pop1(void);
void ring_ini1(void);
void ring_ini0(void);
void putstring1(unsigned char *pui8Buffer);
void random_arr(void);
void BT_9600_19200_set(char *btname);
void uart0_queue_processor(void);
int check_bluetooth_LCD(void);

void LCD_print(char* p, int arr_size);
int check_success(void);
int check_gameover(void);
void LCD_wrong_count(void);
void init_arr_check();

#define PCA9685_ADD	0x40	// 모두 0 Default Address = 0
#define ARR_SIZE 3
#define GAME_LIFE 10
char game_life_count = 0;
int is_hangman_over = 0;

char* arr[ARR_SIZE] = {"APPLE", "COMPUTER", "BANANA"};
char arr_check[17] = {0}; 
int rand1;

int main(void){
unsigned char sw_count;

    setup();
	
	runflag = 0; // flag initial
	
	LEDG(0); // OFF
	LEDR(0); // OFF
	
	UART_Open(UART0, 9600);
	
	//LCD-1
	UART_Open(UART1, 9600); 
	ring_ini1();
	time0_delay(TM_500mS); time0_delay(TM_500mS);
	uart_LCDsend(LCD_FIRST);  // 0xFB head send
	uart_LCDsend(LCD_Clear);  
	Module_LCD_goto_xy(2, 1);  
	textLCD_string("Hangman Game"); 
	time0_delay(TM_500mS);
	
	
	//Hangman 
	
	
	
	
	SE16_OE(1); 
	
	HCPCA9685_Init(SERVO_MODE);
	time0_delay(TM_100mS);
	
	HCPCA9685_OutputOnTime(0, 0);
	HCPCA9685_OutputOnTime(1, 0);
	HCPCA9685_OutputOnTime(2, 0);
	HCPCA9685_OutputOnTime(3, 0);
	HCPCA9685_OutputOnTime(4, 0);
	HCPCA9685_OutputOnTime(5, 0);
	HCPCA9685_OutputOnTime(6, 0);
	HCPCA9685_OutputOnTime(7, 0);
	HCPCA9685_OutputOnTime(8, 0);
	HCPCA9685_OutputOnTime(9, 0);
		
	HCPCA9685_OutputOffTime(0, 0);
	HCPCA9685_OutputOffTime(1, 0);
	HCPCA9685_OutputOffTime(2, 0);
	HCPCA9685_OutputOffTime(3, 0);
	HCPCA9685_OutputOffTime(4, 0);
	HCPCA9685_OutputOffTime(5, 0);
	HCPCA9685_OutputOffTime(6, 0);
	HCPCA9685_OutputOffTime(7, 0);
	HCPCA9685_OutputOffTime(8, 0);
	HCPCA9685_OutputOffTime(9, 0);
	time0_delay(TM_100mS);
	
	ulpass=0;
	HCPCA9685_dgree(0, ulpass);
	HCPCA9685_dgree(1, ulpass);
	HCPCA9685_dgree(2, ulpass);	
	HCPCA9685_dgree(3, ulpass);	
	HCPCA9685_dgree(4, ulpass);	
	HCPCA9685_dgree(5, ulpass);	
	HCPCA9685_dgree(6, ulpass);	
	HCPCA9685_dgree(7, ulpass);	
	HCPCA9685_dgree(8, ulpass);	
	HCPCA9685_dgree(9, ulpass);	
	time0_delay(TM_500mS);	
	HCPCA9685_Sleep(1); // go sleep
	
	HCPCA9685_SetPeriodFreq(50);//50Hz(20ms) for servo motor 19
	time0_delay(TM_1000mS);
	
	sw_count=0;
	
	while(2){			
		uart0_queue_processor();
		// 8888888888888888
		if(isflag(F_IN8_BUT)){
			
			
			random_arr();
			is_hangman_over = 0;
			game_life_count = 0;
			init_arr_check();
			LCD_wrong_count();
			
			
			LEDG(0); // OFF
			LEDR(1); // ON
			HCPCA9685_Sleep(0); // Wakeup 
			time0_delay(TM_200mS);
			
			// bluetooth init
			BT_9600_19200_set("AAA");
			ring_ini0(); 

			sw_count=0;
			ulpass = 0; // 0.6ms, 0 degree start
			HCPCA9685_dgree(0, ulpass); 
				HCPCA9685_dgree(1, ulpass); 
				HCPCA9685_dgree(2, ulpass); 
				HCPCA9685_dgree(3, ulpass); 
				HCPCA9685_dgree(4, ulpass); 
				HCPCA9685_dgree(5, ulpass); 
				HCPCA9685_dgree(6, ulpass); 
				HCPCA9685_dgree(7, ulpass); 
				HCPCA9685_dgree(8, ulpass); 
				HCPCA9685_dgree(9, ulpass); 
			time0_delay(TM_500mS);			
			
		
			
				
			LEDG(0); // OFF
			LEDR(0); // OFF
			HCPCA9685_Sleep(1); // go sleep
			clrflag(F_IN8_BUT);
		}
		
		// 151515151515151515
		if(isflag(F_IN15_BUT)){
			LEDG(1); // ON
			LEDR(0); // OFF
			HCPCA9685_Sleep(0); // Wakeup 
			time0_delay(TM_200mS);
			
			/*Module_LCD_goto_xy(0, 2);  
			char ppp = protocol_cm;
			char* pp = &ppp;
			textLCD_string(pp);*/
			
			if(!is_hangman_over){
				if(!check_bluetooth_LCD()){
					if(sw_count > 10){
						ulpass = 0; 
						HCPCA9685_dgree(0, ulpass); 
						HCPCA9685_dgree(1, ulpass); 
						HCPCA9685_dgree(2, ulpass); 
						HCPCA9685_dgree(3, ulpass); 
						HCPCA9685_dgree(4, ulpass); 
						HCPCA9685_dgree(5, ulpass); 
						HCPCA9685_dgree(6, ulpass); 
						HCPCA9685_dgree(7, ulpass); 
						HCPCA9685_dgree(8, ulpass); 
						HCPCA9685_dgree(9, ulpass); 
				
						time0_delay(TM_1000mS);
						sw_count = 0;
					}
					else{
						ulpass = 180;
						HCPCA9685_dgree(sw_count, ulpass);
						time0_delay(TM_1000mS);
						sw_count++;
					}
				}
			
				if(check_success()){
					Module_LCD_goto_xy(0, 2);
					textLCD_string("     Win!!      ");
					is_hangman_over = 1;
				}
				else if(check_gameover()){
					Module_LCD_goto_xy(0, 2);
					textLCD_string("    Game Over   ");
					is_hangman_over = 1;
				}
			}
			
			HCPCA9685_Sleep(1); // go sleep
			clrflag(F_IN15_BUT);
			LEDG(0); // OFF
			LEDR(0); // OFF
		}
	}
}


void setup(void){
	SYS_UnlockReg();
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);
	CLK_WaitClockReady( CLK_CLKSTATUS_IRC22M_STB_Msk);
	CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC,CLK_CLKDIV_HCLK(1));// 
	
	// Enable IP clock
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_EnableModuleClock(TMR1_MODULE); 
	CLK_EnableModuleClock(I2C1_MODULE); // for PCA9685
	CLK_EnableModuleClock(UART0_MODULE);
	CLK_EnableModuleClock(UART1_MODULE); // for LCD ***********************
	
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HIRC, 0);// 
	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HIRC, 0);//
	CLK_SetModuleClock(I2C1_MODULE, 0, 0);//
	CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));
	CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));//*****************

	
	SystemCoreClockUpdate(); 
    SYS_LockReg();

    SYS->ALT_MFP = 0x00000000;
    SYS->ALT_MFP1 = 0x00000000;
    SYS->ALT_MFP2 = 0x00000000;
	
	// Port A
	SYS->GPA_MFP =  SYS_GPA_MFP_PA10_I2C1_SDA | SYS_GPA_MFP_PA11_I2C1_SCL;
	
	// Port B
	SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD |SYS_GPB_MFP_PB15_INT1 | SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD; // *******************

	
	// Port C
	SYS->GPC_MFP = 0;
	
	// Port D
	SYS->GPD_MFP = 0;
	
	// LED Red, Green
	GPIO_SetMode(PA, BIT6 | BIT7, GPIO_PMD_OUTPUT);
	
	// Buttons
	GPIO_SetMode(PB, BIT15 | BIT8, GPIO_PMD_QUASI);
		
	// Buzzer
	GPIO_SetMode(PA, BIT5, GPIO_PMD_OUTPUT);
	
	GPIO_SetMode(PB, BIT12, GPIO_PMD_OUTPUT);

	// Timer
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100); // 10mS
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);//TMR0_IRQHandler
	TIMER_Start(TIMER0);
	
	TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1); //  1 Sec
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);//TMR1_IRQHandler
	TIMER_Start(TIMER1);
	
	// I2C 1 ini
	I2C_Open(I2C1, 10000); 
	I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
	
	UART_EnableInt(UART0,(UART_IER_RDA_IEN_Msk | UART_IER_TOUT_IEN_Msk));
	NVIC_EnableIRQ(UART02_IRQn);
	
	//******************LCD**********************
	UART_ENABLE_INT(UART1,(UART_IER_RDA_IEN_Msk | UART_IER_TOUT_IEN_Msk));//***********************
	NVIC_EnableIRQ(UART1_IRQn);	//***********************
}

// I2C
void I2C1_IRQHandler(void){
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C1);

    if(I2C_GET_TIMEOUT_FLAG(I2C1))    {
        I2C_ClearTimeoutFlag(I2C1);// Clear I2C1 Timeout Flag
    }
    else {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

void I2C_MasterRx(uint32_t u32Status){
    if(u32Status == 0x08)                      
    {
        I2C_SET_DATA(I2C1, (g_u8DeviceAddr << 1)); 
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)                  
    {
        I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)                  
    {
        I2C_STOP(I2C1);
        I2C_START(I2C1);
    }
    else if(u32Status == 0x28)                  
    {
		if(g_u8MstDataLen != 1) 
        {
            I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_STA_SI); // end
        }
    }
    else if(u32Status == 0x10)
    {
        I2C_SET_DATA(I2C1, ((g_u8DeviceAddr << 1) | 0x01)); 
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
    }
    else if(u32Status == 0x40)
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
    }
    else if(u32Status == 0x58)
    {
        g_u8MstRxData = (unsigned char) I2C_GET_DATA(I2C1);
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else    {
    }
}

void I2C_MasterTx(uint32_t u32Status){
    if(u32Status == 0x08)
    {
       I2C_SET_DATA(I2C1, g_u8DeviceAddr << 1);
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)
    {
        I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)
    {
        I2C_STOP(I2C1);
        I2C_START(I2C1);
    }
    else if(u32Status == 0x28)
    {
		if(g_u8MstDataLen != 2)		
        {
            I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else    {
    }
}

void I2C_Master_2B_Tx(uint32_t u32Status){
    if(u32Status == 0x08)
    {
        I2C_SET_DATA(I2C1, g_u8DeviceAddr << 1);
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)
    {
        I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)
    {
        I2C_STOP(I2C1);
        I2C_START(I2C1);
    }
    else if(u32Status == 0x28)
    {
        if(g_u8MstDataLen != 3)
        {
            I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else    {
    }
}

int32_t I2C1_PCARead_(uint8_t reg_addr){

	g_u8DeviceAddr = PCA9685_ADD;
	g_au8MstTxData[0] = reg_addr;
	g_au8MstTxData[1] = 0;
	
    g_u8MstEndFlag = 0;
	
	s_I2C1HandlerFn = (I2C_FUNC)I2C_MasterRx; 
	g_u8MstDataLen = 0;
	g_u8DeviceAddr = PCA9685_ADD;

	I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_STA); 
	while(g_u8MstEndFlag == 0); 
	
	return g_u8MstRxData; 
}

int32_t I2C1_PCAWrite_(uint8_t reg_addr, uint8_t write_data) {
    g_u8DeviceAddr = PCA9685_ADD;
	g_au8MstTxData[0] = reg_addr;
	g_au8MstTxData[1] = write_data;
	
	g_u8MstDataLen = 0;
	g_u8MstEndFlag = 0;

	s_I2C1HandlerFn = (I2C_FUNC)I2C_MasterTx; 
	CLK_SysTickDelay(5000);

	I2C_START(I2C1);	
	while(g_u8MstEndFlag == 0); 
	g_u8MstEndFlag = 0;
	
	return 0;
}

int32_t I2C1_PCA3B_Write_(uint8_t reg_addr, unsigned int write_2BYTE) {
    g_u8DeviceAddr = PCA9685_ADD;
	g_au8MstTxData[0] = reg_addr;
	g_au8MstTxData[1] = write_2BYTE & 0xFF;
	g_au8MstTxData[2] = write_2BYTE >> 8;
	
	g_u8MstDataLen = 0;
	g_u8MstEndFlag = 0;

	s_I2C1HandlerFn = (I2C_FUNC)I2C_Master_2B_Tx; 
	I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_STA);

	while(g_u8MstEndFlag == 0); 
	g_u8MstEndFlag = 0;
	
	return 0;
}

int32_t I2C1_PCARead_Write_SLAVE(uint8_t slvaddr){
    uint32_t i;

    g_u8DeviceAddr = slvaddr;

    for(i = 0; i < 0x100; i++)    {
        g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
        g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);

        g_u8MstDataLen = 0;
        g_u8MstEndFlag = 0;

        s_I2C1HandlerFn = (I2C_FUNC)I2C_MasterTx;

        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_STA);

        while(g_u8MstEndFlag == 0);
        g_u8MstEndFlag = 0;

        s_I2C1HandlerFn = (I2C_FUNC)I2C_MasterRx;

        g_u8MstDataLen = 0;
        g_u8DeviceAddr = slvaddr;

        I2C_SET_CONTROL_REG(I2C1, I2C_I2CON_STA);

        while(g_u8MstEndFlag == 0);

        if(g_u8MstRxData != g_au8MstTxData[2]){
            return -1;
        }
    }
    return 0;
}


// Interrupt
void TMR0_IRQHandler(void) {// 10ms
	TIMER_ClearIntFlag(TIMER0);
	timer0tick++;	

	if(IN8 == 0){ // Push
		setflag(F_IN8_BUT);
	}		
	
	if(IN15 == 0){ // Push
		setflag(F_IN15_BUT);
	}
}

void TMR1_IRQHandler(void) {//  1sec
	TIMER_ClearIntFlag(TIMER1);
	timer1tick++;	

	if(isflag(F_TIME_UART0)){
		timer_uart0++;
		swtich_uart0 = 1; 
		if(timer_uart0 > TM_3SEC){ 
			timer_uart0 = 0;
			clrflag(F_TIME_UART0);
		}	
	}
}

void time0_delay(unsigned long T0) { 
	timer0tick =0;
	while(timer0tick < T0);
}

void time1_delay(unsigned long T1) {  
	timer1tick =0;
	while(timer1tick < T1);
}

// Utility
unsigned long isflag(unsigned long flag){
	return (runflag & flag);
}

void setflag(unsigned long flag){
	 runflag |= flag;
}

void clrflag(unsigned long flag){
	runflag &= ~flag;
}

void togflag(unsigned long flag){
	runflag ^= flag;
}

//****************
void UART1_IRQHandler(uint32_t param){ 
	uint32_t u32IntSts= UART1->ISR;
	
		if(u32IntSts & UART_ISR_RDA_INT_Msk) {
		while(!UART_GET_RX_EMPTY(UART1)){
			q1.buffer[q1.wp]= UART_READ(UART1);
			q1.wp++;
			if(q1.wp>QSIZE) q1.wp=0;
		}	
	}
} 


void putstring1(unsigned char *pui8Buffer){    
unsigned short rcountx;

	for(rcountx=0; pui8Buffer[rcountx]!='\0'; rcountx++){
		if(pui8Buffer[rcountx]=='\n'){
		}
		else {
			UART_WRITE(UART1, pui8Buffer[rcountx]);			
		}
	}	
}	

unsigned char check_pop1(void){
	if(q1.wp != q1.rp){
		get_char1=(q1.buffer[q1.rp]);	
		q1.rp++;						
		if(q1.rp> QSIZE)	q1.rp=0;	
		return 1;		
	}  
	return 0;
} 

unsigned char check_pop0(void){ // BT
	if(q0.wp != q0.rp){
		get_char0=(q0.buffer[q0.rp]);	
		q0.rp++;						
		if(q0.rp> QSIZE)	q0.rp=0;	
		return 1;		
	}  //end if                      
	return 0;
} //end check_pop

void ring_ini1(void){
	q1.rp=0;
	q1.wp=0;
}	

void ring_ini0(void){
	q0.rp=0;
	q0.wp=0;
}	


void uart0_queue_processor(void){
	if(check_pop0()){
		setflag(F_TIME_UART0);
		uart_timeout=0; // 다시 시간 카운트	
		//UART_WRITE(UART1, get_char0);// for Debug, 수신 즉시 전송 		
		switch(swtich_uart0){
			case 1:
				protocol_cm=get_char0;
				swtich_uart0 = 7; // next
			break;
			/*case 3:
				protocol_cm=get_char0;
					swtich_uart0 = 5; // next
			break;
			case 5:
				protocol_cm=get_char0; // protocol command
				swtich_uart0 = 7; // next								
			break;*/			
			case 7:
				if(get_char0 == 0x0A){
					swtich_uart0 = 9; // next
				}
				else{
					swtich_uart0 = 1; // home
					clrflag(F_TIME_UART0); // disable
				}							
			break;	
			case 9:
				if(get_char0 == 0x0D){
					setflag(F_OK_PROTOCOL);					
				}
				clrflag(F_TIME_UART0); // disable
				swtich_uart0 = 1; // home
			break;			
		}	
	}
}


/*
Random Select Underbar Output
Returns the random number
*/
void random_arr(){
	//line2 init
	char arr_init[16] = "               ";
	Module_LCD_goto_xy(0, 2);  
	textLCD_string(arr_init); 
	time0_delay(TM_500mS);time0_delay(TM_500mS);time0_delay(TM_500mS);
	
	//Random Select Underbar Output
	rand1 = rand() % ARR_SIZE;
	char underbar[16] = {0};
	int i =0;
	while(arr[rand1][i] != '\0'){
		underbar[i] = '_';
		i++;
	}
	
	Module_LCD_goto_xy(0, 2);  
	textLCD_string(underbar); 
	time0_delay(TM_500mS);
	
	arr_check[strlen(arr[rand1])] = 2;
	
	return;
}

void UART02_IRQHandler(uint32_t param){ // UART0_IRQHandler
	
	
	uint32_t u32IntSts0 = UART0->ISR;
	uint32_t u32IntSts2 = UART2->ISR;
	
		if(u32IntSts0 & UART_ISR_RDA_INT_Msk) {
		while(!UART_GET_RX_EMPTY(UART0)){
			q0.buffer[q0.wp]= UART_READ(UART0);
			q0.wp++;
			if(q0.wp>QSIZE) q0.wp=0;
		}	
	}
}

void BT_9600_19200_set(char *btname){
unsigned long bps;

	bps = 9600; // default 9600bps
	UART_Open(UART0, bps); 
	UART_Open(UART1, bps);  
	
	ring_ini0(); // buffer ini
	ring_ini1(); // buffer ini
	time0_delay(TM_200mS);
	
	// AT+NAMECONAN$0D$0A		
	UART_WRITE(UART0, 'A'); time0_delay(TM_10mS);	
	UART_WRITE(UART0, 'T'); time0_delay(TM_10mS);						
	UART_WRITE(UART0, '+'); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 'N'); time0_delay(TM_10mS);				
	UART_WRITE(UART0, 'A'); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 'M'); time0_delay(TM_10mS);				
	UART_WRITE(UART0, 'E'); time0_delay(TM_10mS);	
	
	UART_WRITE(UART0, btname[0]); time0_delay(TM_10mS);	
	UART_WRITE(UART0, btname[1]); time0_delay(TM_10mS);						
	UART_WRITE(UART0, btname[2]); time0_delay(TM_10mS);			
	UART_WRITE(UART0, btname[3]); time0_delay(TM_10mS);				
	UART_WRITE(UART0, btname[4]); time0_delay(TM_10mS);		

	UART_WRITE(UART0, 0x0D); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 0x0A); time0_delay(TM_10mS);	
	
	// AT+BAUD5$0D$0A	
	UART_WRITE(UART0, 'A'); time0_delay(TM_10mS);	
	UART_WRITE(UART0, 'T'); time0_delay(TM_10mS);						
	UART_WRITE(UART0, '+'); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 'B'); time0_delay(TM_10mS);				
	UART_WRITE(UART0, 'A'); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 'U'); time0_delay(TM_10mS);				
	UART_WRITE(UART0, 'D'); time0_delay(TM_10mS);					
	UART_WRITE(UART0, '4'); time0_delay(TM_10mS); // 19200BPS			
	UART_WRITE(UART0, 0x0D); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 0x0A); time0_delay(TM_10mS);		
		
	bps = 9600; // default 9600bps
	UART_Open(UART0, bps); 
	UART_Open(UART1, bps); // 
	
	ring_ini0(); // buffer ini
	ring_ini1(); // buffer ini
	time0_delay(TM_200mS);
	
	// AT+RESET$0D$0A
	UART_WRITE(UART0, 'A'); time0_delay(TM_10mS);	
	UART_WRITE(UART0, 'T'); time0_delay(TM_10mS);						
	UART_WRITE(UART0, '+'); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 'R'); time0_delay(TM_10mS);				
	UART_WRITE(UART0, 'E'); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 'S'); time0_delay(TM_10mS);				
	UART_WRITE(UART0, 'E'); time0_delay(TM_10mS);					
	UART_WRITE(UART0, 'T'); time0_delay(TM_10mS); // 19200BPS			
	UART_WRITE(UART0, 0x0D); time0_delay(TM_10mS);			
	UART_WRITE(UART0, 0x0A); time0_delay(TM_10mS);	
	
	time0_delay(TM_500mS);
}

int check_bluetooth_LCD(){
	char flag = 0;
	int i =0;
	char ppp = protocol_cm;
	char* pp = &ppp;
	
	
	int word_size = strlen(arr[rand1]);
	
	for(i =0;i<word_size;i++){
		if(arr[rand1][i] == ppp){
			Module_LCD_goto_xy(i, 2); 
			textLCD_string(pp);
			time0_delay(TM_100mS);
			arr_check[i] = 1;			
			flag = 1;
		} 
	}
	
	if(flag == 0){
		LCD_print("Wrong!", 6);
		game_life_count++;
		LCD_wrong_count();
		return 0;
	}
	else if(flag == 1){
		LCD_print("Right!", 6);
		return 1;
	}
	return 0;
}

void LCD_print(char* p, int arr_size){
	char *p_a = "<";
	char *p_b = ">";
	int front = (16 - arr_size) / 2;
	int end = front + arr_size;
	
	
	int i_front = front - 1;
	int i_end = end;
	
	Module_LCD_goto_xy(0, 1);  
	textLCD_string("                "); 
	Module_LCD_goto_xy(front, 1); 
	textLCD_string(p);
	
	while(i_front > 0){
		Module_LCD_goto_xy(i_front--, 1); 
		textLCD_string(p_a);
		Module_LCD_goto_xy(i_end++, 1);
		textLCD_string(p_b);
		time0_delay(TM_200mS);
	}
	Module_LCD_goto_xy(0, 1);  
	textLCD_string("  Hangman Game  "); 
	
}

void LCD_wrong_count(){
	char a = game_life_count + 48;
	char *p;
	p = &a;
	Module_LCD_goto_xy(15, 2);  
	textLCD_string(p);
}

int check_gameover(){
	if(game_life_count >= GAME_LIFE)
		return 1;
	
	return 0;
}

int check_success(){
	int i =0, flag = 0;
	while(arr_check[i] != 2){
		if(arr_check[i] == 0){
			flag = 1;
			break;
		}
		i++;
	}
	
	if(flag == 0)
		return 1;
	
	return 0;
}

void init_arr_check(){
	for(int i = 0; i < 17; i++){
		arr_check[i] = 0;
	}
}




