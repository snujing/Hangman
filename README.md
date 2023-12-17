# Hangman

Cortex-M0 MCU NUC130과 LCD, 블루투스 통신, 서보모터를 활용한 행맨 게임 

 - 제작기간 : 2022.05.11 ~ 2022.06.29 
 - 개발규모 : 1인개발

![image](https://github.com/snujing/Hangman/assets/57716676/8c83cbda-ec00-461e-881d-280561cb7e7d)




## 데모 동영상





## LCD

LCD에서 사용자 입력과 결과를 출력

초기화에서 랜덤한 단어를 선택해 단어의 길이만큼 _(underbar)를 출력

사용자 입력값을 받으며 Right, Wrong 결과 표시

맞힌 부분은 _(underbar)를 없애고 알파벳 출력


![image](https://github.com/snujing/Hangman/assets/57716676/071d6aeb-3555-4560-96f1-073c47059b34)

```
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
```




## 서보모터

특정 각도를 제어하기 쉬운 서보모터를 사용해 행맨을 구현

PCA9685라는 PWM 서보모터 모듈을 사용

알파벳을 틀릴 때마다 지정된 서보모터 회전



![image](https://github.com/snujing/Hangman/assets/57716676/6e11daf2-6442-4230-adc9-41d2316a53ce)




## Bluetooth

알파벳 입력을 위한 간단한 앱 제작

15번 버튼을 사용해 가장 마지막에 입력한 알파벳을 입력


![image](https://github.com/snujing/Hangman/assets/57716676/5a372c60-627f-4331-8320-7f2c4af72cc4)



