#include "stm32f10x.h"
#include "delay.h"
#include "motor.h"
#include "usart.h"
#include "Server.h"
#include "usart2.h"
#include "string.h"	 

int serverag_LR = 90;
int serverag_SX = 0;

void left_detection()//左右舵机靠左
{
     serverag_LR =serverag_LR+10;
	  if(serverag_LR >= 175) serverag_LR = 175;	
		SetJointAngle(7,serverag_LR); //产生PWM
}
void right_detection()//左右舵机靠右
{
  	 serverag_LR = serverag_LR-10;
	  if(serverag_LR <= 1) serverag_LR = 1;	
		SetJointAngle(7,serverag_LR); //产生PWM
}
void s_detection()//上下舵机上
{
 
    serverag_SX =serverag_SX-10;
	  if(serverag_SX <= 1) serverag_SX = 1;	
		SetJointAngle(12,serverag_SX); //产生PWM
}
void x_detection()//上下舵机下
{
     serverag_SX = serverag_SX+10;
	  if(serverag_SX >= 175) serverag_SX = 175;	
		SetJointAngle(12,serverag_SX); //产生PWM
}

 int main(void)
 {	
	 u8 reclen=0;
	 delay_init();
	 TIM4_PWM_Init(7199,0);  //初始化PWM
	 uart_init(9600);
	 TIM_PWM_Init(9999,143);               //不分频，PWM频率=72*10^6/(9999+1)/(143+1)=50Hz	 
	 USART2_Init(9600);	 
	 ZYSTM32_brake(500);
	 SetJointAngle(7,serverag_LR); //摄像头左右舵机
	 SetJointAngle(12,serverag_SX);//摄像头上下舵机
	 SetJointAngle(2,90);	       //超声波舵机
	 while(1)
		{ 		
		if(USART2_RX_STA&0X8000)	//接收到一次数据了
		{
			reclen=USART2_RX_STA&0X7FFF;	//得到数据长度
			USART2_RX_BUF[reclen]=0;	 	//加入结束符
	//		printf("USART2_RX_BUF:%s\n",USART2_RX_BUF);
		//	printf("reclen:%d\n",reclen);
			if(reclen==3||reclen==4||reclen==6) 		//控制DS1检测
			{
				if(strcmp((const char*)USART2_RX_BUF,"ONA")==0)
				{
					u2_printf("go forward!"); 
          ZYSTM32_run(80,100);
				}
				else if(strcmp((const char*)USART2_RX_BUF,"ONB")==0)
				{
					u2_printf("go back!"); 
          ZYSTM32_back(80,100);
				}
				else if(strcmp((const char*)USART2_RX_BUF,"ONC")==0)
				{
         u2_printf("go right!"); 
         ZYSTM32_Right(80,100);
				}
			  else if(strcmp((const char*)USART2_RX_BUF,"OND")==0)
				{
					u2_printf("go left!"); 
         ZYSTM32_Left(80,100);
				}
			  else if(strcmp((const char*)USART2_RX_BUF,"ONF")==0)
				{
         u2_printf("Stop!"); 
         ZYSTM32_brake(100);
				}
				else if(strcmp((const char*)USART2_RX_BUF,"ONE")==0)
				{
         u2_printf("Stop!"); 
         ZYSTM32_brake(100);
				}
				
				else if(strcmp((const char*)USART2_RX_BUF,"ONLONF")==0)//左
				{
         left_detection();  
				}
				else if(strcmp((const char*)USART2_RX_BUF,"ONIONF")==0)//右
				{
          right_detection();
				}
				else if(strcmp((const char*)USART2_RX_BUF,"ONJONF")==0)//上
				{
          s_detection();
				}
				else if(strcmp((const char*)USART2_RX_BUF,"ONKONF")==0)//下
				{
          x_detection();
				}
			}
			USART2_RX_STA=0;		
		}	
	}
 }

