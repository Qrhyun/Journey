`src/Timer0.c`
```c
#include <REGX52.H>

/**
  * @brief  定时器0初始化，1毫秒@12.000MHz
  * @param  无
  * @retval 无
  */
void Timer0Init(void)
{
	TMOD &= 0xF0;		//设置定时器模式 TMOD的低四位清0，高四位保持不变
	TMOD |= 0x01;		//设置定时器模式
	TL0 = 0x18;		//设置定时初值
	TH0 = 0xFC;		//设置定时初值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
	ET0=1;
	EA=1;
	PT0=0;
}

/*定时器中断函数模板
void Timer0_Routine() interrupt 1
{
	static unsigned int T0Count;
	TL0 = 0x18;		//设置定时初值
	TH0 = 0xFC;		//设置定时初值
	T0Count++;
	if(T0Count>=1000)
	{
		T0Count=0;
		
	}
}
*/
```
`src/Timer0.h`
```c
#ifndef __TIMER0_H__
#define __TIMER0_H__

void Timer0Init(void);

#endif
```
`src/main.c`
```c
#include <REGX52.H>
#include "Timer0.h"
#include "Key.h"
#include <INTRINS.H>

unsigned char KeyNum,LEDMode;

void main()
{
	P2=0xFE;
	Timer0Init();
	while(1)
	{
		KeyNum=Key();		//获取独立按键键码
		if(KeyNum)			//如果按键按下
		{
			if(KeyNum==1)	//如果K1按键按下
			{
				LEDMode++;	//模式切换
				if(LEDMode>=2)LEDMode=0;
			}
		}
	}
}

void Timer0_Routine() interrupt 1
{
	static unsigned int T0Count;
	TL0 = 0x18;		//设置定时初值
	TH0 = 0xFC;		//设置定时初值
	T0Count++;		//T0Count计次，对中断频率进行分频
	if(T0Count>=500)//分频500次，500ms
	{
		T0Count=0;
		if(LEDMode==0)			//模式判断
			P2=_crol_(P2,1);	//LED输出
		if(LEDMode==1)
			P2=_cror_(P2,1);
	}
}
```