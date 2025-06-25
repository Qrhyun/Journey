/*
#include <REGX52.H>

void main()
{
	
	while(1)
	{
		P2=0xFE;
		P2=0xFF;
	}
}
//这个代码的原理是把第一个灯先点亮后熄灭，由于单片机执行代码速度特别快，看不出来在闪，所以我们可以设置亮和灭的时间间隔为500ms，可以通过stc-isp这个软件生成代码

*/

#include <REG52.H>
#include <INTRINS.H> //定义nop函数的头文件

/**
 * @description: 延时为500ms的子函数，计数以达到延时的目的
 * @return {*}
 */
void Delay500ms()		//@11.0592MHz
{
	unsigned char i, j, k;

	_nop_(); //
	i = 4;
	j = 129;
	k = 119;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}


 void main(){


    while (1){
        P2 = 0xFE; // 点亮1号LED灯
        Delay500ms();
        P2 = 0xFF; // 熄灭所有灯
        Delay500ms();
    }
    
 }