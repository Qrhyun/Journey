`src/Delay.c`
```c
void Delay(unsigned int time) //@11.0592MHz
{
    unsigned char i, j;

    while (time) {
        i = 2;
        j = 199;
        do {
            while (--j);
        } while (--i);
        time--;
    }
}
```
`src/Delay.h`
```c
// 如果没有定义这个，就进行定义
#ifndef __DELAY_H__
#define __DELAY_H__

void Delay(unsigned int time);

#endif
```