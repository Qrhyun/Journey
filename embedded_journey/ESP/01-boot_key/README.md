## BOOT-KEY 按键
### IO大概解释
> ESP32-S3 是 QFN56 封装，GPIO 引脚一共有 45 个，从 GPIO0 到 GPIO21，再从 GPIO26 到 GPIO48。理论上，所有的 IO 都可以作为普通 GPIO 使用，或者复用为任何外设功能使用，但有些引脚用作连接 FLASH 和 PSRAM 后，就不能再做其它用途了。

> 我们开发板上使用的模组型号是 ESP32-S3-WROOM-1-N16R8，它的 FLASH 为 16MB，与 ESP32 采用 4 线制 SPI 连接，它的 PSRAM 为 8MB，与 ESP32 采用 8 线制 SPI 连接。FLASH 与 PSRAM 一共占用了 12 个 IO 引脚。除去这些引脚，IO 就剩下 33 个了。

> 在开发板上，实际上引出了 3 个我们可以用户自定义的 IO，一个是 BOOT 按键连接的 IO，另外两个是外扩接口引出的 GPIO10 和 11。

> ESP32 的 GPIO，可以用作输入、输出，可以配置内部上拉、下拉，可以配置为中断引脚。
### BOOT-KEY 按键
> 本项目把连接 BOOT 按键的 IO0 引脚，设置为 GPIO 中断，接收按键请求
 
### 详解
`app_main`
```md
- 第 3 行~第 9 行，定义一个 gpio 结构体变量，并给该结构体变量成员赋值。
- 第 11 行，配置好 GPIO0
- 第 14 行，创建一个队列处理 GPIO 事件。
- 第 16 行，创建一个 GPIO 任务函数
- 第 18 行，创建 GPIO 中断服务
- 第 20 行，给 GPIO0 添加中断处理。其中，第 1 个参数，表示你要给哪个引脚添加中断功能。第 2 个参数gpio_isr_handler 是中断服务函数名称。第 3 个参数是当第 1 个参数指定的引脚发生中断时，输送给此中断服务函数的参数
```
```c
void app_main(void)
{
    gpio_config_t io0_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // 下降沿中断
        .mode = GPIO_MODE_INPUT, // 输入模式
        .pin_bit_mask = 1<<GPIO_NUM_0, // 选择GPIO0
        .pull_down_en = 0, // 禁能内部下拉
        .pull_up_en = 1 // 使能内部上拉
    };
    // 根据上面的配置 设置GPIO
    gpio_config(&io0_conf);

    // 创建一个队列处理GPIO事件
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // 开启GPIO任务
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    // 创建GPIO中断服务
    gpio_install_isr_service(0);
    // 给GPIO0添加中断处理
    gpio_isr_handler_add(GPIO_NUM_0, gpio_isr_handler, (void*) GPIO_NUM_0);
}
```
`上述内部子函数的实现`
```c
static QueueHandle_t gpio_evt_queue = NULL;  // 定义队列句柄

// GPIO中断服务函数
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;  // 获取入口参数
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL); // 把入口参数值发送到队列
}

// GPIO任务函数
static void gpio_task_example(void* arg)
{
    uint32_t io_num; // 定义变量 表示哪个GPIO
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {  // 死等队列消息
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num)); // 打印相关内容
        }
    }
}
```

###  例程制作过程
> 总的来说，该例程是使用 IDF 例程中的 sample_project 工程（看工程名字就可以知道，这是一个“样例工程”,工程路径为 examples\get-started\sample_project）作为模板，参考例程是 IDF 例程中的 generic_gpio 例程，都是从这个例程中复制粘贴修改而来。
