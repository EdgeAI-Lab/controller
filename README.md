# cotroller

## Motor installation
* m2 -- > left motor
* m1 -- > right motor


## Timer usage
* TIM1 for generate PWM, TIM1-CH1&CH1N for right motor, TIM1-CH2&CH2N for left motor
* TIM3 for left motor encoder
* TIM5 for right motor encoder
* TIM1-CH3 for Stepper_LR_Step, TIM8-CH2 for Stepper_UD_step, TIM8-CH3 for Stepper_Protect_Step
* Stepper_UD_step 和 Stepper_Protect_Step 不能同时使用

## STM32内部FLASH
* 只能以半字(uint16_t)操作，HAL库函数提供的Word和Byte操作其实都是半字操作
* STM32是小端格式，内存和FLASH中都是小端格式
* 从内存和flash中读取或者写入数据都是从底地址开始的
* 数值0x11223344在内存或者flash中的存储形式如下：

|地址|数值|
|----|----|
|0x03|0x11|
|0x02|0x22|
|0x01|0x33|
|0x00|0x44|

## DC motor
* PWM frequency 36K,频率太低电机会啸叫(Maxon Motor)
* 防拆电机采用500RPM的直流减速电机

## Stepper motor

*  步进电机本身能够接受的PWM频率为1200Hz（数据来自淘宝卖家）,也就是说步进电机每步的执行时间大约是0.8ms
* DRV8846能够接受的输入PWM频率是250K, 经测试主频72M，PWM频率18K为最高 ，此时步进电机转速约 6.5秒/圈
* 经测试步进电机细分采用 32 microsteps/step, rising-edge only(M1=1，M0=Z)效果比较理想
* 使用高级定时器的RCR功能产生固定个脉冲的方法，要求1个高级定时器只能控制一个步进电机(因为如果有多个通道使能PWM，那么控制步进电机的PWM通道停止时，UEV事件依然会产生)

## 测距机相关
* 测距机通信波特率115200
* 测距机单次测量指令 ``` 0x0D, 0x0A, 0x4F, 0x3D, 0x31, 0x0D, 0x0A ```
* 测距机停止测量指令(可关闭测距激光) ``` 0x0D, 0x0A, 0x4F, 0x46, 0x46, 0x0D, 0x0A ```
* 测距机返回值，成功测量到距离值时返回 ``` eg: D=35.6m\r\n ```，测量失败时返回 ``` D=-----m\r\n ```


## 光电开关 for Stepper
* 开漏输出
* 无遮挡时，输出低电平
* 有遮挡时，输出高电平


# 电机参数图示
![motor_parameter_illustration](http://github.com/EdgeAI-Lab/controller/blob/master/pictures/motor_parameter.png)
* M1 & M2 is [Maxon DC motor](https://www.maxonmotor.com)
* M3 & M4 is [Chihai DC motor](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.b9052e8dcijeIG&id=538992784317&_u=2nt24adc9aa)

# Maxon电机尺寸
![motor_parameter_illustration](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/maxon-motor-dimen-1.png)
![motor_parameter_illustration](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/maxon-motor-dimen-2.png)
![motor_parameter_illustration](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/maxon-motor-dimen-3.png)
![motor_parameter_illustration](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/maxon-motor-dimen-4.png)

# Maxon电机性能
* Motor - DCX12S EB KL 12V
* Planetary gearhead - GPX12 C 83:1
* Sensor - ENX10 EASY 512IMP

|项目|参数|
|----|----|
|额定电压|12V|
|额定电流|0.159A|
|堵转电流|0.261A|
|电机额定转矩|1.89mNm|
|电机堵转转矩|3.21mNm|
|减速器减速比|83:1|
|减速器额定转矩|0.19Nm|
|减速器堵转转矩|0.24Nm|
|编码器线数|256|

# Maxon编码器的使用
* 经实际测试镜头全量程转动，电机旋转1.46圈 (下面计算都按照1.5圈，留有余量)
* 编码器采用2倍频 256 * 2 * 83 * 1.5 = 63744 (即全量程编码器计数不多于63744) 
* 编码器的理论精度：

|名称|参数|
|----|----| 
|Maxon电机轴精度|360/512=0.703度|
|Maxon减速器轴精度|0.703/83=0.00847度|  

# Maxon电机实际运转情况(电压值是经过放大的)
|状态|电机电压值|ADC采集值|ADC采集值|
|----|----------|---------|---------|
|电机堵转| 1.64V    |2035     |0x07F3   |
|大步运转| 1.25V    |1551     |0x060F   |
|长时运转| 1.0V     |1241     |0x04D9   |
|电机不转| 0.46V    |571      |0x023B   |

# Maxon电机编码器接口
![motor_encode_connector](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/motor_encode_connector.png)

#### 【疑问】理论计算电机堵转时，放大后电压为：0.261A*0.47Ohm*25.097=3.08V，但是实测为1.64V

# Chihai电机性能
|项目|参数|
|----|----|
|额定电压|12V|
|额定电流|<=55mA|
|堵转电流|<=0.42A|
|减速器减速比|1000:1|
|电机额定转矩|3000g.cm|
|电机堵转转矩|3Kg.cm|
|编码器线数|7|

# Chihai电机尺寸及相关图示
![motor_parameter_illustration](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/chihai-motor-dimen.png)
![motor_parameter_illustration](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/chihai-motor-encoder.png)
![motor_parameter_illustration](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/chihai-motor-encoder-electrical-characteristics.png)


# 程序下载接口(SW)
![SW](http://192.168.10.100/EdgeAI-Lab/controller/raw/master/pictures/sw_jtag.png)


## Debg info

### Jlink
* S/N: 583648125 for main board

### 1. 系统带电机上电，程序会卡死在 ```configASSERT( xTaskToNotify );```
* 原因：带电机上电，中断可能会在任务初始化之前触发，这样可能非法使用操作系统的资源，导致程序异常
* 解决：系统上电进入临界区（在所有的中断使能之前），然后在所有任务初始化完成之后退出临界区


### 2. object_find()取出结构体时，其内容被修改
* 原因：变量重命名，在init.c中和system_cmd.c中分别以结构体和结构体指针的形式重复命名变量


```
// init.c中以结构体的形式命名
bracket bracket;
protect_structure protect_structure;

```


```
// system_cmd.c中以结构体指针的形式命名
bracket_t bracket;
protect_structure_t protect_structure;
```

### 3. osDelayUntil()
* 使用此函数请在FreeRTOS中将vTaskDelayUntil选项使能

### 4.ADC DMA完成中断
* 请不要在DMA传输完成中断函数中处理数据，因为中断的优先级本身就高于任务，且这个中断发生的频率非常高，会影响其他任务的正常执行
* 解决方案是：在任务中主动处理ADC数据
* STM32 CubeMX默认是开启DMA传输完成中断的 ``` HAL_ADC_Start_DMA() -> HAL_DMA_Start_IT(){__HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_TE));} ``` 
* ADC DMA的三个中断(DMA_IT_TC, DMA_IT_HT, DMA_IT_TE)相对应的中断函数，在HAL库中本身就有实现，回调函数也是从这些函数中回调的
* 不知道官方为什么这么做

### 5.工程名问题
* 从码云远程仓库下载来的工程文件夹与本地最初创建的不一样，导致各种问题：1. xxx.ico无法正常打开；2. xxx.elf.launch文件无法正常使用
* 所以尽量是工程名保持一致，避免这些不必要的问题

### 6.进出临界区

```
taskENTER_CRITICAL();

taskEXIT_CRITICAL();
```
### 7. Flash磨损均衡算法写入速度测试
* 写入速度非常快，从开始写入到串口打印出数据不超过1s
* 测试代码如下：

```

taskENTER_CRITICAL();
for(cnt=0;cnt<2048;cnt++)
{
	writer.data[0] = (uint8_t)(cnt & 0xFF);
	writer.data[1] = (uint8_t)((cnt>>8) & 0xFF);
	write_word_to_flash(writer);
}

taskEXIT_CRITICAL();
usart1_printf("%x\n",find_used_entry());

```
