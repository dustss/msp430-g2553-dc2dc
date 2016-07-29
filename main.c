/*
 * GPIO说明
 * P1.6、P1.7口用作I2C 扩展出16个IO (4个驱动LCD 4个按键 8个LED)
 * P2.2口  做 PWM波输出口
 * P1.1口  做 ADC采样输入口
 *
 * 任务分解与进度
 * 显示屏显示输出电压 …………………………………………………………………………………………………………………………………………完成
 * 扩展板按键控制输出电压大小，左侧按钮0.1V增减，右侧按钮1V增减…………………………………………完成
 * P2.2口 输出 PWM波………………………………………………………………………………………………………………………………………………完成
 * P1.1口 输入分压后的电压反馈值 ADC采样………………………………………………………………………………………………完成
 * 根据采样所得值temp与目标值goal 通过PID控制，调节输出占空比permill………………………完成
 * 测定 votage 与  采样值的关系 即  采样值 = a* votage +b 中的a,b…………………………………完成
 *  整定PID参数……………………………………………………………………………………………………………………………………………………
 * 用万用表测定电压稳定度，电压效率，完善程序结构
 *
 *  last edit on: 2015-10-09
 *  Author: dusts
 */
#include"MSP430G2553.h"

#include "HT1621.h"   //IO扩展    按键和显示屏
#include "LCD_128.h"
#include "TCA6416A.h"

#include "TA_PWM.h"  // PWM波

int Kp=5;                       //PID参数
int Ki=1;
int Kd=0;
//lmc:716,861   LYT:617,696
int a=608;                        // 采样值 = (a/10* Votage +b)/10
int b=120;						//matlab拟合结果为 temp=0.7160*v +86.0752

int Votage = 12;                 //输出电压，避免浮点数设为整型，实际数值为1/10
int temp = 0;                    //采样值
int goal = 0;					 //目标值
int permill=0;      			 //0-1000   即0.0%-100.0%

void WDT_init();
void I2C_IODect();		                 //检测事件
void votageCheck();
void LCD_Init();
void ADC10_init();                        //ADC初始化
void getTemp();
void calPermill();

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;	//关狗
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
    //初始化1.1口为输入口
    P1IN |=BIT1;
	//-----初始化TA1为ACLK/SMCLK输入，不分频，通道1普通超前PWM，通道2关闭----- P.S. TA0的P1.6口被I2C占用，所以不能用TA0
	TA1_PWM_Init('a',1,'F',0); //TA1的1通道对应 P2.2      自动初始化P2.2
	TA1_PWM_SetPeriod(50);				//设定PWM周期
	//ADC初始化
	ADC10_init();
    TCA6416A_Init();			// 初始化IO扩展口
	HT1621_init();				// 初始化lcd_128
	LCD_Init();              	//清屏 显示默认值
	WDT_init();
	while(1)
	{
		getTemp();
	     PinIN();
		I2C_IODect();                            //按键事件检测

	     calPermill();                          //计算需要的占空比
	  	TA1_PWM_SetPermill(1,permill);		     //更新PWM占空比

		 _bis_SR_register(LPM0_bits);             //进入休眠
	}

}
/**************************ADC10*****************************/
void ADC10_init()
{
	ADC10CTL0 &= ~ENC;

	/******** 开启***2.5v参考电压***64x采样保持时间*参考电压模式****/
//	ADC10CTL0 =  ADC10ON + REFON +REF2_5V+ ADC10SHT_3 + SREF_1;
	ADC10CTL0 =  ADC10ON + REFON + ADC10SHT_3 + SREF_1;
	/*单通道单次采样*****专用ADC时钟****时钟分频****采样开始信号***P1.1/A1 CH1***/
    ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_3 + SHS_0 + INCH_1;
      __delay_cycles(30000);
    ADC10CTL0 |= ENC;
}
/*************************ADC采样***********************************************/
void getTemp()
{
	static int i=0;                     //声明为局部静态变量，只初始化一次
    static int t[10]={0};

    ADC10CTL0 |= ENC + ADC10SC;            // 采样开始信号
    __delay_cycles(1000);              	   //等待ADC转换完成
    t[i] = (int)ADC10MEM;                  //拿到采样值

	i++;       							//采样值波动较大，为稳定，十次采样取平均
	if(i>9)
		  i=0;
	int j;
	for(j=0,temp=0;j<10;j++)
	{
		  temp+=t[j];
	}
	temp /=10;
//	    ADC10CTL0 |= ENC + ADC10SC;            // 采样开始信号
//	    __delay_cycles(1000);              	   //等待ADC转换完成
//	    temp = (int)ADC10MEM;                  //拿到采样值
}



/********初始化LCD显示相关的硬件，并预设固定不变的显示内容**********/

void LCD_Init()
{
	LCD_DisplayNum(Votage);        // 电压数值
	LCD_DisplaySeg(_LCD_DOT4);     // 小数点
	LCD_DisplaySeg(_LCD_V);        // 电压符号“V”
	LCD_DisplaySeg(_LCD_DC);       // 直流符号“DC”
	HT1621_Reflash(LCD_Buffer);    // 刷新显存
}

/********WDT初始化函数**********/
void WDT_init()
{
	//-----设定WDT为-----------
	WDTCTL=WDT_ADLY_16;
	//-----WDT中断使能--------
    IE1 |= WDTIE;
}

/***********WDT定时中断函数*************/
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
__bic_SR_register_on_exit(LPM0_bits );
}

/**********I2C扩展IO中断事件处理函数***************/
void I2C_IO10_Onclick()
{
	Votage += 1;
	votageCheck();
//	LCD_DisplayNum(temp);       //  测试用temp
	LCD_DisplayNum(Votage);
	HT1621_Reflash(LCD_Buffer);
}
void I2C_IO11_Onclick()
{
	Votage -= 1;
	votageCheck();
	LCD_DisplayNum(Votage);
//	LCD_DisplayNum(goal);       //  测试用goal
	HT1621_Reflash(LCD_Buffer);
}
void I2C_IO12_Onclick()
{
//	Votage += 10;
//
	Kp+=1;
	if(Kp==100)
		Kp=0;
//	permill +=50;
//	if(permill>1000||permill<0)
//		permill=0;
//	LCD_DisplayNum(permill);
	votageCheck();
	LCD_DisplayNum(Kp);

	HT1621_Reflash(LCD_Buffer);
}
void I2C_IO13_Onclick()
{
//	Votage -= 10;
//	Votage += 1;
	Ki+=1;
	if(Ki==100)
		Ki=0;
	LCD_DisplayNum(Ki);

//	permill -=50;
//	if(permill>1000||permill<0)
//		permill=0;
	votageCheck();
//	LCD_DisplayNum(permill);
	HT1621_Reflash(LCD_Buffer);
}

void votageCheck()
{
	if (Votage<12)
		Votage=12;
	else if(Votage>120)
		Votage=120;
    goal= (a * Votage/10 + b)/10 ;                  //计算目标值
}
void I2C_IODect()			                 //检测按键事件
{
	static unsigned char KEY_Past=0,KEY_Now=0;
	KEY_Past=KEY_Now;
	//----判断I2C_IO10所连的KEY1按键是否被按下------
	if((TCA6416A_InputBuffer&BIT8) == BIT8)
		KEY_Now |=BIT0;
	else
		KEY_Now &=~BIT0;
	if(((KEY_Past&BIT0)==BIT0)&&(KEY_Now&BIT0) !=BIT0)
		I2C_IO10_Onclick();
	//----判断I2C_IO11所连的KEY2按键是否被按下------
	if((TCA6416A_InputBuffer&BIT9)== BIT9)
		KEY_Now |=BIT1;
	else
		KEY_Now &=~BIT1;
	if(((KEY_Past&BIT1)==BIT1)&&(KEY_Now&BIT1)!=BIT1)
		I2C_IO11_Onclick();
	//----判断I2C_IO12所连的KEY3按键是否被按下------
	if((TCA6416A_InputBuffer&BITA) == BITA)
		KEY_Now |=BIT2;
	else
		KEY_Now &=~BIT2;
	if(((KEY_Past&BIT2)==BIT2)&&(KEY_Now&BIT2) ==0)
	{
		I2C_IO12_Onclick();
	}
	//----判断I2C_IO13所连的KEY4按键是否被按下------
	if((TCA6416A_InputBuffer&BITB) ==  BITB)
		KEY_Now |=BIT3;
	else
		KEY_Now &=~BIT3;
	if(((KEY_Past&BIT3) == BIT3)&& (KEY_Now&BIT3) == 0)    //
	{
		I2C_IO13_Onclick();
	}
}
/************************PI控制输出占空比********************************************/
//假设现在已经拿到temp值，建立起V与temp的对应关系  temp=a*真实电压+b
//  temp的目标值  即  goal = a*Voatge(目标电压12-120)+b
void calPermill()
{
	  static int last_error;
	  static int last_last_error;

	  int error    = goal - temp;                  //数量级是 100
	  int d_error  = error - last_error;              //数量级是10
	  int dd_error = error - 2*last_error + last_last_error;

	  permill += Kp * d_error/10 + Ki * error/10 + Kd * dd_error;   //PID控制

	  last_last_error = last_error;
	  last_error = error;

	  if(permill>800)                           //输出合法性检查  留20%余量
	    permill=800;
	  else if(permill<0)
		permill=0;
}

