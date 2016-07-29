/*
 * GPIO˵��
 * P1.6��P1.7������I2C ��չ��16��IO (4������LCD 4������ 8��LED)
 * P2.2��  �� PWM�������
 * P1.1��  �� ADC���������
 *
 * ����ֽ������
 * ��ʾ����ʾ�����ѹ �����������������������������������������������������������������������������������������������������������
 * ��չ�尴�����������ѹ��С����ఴť0.1V�������Ҳఴť1V���������������������������������������
 * P2.2�� ��� PWM�����������������������������������������������������������������������������������������������������������������
 * P1.1�� �����ѹ��ĵ�ѹ����ֵ ADC�������������������������������������������������������������������������������
 * ���ݲ�������ֵtemp��Ŀ��ֵgoal ͨ��PID���ƣ��������ռ�ձ�permill���������������������
 * �ⶨ votage ��  ����ֵ�Ĺ�ϵ ��  ����ֵ = a* votage +b �е�a,b�����������������������������
 *  ����PID��������������������������������������������������������������������������������������������������������������������
 * �����ñ�ⶨ��ѹ�ȶ��ȣ���ѹЧ�ʣ����Ƴ���ṹ
 *
 *  last edit on: 2015-10-09
 *  Author: dusts
 */
#include"MSP430G2553.h"

#include "HT1621.h"   //IO��չ    ��������ʾ��
#include "LCD_128.h"
#include "TCA6416A.h"

#include "TA_PWM.h"  // PWM��

int Kp=5;                       //PID����
int Ki=1;
int Kd=0;
//lmc:716,861   LYT:617,696
int a=608;                        // ����ֵ = (a/10* Votage +b)/10
int b=120;						//matlab��Ͻ��Ϊ temp=0.7160*v +86.0752

int Votage = 12;                 //�����ѹ�����⸡������Ϊ���ͣ�ʵ����ֵΪ1/10
int temp = 0;                    //����ֵ
int goal = 0;					 //Ŀ��ֵ
int permill=0;      			 //0-1000   ��0.0%-100.0%

void WDT_init();
void I2C_IODect();		                 //����¼�
void votageCheck();
void LCD_Init();
void ADC10_init();                        //ADC��ʼ��
void getTemp();
void calPermill();

void main(void)
{
	WDTCTL = WDTPW + WDTHOLD;	//�ع�
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
    //��ʼ��1.1��Ϊ�����
    P1IN |=BIT1;
	//-----��ʼ��TA1ΪACLK/SMCLK���룬����Ƶ��ͨ��1��ͨ��ǰPWM��ͨ��2�ر�----- P.S. TA0��P1.6�ڱ�I2Cռ�ã����Բ�����TA0
	TA1_PWM_Init('a',1,'F',0); //TA1��1ͨ����Ӧ P2.2      �Զ���ʼ��P2.2
	TA1_PWM_SetPeriod(50);				//�趨PWM����
	//ADC��ʼ��
	ADC10_init();
    TCA6416A_Init();			// ��ʼ��IO��չ��
	HT1621_init();				// ��ʼ��lcd_128
	LCD_Init();              	//���� ��ʾĬ��ֵ
	WDT_init();
	while(1)
	{
		getTemp();
	     PinIN();
		I2C_IODect();                            //�����¼����

	     calPermill();                          //������Ҫ��ռ�ձ�
	  	TA1_PWM_SetPermill(1,permill);		     //����PWMռ�ձ�

		 _bis_SR_register(LPM0_bits);             //��������
	}

}
/**************************ADC10*****************************/
void ADC10_init()
{
	ADC10CTL0 &= ~ENC;

	/******** ����***2.5v�ο���ѹ***64x��������ʱ��*�ο���ѹģʽ****/
//	ADC10CTL0 =  ADC10ON + REFON +REF2_5V+ ADC10SHT_3 + SREF_1;
	ADC10CTL0 =  ADC10ON + REFON + ADC10SHT_3 + SREF_1;
	/*��ͨ�����β���*****ר��ADCʱ��****ʱ�ӷ�Ƶ****������ʼ�ź�***P1.1/A1 CH1***/
    ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_3 + SHS_0 + INCH_1;
      __delay_cycles(30000);
    ADC10CTL0 |= ENC;
}
/*************************ADC����***********************************************/
void getTemp()
{
	static int i=0;                     //����Ϊ�ֲ���̬������ֻ��ʼ��һ��
    static int t[10]={0};

    ADC10CTL0 |= ENC + ADC10SC;            // ������ʼ�ź�
    __delay_cycles(1000);              	   //�ȴ�ADCת�����
    t[i] = (int)ADC10MEM;                  //�õ�����ֵ

	i++;       							//����ֵ�����ϴ�Ϊ�ȶ���ʮ�β���ȡƽ��
	if(i>9)
		  i=0;
	int j;
	for(j=0,temp=0;j<10;j++)
	{
		  temp+=t[j];
	}
	temp /=10;
//	    ADC10CTL0 |= ENC + ADC10SC;            // ������ʼ�ź�
//	    __delay_cycles(1000);              	   //�ȴ�ADCת�����
//	    temp = (int)ADC10MEM;                  //�õ�����ֵ
}



/********��ʼ��LCD��ʾ��ص�Ӳ������Ԥ��̶��������ʾ����**********/

void LCD_Init()
{
	LCD_DisplayNum(Votage);        // ��ѹ��ֵ
	LCD_DisplaySeg(_LCD_DOT4);     // С����
	LCD_DisplaySeg(_LCD_V);        // ��ѹ���š�V��
	LCD_DisplaySeg(_LCD_DC);       // ֱ�����š�DC��
	HT1621_Reflash(LCD_Buffer);    // ˢ���Դ�
}

/********WDT��ʼ������**********/
void WDT_init()
{
	//-----�趨WDTΪ-----------
	WDTCTL=WDT_ADLY_16;
	//-----WDT�ж�ʹ��--------
    IE1 |= WDTIE;
}

/***********WDT��ʱ�жϺ���*************/
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
__bic_SR_register_on_exit(LPM0_bits );
}

/**********I2C��չIO�ж��¼�������***************/
void I2C_IO10_Onclick()
{
	Votage += 1;
	votageCheck();
//	LCD_DisplayNum(temp);       //  ������temp
	LCD_DisplayNum(Votage);
	HT1621_Reflash(LCD_Buffer);
}
void I2C_IO11_Onclick()
{
	Votage -= 1;
	votageCheck();
	LCD_DisplayNum(Votage);
//	LCD_DisplayNum(goal);       //  ������goal
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
    goal= (a * Votage/10 + b)/10 ;                  //����Ŀ��ֵ
}
void I2C_IODect()			                 //��ⰴ���¼�
{
	static unsigned char KEY_Past=0,KEY_Now=0;
	KEY_Past=KEY_Now;
	//----�ж�I2C_IO10������KEY1�����Ƿ񱻰���------
	if((TCA6416A_InputBuffer&BIT8) == BIT8)
		KEY_Now |=BIT0;
	else
		KEY_Now &=~BIT0;
	if(((KEY_Past&BIT0)==BIT0)&&(KEY_Now&BIT0) !=BIT0)
		I2C_IO10_Onclick();
	//----�ж�I2C_IO11������KEY2�����Ƿ񱻰���------
	if((TCA6416A_InputBuffer&BIT9)== BIT9)
		KEY_Now |=BIT1;
	else
		KEY_Now &=~BIT1;
	if(((KEY_Past&BIT1)==BIT1)&&(KEY_Now&BIT1)!=BIT1)
		I2C_IO11_Onclick();
	//----�ж�I2C_IO12������KEY3�����Ƿ񱻰���------
	if((TCA6416A_InputBuffer&BITA) == BITA)
		KEY_Now |=BIT2;
	else
		KEY_Now &=~BIT2;
	if(((KEY_Past&BIT2)==BIT2)&&(KEY_Now&BIT2) ==0)
	{
		I2C_IO12_Onclick();
	}
	//----�ж�I2C_IO13������KEY4�����Ƿ񱻰���------
	if((TCA6416A_InputBuffer&BITB) ==  BITB)
		KEY_Now |=BIT3;
	else
		KEY_Now &=~BIT3;
	if(((KEY_Past&BIT3) == BIT3)&& (KEY_Now&BIT3) == 0)    //
	{
		I2C_IO13_Onclick();
	}
}
/************************PI�������ռ�ձ�********************************************/
//���������Ѿ��õ�tempֵ��������V��temp�Ķ�Ӧ��ϵ  temp=a*��ʵ��ѹ+b
//  temp��Ŀ��ֵ  ��  goal = a*Voatge(Ŀ���ѹ12-120)+b
void calPermill()
{
	  static int last_error;
	  static int last_last_error;

	  int error    = goal - temp;                  //�������� 100
	  int d_error  = error - last_error;              //��������10
	  int dd_error = error - 2*last_error + last_last_error;

	  permill += Kp * d_error/10 + Ki * error/10 + Kd * dd_error;   //PID����

	  last_last_error = last_error;
	  last_error = error;

	  if(permill>800)                           //����Ϸ��Լ��  ��20%����
	    permill=800;
	  else if(permill<0)
		permill=0;
}

