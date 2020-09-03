#include "stm32f10x.h"
#include "bsp_ov7725.h"
#include "bsp_ili9341_lcd.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "lcd.h"
#include <misc.h>
#include <stdio.h>

#define RCV_BUFFER_SIZE 4

extern uint8_t Ov7725_vsync;

GPIO_InitTypeDef GPIO_InitStructure;
ADC_InitTypeDef  ADC_InitStructure;
TIM_TimeBaseInitTypeDef TIM_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
USART_InitTypeDef USART_InitStructure;

float ADC1_ConvertedValueLocal;
float ADC2_ConvertedValueLocal;
char c[8];
char d[10];
char e[10];
char f[30];
float limitOfHumidity = 30;
float limitOfLight = 150;
volatile uint32_t ticks;
volatile uint32_t lastTicks;
volatile uint32_t startRunTicks;
volatile uint32_t second;
volatile uint32_t minute;
volatile uint32_t hour;
static volatile char rcvBuffer[5];
static volatile char rcvMessage[5];
static volatile uint32_t rcvIndex = 0;
static volatile uint8_t rcvMessageFlag = 0;
static volatile uint8_t rcvMessageSize = 0;
int i;
int j;
uint8_t checkedLight = 0;
uint8_t checkedHumidity = 0;
uint8_t count = 0;
uint8_t stop = 0;
const unsigned char command[] = " \n0001:get brightness\n0002:get humidity\n0003:find bright place\n0004:add water\n0005:take photo\n";
uint16_t bright[30];
uint16_t max;
uint16_t maxpt;
uint8_t IR1 = 0;
uint8_t IR2 = 0;


u16 readADC1(void);
u16 readADC2(void);
void GPIO_Config(void);
void TIM_Config(void);
void TIM_NVIC_Config(void);
void TIM2_IRQHandler(void);
void ADC_Config(void);
void EXTI_Config(void);
void NVIC_Config(void);
void SysTick_Handler(void);
void USART_Config(void);
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);
void USART1_IRQHandler(void);

int main(void)

{	
	ADC_Config();
	LCD_Init();
	
	TIM_Config();
	TIM_NVIC_Config();
	GPIO_Config();
	EXTI_Config();
	NVIC_Config();
	SysTick_Config(SystemCoreClock/10);
	USART_Config();
	lastTicks = 0;
	
	Ov7725_GPIO_Config();
	while(Ov7725_Init() != SUCCESS);
	VSYNC_Init();	
	Ov7725_vsync = 0;
	
	LCD_DrawString(0,0, "Brightness(0-400):");
	LCD_DrawString(0,20,"Humidity(0-100):  ");
	
	while(1)
	{
		/*
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) == 0)
		{
			FIFO_PREPARE;
			ImagDisp();
			Ov7725_vsync = 0;			
		}
		*/
		if(ticks%10 == 0)
		{
			ADC1_ConvertedValueLocal = 409 - (readADC1()/10);
			sprintf(d, "%03g      ", ADC1_ConvertedValueLocal);
			LCD_DrawString(160,0,d);
			
			ADC2_ConvertedValueLocal = 200 - readADC2()/20;
			sprintf(e, "%03g      ", ADC2_ConvertedValueLocal);
			LCD_DrawString(160,20,e);
		}
		
		if(409 - (readADC1()/10) < limitOfLight && stop == 0 && GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == 1)
		{
			TIM_Cmd(TIM3, ENABLE);
			TIM_OCInitStructure.TIM_Pulse = 99;
			TIM_OC1Init(TIM3, &TIM_OCInitStructure);
			LCD_DrawString(0,40,"RUN");
			while(!(409 - (readADC1()/10) >= limitOfLight || stop == 1 || GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == 0))
			{
				ADC1_ConvertedValueLocal = 409 - (readADC1()/10);
				sprintf(d, "%03g ", ADC1_ConvertedValueLocal);
				LCD_DrawString(160,0,d);
			}
			//TIM_Cmd(TIM3, DISABLE);
			TIM_OCInitStructure.TIM_Pulse = 1000;
			TIM_OC1Init(TIM3, &TIM_OCInitStructure);
			LCD_DrawString(0,40,"    ");
			
			ADC1_ConvertedValueLocal = 409 - (readADC1()/10);
			sprintf(d, "%03g ", ADC1_ConvertedValueLocal);
			LCD_DrawString(160,0,d);
		}
		
		if(200 - readADC2()/20 < limitOfHumidity)
		{
			TIM_Cmd(TIM4, ENABLE);
			TIM_OCInitStructure.TIM_Pulse = 499;
			TIM_OC1Init(TIM4, &TIM_OCInitStructure);
			LCD_DrawString(0,60,"Adding Water");
			while(200 - readADC2()/20 < limitOfHumidity)
			{
				ADC2_ConvertedValueLocal = 200 - readADC2()/20;
				sprintf(e, "%03g      ", ADC2_ConvertedValueLocal);
				LCD_DrawString(160,20,e);
			}
			//TIM_Cmd(TIM4, DISABLE);
			TIM_OCInitStructure.TIM_Pulse = 1000;
			TIM_OC1Init(TIM4, &TIM_OCInitStructure);
			
			LCD_DrawString(0,60,"             ");
			
			ADC2_ConvertedValueLocal = 200 - readADC2()/20;
			sprintf(e, "%03g      ", ADC2_ConvertedValueLocal);
			LCD_DrawString(160,20,e);
			
		}
		
		if(rcvMessageFlag == 1 && count == 0)//set curr time
		{
			hour = rcvMessage[1]+rcvMessage[0]*10-528;
			minute = rcvMessage[3]+rcvMessage[2]*10-528;
			if(hour >= 24 ||hour < 0||minute < 0 || minute >= 60){
				
					const unsigned char errormsg[] = " Invalid! Please enter the current time:(hhmm) again!\r";
					UARTSend(errormsg, sizeof(errormsg));
					USART_SendData(USART1, '\n');
					rcvMessageFlag = 0;
			}else{
				sprintf(c, "%02i:%02i:00 ", hour, minute);
				LCD_DrawString(0,300,c);
				if(hour > 6 && hour < 18)
				{
				LCD_DrawString(100,300,"DAY  ");
				}else{
				LCD_DrawString(100,300,"NIGHT");
				}
				rcvMessageFlag = 0;
				count = 1;
				UARTSend(command, sizeof(command));
			}
		}
		
		if(rcvMessageFlag == 1 && count == 1)//get data
		{
			if(rcvMessage[3] == '1'){
				ADC1_ConvertedValueLocal = 409 - readADC1()/10;
				sprintf(f, "Current brightness: %03g\r", ADC1_ConvertedValueLocal);
				UARTSend(f, sizeof(f));
				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
				USART_SendData(USART1, '\n');
				rcvMessageFlag = 0;
			}
			else if(rcvMessage[3] == '2'){
				ADC2_ConvertedValueLocal = 200 - readADC2()/20;
				sprintf(f, "Current humidity: %03g\r", ADC2_ConvertedValueLocal);
				UARTSend(f, sizeof(f));
				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
				USART_SendData(USART1, '\n');
				rcvMessageFlag = 0;
			}
			else if(rcvMessage[3] == '3'){
				/*
				TIM_Cmd(TIM3, ENABLE);
				TIM_OCInitStructure.TIM_Pulse = 199;
				TIM_OC1Init(TIM3, &TIM_OCInitStructure);
				LCD_DrawString(0,40,"RUN");
				while(!(409 - (readADC1()/10) >= limitOfLight || stop == 1 || GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == 0))
				{
					ADC1_ConvertedValueLocal = 409 - (readADC1()/10);
					sprintf(d, "%03g      ", ADC1_ConvertedValueLocal);
					LCD_DrawString(160,0,d);
				}
				//TIM_Cmd(TIM3, DISABLE);
				TIM_OCInitStructure.TIM_Pulse = 1000;
				TIM_OC1Init(TIM3, &TIM_OCInitStructure);
				LCD_DrawString(0,40,"    ");
				rcvMessageFlag = 0;
				*/
				
				
				j = 0;
				max = 0;
				IR1 = IR2 = 0;
				TIM_Cmd(TIM3, ENABLE);
				TIM_OCInitStructure.TIM_Pulse = 99;
				TIM_OC1Init(TIM3, &TIM_OCInitStructure);
				LCD_DrawString(0,40,"Finding Light");
				
				while(IR2 == 0)
				{
					if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 1)
					{
						IR1 = 1;
					}
					if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 1)
					{
						IR2 = 1;
					}
					if(IR1 == 1)
					{
						startRunTicks = ticks;
						GPIOA->BSRR = GPIO_Pin_7;
						TIM_OCInitStructure.TIM_Pulse = 299;
						TIM_OC1Init(TIM3, &TIM_OCInitStructure);
						if(ticks - startRunTicks >= 5)
						{
							LCD_DrawDot(0,60+readADC1()/100,WHITE);
							bright[j++] = 409 - (readADC1()/10);
							if(max < bright[j])
							{
								max = bright[j];
								maxpt = j;
							}
						}
					}
					else
					{
						startRunTicks = ticks;
						GPIOA->BRR = GPIO_Pin_7;
						while(ticks - startRunTicks >= (j-maxpt)*5 && IR1 != 1);
						TIM_OCInitStructure.TIM_Pulse = 1000;
						TIM_OC1Init(TIM3, &TIM_OCInitStructure);
					}
				}
				TIM_OCInitStructure.TIM_Pulse = 1000;
				TIM_OC1Init(TIM3, &TIM_OCInitStructure);
				LCD_DrawString(0,40,"              ");
				rcvMessageFlag = 0;
				
			}
			else if(rcvMessage[3] == '4')
			{
				TIM_Cmd(TIM4, ENABLE);
				TIM_OCInitStructure.TIM_Pulse = 499;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				LCD_DrawString(0,60,"Adding Water");
				lastTicks = ticks;
				while(!(ticks - lastTicks >= 20 ));
				//TIM_Cmd(TIM4, DISABLE);
				TIM_OCInitStructure.TIM_Pulse = 1000;
				TIM_OC1Init(TIM4, &TIM_OCInitStructure);
				
				LCD_DrawString(0,60,"             ");
				
				ADC2_ConvertedValueLocal = 200 - readADC2()/20;
				sprintf(e, "%03g      ", ADC2_ConvertedValueLocal);
				LCD_DrawString(160,20,e);
			}
			else if(rcvMessage[3] == '5')
			{
				if( Ov7725_vsync == 2 )
					{
						FIFO_PREPARE;
						ImagDisp();
						Ov7725_vsync = 0;			
					}		
			}
			rcvMessageFlag = 0;
		}
		
	}
}
	


void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	
}

void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
			rcvBuffer[rcvIndex] = USART_ReceiveData(USART1);
			
			//Buffer full or end of current message
			if(rcvIndex+1>=RCV_BUFFER_SIZE || rcvBuffer[rcvIndex]=='\n')
			{
				//Placing message in message buffer - setting flag
				rcvMessageFlag = 1;
				for(i=0;i<RCV_BUFFER_SIZE;i++)
				{
					if(i<=rcvIndex)
					{
						rcvMessage[i]=rcvBuffer[i];
					}
					else
					{
						rcvMessage[i]=0x00;
					}
				}

				//Clear receive buffer
				for(i=0;i<RCV_BUFFER_SIZE;i++)
				{
					rcvBuffer[i]=0x00;
				}
				rcvMessageSize=rcvIndex;
				rcvIndex=0;
			}
			else
			{
				rcvIndex++;
			}
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}


void SysTick_Handler(void)
{
	if(count == 1)
	{
		if((++ticks)%10== 0)
		{
			second = (second < 59) ? (second + 1) : 0;
			minute = (second == 0) ? ((minute < 59) ? (minute + 1) : 0) : minute;
			if(!(hour == 0 && minute == 0)){
			hour = (minute == 0) ? ((hour < 24) ? (hour + 1) : 0) : hour;
			}
			sprintf(c, "%02i:%02i:%02i ", hour, minute, second);
			LCD_DrawString(0,300,c);
		}
	}
	
}

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		if(stop == 0)
		{
			LCD_DrawString(0,40,"     STOP");
			stop = 1;
		}else{
			stop = 0;
			LCD_DrawString(0,40,"          ");
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}


void GPIO_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//K1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//K2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//IR
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//dir
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIOB->BSRR = GPIO_Pin_7;
	GPIOA->BSRR = GPIO_Pin_11;
}
void TIM_Config(void)
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//TIM2
	TIM_InitStructure.TIM_Prescaler = 40000;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period = 5000;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//for interrupt
	//TIM3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_InitStructure.TIM_Period = 1000; //<- TIMx_ARR register
	TIM_InitStructure.TIM_Prescaler = 72;
	TIM_InitStructure.TIM_ClockDivision = 0;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_InitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 99; //<- TIMx_CCRx register
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	//TIM4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseInit(TIM4, &TIM_InitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 499; //<- TIMx_CCRx register
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
}

void TIM_NVIC_Config(void)
{
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
}
void ADC_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  /* PCLK2 is the APB2 clock */
  /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);

  /* Enable ADC1 clock so that we can talk to it */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  /* Put everything back to power-on defaults */
  ADC_DeInit(ADC1);
	ADC_DeInit(ADC2);

  /* ADC1 Configuration ------------------------------------------------------*/
  /* ADC1 and ADC2 operate independently */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  /* Disable the scan conversion so we do one at a time */
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  /* Don't do contimuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  /* Start conversin by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  /* Conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  /* Say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfChannel = 1;

  /* Now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
	
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Cmd(ADC2, ENABLE);
	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2));
	ADC_StartCalibration(ADC2);
	while(ADC_GetCalibrationStatus(ADC2));
}

u16 readADC1(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_55Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ADC_GetConversionValue(ADC1);
}

u16 readADC2(void)
{
  ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC2, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ADC_GetConversionValue(ADC2);
}
void EXTI_Config(void)
{
	GPIO_EXTILineConfig(RCC_APB2Periph_GPIOA, GPIO_Pin_0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
void NVIC_Config(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART_Config(void)
{
	const unsigned char welcome_str[] = " Please enter the current time:(hhmm)\r";
	/* Enable USART1 and GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART1 configuration*/
  USART_InitStructure.USART_BaudRate = 9600;        // Baud Rate
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
	
	/* Enable the USART1 Receive interrupt: this interrupt is generated when the
         USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	/* print welcome information */
  UARTSend(welcome_str, sizeof(welcome_str));
}

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART1, (uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}

