#include "stm32f10x.h"
#include <stdio.h>

#define DHT11_OUT_1 GPIO_SetBits(GPIOA, GPIO_Pin_1);		//输出1
#define DHT11_OUT_0 GPIO_ResetBits(GPIOA, GPIO_Pin_1);	//输出0
#define DHT11_IN GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1)	//读取

uint16_t adc_conv_value = 0;
float pressure_value = 0.0;		//压力值
uint8_t temp_hum_buffer[5] ={0};
uint8_t lcd_0_buffer[17] = "Temp:  C Hum:  %";
uint8_t lcd_1_buffer[17] = "     G:         ";
#define E_0     GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define E_1     GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define RW_0    GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define RW_1    GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define RS_0    GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define RS_1    GPIO_SetBits(GPIOB, GPIO_Pin_9)

void delay1(void);
void delay(void);
void led_init(void);
void usart_init(void);
int fputc(int ch, FILE *f);
void motor_stepper_init(void);
void motor_stepper_m_0_90(void);
void motor_stepper_m_90_0(void);
void motor_stepper_u_0_45(void);
void motor_stepper_u_45_0(void);
void motor_stepper_d_0_90(void);
void motor_stepper_d_90_0(void);
void motor_stepper_d_0_n90(void);
void motor_stepper_d_n90_0(void);
void adc_init(void);
void dht11_init(void);
void dht11_config_output(void);
void dht11_config_input(void);
uint8_t dht11_readbyte(void);
void delay_us(__IO uint32_t nus);
void delay_ms(uint16_t nus);
uint8_t dht11_read_temp_hum(uint8_t *temp_hum);
void lcd1602_config(void);
void lcd1602_write_comm(uint8_t comm);
void lcd1602_write_data(uint8_t data);
void lcd1602_init(void);
void lcd1602_show_string(uint8_t addr_offset, uint8_t *string);
void motor_stepper_u_50(void);

int main(void){
  led_init();
	usart_init();
	motor_stepper_init();
	adc_init();
	lcd1602_init();    // LCD1602初始化
	
	printf("#### My System ####\r\n");
	printf("#### wpf ####\r\n");
	float pressure_value = 0.0;
	float pressure_value_pre = 0.0;
	
  while (1){
		if(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)){
			adc_conv_value = ADC_GetConversionValue(ADC1);
			ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		}
		
		//通过公式获取压力值
		pressure_value = (((float)adc_conv_value - 218.0) * (115.0 - 15.0) / (3883.0 - 218.0) + 15.0);
		printf("Press Value: %d\r\n", (uint16_t)(pressure_value * 10));
		
		if(dht11_read_temp_hum(temp_hum_buffer) == 1)
			printf("temp: %d  hum: %d\r\n", temp_hum_buffer[2], temp_hum_buffer[0]);
		else printf("DHT11 Error\r\n");
		
		delay1();
		lcd_0_buffer[5] = temp_hum_buffer[2] / 10 + '0';
    lcd_0_buffer[6] = temp_hum_buffer[2] % 10 + '0';
    lcd_0_buffer[13] = temp_hum_buffer[0] / 10 + '0';
    lcd_0_buffer[14] = temp_hum_buffer[0] % 10 + '0';
    lcd_0_buffer[16] = '\0';
    lcd1602_show_string(0, lcd_0_buffer);
 
    lcd_1_buffer[7] = (uint16_t)pressure_value / 10 + '0';
    lcd_1_buffer[8] = (uint16_t)pressure_value % 10 + '0';
    lcd_1_buffer[9] = '.';
    lcd_1_buffer[10] = (uint16_t)(pressure_value * 10) % 10 + '0';
    lcd_1_buffer[16] = '\0';
    lcd1602_show_string(1, lcd_1_buffer);
 
    if(pressure_value_pre != pressure_value){
			if(pressure_value >= 50.0){
				lcd_1_buffer[0] = ' ';
				lcd_1_buffer[1] = ' ';
				lcd_1_buffer[2] = ' ';
				lcd_1_buffer[13] = '-';
				lcd_1_buffer[14] = '-';
				lcd_1_buffer[15] = '>';
				lcd_1_buffer[16] = '\0';
				lcd1602_show_string(1, lcd_1_buffer);
   
				motor_stepper_u_50();
		}
		else // pressure_value < 50.0
		{
			lcd_1_buffer[0] = '<';
			lcd_1_buffer[1] = '-';
			lcd_1_buffer[2] = '-';
			lcd_1_buffer[13] = ' ';
			lcd_1_buffer[14] = ' ';
			lcd_1_buffer[15] = ' ';
			lcd_1_buffer[16] = '\0';
			lcd1602_show_string(1, lcd_1_buffer);
  }
   
  pressure_value_pre = pressure_value;
 }
  }
}

void motor_stepper_u_50(void){
	motor_stepper_m_0_90();
	motor_stepper_u_0_45();
	motor_stepper_m_90_0();
	motor_stepper_d_0_90();
	motor_stepper_m_0_90();
	motor_stepper_u_45_0();
	motor_stepper_m_90_0();
	motor_stepper_d_90_0();
}



void delay_us(__IO uint32_t nus){	//__IO：防止编译器自动优化掉
	while(nus--);
}

void delay_ms(uint16_t nms){
	delay_us(1000 * nms);
}

void dht11_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//打开时钟
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	DHT11_OUT_1;
}

void dht11_config_output(void){		//输出
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void dht11_config_input(void){	//输入
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t dht11_readbyte(void){
	uint8_t temp = 0;
	for(uint8_t i = 0; i < 8; ++i){
		while(DHT11_IN == 0);		//等低电平过去
		delay_us(40);
		if(DHT11_IN == 1){
			while(DHT11_IN == 1);
			temp |= (uint8_t)(0x01 << (7 - i));
		}
		else{
			temp &= (uint8_t)~(0x01 << (7 - i));
		}
	}
	return temp;
}

uint8_t dht11_read_temp_hum(uint8_t *temp_hum){	//读取温度和湿度
	dht11_config_output();
	DHT11_OUT_0;
	delay_ms(18);
	DHT11_OUT_1;
	delay_us(30);
	dht11_config_input();
	if(DHT11_IN == 0){
		while(DHT11_IN == 0);
		while(DHT11_IN == 1);
		temp_hum[0] = dht11_readbyte();
		temp_hum[1] = dht11_readbyte();
		temp_hum[2] = dht11_readbyte();
		temp_hum[3] = dht11_readbyte();
		temp_hum[4] = dht11_readbyte();
		
		dht11_config_output();
		DHT11_OUT_1;
		
		if(temp_hum[4] == temp_hum[0] + temp_hum[1] + temp_hum[2] + temp_hum[3])
			return 1;
		else return 0;
	}
	else return 0;
}

void adc_init(void){
	ADC_InitTypeDef ADC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);		//两分频  将频率降低两倍
	
	//引脚初始化
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign = 1;
	ADC_Init(ADC1, &ADC_InitStruct);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_1Cycles5);	//规则通道
	
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void motor_stepper_init(void){		//步进电机初始化函数
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);	
}

void delay(void){			//延时函数
	uint32_t cnt = 0xFFFF;
	while(cnt--);
}

void delay1(void){			//延时函数
	uint32_t cnt = 0xFFFFF;
	while(cnt--);
}

void motor_stepper_m_0_90(void){	//中间电机旋转0~90度
	//AB-B-BC
	//AB
	GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_SetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
	delay();
	//B
	GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_SetBits(GPIOC, GPIO_Pin_5);
	delay();
	//BC
	GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_7);
	GPIO_SetBits(GPIOC, GPIO_Pin_5 | GPIO_Pin_6);
	delay();
}

void motor_stepper_m_90_0(void){	//中间电机旋转90~0度
	//BC-B-AB
	//BC
	GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_7);
	GPIO_SetBits(GPIOC, GPIO_Pin_5 | GPIO_Pin_6);
	delay();
	//B
	GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_SetBits(GPIOC, GPIO_Pin_5);
	delay();
	//AB
	GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_SetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
	delay();
}

void motor_stepper_u_0_45(void){	//上方电机旋转0~45度
	//AB-B
	//AB
	GPIO_ResetBits(GPIOC, GPIO_Pin_10 | GPIO_Pin_11);
	GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);
	delay();
	//B
	GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11);
	GPIO_SetBits(GPIOC, GPIO_Pin_9);
	delay();
}

void motor_stepper_u_45_0(void){	//上方电机旋转45~0度
	//B-AB
	//B
	GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11);
	GPIO_SetBits(GPIOC, GPIO_Pin_9);
	delay();
	//AB
	GPIO_ResetBits(GPIOC, GPIO_Pin_10 | GPIO_Pin_11);
	GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9);
	delay();
}

void motor_stepper_d_0_90(void){	//下侧电机旋转0~90度
	//AB
	GPIO_SetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3);
	delay();
	//B
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3);
	delay();
	//BC
	GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_3);
	delay();
}

void motor_stepper_d_90_0(void){	//下侧电机旋转90~0度
	//BC
	GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_3);
	delay();
	//B
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3);
	delay();
	//AB
	GPIO_SetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3);
	delay();
}

void motor_stepper_d_0_n90(void){	//下侧电机旋转0~-90度
	//AB
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3);
	delay();
	//A
	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	GPIO_ResetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
	delay();
	//DA
	GPIO_SetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_3);
	GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_1);
	delay();
}

void motor_stepper_d_n90_0(void){	//下侧电机旋转-90~0度
	//DA
	GPIO_SetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_3);
	GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_1);
	delay();
	//A
	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	GPIO_ResetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
	delay();
	//AB
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3);
	delay();
}

void led_init(void){				//Led灯初始化函数
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);	//PB15
}

void lcd1602_config(void){
	GPIO_InitTypeDef GPIO_InitStruct;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	// PB1-PB11
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
 
	GPIO_ResetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
}

void lcd1602_write_comm(uint8_t comm){
	E_0;
	RW_0;
	RS_0;
	GPIOB->ODR &= ~(0x00FF << 1);
	GPIOB->ODR |= comm << 1;
	delay_ms(2);
	E_1;
	delay_ms(2);
	E_0;
}

void lcd1602_write_data(uint8_t data){
	E_0;
	RW_0;
	RS_1;
	GPIOB->ODR &= ~(0x00FF << 1);
	GPIOB->ODR |= data << 1;
	delay_ms(2);
	E_1;
	delay_ms(2);
	E_0;
}

void lcd1602_init(void){
	lcd1602_config();
	lcd1602_write_comm(0x38);//16*2显示，5*7点阵，8位数据接口
	lcd1602_write_comm(0x0C);//显示器开，光标关闭
	lcd1602_write_comm(0x06);//文字不动，地址自动+1
	lcd1602_write_comm(0x01);//清屏
	lcd1602_write_comm(0x80);//显示地址
}

void lcd1602_show_string(uint8_t addr_offset, uint8_t *string){
	lcd1602_write_comm(0x80 + (addr_offset * 0x40));
	for(uint8_t i = 0; string[i] != '\0'; i++){
		lcd1602_write_data(string[i]);
	}
}

void usart_init(void){			//串口初始化函数
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStruct);
	
	USART_Cmd(USART1, ENABLE);
}

int fputc(int ch, FILE *f){	//输出函数
	USART_SendData(USART1, (uint8_t)ch);
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
	return ch;
}
