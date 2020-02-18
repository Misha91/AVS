#include "lcd.h"
#include <math.h>


void initLCD(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);     
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
    
    SPI_InitTypeDef SPI_InitStructure;
    
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
 
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(LCD_SPI, &SPI_InitStructure);
 
    /*!< Enable the LCD_SPI  */
    SPI_Cmd(LCD_SPI, ENABLE);
    startLCD();
    
}

void LCD_SendByte(uint8_t byte, char cmnd){
    
    if (cmnd && (GPIOD->ODR & 0x4000)){
        GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    }
    
    if (!(cmnd) && !(GPIOD->ODR & 0x4000)){
        GPIO_SetBits(GPIOD, GPIO_Pin_14);
    }    
    
    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(LCD_SPI, byte);
    delay(0x30);
}

void startLCD(void){
    GPIO_ResetBits(GPIOD, GPIO_Pin_11);    
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);
    //delay(0x0F);
    GPIO_SetBits(GPIOD, GPIO_Pin_11);
    GPIO_SetBits(GPIOC, GPIO_Pin_7);
    //delay(0x0F);
    //GPIO_SetBits(GPIOD, GPIO_Pin_11);
    LCD_SendByte(CMD_SET_BIAS_7, 1);
    LCD_SendByte(CMD_SET_ADC_NORMAL, 1);
    LCD_SendByte(CMD_SET_COM_NORMAL, 1);
    LCD_SendByte(CMD_SET_DISP_START_LINE, 1);
    LCD_SendByte(CMD_SET_POWER_CONTROL  | 0x4, 1);
    //delay(0x0F);
    LCD_SendByte(CMD_SET_POWER_CONTROL  | 0x6, 1);
   // delay(0x0F);
    LCD_SendByte(CMD_SET_POWER_CONTROL  | 0x7 , 1);
   //delay(0x0F);
    LCD_SendByte(CMD_SET_RESISTOR_RATIO  | 0x6 , 1);
    LCD_SendByte(CMD_SET_ADC_REVERSE, 1);
    LCD_SendByte(CMD_DISPLAY_ON, 1);
    //delay(0x0F);
    LCD_SendByte(CMD_SET_VOLUME_FIRST, 1);
    LCD_SendByte(CMD_SET_VOLUME_SECOND | (0x10 & 0x3f), 1); 
    //LCD_SendByte(CMD_SET_DISP_START_LINE, 1);
    clearLCD();
}

void clearLCD(void){
    uint8_t p, c;
  //LCD_SendByte(CMD_SET_DISP_START_LINE, 1);
  for(p = 0; p < 8; p++) {

    //LCD_SendByte(CMD_SET_DISP_START_LINE | p, 1);
    LCD_SendByte(CMD_SET_PAGE | p, 1);
    LCD_SendByte(CMD_SET_COLUMN_LOWER, 1);
    LCD_SendByte(CMD_SET_COLUMN_UPPER , 1);
    for(c = 0; c < 132; c++) { 
      
      LCD_SendByte(0x00, 0);
    }    
  }
}

void writeNumb(uint8_t x, uint8_t y, int numb){
    char tmp[10];
    sprintf(tmp, "%d ", numb);
    writeLine(x, y, tmp, sizeof(tmp));
}

void writeWord(uint8_t x, uint8_t y, char line[]){
    writeLine(x, y, line, sizeof(line)/sizeof(char));
}

void writeEnc(uint8_t x, uint8_t y, vu32 speed){
    uint8_t c;
    TIM_SetCounter(TIM4, 65535/2);
    uint32_t init  = TIM_GetCounter(TIM4);
    uint32_t last = -1;
    //drawCircle(x, y);
    for (c = 0; c < 50; c++){
        uint32_t temp = TIM_GetCounter(TIM4);
        if (temp != last){
            int32_t shift = temp - init;
            int16_t x_new = x + shift;
            x_new = x_new < 0 ? 0 : x_new > 128 ? 128 : x_new;
            //drawCircle(x_new, y);
            last = temp;
            
        }
        else delay(0xFF);
        //writeLine(x, y+1, tmp2, sizeof(tmp2)/sizeof(char));
        delay(speed);
    }
    clearLCD();

}

void writeTimer(uint8_t x, uint8_t y, vu32 speed){
    uint8_t c;
    uint32_t last = -1;
    for (c = 0; c < 50; c++){
        uint32_t temp = TIM_GetCounter(TIM3);
        if (temp != last){
            int len = floor(log10(abs(temp)));
             
            last = temp;
            char tmp[10];
            sprintf(tmp, "%d      ", (int)temp);
            char tmp2[5];
            sprintf(tmp2, "%d     ", sizeof(tmp));
            writeLine(x - (int) 3*len, y, tmp, sizeof(tmp)/sizeof(char));
        }
        else delay(0xFF);
        //writeLine(x, y+1, tmp2, sizeof(tmp2)/sizeof(char));
        delay(speed);
    }
    clearLCD();
}



void writeLine(uint8_t x, uint8_t y, char *arr, int size){
    x += 4;
    LCD_SendByte(CMD_SET_PAGE | y, 1);
    int tmp = (x - 20) > 0 ? (x - 20) : 0;
    LCD_SendByte(CMD_SET_COLUMN_LOWER | (tmp & 0xf), 1);
    LCD_SendByte(CMD_SET_COLUMN_UPPER | ((tmp >> 4) & 0xf) , 1);
    uint8_t i, c;
    for(c = tmp; c < 132; c++) { 
        LCD_SendByte(0x00, 0);
    }    
    LCD_SendByte(CMD_SET_PAGE | y, 1);
    LCD_SendByte(CMD_SET_COLUMN_LOWER | (x & 0xf), 1);
    LCD_SendByte(CMD_SET_COLUMN_UPPER | ((x >> 4) & 0xf) , 1);
    for (i = 0; i < size; i++){
        uint8_t t;
        uint16_t a = (*(arr+i)-0x21)*5;
        if((a >= 0x41) && (a <= 0xEF)){ //
            for (t = 0; t < 5; t++){
                //uint8_t a = lcd_charset[i];
                LCD_SendByte(lcd_charset[a+t], 0);
            }
            LCD_SendByte(0x0, 0);
        }
        else 
	{
		LCD_SendByte(0x0, 0);
                LCD_SendByte(0x0, 0);
                LCD_SendByte(0x0, 0);
	}
    }
    
    
}

void horLine(vu32 speed, char led){
    uint8_t p, c;
  //LCD_SendByte(CMD_SET_DISP_START_LINE, 1);
  for(p = 0; p < 8; p++) {

    //LCD_SendByte(CMD_SET_DISP_START_LINE | p, 1);
    LCD_SendByte(CMD_SET_PAGE | p, 1);
    LCD_SendByte(CMD_SET_COLUMN_LOWER, 1);
    LCD_SendByte(CMD_SET_COLUMN_UPPER , 1);
    
    for(c = 0; c < 132; c++) { 

      LCD_SendByte(0x1, 0);
    }
    
    if (led) STM_EVAL_LEDOn(LED5);
    delay(speed);
        
    
    if (led) STM_EVAL_LEDOff(LED5);
    delay(speed);
    
  }
clearLCD();
}

void delay(vu32 nCount) {
    for (; nCount != 0; nCount--);
}

void drawCircle(uint8_t x,uint8_t y){
    //x += 4;
    LCD_SendByte(CMD_SET_PAGE | y, 1); 
    LCD_SendByte(CMD_SET_COLUMN_LOWER, 1);
    LCD_SendByte(CMD_SET_COLUMN_UPPER, 1);
    uint8_t c = 0;
    for(c = 0; c < 132; c++) { 
        LCD_SendByte(0x00, 0);
    }  
    
    LCD_SendByte(CMD_SET_COLUMN_LOWER | (x & 0xf), 1);
    LCD_SendByte(CMD_SET_COLUMN_UPPER | ((x >> 4) & 0xf) , 1);
    LCD_SendByte(0x3C, 0); //18
    LCD_SendByte(0x66, 0);
    LCD_SendByte(0xC3, 0);
    LCD_SendByte(0x81, 0);
    LCD_SendByte(0x81, 0);
    LCD_SendByte(0xC3, 0);
    LCD_SendByte(0x66, 0);
    LCD_SendByte(0x3C, 0);
}