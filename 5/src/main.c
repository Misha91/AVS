/**
 ******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    07-October-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32_eval.h"
#include "serial_driver.h"
#include "stdio.h"

#define LCD_MAX_CHARS (int)(LCD_PIXEL_WIDTH/ LCD_FONT_WIDTH)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void delay(vu32 nCount);
void InitPeripherals(void);
void InitADC(void);
void InitDAC(void);
/* Private functions ---------------------------------------------------------*/

/**
 * Main program.
 * @return Nothing.
 */
int main(void) {

    /* Initialize SCKit's LEDs, buttons, LCD and COM2 */
    InitPeripherals();

    //LCD_Display(LCD_Bitmap_SCKIT);

    /**
     * BNC1: ADC1_Channel3
     * BNC2: DAC_Channel2
     * POT:  ADC2_Channel10
     */

    /* ADC initialization */
    InitADC();
    
    /* DAC initialization */
    InitDAC();
    int c;
    /* Program loop */
    while (1) {

        /* Now you can read from ADC data register */
        /* and write to DAC data register */
        LCD_Clear();
        char sTmp[10];
        STM_EVAL_LEDToggle(LED1);
        int pot = 0;
        int res = 0;
        for (c = 0; c < 10; c++){
            pot += Get_ADC_Converted_Value(2);
            delay(0xFF);
        }
        pot /= 10;
        snprintf(sTmp, LCD_MAX_CHARS, "Pot: %d", pot);
        LCD_DisplayStringLine(Line1, sTmp); 
        DAC_SetChannel2Data(DAC_Align_12b_R, pot);
        
        for (c = 0; c < 10; c++){
            res += Get_ADC_Converted_Value(1);
            delay(0xFF);
        }
        res /= 10;
        snprintf(sTmp, LCD_MAX_CHARS, "BNC: %d", res);
        LCD_DisplayStringLine(Line2, sTmp); 
        delay(0x4FFFFF);
    }
}

/**
 * Initialize ADC
 */
void InitADC(void) {

    /* ADC configuration */
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    /* Enable GPIO clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

    /* ADC Peripheral clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);

    /* Configure ADC12 Channel3 pin as analog input */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure ADC123 Channel10 pin as analog input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Reset ADC */
    ADC_DeInit();

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* Configure the ADC1 */
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Add channel 3 to ADC1 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_3Cycles);

    /* Configure the ADC2 */
    ADC_Init(ADC2, &ADC_InitStructure);

    /* Add channel 10 to ADC2 */
    ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);

    /* Enable ADC1 and ADC2 */
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    
    ADC_SoftwareStartConv(ADC1);
    ADC_SoftwareStartConv(ADC2);

}

/**
 * Initialize DAC
 */
void InitDAC(void) {

    /* DAC configuration */
    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef DAC_InitStructure;

    /* DAC Pin clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* DAC Peripheral clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    /* DAC channel 2 (DAC_OUT2 = PA.5) configuration */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Reset DAC */
    DAC_DeInit();

    /* DAC channel2 Configuration */
    DAC_StructInit(&DAC_InitStructure);
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_2, &DAC_InitStructure);

    /* Enable DAC channel2 */
    DAC_Cmd(DAC_Channel_2, ENABLE);
}

/**
 * Initialize SCKit's LEDs, buttons, LCD and COM2
 */
void InitPeripherals(void) {
    Led_TypeDef led;

    for (led = LED1; led < (LED1 + LEDn); led++) {
        /* Initialize SCKit's LEDs */
        STM_EVAL_LEDInit(led);
        /* LEDs off */
        STM_EVAL_LEDOff(led);
    }

    /* Initialize buttons */
    STM_EVAL_PBInit(BUTTON_SW1, BUTTON_MODE_GPIO);
    STM_EVAL_PBInit(BUTTON_SW2, BUTTON_MODE_GPIO);

    /* Configure Serial port */
    ComPort_Init(COM2); // J15 - UART3
    /* Configure stdin,  stdout, stderr */
    ComPort_ConfigStdio(COM2, COM2, COM2);

    /* Initialize LCD */
    LCD_Init();
    LCD_SetBacklight(100);
}

/**
 * Delay function
 * @param nCount = Number of cycles to delay.
 */
void delay(vu32 nCount) {
    for (; nCount != 0; nCount--);
}

int Get_ADC_Converted_Value(int chn)
{
    if (chn == 1){
        ADC_SoftwareStartConv(ADC1);
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
        {
                return ADC_GetConversionValue(ADC1);
        }
    }
    
    else if (chn == 2){
        ADC_SoftwareStartConv(ADC2);
        while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC))
        {
                return ADC_GetConversionValue(ADC2);
        }
    }  
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
