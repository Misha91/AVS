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
#include "../SC-IDE/lcd.h"

#define FREQ 0xAFFFF

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/


/**
 * Main program.
 * @return Nothing.
 */

void encInit(void);
void timerInit(void);

int main(void) {
    initLCD();    
    
    STM_EVAL_LEDInit(LED5);
    encInit();
    timerInit();
    while (1) {    
        //clearLCD();
        
        vertLine(FREQ, 0);
        horLine(FREQ, 0);
        sawLine(FREQ, 0);
        drawRect(FREQ);
        writeTimer(62,3, FREQ);
        writeEnc(62,3, FREQ);
        
    }
}

/**
 * Delay function
 * @param nCount = Number of cycles to delay.
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

void encInit(void){
    GPIO_InitTypeDef GPIO_InitStructure;     
    GPIO_StructInit(&GPIO_InitStructure); /* !! important !!, initialization of the structure. */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    
    RCC->AHB1ENR |= RCC_AHB1LPENR_GPIOELPEN;
    
    //timer setup
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 0;    
    TIM4->ARR = 0xffff;	          
    TIM4->DIER = TIM_DIER_UIE; 
    TIM4->CR1 = TIM_CR1_CEN;
    //NVIC_EnableIRQ(TIM4_IRQn); 
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM4, ENABLE);
}

void timerInit(void){
    RCC->AHB1ENR |= RCC_AHB1LPENR_GPIOELPEN;
    
    //timer setup
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 65535>>5;    
    TIM3->ARR = 65535;	          
    TIM3->DIER = TIM_DIER_UIE; 
    TIM3->CR1 = TIM_CR1_CEN;
    //NVIC_EnableIRQ(TIM3_IRQn); 
    
}