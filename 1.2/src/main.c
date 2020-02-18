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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void delay(vu32 nCount);
/* Private functions ---------------------------------------------------------*/
volatile uint8_t k = 0;
char dataToShow[] = {1,0,0,1,0,1,1,0};
/**
 * Main program.
 * @return Nothing.
 */
int main(void) {

    RCC->AHB1ENR |= RCC_AHB1LPENR_GPIOELPEN;
    
    //timer setup
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 20000;    
    TIM3->ARR = 2;	          
    TIM3->DIER = TIM_DIER_UIE; 
    TIM3->CR1 = TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn); 
    
    //LED setup
    STM_EVAL_LEDInit(LED7);  
    
    while (1) {           
    }
}

/**
 * Delay function
 * @param nCount = Number of cycles to delay.
 */
void delay(vu32 nCount) {
    for (; nCount != 0; nCount--);
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

//IRQ for TIMER 
void TIM3_IRQHandler()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        k++;
        if (k == 8) k = 0;
        if (dataToShow[k]) STM_EVAL_LEDOn(LED7);
        else STM_EVAL_LEDOff(LED7);
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        //GPIO_ToggleBits(GPIOE, GPIO_Pin_14);
    }
}