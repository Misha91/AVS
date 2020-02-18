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
#include <stdio.h>
#include "stm32_eval.h"
#include "serial_driver.h"
//#include <stm32f32xx_tim.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void delay(vu32 nCount);
void LED_Init(void);
/* Private functions ---------------------------------------------------------*/

/**
 * Main program.
 * @return Nothing.
 */
int main(void) {

    /* Initialize all LEDs */
    LED_Init();

    /* Initialize COM2 == USART3 (USB <==> Serial Port) */
    ComPort_Init(COM2);

    /* Configure COM2 as stdin, stdout and stderr */
    ComPort_ConfigStdio(COM2, COM2, COM2);

    /* INITIALIZE ECODER */
    /* It is not that hard, no help today. */
    /* Look into SCKit schematics for the encoder connection */
    /* Read STM32F207 datasheet and reference manual */
    /* Do not forget to configure the encoder pins */
    /* Do not forget to configure clock for a counter */
    
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
    uint32_t last = 0;
    iprintf("\nRun.");
    while (1) {
        STM_EVAL_LEDToggle(LED1);
        uint32_t temp = TIM_GetCounter(TIM4);
        if (temp != last){
            last = temp;
            char tmp[24];
            sprintf(tmp, "%d ", last);
            iprintf(tmp);
        }
        
        
        /* USE ECODER :) */
        /* Look at the A4M38AVS web pages for your assignment*/
        
        delay(0x1FFFFF);
    }
}

/**
 * Initialize all LEDs 
 */
void LED_Init(void) {
    Led_TypeDef led;

    for (led = LED1; led < (LED1 + LEDn); led++) {
        /* Initialize SCKit's LEDs */
        STM_EVAL_LEDInit(led);
        /* Set LEDs off */
        STM_EVAL_LEDOff(led);
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
