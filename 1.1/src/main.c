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

/**
 * Main program.
 * @return Nothing.
 */
int main(void) {

    /* Hello, I'm a comment */

    // Hello, I'm also a comment, but not according to C 89   :(


    /***** Initialization using registers */
    /* Enable IO port E */
    RCC->AHB1ENR |= RCC_AHB1LPENR_GPIOELPEN;

    /* Reseting registers, pins 8 - 15 */
    GPIOE->MODER &= 0x0000FFFF; /* Two bits per pin */
    GPIOE->OSPEEDR &= 0x0000FFFF; /* Two bits per pin */
    GPIOE->OTYPER &= 0xFFFF00FF; /* One bit per pin */

    /* LED1, configure GPIOE (PE8: output, 50 MHz, push-pull) */
    GPIOE->MODER |= 0x00010000; /* 0x1, output */
    GPIOE->OSPEEDR |= 0x00020000; /* 0x2, 50 MHz */
    GPIOE->OTYPER |= 0x00000000; /* 0x00000000 for PP, 0x00000100 for OD = (1 << 8) */

    /* LED2, configure GPIOE (PE9: output, 50 MHz, push-pull) */
    GPIOE->MODER |= 0x00040000; /* 0x1 << 2 = 0x4, output  */
    GPIOE->OSPEEDR |= 0x00080000; /* 0x2 << 2 = 0x8, 50 MHz */
    GPIOE->OTYPER |= 0x00000000; /* 0x00000000 for PP, 0x00000200 for OD = (1 << 9) */


    ///***** Initialization using periph. library */
    /* Structure declaration */
    GPIO_InitTypeDef GPIO_InitStructure;
     /* hmm, I should be declared on function start :) */

    GPIO_StructInit(&GPIO_InitStructure); /* !! important !!, initialization of the structure. */
    
    /* Enable the GPIO_LED Clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);


    /***** Initialization using eval. board library */
    /* Init LED5 and LED6*/
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);
    
    /* SysTick */
    STM_EVAL_LEDInit(LED7);
    STM_EVAL_LEDInit(LED8);
    
    //Init BT2
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    
    //Init BT1
    GPIOC->MODER &= ~(0x30000000); /* Set Pin 14 to 0 */
    GPIOC->OSPEEDR &= ~(0x30000000); /* Speed 50 MHz */
    GPIOC->PUPDR &= ~(0x30000000); /* No Pull up/down */
    
    int bt1, bt2; //variables to store button read result
    
    while (1) {  
                     
        bt1 = GPIOC->IDR & 0x4000;        
        if (bt1) GPIOE->BSRRH |= 0x4000;           
        else if (bt1 == 0) GPIOE->BSRRL |= 0x4000;   
        
        bt2 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);        
        if ((bt2 == 1) ) GPIO_ResetBits(GPIOE, GPIO_Pin_15);        
        else if ((bt2 == 0) ) GPIO_SetBits(GPIOE, GPIO_Pin_15);
         
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
