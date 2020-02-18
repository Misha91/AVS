/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f2xx_it.h"
#include "main.h"
#include "sckit_debug.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint32_t sEE_TIMEOUT_UserCallback(void) {

    print_assert("STM", "EEPROM Timeout", NULL, 0);

    return 1;
}


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) __attribute__((naked));

void HardFault_Handler(void) {
    __asm(
            "movs         r0, #4\n"
            "mov          r1, lr\n"
            "tst          r0, r1\n"
            "beq          hf_stacking_used_MSP\n"
            "mrs          r0, psp\n"
            "ldr          r1,=print_hardFault\n"
            "bx           r1\n"
            "hf_stacking_used_MSP:\n"
            "mrs          r0, msp\n"
            "ldr          r1,=print_hardFault\n"
            "bx           r1\n"
            );

}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void) {

    /* Go to infinite loop when Memory Manage exception occurs */

    print_assert("Fault", "MemManage", NULL, 0);
    halt();
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void) {

    /* Go to infinite loop when Bus Fault exception occurs */
    print_assert("Fault", "BusFault", NULL, 0);
    halt();

}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {

    /* Go to infinite loop when Usage Fault exception occurs */
    print_assert("Fault", "UsageFault", NULL, 0);
    halt();

}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  /* Update the LocalTime by adding SYSTEMTICK_PERIOD_MS each SysTick interrupt */
}

/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
//  if(EXTI_GetITStatus(ETH_LINK_EXTI_LINE) != RESET)
//  {
//    Eth_Link_ITHandler(DP83848_PHY_ADDRESS);
//    /* Clear interrupt pending bit */
//    EXTI_ClearITPendingBit(ETH_LINK_EXTI_LINE);
//  }
    if (EXTI_GetITStatus(SW1_BUTTON_EXTI_LINE) != RESET) {
        // button1_from_isr();
        /* Clear the EXTI line 15 pending bit */
        EXTI_ClearITPendingBit(SW1_BUTTON_EXTI_LINE);
    }

    if (EXTI_GetITStatus(SW2_BUTTON_EXTI_LINE) != RESET) {
        // button2_from_isr();
        /* Clear the EXTI line 15 pending bit */
        EXTI_ClearITPendingBit(SW2_BUTTON_EXTI_LINE);
    }    
}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
