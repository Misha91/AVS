/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32_eval.h"
#include "serial_driver.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void delay(vu32 nCount);
void initUART(void);
void initLED(void);
void initKeyboard(void);
/* Private functions ---------------------------------------------------------*/

/**
 * Main program.
 * @return Nothing.
 */
int main(void) {
    /* Welcome text messages */
    const char c_sWelcomePer[] = "This is HW Task 2'.\n";

    
    unsigned int i, k;

    initKeyboard();
    initUART();
    initLED();
    
    for (i = 0; i < strlen(c_sWelcomePer); i++) {
            /* Check Transmission Complete flag */
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
            /* Send byte */
        USART_SendData(USART3, c_sWelcomePer[i]);
    }
    
    uint16_t bitsReset[] = {GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10}; 
    uint16_t bitsSet[] = {GPIO_Pin_9 | GPIO_Pin_10, GPIO_Pin_8 | GPIO_Pin_10, GPIO_Pin_8 | GPIO_Pin_9}; 
    uint8_t c1, c2, c3, c4;
    char answ[] = {'1', '2', '3', 'F', '4', '5', 'N', '9', '0', '6', '7', '8'};
    char last;
    uint8_t lastR = 0;
    /* Program loop */
    while (1) {                 

            for (i = 0; i < 3; i ++){
                //STM_EVAL_LEDOn(LED5);
                GPIOB->BSRRL |= bitsSet[i];
                GPIOB->BSRRH |= bitsReset[i]; 
                delay(0x00FFFF);
                c1 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                c2 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
                c3 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
                c4 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
                if (!(c1 && c2 && c3 && c4)){
                        lastR = i;
                        uint8_t c = ((~(c1))*1 + (~(c2))*2 + (~(c3))*3 + (~(c4))*4) - 237;
                        uint8_t r = ((GPIOB->ODR >> 8) ^ 0b111)/2;               
                        char sTmp[24];
                        if (last != answ[c*3 + r]){
                                last = answ[c*3 + r];
                                sprintf(sTmp, "%c ", last);
                                for (k = 0; k < strlen(sTmp); k++) {
                                        /* Check Transmission Complete flag */
                                        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
                                        /* Send byte */
                                        USART_SendData(USART3, sTmp[k]);
                                }
                        }                        
                }
                else{
                   if (i == lastR) last = ""; 
                }
                delay(0x00FFFF);                  
            }            
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

void initKeyboard(void){
    GPIO_InitTypeDef GPIO_InitStructure;
     /* hmm, I should be declared on function start :) */

    GPIO_StructInit(&GPIO_InitStructure); /* !! important !!, initialization of the structure. */
    
    /* Enable the GPIO_LED Clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);    
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);    
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);    
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
    
}


void initUART(void){
    USART_InitTypeDef USART_InitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;
        /* Signal demo type */


        /* Initialize USART3 (USB <==> Serial Port) */

        /* Enable clock to GPIOD */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        /* Enable clock to USART3 */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

        /* Set RX and TX pins as Alternate Function USART3 */
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);

        GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

        /* Configure RX and TX pins */
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /* Configure USART 3 */
        USART_StructInit(&USART_InitStructure);
        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_Init(USART3, &USART_InitStructure);
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        /* Enable USART 3 */
        USART_Cmd(USART3, ENABLE);
        NVIC_EnableIRQ(USART3_IRQn);

}

void USART3_IRQHandler(void)
{
    /* RXNE handler */
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        char r = (char)USART_ReceiveData(USART3);        
        GPIOE->ODR = (GPIOE->ODR & ~(0xFF00)) | r << 8;       
           
   }
}   

void initLED(void){
    STM_EVAL_LEDInit(LED1);
    STM_EVAL_LEDInit(LED2);
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);
    STM_EVAL_LEDInit(LED5);
    STM_EVAL_LEDInit(LED6);
    STM_EVAL_LEDInit(LED7);
    STM_EVAL_LEDInit(LED8);
    STM_EVAL_LEDInit(LED1);
}
