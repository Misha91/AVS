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
#include "../SC-IDE/lcd.h"
#define POINTS 500
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LCD_MAX_CHARS (int)(LCD_PIXEL_WIDTH/ LCD_FONT_WIDTH)
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
    char sTmp[LCD_MAX_CHARS];
    uint8_t last;
    int16_t lastX;
    last = -1;
    lastX = -1;
    /* Accelerometer initialization */
    sACCEL_Init();
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* LCD initialization */
    LCD_Init();
    LCD_SetBacklight(100);
    LCD_DisplayStringLine(Line0, "Hello.");
    
    //
    uint32_t k = 0;
 
    sACCEL_ReadReg(0x0F);    
    sACCEL_WriteReg(0x20, 0x87);
    
    uint8_t xH = 0;
    uint8_t xL = 0;
    uint8_t yH = 0;
    uint8_t yL = 0;
    
    uint64_t x_shift = 0; //60
    uint64_t y_shift = 0; //28
    
    while (1) {

        xL = sACCEL_ReadReg(0x28);        
        xH = sACCEL_ReadReg(0x29);      
        yL = sACCEL_ReadReg(0x2A);        
        yH = sACCEL_ReadReg(0x2B);     
        
        if (xH >= 0x00 && xH < 0x08){
                x_shift += 64 - (int)(xH * 256 + xL) / 5;
        }
        else if (xH <= 0xFF && xH > 0xFB){
            x_shift += 64 + (int)((0xFF-xH) * 256 + (0xFF-xL)) / 5;
        }
        if (yH >= 0x00 && yH < 0x08){
            y_shift += 28 + (int)(yH * 256 + yL) / 5;
        }
        else if (yH <= 0xFF && yH > 0xFB){
            y_shift += 28 - (int)((0xFF-yH) * 256 + (0xFF-yL)) / 5;
        }
        
        uint8_t bt2 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);   
        
        if ((bt2 == 0) ){
            if (last != 51){
                last = 1;
                snprintf(sTmp, LCD_MAX_CHARS, "xL = 0x%02X", xL);
                LCD_DisplayStringLine(Line2, sTmp);        
                snprintf(sTmp, LCD_MAX_CHARS, "xH = 0x%02X", xH);
                LCD_DisplayStringLine(Line3, sTmp);
                snprintf(sTmp, LCD_MAX_CHARS, "yL = 0x%02X", yL);
                LCD_DisplayStringLine(Line5, sTmp);
                snprintf(sTmp, LCD_MAX_CHARS, "yH = 0x%02X", xH);
                LCD_DisplayStringLine(Line6, sTmp);
                snprintf(sTmp, LCD_MAX_CHARS, "Counter 0x%02X", k);
                LCD_DisplayStringLine(Line7, sTmp);
            }
        }        
        else if ((bt2 == 1) && (last != 0) ){
            last = 0; 
            LCD_Clear();
        }
        
        k++;
        
        if (k >= POINTS){
            
            k = 0;
            x_shift /= POINTS;
            y_shift /= POINTS;
            
            if (lastX != x_shift){
                lcdDrawCircle(x_shift, y_shift);  
                
                lastX = x_shift;                
                delay(0xFFFF);
            }

            x_shift = 0;
            y_shift = 0;
        }
        delay(0x4F);
     }     
}

/**
 * Delay function
 * @param nCount = Number of cycles to delay.
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


void lcdDrawCircle(uint64_t x, uint64_t y){
   
    uint8_t k = y % 8;
    y = y/8;
    
    LCD_Clear();
    
    int c;
    for (c = 0; c < 128; c++){
        LCD_WriteTo(4, c, 0x01);
    }
    if (y <= 7){
        LCD_WriteTo(y, x-4, 0x3C<<k | ((y == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y+1, x-4, 0x3C>>(8-k) | ((y + 1 == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y, x-3,0x66<<k | ((y == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y+1, x-3,0x66>>(8-k) | ((y + 1 == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y, x-2,0xC3<<k | ((y == 4) ? 0x01 : 0x00));   
        LCD_WriteTo(y+1, x-2,0xC3>>(8-k) | ((y + 1 == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y, x-1,0x81<<k | ((y == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y+1, x-1,0x81>>(8-k) | ((y + 1 == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y,  x,  0x81<<k | ((y == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y+1, x,0x81>>(8-k) | ((y + 1 == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y, x+1,0xC3<<k | ((y == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y+1, x+1,0xC3>>(8-k)  | ((y + 1 == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y, x+2,0x66<<k | ((y == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y+1, x+2,0x66>>(8-k) | ((y + 1 == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y, x+3,0x3C<<k | ((y == 4) ? 0x01 : 0x00));
        LCD_WriteTo(y+1, x+3,0x3C>>(8-k) | ((y + 1 == 4) ? 0x01 : 0x00));
    }
  
    LCD_DrawYLine(64, 1, 0, 7);
}