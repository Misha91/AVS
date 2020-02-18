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
#include <stdlib.h>
#include "../SC-IDE/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_flash.c"
#include "stm32_eval.h"
#include "stm32f2xx.h"
#include "stm32f2xx_conf.h"



#define POINTS 500

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LCD_MAX_CHARS (int)(LCD_PIXEL_WIDTH/ LCD_FONT_WIDTH)
//#define EnableTim(n)               (n.TIMx->CR1 |= TIM_CR1_CEN)
//#define DisableTim(n)              (n.TIMx->CR1 &= (~((U16)TIM_CR1_CEN)))
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void delay(vu32 nCount);
void updateAcc(int* xVal, int* yVal);
void drawWorld(uint8_t* world);
void lcdClearCircle(uint64_t x, uint64_t y, uint8_t* world);
void lcdDrawCircle(uint64_t x, uint64_t y, uint8_t* world);
uint16_t readFlash(uint32_t shift);
/* Private functions ---------------------------------------------------------*/

/**
 * Main program.
 * @return Nothing.
 */


//global pointers
int *xVal_ptr;
int *yVal_ptr;

int *xPos_ptr;
int *yPos_ptr;

int *ticker_ptr;
int *xSpeed_ptr;
int *ySpeed_ptr;
int *xSpeedPrev_ptr;
int *ySpeedPrev_ptr;

char *stopGame_ptr;

uint8_t *world_ptr;
uint8_t *finish_ptr;
uint32_t *score_ptr;
float *difficulty_ptr;


int main(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;    
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    
    char sTmp[LCD_MAX_CHARS];
    char pauseIsSet = 0;
    char stopGame = 0;
    uint8_t world[8][128];
    uint8_t finish[8][128];
    int xVal = 0;
    int yVal = 0;
    int xPos = 0;
    int yPos = 0;
    
    int xSpeed = 0;
    int ySpeed = 0;
    int xSpeedPrev = 0;
    int ySpeedPrev = 0;
    
    int ticker = 0;
    float difficulty = 0.6;
    
    uint8_t currentState = 0;
    uint8_t levelChosen = 0;
    uint8_t maxLevel = 6;
    uint32_t score = 0;
    difficulty_ptr = &difficulty;
    xVal_ptr = &xVal;
    yVal_ptr = &yVal;
    xPos_ptr = &xPos;
    yPos_ptr = &yPos;
    xSpeed_ptr = &xSpeed;
    ySpeed_ptr = &ySpeed;
    xSpeedPrev_ptr = &xSpeedPrev;
    ySpeedPrev_ptr = &ySpeedPrev;
    score_ptr = &score;
    stopGame_ptr = &stopGame;
    ticker_ptr = &ticker;
    /* Accelerometer initialization */
    sACCEL_Init();
    periph_setup();    
    
    /* LCD initialization */
    LCD_Init();
    LCD_SetBacklight(100);
    LCD_Clear();
    
    LCD_Display(LCD_Bitmap_SCKIT);
    delay(0xFFFFFF);
    //world initialization
    uint8_t i, j;
    for (i = 0; i < 8; i++){
        for (j = 0; j < 128; j++){            
            world[i][j] = 0x00;
            finish[i][j] = 0x00;
        }
    }
    world_ptr = &world[0][0];
    finish_ptr = &finish[0][0];
    //
    uint32_t k = 0;
    
    encInit();
    TIM_Cmd(TIM4, DISABLE);
    
    sACCEL_ReadReg(0x0F);    
    sACCEL_WriteReg(0x20, 0x83);
    sACCEL_WriteReg(0x21, 0x90);
    TIM_setup();
    ComPort_Init(COM2);
    ComPort_ConfigStdio(COM2, COM2, COM2);    
    char firstTime = 1;
    int gameStatus;
    

    //Score table reset
    //uint16_t toWrite[] = {0,0,0,0,0};
    //writeFlash(toWrite, 0);
       
    
    while (1) {
        switch(currentState){
            //logo screen
            case 0:
                logoScreen();
                currentState = 1;
                break;
            //menu screen    
            case 1:
                startMenu:
                TIM_Cmd(TIM4, ENABLE);
                TIM_SetCounter(TIM4, 65535/2);                
                currentState = menuScreen(firstTime);
                if (firstTime) firstTime = 0;               
                TIM_Cmd(TIM4, DISABLE);
                break;
            //campaign game mode    
            case 2:                
                sprintf(sTmp, "       Level %d", levelChosen+1);               
                LCD_Clear();
                LCD_DisplayStringLine(Line0, sTmp);
                LCD_DisplayStringLine(Line4, "  READY, STEADY, GO!");
                delay(0x1900000);
                worldSelect(levelChosen, world_ptr, finish_ptr);
                drawWorldInit(world_ptr, finish_ptr);
                updateWorld();
                unpauseTimers();
                currentState = 3;                
                break;
            //custom game mode    
            case 123:                
                sprintf(sTmp, "    Custom game!");                
                levelChosen = maxLevel - 1;
                LCD_Clear();
                LCD_DisplayStringLine(Line0, sTmp);
                LCD_DisplayStringLine(Line4, "  READY, STEADY, GO!");
                delay(0x1900000);                
                drawWorldInit(world_ptr, finish_ptr);
                updateWorld();
                unpauseTimers();
                currentState = 3;                
                break;
            //in game mode    
            case 3:
                gameStatus = gameChecker();
                if (gameStatus) {
                    currentState = 4;
                    LCD_ClearLine(Line7);
                }
                if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))){
                    pauseTimers();
                    levelChosen = 0;
                    worldSelect(99, world_ptr, finish_ptr);
                    LCD_Clear();
                    *score_ptr = 0;
                    currentState = 1;                    
                }
                if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))){
                    if (!pauseIsSet){
                        pauseTimers();
                        pauseIsSet=1;
                        delay(0x8FFFFF);
                    }
                    else if (pauseIsSet){
                        unpauseTimers();
                        pauseIsSet=0;
                        delay(0x8FFFFF);
                    }
                }
                break;
            //game finished mode    
            case 4:                
                if (gameStatus == 2) *score_ptr += (int)((3000*(levelChosen + 1))/((*ticker_ptr))+3);
                *ticker_ptr = 0;
                
                while(1){
                    int g = 0;
                    LCD_DisplayStringLine(Line7, " - PUSH ANY BUTTON -");
                    while (g < 172555){
                        if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)) || !(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))){
                            if (gameStatus == 2){
                                levelChosen++;
                                if (levelChosen == maxLevel){
                                    LCD_Clear();
                                    LCD_DisplayStringLine(Line3, "      GOOD JOB!");
                                    LCD_DisplayStringLine(Line4, " ALL LEVELS ARE DONE!");
                                    delay(0xFFFFFF);
                                    LCD_Clear();
                                    checkRecords();
                                    worldSelect(99, world_ptr, finish_ptr);
                                    levelChosen = 0;
                                    *score_ptr = 0;
                                    currentState = 0;
                                    goto here123;
                                }
                                currentState = 2;
                                goto here123;
                            }
                            else{
                                LCD_Clear();
                                checkRecords();
                                *score_ptr = 0;
                                levelChosen = 0;
                                worldSelect(99, world_ptr, finish_ptr);
                                currentState = 1;                            
                                goto here123;
                            }
                        }
                        g++;
                    }
                    LCD_ClearLine(Line7);
                    g = 0;
                    while (g < 172555){
                        if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)) || !(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))){
                            if (gameStatus == 2){
                                levelChosen++;
                                if (levelChosen == maxLevel){
                                    LCD_Clear();
                                    LCD_DisplayStringLine(Line3, "      GOOD JOB!");
                                    LCD_DisplayStringLine(Line4, " ALL LEVELS ARE DONE!");
                                    delay(0xFFFFFF);
                                    LCD_Clear();
                                    checkRecords();
                                    worldSelect(99, world_ptr, finish_ptr);
                                    levelChosen = 0;
                                    *score_ptr = 0;
                                    currentState = 0;
                                    goto here123;
                                }
                                currentState = 2;
                                goto here123;
                            }
                            else{
                                LCD_Clear();
                                checkRecords();
                                *score_ptr = 0;
                                levelChosen = 0;
                                worldSelect(99, world_ptr, finish_ptr);
                                currentState = 1;
                                goto here123;
                            }
                        }
                        g++;
                    }                
                }
                here123:
                break;            
                
            deafult:
                break;                
        }        
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
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM4, ENABLE);
}

//score screen and records update
void checkRecords(){
    char sTmp[25];
    if (*score_ptr < 10) sprintf(sTmp, "       Score %d", *score_ptr);
    
    else if (*score_ptr < 100) sprintf(sTmp, "      Score %d", *score_ptr);
    else if (*score_ptr < 1000) sprintf(sTmp, "      Score %d", *score_ptr);
    else if (*score_ptr < 10000) sprintf(sTmp, "     Score %d", *score_ptr);
    else if (*score_ptr < 100000) sprintf(sTmp, "    Score %d", *score_ptr);
    LCD_DisplayStringLine(Line2, sTmp);
    
    uint32_t k, j;    
    uint16_t scoresTable[5];
    uint16_t scoresTableTmp[5];
    for (k=0; k<20; k+=4){
        scoresTable[k/4] = readFlash(k);
    }
    for (k=0; k<5; k++){
        if (*score_ptr > scoresTable[k]){
            for (j=0; j<k; j++){
                scoresTableTmp[j] = scoresTable[j];
            }
            scoresTableTmp[k] = *score_ptr;
            for (j=k; j<4; j++){
                scoresTableTmp[j+1] = scoresTable[j];
            }
            writeFlash(scoresTableTmp, 0);
            LCD_DisplayStringLine(Line5, "     NEW RECORD!");
            break;
        }
    }    
    delay(0xFFFFFF);
    LCD_Clear();
}

//draw ball on screen
void lcdDrawCircle(uint64_t x, uint64_t y, uint8_t* world){
   
    uint8_t k = y % 8;
    y = y/8;   
   
    
    if (y < 5){
        LCD_WriteTo(y, x,0x06<<k | *(world + (y*128) + x));
        LCD_WriteTo(y+1, x,0x06>>(8-k) | *(world + ((y+1)*128) + x));
        LCD_WriteTo(y,  x+1,  0x0F<<k | *(world + (y*128) + x + 1));
        LCD_WriteTo(y+1, x+1,0x0F>>(8-k) | *(world + ((y+1)*128) + x + 1));
        LCD_WriteTo(y, x+2,0x0F<<k | *(world + (y*128) + x + 2));
        LCD_WriteTo(y+1, x+2,0x0F>>(8-k) | *(world + ((y+1)*128) + x + 2));
        LCD_WriteTo(y, x+3,0x06<<k | *(world + (y*128) + x + 3));
        LCD_WriteTo(y+1, x+3,0x06>>(8-k) | *(world + ((y+1)*128) + x + 3));
    }
    if (y == 5){
        LCD_WriteTo(y, x,0x06<<k | *(world + (y*128) + x));
        LCD_WriteTo(y+1, x,0x06>>(8-k) | *(world + ((y+1)*128) + x) | 0x80);
        LCD_WriteTo(y,  x+1,  0x0F<<k | *(world + (y*128) + x + 1));
        LCD_WriteTo(y+1, x+1,0x0F>>(8-k) | *(world + ((y+1)*128) + x + 1) | 0x80);
        LCD_WriteTo(y, x+2,0x0F<<k | *(world + (y*128) + x + 2));
        LCD_WriteTo(y+1, x+2,0x0F>>(8-k) | *(world + ((y+1)*128) + x + 2) | 0x80);
        LCD_WriteTo(y, x+3,0x06<<k | *(world + (y*128) + x + 3));
        LCD_WriteTo(y+1, x+3,0x06>>(8-k) | *(world + ((y+1)*128) + x + 3) | 0x80);
    }
    if (y == 6){
        LCD_WriteTo(y, x,0x06<<k | *(world + (y*128) + x) | 0x80);
        LCD_WriteTo(y,  x+1,  0x0F<<k | *(world + (y*128) + x + 1) | 0x80);
        LCD_WriteTo(y, x+2,0x0F<<k | *(world + (y*128) + x + 2) | 0x80);
        LCD_WriteTo(y, x+3,0x06<<k | *(world + (y*128) + x + 3) | 0x80);
    } 
}

//clean ball from screen 
void lcdClearCircle(uint64_t x, uint64_t y, uint8_t* world){
    
    uint8_t k = y % 8;
    y = y/8;       

    if (y < 5){
        LCD_WriteTo(y, x,0x00<<k | *(world + (y*128) + x));
        LCD_WriteTo(y+1, x,0x00>>(8-k) | *(world + ((y+1)*128) + x));
        LCD_WriteTo(y,  x+1,  0x00<<k | *(world + (y*128) + x + 1));
        LCD_WriteTo(y+1, x+1,0x00>>(8-k) | *(world + ((y+1)*128) + x + 1));
        LCD_WriteTo(y, x+2,0x00<<k | *(world + (y*128) + x + 2));
        LCD_WriteTo(y+1, x+2,0x00>>(8-k)  | *(world + ((y+1)*128) + x + 2));
        LCD_WriteTo(y, x+3,0x00<<k | *(world + (y*128) + x + 3));
        LCD_WriteTo(y+1, x+3,0x00>>(8-k) | *(world + ((y+1)*128) + x + 3));
    }
    if (y == 5){
        LCD_WriteTo(y, x,0x00<<k | *(world + (y*128) + x));
        LCD_WriteTo(y+1, x,0x00>>(8-k) | *(world + ((y+1)*128) + x) | 0x80);
        LCD_WriteTo(y,  x+1,  0x00<<k | *(world + (y*128) + x + 1));
        LCD_WriteTo(y+1, x+1,0x00>>(8-k) | *(world + ((y+1)*128) + x + 1) | 0x80);
        LCD_WriteTo(y, x+2,0x00<<k | *(world + (y*128) + x + 2));
        LCD_WriteTo(y+1, x+2,0x00>>(8-k)  | *(world + ((y+1)*128) + x + 2) | 0x80);
        LCD_WriteTo(y, x+3,0x00<<k | *(world + (y*128) + x + 3));
        LCD_WriteTo(y+1, x+3,0x00>>(8-k) | *(world + ((y+1)*128) + x + 3) | 0x80);
    }
    if (y == 6){
        LCD_WriteTo(y, x,0x00<<k | *(world + (y*128) + x) | 0x80);
        LCD_WriteTo(y,  x+1,  0x00<<k | *(world + (y*128) + x + 1) | 0x80);
        LCD_WriteTo(y, x+2,0x00<<k | *(world + (y*128) + x + 2) | 0x80);
        LCD_WriteTo(y, x+3,0x00<<k | *(world + (y*128) + x + 3) | 0x80);
    }
}

//draw world on screen
void drawWorldInit(uint8_t* world, uint8_t* finish){
    uint8_t x, y;
    
    for (y = 0; y < 7; y++){
        for (x = 0; x < 128; x++){ 
            if (y != 6) LCD_WriteTo(y, x, *(world + y*128 + x) | *(finish + y*128 + x));    
            else LCD_WriteTo(y, x, 0x80 | *(world + y*128 + x) | *(finish + y*128 + x));
        }
    }
    char sTmp2[LCD_MAX_CHARS];
    if (*score_ptr < 10) sprintf(sTmp2, "PAUSE EXIT    SCORE:%d", *score_ptr);
    else if (*score_ptr < 100) sprintf(sTmp2, "PAUSE EXIT   SCORE:%d", *score_ptr);
    else if (*score_ptr < 1000) sprintf(sTmp2, "PAUSE EXIT  SCORE:%d", *score_ptr);
    else if (*score_ptr < 10000) sprintf(sTmp2, "PAUSE EXIT SCORE:%d", *score_ptr);
    
    LCD_ClearLine(Line7);
    LCD_DisplayStringLine(Line7, sTmp2); 
}

//checker if game is finished
int gameChecker(){
    uint8_t i, y, k, x;
    pauseTimers();
    x = *xVal_ptr/500;
    y = *yVal_ptr/8000;
    k = (*yVal_ptr/1000)%8;
    unpauseTimers();    
    for (i = x; i < x+4; i++){
            if ((*(world_ptr + (y*128) + i) & (0x0F<<k)) || (*(world_ptr + ((y+1)*128) + i) & (0x0F>>(8-k))))
            {                
                pauseTimers();
                updateWorld();
                LCD_ClearLine(Line3);
                LCD_DisplayStringLine(Line3, "      Game Over");
                *stopGame_ptr = 1;
                return 1;                          
            } 
            if ((*(finish_ptr + (y*128) + i) & (0x0F<<k)) || (*(finish_ptr + ((y+1)*128) + i) & (0x0F>>(8-k))))
            {                
                pauseTimers();
                updateWorld();
                LCD_ClearLine(Line3);
                LCD_DisplayStringLine(Line3, "   Level Complete");
                *stopGame_ptr = 1;
                return 2;                      
            }            
        }
    return 0;
}

//update accelerometer data
void updateAcc(int* xSpeed, int* ySpeed){

    uint8_t xL = sACCEL_ReadReg(0x28);        
    uint8_t xH = sACCEL_ReadReg(0x29);      
    uint8_t yL = sACCEL_ReadReg(0x2A);        
    uint8_t yH = sACCEL_ReadReg(0x2B);  


     if (xH >= 0x00 && xH < 0x08){
             *xSpeed = (int)(xH * 256 + xL);
     }
     else if (xH <= 0xFF && xH > 0xFB){
             *xSpeed = - (int)((0xFF-xH) * 256 + (0xFF-xL));
     }
     if (yH >= 0x00 && yH < 0x08){
         *ySpeed = -(int)(yH * 256 + yL);
     }
     else if (yH <= 0xFF && yH > 0xFB){
         *ySpeed = +(int)((0xFF-yH) * 256 + (0xFF-yL));
     }
}

//buttons setup
void periph_setup(){
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//ball update and LCD update timers setup
void TIM_setup(){
    //RCC->AHB1ENR |= RCC_AHB1LPENR_GPIOELPEN;
    
    //timer setup
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 24999;	        // Set prescaler to 25 000 (PSC + 1)
    TIM2->ARR = 250;	          // Auto reload value 2400 - 1Hz, 40 - 60Hz
    TIM2->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
    TIM2->CR1 = TIM_CR1_CEN;   // Enable timer

    //NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt from TIM3 (NVIC level)
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->PSC = 2399;	        // Set prescaler to 25 000 (PSC + 1)
    TIM5->ARR = 125;	          // 500 Hz
    TIM5->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
    TIM5->CR1 = TIM_CR1_CEN;   // Enable timer

    //NVIC_EnableIRQ(TIM5_IRQn); // Enable interrupt from TIM3 (NVIC level)
    
    //LED setup
    STM_EVAL_LEDInit(LED7);
    STM_EVAL_LEDOff(LED7);
}

//screen update timer, 10Hz
void TIM2_IRQHandler()
{
    if(TIM2->SR & TIM_SR_UIF)
    {
        NVIC_DisableIRQ(TIM5_IRQn);
        TIM2->SR &= ~TIM_SR_UIF;
        updateWorld();
        *ticker_ptr += 1;
        NVIC_EnableIRQ(TIM5_IRQn);
    }
}

//ball position update timer, 500 Hz
void TIM5_IRQHandler()
{
    if(TIM5->SR & TIM_SR_UIF)
    {
        NVIC_DisableIRQ(TIM2_IRQn);
        TIM5->SR &= ~TIM_SR_UIF;
        updateAcc(xSpeed_ptr, ySpeed_ptr);
        if (abs(*xSpeed_ptr)<15) *xSpeed_ptr = 0;
        if (abs(*ySpeed_ptr)<15) *ySpeed_ptr = 0;
        
        int xOld = *xVal_ptr;
        int yOld = *yVal_ptr;
        *xVal_ptr = (*xVal_ptr + *xSpeed_ptr + *xSpeedPrev_ptr);
        
        *xVal_ptr = (*xVal_ptr > 0 ? (*xVal_ptr < 62000 ? *xVal_ptr : 62000) : 0);
        if (*xVal_ptr == 0 || *xVal_ptr == 62000) {
            *xSpeedPrev_ptr = 0;
            *xSpeed_ptr = 0;
        }
        else *xSpeedPrev_ptr = *xSpeed_ptr;
        
        //*ySpeed_ptr = ((*ySpeed_ptr)*(*difficulty_ptr))
        *yVal_ptr = (*yVal_ptr + *ySpeed_ptr + *ySpeedPrev_ptr);
        
        *yVal_ptr = (*yVal_ptr > 0 ? (*yVal_ptr < 51000 ? *yVal_ptr : 51000) : 0);
        if (*yVal_ptr == 0 || *yVal_ptr == 51000) {
            *ySpeedPrev_ptr = 0;
            *ySpeed_ptr = 0;
        }
        else *ySpeedPrev_ptr = *ySpeed_ptr;  
               
        NVIC_EnableIRQ(TIM2_IRQn);
            
    }    
}

//pause timers
void pauseTimers(){
    TIM_Cmd(TIM2, DISABLE);    
    NVIC_DisableIRQ(TIM2_IRQn);
    TIM_Cmd(TIM5, DISABLE);
    NVIC_DisableIRQ(TIM5_IRQn);
    TIM2->CR1 &= (~(TIM_CR1_CEN));
    TIM5->CR1 &= (~(TIM_CR1_CEN));
}

//unpause timers
void unpauseTimers(){
    TIM_Cmd(TIM2, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM_Cmd(TIM5, ENABLE);
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM5->CR1 |= TIM_CR1_CEN;
}

//draw world on LCD routine
void updateWorld(){
    int tmpX_val, tmpY_val, tmpX_pos, tmpY_pos;
    tmpX_val = (int)*xVal_ptr/500;
    tmpY_val = (int)*yVal_ptr/1000;
    tmpX_pos = (int)*xPos_ptr/500;
    tmpY_pos = (int)*yPos_ptr/1000;
    if (tmpX_val != tmpX_pos || tmpY_val != tmpY_pos){        
        lcdClearCircle(tmpX_pos, tmpY_pos, world_ptr); 
        *xPos_ptr = *xVal_ptr;
        *yPos_ptr = *yVal_ptr;
        lcdDrawCircle(tmpX_val, tmpY_val, world_ptr);
    }
    
}

//startup logo screen
void logoScreen(){
    
    LCD_Clear();
    delay(0xFFFFF);
    uint16_t img[] = {148, 128, 149, 192, 150, 240, 151, 248, 152, 252, 153, 12, 154, 6, 271, 192, 272, 248, 273, 254, 274, 3, 275, 56, 276, 255, 277, 255, 278, 255, 279, 255, 280, 255, 281, 224, 282, 128, 396, 128, 399, 255, 400, 255, 401, 255, 402, 248, 403, 224, 404, 195, 405, 143, 406, 31, 407, 127, 408, 255, 409, 255, 410, 255, 411, 255, 412, 254, 413, 248, 414, 240, 415, 224, 416, 192, 417, 128, 425, 255, 426, 255, 430, 128, 431, 224, 432, 252, 433, 127, 434, 103, 435, 126, 436, 248, 437, 192, 438, 3, 439, 31, 440, 124, 441, 224, 442, 192, 443, 240, 444, 126, 445, 15, 446, 131, 447, 224, 448, 252, 449, 127, 450, 111, 451, 126, 452, 248,453, 192, 458, 15, 459, 62, 460, 240, 461, 192, 462, 240, 463,126, 464, 15, 465, 63, 466, 248, 467, 224, 468, 240, 469, 124,470, 31, 471, 3, 472, 192, 473, 248, 474, 126, 475, 103, 476, 127, 477, 124, 478, 224, 479, 128, 481, 255, 482, 254, 487, 255, 488, 255, 493, 14, 494, 31, 495, 27, 496, 59, 497, 243, 498, 227, 524, 131, 525, 156, 526, 184, 527, 160, 528, 7, 529, 15, 530, 63, 531, 127, 532, 255, 533, 255, 534, 255, 535, 254, 536, 248, 537, 241, 538, 3, 539, 15, 540, 63, 541, 255, 542, 255, 543, 255, 544, 255, 545, 255, 546, 254, 547, 252, 548, 240, 549, 128, 553, 3, 554, 3, 555, 3, 556, 3, 557, 3, 558, 1, 559, 1, 565, 1, 566, 1, 569, 1, 570, 3, 571, 1, 574, 1, 575, 1, 581, 1, 582, 1, 588, 1, 589, 3, 590, 1, 594, 1, 595, 3, 596, 1, 599, 1, 600, 3, 606, 1, 607, 3, 609, 3, 610, 3, 611, 3, 612, 3, 613, 3,615, 1, 616, 3, 617, 3, 618, 3, 619, 3, 620, 1, 621, 1, 622, 1, 623, 3, 624, 3, 625, 1, 626, 1, 651, 7, 652, 15, 653, 31, 654, 63, 655, 127, 656, 127, 657, 255, 658, 255, 659, 255, 660, 254, 661, 254, 662, 255, 663, 189, 664, 61, 665, 60, 666, 60, 667, 60, 668, 62, 669, 63, 670, 63, 671, 63, 672, 63, 673, 63, 674, 63, 675, 31, 676, 15, 677, 3, 787, 1, 788, 1, 789, 1, 790, 3,791, 3, 792, 3, 793, 3, 794, 2};
    int x, y;
    for (y = 0; y < 8; y++){
        for (x = 0; x < 128; x++){                   
                *(world_ptr + y*128 + x) = 0x00;    
            }
    }
    for (x = 0; x<sizeof(img)/sizeof(img[0]); x+=2){
        *(world_ptr + img[x]) = img[x+1];
    }
    
    for (y = 0; y < 7; y++){
        for (x = 0; x < 128; x++){ 
            LCD_WriteTo(y, x, *(world_ptr + y*128 + x));    
            //else LCD_WriteTo(y, x, 0x80 | *(world + y*128 + x) );
        }
    }
    
    for (y = 0; y < 8; y++){
        for (x = 0; x < 128; x++){                   
                *(world_ptr + y*128 + x) = 0x00;    
            }
    }
    
    while(1){
        int g = 0;
        LCD_DisplayStringLine(Line7, " - PUSH ANY BUTTON -");
        while (g < 172555){
            if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)) || !(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))){
               goto here12;
            }
            g++;
        }
        LCD_ClearLine(Line7);
        g = 0;
        while (g < 172555){
            if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)) || !(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))){
                goto here12;
            }
            g++;
        }        
    }    
    here12:;
    delay(0x3AFFFF);
}

//main menu routine
int menuScreen(int beginning){
    
    uint8_t h = 0;
    char first = beginning;
    uint8_t pos = 0;
    uint8_t menuLevel = 0;
    uint8_t upperBound = 2;
    uint32_t init, last;
    initMenu:
    
    init  = TIM_GetCounter(TIM4);
    last = init;    
    LCD_Clear();
    delay(0xFFFF);
    LCD_DisplayStringLine(Line2, "       NEW GAME");
    LCD_DisplayStringLine(Line3, "       RECORDS");
    LCD_DisplayStringLine(Line4, "       CREDITS");
    
    if (first){
        for (h=0; h<1; h++){
            LCD_ClearLine(Line7);
            delay(0X5FFFFF);
            LCD_DisplayStringLine(Line7, "SELECT BACK RST BOOT");
            delay(0X5FFFFF);
            LCD_ClearLine(Line7);
        }
        first = 0;
    }
    
    lcdDrawCircle(35, (pos + 2)*8+2, world_ptr);
    while(1){
        
        init  = TIM_GetCounter(TIM4);
        delay(0XFFFF);
        int diff = (init-last);
        if (menuLevel == 0 || menuLevel == 1){
            if (diff > 1){
                if (pos < upperBound) {
                    lcdClearCircle(35, (pos + 2)*8+2, world_ptr); 
                    pos++;
                    lcdDrawCircle(35, (pos + 2)*8+2, world_ptr);
                }
            }

            else if (diff < -1){
                if (pos > 0) {
                    lcdClearCircle(35, (pos + 2)*8+2, world_ptr); 
                    pos--;
                    lcdDrawCircle(35, (pos + 2)*8+2, world_ptr);
                }
            }        
        }
        if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))){
            if (menuLevel == 2 || menuLevel == 3) {
                pos = 0;
                menuLevel = 0;
                upperBound = 2;
                delay(0x3AFFFF);
                goto initMenu;
            }
            switch(pos){
                case 0:
                    if (menuLevel == 1) {
                        return 2;
                    }
                    else{
                        menuLevel = 1;                        
                        pos = 0;
                        upperBound = 1;
                        LCD_Clear();
                        LCD_DisplayStringLine(Line2, "       CAMPAIGN");
                        LCD_DisplayStringLine(Line3, "       CUSTOM GAME");
                        lcdDrawCircle(35, (pos + 2)*8+2, world_ptr);
                        delay(0x3FFFFF);
                    }
                    break;
                case 1:
                    if (menuLevel == 0){
                        
                        LCD_Clear();
                        LCD_DisplayStringLine(Line0, "     BEST SCORES:");                        
                        uint32_t k;
                        char sTmp[20];
                        uint16_t scoresTable[5];
                        
                        for (k=0; k<20; k+=4){  
                            scoresTable[k/4] = readFlash(k);
                         }
                        
                         if (scoresTable[0])
                         sprintf(sTmp, "       1. %d", scoresTable[0]);
                         else sprintf(sTmp, "       1. N/A");
                         LCD_DisplayStringLine(Line2, sTmp);
                         
                         if (scoresTable[1])
                         sprintf(sTmp, "       2. %d", scoresTable[1]);
                         else sprintf(sTmp, "       2. N/A");
                         LCD_DisplayStringLine(Line3, sTmp);  
                         
                         if (scoresTable[2])
                         sprintf(sTmp, "       3. %d", scoresTable[2]);
                         else sprintf(sTmp, "       3. N/A");                         
                         LCD_DisplayStringLine(Line4, sTmp);
                         
                         if (scoresTable[3])
                         sprintf(sTmp, "       4. %d", scoresTable[3]);
                         else sprintf(sTmp, "       4. N/A");
                         LCD_DisplayStringLine(Line5, sTmp);
                         
                         if (scoresTable[4])
                         sprintf(sTmp, "       5. %d", scoresTable[4]);
                         else sprintf(sTmp, "       5. N/A");
                         LCD_DisplayStringLine(Line6, sTmp);
                         
                         menuLevel = 2;
                         delay(0x3FFFFF);
                    }
                    else{
                        LCD_Clear();
                        LCD_DisplayStringLine(Line1, "Make sure you are");
                        LCD_DisplayStringLine(Line2, "connected to PC and");
                        LCD_DisplayStringLine(Line3, "follow instructions");
                        iprintf("The map will be composed of 16*7 blocks (112 blocks total)\n");
                        iprintf("Each block will represent 8x8 pixels\n");
                        iprintf("Please enter sequence of 16 blocks for each line in format\n");
                        iprintf("X - you can put 0 if you want empty block, or 1 otherwise\n");
                        iprintf("Please send each block separately\n");
                        int d, m;
                        //d = fdgetc(COM2);
                        uint8_t customWorld[7][16];
                        for (d = 0; d < 7; d++){
                            iprintf("Please enter %d line\n", d+1);
                            for (m = 0; m <16; m++){
                                    customWorld[d][m] = fdgetc(COM2);
                                    iprintf("%d - %d blocks of %d lines received\n", customWorld[d][m], m+1, d+1);
                            }
                        }
                        iprintf("World is successfully received!\n");
                        iprintf("Please indicate the start block, from 0 to 111, 16 blocks per line (from top)\n");
                        uint8_t customStart = fdgetc(COM2);
                        iprintf("Start position is successfully received!\n");
                        iprintf("Please indicate the finish block, from 0 to 111, 16 blocks per line (from top)\n");
                        uint8_t customFinish = fdgetc(COM2);
                        iprintf("Finish position is successfully received!\n");
                        int x, y, s;
                        *xVal_ptr = (((int)customStart%16)*8+4)*500; //x /500, y/1000
                        *yVal_ptr = (((int)customStart/16)*8+4)*1000;
                        for (y = 0; y < 8; y++){
                            for (x = 0; x < 128; x++){
                                *(finish_ptr + y*128 + x) = 0x00;
                                *(world_ptr + y*128 + x) = 0x00;                    
                            }
                        }
                        for (y = 0; y < 7; y++){
                            for (x = 0; x < 16; x++){                            
                                for (s=0; s<8; s++)
                                *(world_ptr + y*128 + x*8 + s) = 0xFF*customWorld[y][x];                    
                            }
                        }
                        x = (customFinish%16)*8;
                        y = ((int)customFinish/16);
                        for (s=0; s<8; s++){
                                if (s%2==0) *(finish_ptr + y*128 + x + s) = 0x55; //55
                                else *(finish_ptr + y*128 + x + s) = 0xAA; //AA
                        }

                        LCD_Clear();
                        LCD_DisplayStringLine(Line1, "Map is ready!");                    
                        LCD_DisplayStringLine(Line3, "Press any button to");
                        LCD_DisplayStringLine(Line4, "start!");
                        iprintf("Custom game is ready! Press any button when you will be ready!\n");
                        while(1){
                            if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15)) || !(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))){
                            return 123;
                            }
                        }
                    }
                    break;
                case 2:
                    LCD_Clear();
                    LCD_DisplayStringLine(Line0, "       CREDITS:");
                    LCD_DisplayStringLine(Line2, " (c) Mikhail Ivanov");
                    LCD_DisplayStringLine(Line3, "ivanomik@fel.cvut.cz");
                    LCD_DisplayStringLine(Line6, "   Czech Technical");
                    LCD_DisplayStringLine(Line7, "University in Prague");
                    //LCD_DisplayStringLine(Line7, "       BACK");
                    menuLevel = 3;
                    delay(0x3FFFFF);
                    break;
                                    
                deafult:
                    break;
            }
            
        }
        
        if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))){
            pos = 0;
            menuLevel = 0;
            upperBound = 2;
            goto initMenu;            
        }
                    
        last = init;
        delay(0XFFFF);
    }
}

//fill world parameters based on level chosen
void worldSelect(uint8_t level, uint8_t* world, uint8_t* finish){
    uint16_t x, y, s;
    
    switch(level)
    {   
        case 0:
            *xVal_ptr = (((int)0%16)*8+4)*500; //x /500, y/1000
            *yVal_ptr = (((int)48/16)*8+4)*1000;
            uint16_t img0[] = {286, 254, 287, 2, 288, 2, 289, 2, 290, 2, 291, 2, 292, 2, 293, 2, 294, 2, 295, 2, 296, 2, 297, 2, 298, 2, 299, 2, 305, 14, 306, 56, 307, 128, 315, 192, 316, 120, 317, 14, 323, 254, 336, 254, 342, 2, 343, 2, 344, 2, 345, 2, 346, 2, 347, 2, 348, 254, 349, 2, 350, 2, 351, 2, 352, 2, 353, 2, 354, 2, 355, 2, 414, 255, 435, 3, 436, 30, 437, 224, 441, 224, 442, 62, 443, 3, 451, 255, 464, 255, 476, 255, 542, 63, 543, 32, 544, 32, 545, 32, 546,32, 547, 32, 548, 32, 549, 32, 550, 32, 551, 32, 552, 32, 553,32, 554, 32, 555, 32, 566, 15, 567, 56, 568, 15, 569, 1, 579, 63, 580, 32, 581, 32, 582, 32, 583, 32, 584, 32, 585, 32, 586, 32, 587, 32, 588, 32, 589, 32, 590, 32, 591, 32, 592, 63, 604, 63};
     
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){                   
                    *(world + y*128 + x) = 0x00;                
                    *(finish + y*128 + x) = 0x00;
                }
            }
            for (x = 0; x<sizeof(img0)/sizeof(img0[0]); x+=2){
                *(world + img0[x]) = img0[x+1];
            }
            x = 120;
            y = 3;
            for (s=0; s<8; s++){
                if (s%2==0) *(finish_ptr + y*128 + x + s) = 0x55; //55
                else *(finish_ptr + y*128 + x + s) = 0xAA; //AA
            }
            
            break;
        
        
        case 1:
            *xVal_ptr = (((int)96%16)*8+4)*500; //x /500, y/1000
            *yVal_ptr = (((int)96/16)*8+4)*1000;
            uint16_t img1[] = {384, 8, 385, 8, 386, 8, 387, 8, 388, 8, 389, 8, 390, 8, 391, 8, 392, 8, 393, 8, 394, 8, 395, 8, 396, 8, 397, 8, 398, 8, 399,8, 400, 8, 401, 8, 402, 8, 403, 8, 404, 8, 405, 8, 406, 8, 407, 8, 408, 8, 409, 8, 410, 8, 411, 8, 412, 8, 413, 8, 414, 8, 415, 8, 416, 8, 417, 8, 418, 8, 419, 8, 420, 8, 421, 8, 422, 8, 423, 8, 424, 8, 425, 8, 426, 8, 427, 8, 428, 8, 429, 8, 430, 8, 431, 8, 432, 8, 433, 8, 434, 8, 435, 8, 436, 8, 437, 8, 438, 8,439, 8, 440, 8, 441, 8, 442, 8, 443, 8, 444, 8, 445, 8, 446, 8, 447, 8, 448, 8, 449, 8, 450, 8, 451, 8, 452, 8, 453, 8, 454, 8, 455, 8, 456, 8, 457, 8, 458, 8, 459, 8, 460, 8, 461, 8, 462,8, 463, 8, 464, 8, 465, 8, 466, 8, 467, 8, 468, 8, 469, 8, 470, 8, 471, 8, 472, 8, 473, 8, 474, 8, 475, 8, 476, 8, 477, 8, 478, 8, 479, 8, 480, 8, 481, 8, 482, 8, 483, 8, 484, 8, 485, 8, 486, 8, 487, 8, 488, 8, 489, 8, 490, 8, 491, 8, 492, 8, 493, 8, 494, 8, 495, 8};
     
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){                   
                    *(world + y*128 + x) = 0x00;                
                    *(finish + y*128 + x) = 0x00;
                }
            }
            for (x = 0; x<sizeof(img1)/sizeof(img1[0]); x+=2){
                *(world + img1[x]) = img1[x+1];
            }
            x = 4;
            y = 0;
            for (s=0; s<8; s++){
                if (s%2==0) *(finish_ptr + y*128 + x + s) = 0x55; //55
                else *(finish_ptr + y*128 + x + s) = 0xAA; //AA
            }
            
            break;
            
        case 2:          
            
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){
                    if (((x == 35 || x == 36 || x == 37) || (x == 93 || x == 94 || x == 95)) && (y >= 0 && y < 3) ) *(world + y*128 + x) = 0xFF;  
                    else if (((x == 35 || x == 36 || x == 37) || (x == 93 || x == 94 || x == 95)) && (y ==3) ) *(world + y*128 + x) = 0x0F;
                    else if (((x == 64 || x == 65 || x == 66) ) && (y > 3 && y < 7) ) *(world + y*128 + x) = 0xFF;  
                    else if (((x == 64 || x == 65 || x == 66) ) && (y == 3) ) *(world + y*128 + x) = 0xF0;  
                    else *(world + y*128 + x) = 0x00;
                    
                    if (( x == 120 || x == 122 || x == 124 || x == 126) && (y == 0)) *(finish + y*128 + x) = 0x55;
                    else if (( x == 121 || x == 123 || x == 125 || x == 127) && (y == 0)) *(finish + y*128 + x) = 0xAA;
                    else *(finish + y*128 + x) = 0x00;
                }
            } 
            break;
            
        case 3:
            *xVal_ptr = (((int)96%16)*8+4)*500; //x /500, y/1000
            *yVal_ptr = (((int)96/16)*8+4)*1000;
            uint16_t img3[] = {272, 2, 273, 2, 274, 2, 275, 2, 276, 2, 277, 2, 278, 2, 279, 2, 280, 2, 281, 2, 282, 2, 283, 2, 284, 2, 285, 2, 286, 2, 287,2, 288, 2, 289, 2, 290, 2, 291, 2, 292, 2, 293, 2, 294, 2, 295, 2, 296, 2, 297, 2, 298, 2, 299, 2, 300, 2, 301, 2, 302, 2, 303, 2, 304, 2, 305, 2, 306, 2, 307, 2, 308, 2, 309, 2, 310, 2, 311, 2, 312, 2, 313, 2, 314, 2, 315, 2, 316, 2, 317, 2, 318, 2, 319, 2, 320, 2, 321, 2, 322, 2, 323, 2, 324, 2, 325, 2, 326, 2,327, 2, 328, 2, 329, 2, 330, 2, 331, 2, 332, 2, 333, 2, 334, 2, 335, 2, 336, 2, 337, 2, 338, 2, 339, 2, 340, 2, 341, 2, 342, 2, 343, 2, 344, 2, 345, 2, 346, 2, 347, 2, 348, 2, 349, 2, 350,2, 351, 2, 352, 2, 353, 2, 354, 2, 355, 2, 356, 2, 357, 2, 358, 2, 359, 2, 360, 2, 361, 2, 362, 2, 363, 2, 364, 2, 365, 2, 366, 2, 367, 254, 495, 255, 512, 32, 513, 32, 514, 32, 515, 32, 516, 32, 517, 32, 518, 32, 519, 32, 520, 32, 521, 32, 522, 32, 523, 32, 524, 32, 525, 32, 526, 32, 527, 32, 528, 32, 529, 32, 530, 32, 531, 32, 532, 32, 533, 32, 534, 32, 535, 32, 536, 32, 537, 32, 538, 32, 539, 32, 540, 32, 541, 32, 542, 32, 543, 32, 544, 32, 545, 32, 546, 32, 547, 32, 548, 32, 549, 32, 550, 32, 551, 32, 552, 32, 553, 32, 554, 32, 555, 32, 556, 32, 557, 32, 558, 32, 559, 32, 560, 32, 561, 32, 562, 32, 563, 32, 564, 32, 565, 32, 566, 32, 567, 32, 568, 32, 569, 32, 570, 32, 571, 32, 572, 32, 573, 32, 574, 32, 575, 32, 576, 32, 577, 32, 578, 32, 579, 32, 580, 32, 581, 32, 582, 32, 583, 32, 584, 32, 585, 32, 586, 32, 587, 32, 588, 32, 589, 32, 590, 32, 591, 32, 592, 32, 593, 32, 594, 32, 595, 32, 596, 32, 597, 32, 598, 32, 599, 32, 600, 32, 601, 32, 602, 32, 603, 32, 604, 32, 605, 32, 606, 32, 607, 32, 608, 32, 609, 32, 610, 32, 611, 32, 612, 32, 613, 32, 614, 32, 615, 32, 616, 32, 617, 32, 618, 32, 619, 32, 620, 32, 621, 32, 622, 32, 623, 63};
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){                   
                    *(world + y*128 + x) = 0x00;                
                    *(finish + y*128 + x) = 0x00;
                }
            }
            for (x = 0; x<sizeof(img3)/sizeof(img3[0]); x+=2){
                *(world + img3[x]) = img3[x+1];
            }
            x = 100;
            y = 3;
            for (s=0; s<8; s++){
                if (s%2==0) *(finish_ptr + y*128 + x + s) = 0x55; //55
                else *(finish_ptr + y*128 + x + s) = 0xAA; //AA
            }
            
            break;    
        case 5:
            *xVal_ptr = (((int)96%16)*8+4)*500; //x /500, y/1000
            *yVal_ptr = (((int)96/16)*8+4)*1000;
            uint16_t img4[] = {144, 248, 145, 8, 146, 8, 147, 8, 148, 8, 149, 8, 150, 8, 151, 8, 152, 8, 153, 8, 154, 8, 155, 8, 156, 8, 157, 8, 158, 8, 159, 8, 160, 8, 161, 8, 162, 8, 163, 8, 164, 8, 165, 8, 166, 8, 167, 8, 168, 8, 169, 8, 170, 8, 171, 8, 172, 8, 173, 8, 174, 8, 175, 8, 176, 8, 177, 8, 178, 8, 179, 8, 180, 8, 181, 8, 182, 8,183, 8, 184, 8, 185, 8, 186, 8, 187, 8, 188, 8, 189, 8, 190, 8, 191, 8, 192, 8, 193, 8, 194, 8, 195, 8, 196, 8, 197, 8, 198, 8, 199, 8, 200, 8, 201, 8, 202, 8, 203, 8, 204, 8, 205, 8, 206,8, 207, 8, 208, 8, 209, 8, 210, 8, 211, 8, 212, 8, 213, 8, 214, 8, 215, 8, 216, 8, 217, 8, 218, 8, 219, 8, 220, 8, 221, 8, 222, 8, 223, 8, 224, 8, 225, 8, 226, 8, 227, 8, 228, 8, 229, 8, 230, 8, 231, 8, 232, 8, 233, 8, 234, 8, 235, 8, 236, 8, 237, 8, 238, 8, 239, 248, 272, 255, 289, 128, 290, 128, 291, 128, 292, 128, 293, 128, 294, 128, 295, 128, 296, 128, 297, 128, 298, 128, 299, 128, 300, 128, 301, 128, 302, 128, 303, 128, 304, 128, 305, 128, 306, 128, 307, 128, 308, 128, 309, 128, 310, 128, 311,128, 312, 128, 313, 128, 314, 128, 315, 128, 316, 128, 317, 128, 318, 128, 319, 128, 320, 128, 321, 128, 322, 128, 323, 128, 324, 128, 325, 128, 326, 128, 327, 128, 328, 128, 329, 128, 330, 128, 331, 128, 332, 128, 333, 128, 334, 128, 335, 128, 336, 128, 337, 128, 338, 128, 339, 128, 340, 128, 341, 128, 342, 128,343, 128, 344, 128, 345, 128, 346, 128, 347, 128, 348, 128, 349, 128, 350, 128, 367, 255, 400, 255, 478, 255, 495, 255, 528, 3, 529, 2, 530, 2, 531, 2, 532, 2, 533, 2, 534, 2, 535, 2, 536,2, 537, 2, 538, 2, 539, 2, 540, 2, 541, 2, 542, 2, 543, 2, 544, 2, 545, 2, 546, 2, 547, 2, 548, 2, 549, 2, 550, 2, 551, 2, 552, 2, 553, 2, 554, 2, 555, 2, 556, 2, 557, 2, 558, 2, 559, 2, 560, 2, 561, 2, 562, 2, 563, 2, 564, 2, 565, 2, 566, 2, 567, 2, 568, 2, 569, 2, 570, 2, 571, 2, 572, 2, 573, 2, 574, 2, 575, 2,576, 2, 577, 2, 578, 2, 579, 2, 580, 2, 581, 2, 582, 2, 583, 2, 584, 2, 585, 2, 586, 2, 587, 2, 588, 2, 589, 2, 590, 2, 591, 2, 592, 2, 593, 2, 594, 2, 595, 2, 596, 2, 597, 2, 598, 2, 599,2, 600, 2, 601, 2, 602, 2, 603, 2, 604, 2, 605, 2, 606, 3, 623, 255, 640, 32, 641, 32, 642, 32, 643, 32, 644, 32, 645, 32, 646, 32, 647, 32, 648, 32, 649, 32, 650, 32, 651, 32, 652, 32, 653, 32, 654, 32, 655, 32, 656, 32, 657, 32, 658, 32, 659, 32, 660, 32, 661, 32, 662, 32, 663, 32, 664, 32, 665, 32, 666, 32, 667, 32, 668, 32, 669, 32, 670, 32, 671, 32, 672, 32, 673, 32, 674, 32, 675, 32, 676, 32, 677, 32, 678, 32, 679, 32, 680, 32, 681, 32, 682, 32, 683, 32, 684, 32, 685, 32, 686, 32, 687, 32, 688, 32, 689, 32, 690, 32, 691, 32, 692, 32, 693, 32, 694, 32, 695, 32, 696, 32, 697, 32, 698, 32, 699, 32, 700, 32, 701, 32, 702, 32, 703, 32, 704, 32, 705, 32, 706, 32, 707, 32, 708, 32, 709, 32, 710, 32, 711, 32, 712, 32, 713, 32, 714, 32, 715, 32, 716, 32, 717, 32, 718, 32, 719, 32, 720, 32, 721, 32, 722, 32, 723, 32, 724, 32, 725, 32, 726, 32, 727, 32, 728, 32, 729, 32, 730, 32, 731, 32, 732, 32, 733, 32, 734, 32, 735, 32, 736, 32, 737, 32, 738, 32, 739, 32, 740, 32, 741, 32, 742, 32, 743, 32, 744, 32, 745, 32, 746, 32, 747, 32, 748, 32, 749, 32, 750, 32, 751, 63};
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){                   
                    *(world + y*128 + x) = 0x00;                
                    *(finish + y*128 + x) = 0x00;
                }
            }
            for (x = 0; x<sizeof(img4)/sizeof(img4[0]); x+=2){
                *(world + img4[x]) = img4[x+1];
            }
            x = 86;
            y = 3;
            for (s=0; s<8; s++){
                if (s%2==0) *(finish_ptr + y*128 + x + s) = 0x55; //55
                else *(finish_ptr + y*128 + x + s) = 0xAA; //AA
            }            
            break;    
            
        case 4:
            *xVal_ptr = 5000;
            *yVal_ptr = 5500;
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){
                    if (((x == 30 || x == 31 || x == 32) || (x == 61 || x == 62 || x == 63 || x == 64) || (x == 93 || x == 94 || x == 95)) && (y >= 0 && y < 3) ) *(world + y*128 + x) = 0xFF;
                    else if (((x == 30 || x == 31 || x == 32) || (x == 61 || x == 62 || x == 63 || x == 64) || (x == 93 || x == 94 || x == 95)) && (y == 3) ) *(world + y*128 + x) = 0x0F;
                    else if (((x == 46 || x == 47 || x == 48) || (x == 78 || x == 79 || x == 80)) && (y >= 4 && y < 7) ) *(world + y*128 + x) = 0xFF; 
                    else if (((x == 46 || x == 47 || x == 48) || (x == 78 || x == 79 || x == 80)) && (y == 3) ) *(world + y*128 + x) = 0xF0;   
                    else *(world + y*128 + x) = 0x00;
                    
                    if (( x == 100 || x == 102 || x == 104 || x == 106) && (y == 0)) *(finish + y*128 + x) = 0x55;
                    else if (( x == 101 || x == 103 || x == 105 || x == 107) && (y == 0)) *(finish + y*128 + x) = 0xAA;
                    else *(finish + y*128 + x) = 0x00;
                }
            } 
            break;
            
        case 6:
            *xVal_ptr = (((int)96%16)*8+4)*500; //x /500, y/1000
            *yVal_ptr = (((int)96/16)*8+4)*1000;
            uint16_t img6[] = {144, 248, 145, 8, 146, 8, 147, 8, 148, 8, 149, 8, 150, 8, 151, 8, 152, 8, 153, 8, 154, 8, 155, 8, 156, 8, 157, 8, 158, 8, 159, 8, 160, 8, 161, 8, 162, 8, 163, 8, 164, 8, 165, 8, 166, 8, 167, 8, 168, 8, 169, 8, 170, 8, 171, 8, 172, 8, 173, 8, 174, 8, 175, 8, 176, 8, 177, 8, 178, 8, 179, 8, 180, 8, 181, 8, 182, 8,183, 8, 184, 8, 185, 8, 186, 8, 187, 8, 188, 8, 189, 8, 190, 8, 191, 8, 192, 8, 193, 8, 194, 8, 195, 8, 196, 8, 197, 8, 198, 8, 199, 8, 200, 8, 201, 8, 202, 8, 203, 8, 204, 8, 205, 8, 206,8, 207, 8, 208, 8, 209, 8, 210, 8, 211, 8, 212, 8, 213, 8, 214, 8, 215, 8, 216, 8, 217, 8, 218, 8, 219, 8, 220, 8, 221, 8, 222, 8, 223, 8, 224, 8, 225, 8, 226, 8, 227, 8, 228, 8, 229, 8, 230, 8, 231, 8, 232, 8, 233, 8, 234, 8, 235, 8, 236, 8, 237, 8, 238, 8, 239, 248, 272, 255, 289, 128, 290, 128, 291, 128, 292, 128, 293, 128, 294, 128, 295, 128, 296, 128, 297, 128, 298, 128, 299, 128, 300, 128, 301, 128, 302, 128, 303, 128, 304, 128, 305, 128, 306, 128, 307, 128, 308, 128, 309, 128, 310, 128, 311,128, 312, 128, 313, 128, 314, 128, 315, 128, 316, 128, 317, 128, 318, 128, 319, 128, 320, 128, 321, 128, 322, 128, 323, 128, 324, 128, 325, 128, 326, 128, 327, 128, 328, 128, 329, 128, 330, 128, 331, 128, 332, 128, 333, 128, 334, 128, 335, 128, 336, 128, 337, 128, 338, 128, 339, 128, 340, 128, 341, 128, 342, 128,343, 128, 344, 128, 345, 128, 346, 128, 347, 128, 348, 128, 349, 128, 350, 128, 367, 255, 400, 255, 478, 255, 495, 255, 528, 3, 529, 2, 530, 2, 531, 2, 532, 2, 533, 2, 534, 2, 535, 2, 536,2, 537, 2, 538, 2, 539, 2, 540, 2, 541, 2, 542, 2, 543, 2, 544, 2, 545, 2, 546, 2, 547, 2, 548, 2, 549, 2, 550, 2, 551, 2, 552, 2, 553, 2, 554, 2, 555, 2, 556, 2, 557, 2, 558, 2, 559, 2, 560, 2, 561, 2, 562, 2, 563, 2, 564, 2, 565, 2, 566, 2, 567, 2, 568, 2, 569, 2, 570, 2, 571, 2, 572, 2, 573, 2, 574, 2, 575, 2,576, 2, 577, 2, 578, 2, 579, 2, 580, 2, 581, 2, 582, 2, 583, 2, 584, 2, 585, 2, 586, 2, 587, 2, 588, 2, 589, 2, 590, 2, 591, 2, 592, 2, 593, 2, 594, 2, 595, 2, 596, 2, 597, 2, 598, 2, 599,2, 600, 2, 601, 2, 602, 2, 603, 2, 604, 2, 605, 2, 606, 3, 623, 255, 640, 32, 641, 32, 642, 32, 643, 32, 644, 32, 645, 32, 646, 32, 647, 32, 648, 32, 649, 32, 650, 32, 651, 32, 652, 32, 653, 32, 654, 32, 655, 32, 656, 32, 657, 32, 658, 32, 659, 32, 660, 32, 661, 32, 662, 32, 663, 32, 664, 32, 665, 32, 666, 32, 667, 32, 668, 32, 669, 32, 670, 32, 671, 32, 672, 32, 673, 32, 674, 32, 675, 32, 676, 32, 677, 32, 678, 32, 679, 32, 680, 32, 681, 32, 682, 32, 683, 32, 684, 32, 685, 32, 686, 32, 687, 32, 688, 32, 689, 32, 690, 32, 691, 32, 692, 32, 693, 32, 694, 32, 695, 32, 696, 32, 697, 32, 698, 32, 699, 32, 700, 32, 701, 32, 702, 32, 703, 32, 704, 32, 705, 32, 706, 32, 707, 32, 708, 32, 709, 32, 710, 32, 711, 32, 712, 32, 713, 32, 714, 32, 715, 32, 716, 32, 717, 32, 718, 32, 719, 32, 720, 32, 721, 32, 722, 32, 723, 32, 724, 32, 725, 32, 726, 32, 727, 32, 728, 32, 729, 32, 730, 32, 731, 32, 732, 32, 733, 32, 734, 32, 735, 32, 736, 32, 737, 32, 738, 32, 739, 32, 740, 32, 741, 32, 742, 32, 743, 32, 744, 32, 745, 32, 746, 32, 747, 32, 748, 32, 749, 32, 750, 32, 751, 63};
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){                   
                    *(world + y*128 + x) = 0x00;                
                    *(finish + y*128 + x) = 0x00;
                }
            }
            for (x = 0; x<sizeof(img6)/sizeof(img6[0]); x+=2){
                *(world + img6[x]) = img6[x+1];
            }
            x = 86;
            y = 3;
            for (s=0; s<8; s++){
                if (s%2==0) *(finish_ptr + y*128 + x + s) = 0x55; //55
                else *(finish_ptr + y*128 + x + s) = 0xAA; //AA
            }
            
            break; 
            
        case 99:
            *xVal_ptr = 0;
            *yVal_ptr = 0;
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){                   
                    *(world + y*128 + x) = 0x00;                
                    *(finish + y*128 + x) = 0x00;
                }
            }
            break;
            
        deafult: 
            for (y = 0; y < 8; y++){
                for (x = 0; x < 128; x++){                   
                    *(world + y*128 + x) = 0x00;                
                    *(finish + y*128 + x) = 0x00;
                }
            }
            break;
    }
}

//store records data
void writeFlash(uint16_t* data, uint32_t shift)
{
    uint32_t i;
    uint16_t j = 0;
    FLASH_Unlock();
    /* Clear All pending flags */
    FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    //you need to erase entire sector before write anything
    FLASH_EraseSector(FLASH_Sector_11, VoltageRange_2);
    
    for (i=0; i<5; i++){
        FLASH_ProgramHalfWord((0x080E0000 + i*4), *(data+j));
        j++;
    }
    FLASH_Lock();

}

//read records data
uint16_t readFlash(uint32_t shift)
{
   return *(uint16_t *)(0x080E0000 + shift);
}