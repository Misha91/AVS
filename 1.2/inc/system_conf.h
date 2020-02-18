#ifndef __SYSTEM_CONF_H_
#define __SYSTEM_CONF_H_

/* StdPeriph Library configuration*/
#include "stm32f2xx_conf.h"

/* enable LCD  */
#define USE_LCD
   
/* default serial port for printf */
#define SERIAL_PORT   COM2

/* default size of heap for malloc */
/* heap for FreeRTOS is in configTOTAL_HEAP_SIZE */
/* heap for LwIP is in MEM_SIZE */
#define MALLOC_HEAP_SIZE 4096
     

#endif /* __SYSTEM_CONF_H_ */

