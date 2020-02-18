#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=arm-none-eabi-gcc
CCC=arm-none-eabi-g++
CXX=arm-none-eabi-g++
FC=gfortran
AS=arm-none-eabi-gcc

# Macros
CND_PLATFORM=CodeSourceryARM-Windows
CND_DLIB_EXT=dll
CND_CONF=SC-KIT
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/540162733/misc.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_adc.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_dac.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_dma.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_exti.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_gpio.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_i2c.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_rcc.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_sdio.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_spi.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_syscfg.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_tim.o \
	${OBJECTDIR}/_ext/540162733/stm32f2xx_usart.o \
	${OBJECTDIR}/_ext/1085738613/stm32_eval.o \
	${OBJECTDIR}/_ext/513150005/iprintf.o \
	${OBJECTDIR}/_ext/513150005/sckit_debug.o \
	${OBJECTDIR}/_ext/513150005/serial_driver.o \
	${OBJECTDIR}/_ext/513150005/syscalls.o \
	${OBJECTDIR}/_ext/1360937237/main.o \
	${OBJECTDIR}/_ext/1360937237/stm32f2xx_it.o \
	${OBJECTDIR}/_ext/1360937237/system_stm32f2xx.o \
	${OBJECTDIR}/startup_stm32f2xx.o


# C Compiler Flags
CFLAGS=-mthumb -mcpu=cortex-m3 -ggdb -Wextra -ffunction-sections -fdata-sections -O0

# CC Compiler Flags
CCFLAGS=-mthumb -mcpu=cortex-m3 -ggdb -Wextra -ffunction-sections -fdata-sections -O0
CXXFLAGS=-mthumb -mcpu=cortex-m3 -ggdb -Wextra -ffunction-sections -fdata-sections -O0

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=-c -mthumb -mcpu=cortex-m3 -g3 -Wa,--warn -x assembler-with-cpp

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/Application.elf.exe

${CND_DISTDIR}/${CND_CONF}/Application.elf.exe: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}
	${LINK.c} -o ${CND_DISTDIR}/${CND_CONF}/Application.elf ${OBJECTFILES} ${LDLIBSOPTIONS} -mthumb -mcpu=cortex-m3 -Tstm32_flash.ld -Wl,-Map=${CND_DISTDIR}/${CND_CONF}/Application.map -Wl,--gc-sections

${OBJECTDIR}/_ext/540162733/misc.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/misc.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/misc.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/misc.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_adc.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_adc.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_adc.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_adc.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_dac.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_dac.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_dac.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_dac.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_dma.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_dma.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_dma.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_dma.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_exti.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_exti.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_exti.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_exti.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_gpio.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_gpio.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_gpio.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_gpio.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_i2c.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_i2c.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_i2c.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_i2c.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_rcc.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_rcc.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_rcc.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_rcc.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_sdio.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_sdio.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_sdio.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_sdio.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_spi.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_spi.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_spi.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_spi.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_syscfg.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_syscfg.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_syscfg.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_syscfg.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_tim.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_tim.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_tim.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_tim.c

${OBJECTDIR}/_ext/540162733/stm32f2xx_usart.o: ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_usart.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/540162733
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/540162733/stm32f2xx_usart.o ../../../../Libraries/STM32F2xx_StdPeriph_Driver/src/stm32f2xx_usart.c

${OBJECTDIR}/_ext/1085738613/stm32_eval.o: ../../../../Utilities/STM32_EVAL/stm32_eval.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1085738613
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1085738613/stm32_eval.o ../../../../Utilities/STM32_EVAL/stm32_eval.c

${OBJECTDIR}/_ext/513150005/iprintf.o: ../../../../Utilities/Third_Party/System/iprintf.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/513150005
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/513150005/iprintf.o ../../../../Utilities/Third_Party/System/iprintf.c

${OBJECTDIR}/_ext/513150005/sckit_debug.o: ../../../../Utilities/Third_Party/System/sckit_debug.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/513150005
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/513150005/sckit_debug.o ../../../../Utilities/Third_Party/System/sckit_debug.c

${OBJECTDIR}/_ext/513150005/serial_driver.o: ../../../../Utilities/Third_Party/System/serial_driver.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/513150005
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/513150005/serial_driver.o ../../../../Utilities/Third_Party/System/serial_driver.c

${OBJECTDIR}/_ext/513150005/syscalls.o: ../../../../Utilities/Third_Party/System/syscalls.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/513150005
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/513150005/syscalls.o ../../../../Utilities/Third_Party/System/syscalls.c

${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1360937237
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c

${OBJECTDIR}/_ext/1360937237/stm32f2xx_it.o: ../src/stm32f2xx_it.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1360937237
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1360937237/stm32f2xx_it.o ../src/stm32f2xx_it.c

${OBJECTDIR}/_ext/1360937237/system_stm32f2xx.o: ../src/system_stm32f2xx.c 
	${MKDIR} -p ${OBJECTDIR}/_ext/1360937237
	${RM} $@.d
	$(COMPILE.c) -Wall -DSTM32F2XX -DUSE_SCKIT_EVAL -DUSE_STDPERIPH_DRIVER -I../inc -I../../../../Libraries/CMSIS/Include -I../../../../Libraries/CMSIS/Device/ST/STM32F2xx/Include -I../../../../Libraries/STM32F2xx_StdPeriph_Driver/inc -I../../../../Utilities/STM32_EVAL -I../../../../Utilities/STM32_EVAL/Common -I../../../../Utilities/STM32_EVAL/SCKIT_EVAL -I../../../../Utilities/Third_Party/System -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1360937237/system_stm32f2xx.o ../src/system_stm32f2xx.c

${OBJECTDIR}/startup_stm32f2xx.o: startup_stm32f2xx.s 
	${MKDIR} -p ${OBJECTDIR}
	$(AS) $(ASFLAGS) -o ${OBJECTDIR}/startup_stm32f2xx.o startup_stm32f2xx.s

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/Application.elf.exe

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
