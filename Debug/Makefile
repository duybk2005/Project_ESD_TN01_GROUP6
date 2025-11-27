################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../APP \
../BSP \
../COM_MAC \
../DRIVER \
../DRIVER/components \
../DRIVER/devices \
../MW \
../TEST


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../APP/main.c \
../BSP/bsp_adc.c \
../BSP/bsp_eeprom.c \
../BSP/bsp_i2c.c \
../BSP/bsp_interrupt.c \
../BSP/bsp_timer.c \
../BSP/bsp_uart.c \
../DRIVER/components/adc.c \
../DRIVER/components/eeprom.c \
../DRIVER/components/gpio.c \
../DRIVER/components/i2c.c \
../DRIVER/components/interrupt.c \
../DRIVER/components/timer.c \
../DRIVER/components/uart.c \
../DRIVER/devices/bme.c \
../DRIVER/devices/mq.c \
../MW/alarm.c \
../MW/print_uart.c \
../TEST/test_bme.c \
../TEST/test_interrupt.c \
../TEST/test_timer.c \
../TEST/test_mq.c \
../TEST/test_uart.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
APP/main.o \
BSP/bsp_adc.o \
BSP/bsp_eeprom.o \
BSP/bsp_i2c.o \
BSP/bsp_interrupt.o \
BSP/bsp_timer.o \
BSP/bsp_uart.o \
DRIVER/components/adc.o \
DRIVER/components/eeprom.o \
DRIVER/components/gpio.o \
DRIVER/components/i2c.o \
DRIVER/components/interrupt.o \
DRIVER/components/timer.o \
DRIVER/components/uart.o \
DRIVER/devices/bme.o \
DRIVER/devices/mq.o \
MW/alarm.o \
MW/print_uart.o \
TEST/test_bme.o \
TEST/test_interrupt.o \
TEST/test_timer.o \
TEST/test_mq.o \
TEST/test_uart.o

OBJS_AS_ARGS +=  \
APP/main.o \
BSP/bsp_adc.o \
BSP/bsp_eeprom.o \
BSP/bsp_i2c.o \
BSP/bsp_interrupt.o \
BSP/bsp_timer.o \
BSP/bsp_uart.o \
DRIVER/components/adc.o \
DRIVER/components/eeprom.o \
DRIVER/components/gpio.o \
DRIVER/components/i2c.o \
DRIVER/components/interrupt.o \
DRIVER/components/timer.o \
DRIVER/components/uart.o \
DRIVER/devices/bme.o \
DRIVER/devices/mq.o \
MW/alarm.o \
MW/print_uart.o \
TEST/test_bme.o \
TEST/test_interrupt.o \
TEST/test_timer.o \
TEST/test_mq.o \
TEST/test_uart.o

C_DEPS +=  \
APP/main.d \
BSP/bsp_adc.d \
BSP/bsp_eeprom.d \
BSP/bsp_i2c.d \
BSP/bsp_interrupt.d \
BSP/bsp_timer.d \
BSP/bsp_uart.d \
DRIVER/components/adc.d \
DRIVER/components/eeprom.d \
DRIVER/components/gpio.d \
DRIVER/components/i2c.d \
DRIVER/components/interrupt.d \
DRIVER/components/timer.d \
DRIVER/components/uart.d \
DRIVER/devices/bme.d \
DRIVER/devices/mq.d \
MW/alarm.d \
MW/print_uart.d \
TEST/test_bme.d \
TEST/test_interrupt.d \
TEST/test_timer.d \
TEST/test_mq.d \
TEST/test_uart.d

C_DEPS_AS_ARGS +=  \
APP/main.d \
BSP/bsp_adc.d \
BSP/bsp_eeprom.d \
BSP/bsp_i2c.d \
BSP/bsp_interrupt.d \
BSP/bsp_timer.d \
BSP/bsp_uart.d \
DRIVER/components/adc.d \
DRIVER/components/eeprom.d \
DRIVER/components/gpio.d \
DRIVER/components/i2c.d \
DRIVER/components/interrupt.d \
DRIVER/components/timer.d \
DRIVER/components/uart.d \
DRIVER/devices/bme.d \
DRIVER/devices/mq.d \
MW/alarm.d \
MW/print_uart.d \
TEST/test_bme.d \
TEST/test_interrupt.d \
TEST/test_timer.d \
TEST/test_mq.d \
TEST/test_uart.d

OUTPUT_FILE_PATH +=FIRMWARE.elf

OUTPUT_FILE_PATH_AS_ARGS +=FIRMWARE.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
APP/main.o: ../APP/main.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

BSP/bsp_adc.o: ../BSP/bsp_adc.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

BSP/bsp_eeprom.o: ../BSP/bsp_eeprom.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

BSP/bsp_i2c.o: ../BSP/bsp_i2c.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

BSP/bsp_interrupt.o: ../BSP/bsp_interrupt.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

BSP/bsp_timer.o: ../BSP/bsp_timer.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

BSP/bsp_uart.o: ../BSP/bsp_uart.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/components/adc.o: ../DRIVER/components/adc.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/components/eeprom.o: ../DRIVER/components/eeprom.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/components/gpio.o: ../DRIVER/components/gpio.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/components/i2c.o: ../DRIVER/components/i2c.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/components/interrupt.o: ../DRIVER/components/interrupt.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/components/timer.o: ../DRIVER/components/timer.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/components/uart.o: ../DRIVER/components/uart.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/devices/bme.o: ../DRIVER/devices/bme.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

DRIVER/devices/mq.o: ../DRIVER/devices/mq.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

MW/alarm.o: ../MW/alarm.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

MW/print_uart.o: ../MW/print_uart.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

TEST/test_bme.o: ../TEST/test_bme.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

TEST/test_interrupt.o: ../TEST/test_interrupt.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

TEST/test_timer.o: ../TEST/test_timer.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

TEST/test_mq.o: ../TEST/test_mq.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

TEST/test_uart.o: ../TEST/test_uart.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega324P__ -DDEBUG  -I"../BSP" -I"../COM_MAC" -I"../MW" -I"../TEST" -I"../DRIVER/components" -I"../DRIVER/devices" -I"../APP"  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking:  XC8 C Linker : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -mcpu=ATmega324P  -mdfp="E:\MICROCHIP STUDIO\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -Wl,-Map="FIRMWARE.map" -funsigned-char -funsigned-bitfields -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums --memorysummary,memoryfile.xml  
	@echo Finished building target: $@
	"C:\Program Files\Microchip\xc8\v2.36\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "FIRMWARE.elf" "FIRMWARE.hex"
	"C:\Program Files\Microchip\xc8\v2.36\bin\avr-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex "FIRMWARE.elf" "FIRMWARE.eep" || exit 0
	"C:\Program Files\Microchip\xc8\v2.36\bin\avr-objdump.exe" -h -S "FIRMWARE.elf" > "FIRMWARE.lss"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "FIRMWARE.elf" "FIRMWARE.hex" "FIRMWARE.eep" "FIRMWARE.map" "FIRMWARE.usersignatures" "FIRMWARE.a"  "FIRMWARE.lss"
	