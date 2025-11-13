################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/RDB_Depth_Digits36x58.c \
../Core/Src/RDB_Functions.c \
../Core/Src/ST7565.c \
../Core/Src/bmi3.c \
../Core/Src/bmi323.c \
../Core/Src/common.c \
../Core/Src/glcdfont.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/RDB_Depth_Digits36x58.o \
./Core/Src/RDB_Functions.o \
./Core/Src/ST7565.o \
./Core/Src/bmi3.o \
./Core/Src/bmi323.o \
./Core/Src/common.o \
./Core/Src/glcdfont.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/RDB_Depth_Digits36x58.d \
./Core/Src/RDB_Functions.d \
./Core/Src/ST7565.d \
./Core/Src/bmi3.d \
./Core/Src/bmi323.d \
./Core/Src/common.d \
./Core/Src/glcdfont.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/RDB_Depth_Digits36x58.cyclo ./Core/Src/RDB_Depth_Digits36x58.d ./Core/Src/RDB_Depth_Digits36x58.o ./Core/Src/RDB_Depth_Digits36x58.su ./Core/Src/RDB_Functions.cyclo ./Core/Src/RDB_Functions.d ./Core/Src/RDB_Functions.o ./Core/Src/RDB_Functions.su ./Core/Src/ST7565.cyclo ./Core/Src/ST7565.d ./Core/Src/ST7565.o ./Core/Src/ST7565.su ./Core/Src/bmi3.cyclo ./Core/Src/bmi3.d ./Core/Src/bmi3.o ./Core/Src/bmi3.su ./Core/Src/bmi323.cyclo ./Core/Src/bmi323.d ./Core/Src/bmi323.o ./Core/Src/bmi323.su ./Core/Src/common.cyclo ./Core/Src/common.d ./Core/Src/common.o ./Core/Src/common.su ./Core/Src/glcdfont.cyclo ./Core/Src/glcdfont.d ./Core/Src/glcdfont.o ./Core/Src/glcdfont.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

