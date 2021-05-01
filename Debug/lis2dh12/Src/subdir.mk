################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lis2dh12/Src/lis2dh12_read_data_polling.c \
../lis2dh12/Src/lis2dh12_reg.c 

OBJS += \
./lis2dh12/Src/lis2dh12_read_data_polling.o \
./lis2dh12/Src/lis2dh12_reg.o 

C_DEPS += \
./lis2dh12/Src/lis2dh12_read_data_polling.d \
./lis2dh12/Src/lis2dh12_reg.d 


# Each subdirectory must supply rules for building sources it contributes
lis2dh12/Src/lis2dh12_read_data_polling.o: ../lis2dh12/Src/lis2dh12_read_data_polling.c lis2dh12/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L476xx -DDEBUG -c -I../Core/Inc -I"D:/my Git clones/STM32/Nucleo_LIS/lis2dh12/Inc" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lis2dh12/Src/lis2dh12_read_data_polling.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
lis2dh12/Src/lis2dh12_reg.o: ../lis2dh12/Src/lis2dh12_reg.c lis2dh12/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L476xx -DDEBUG -c -I../Core/Inc -I"D:/my Git clones/STM32/Nucleo_LIS/lis2dh12/Inc" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"lis2dh12/Src/lis2dh12_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

