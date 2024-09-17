################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AudioFile/stop_command_m.c 

OBJS += \
./AudioFile/stop_command_m.o 

C_DEPS += \
./AudioFile/stop_command_m.d 


# Each subdirectory must supply rules for building sources it contributes
AudioFile/%.o AudioFile/%.su AudioFile/%.cyclo: ../AudioFile/%.c AudioFile/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-AudioFile

clean-AudioFile:
	-$(RM) ./AudioFile/stop_command_m.cyclo ./AudioFile/stop_command_m.d ./AudioFile/stop_command_m.o ./AudioFile/stop_command_m.su

.PHONY: clean-AudioFile

