################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../startup/startup_stm32l433xx.S 

OBJS += \
./startup/startup_stm32l433xx.o 

S_UPPER_DEPS += \
./startup/startup_stm32l433xx.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra  -g -x assembler-with-cpp -DDEBUG -DSTM32L433xx -I"/home/jacob/Dev/StoveController/StoveControllerCode/StoveControllerCPP/Drivers/CMSIS/Include" -I"/home/jacob/Dev/StoveController/StoveControllerCode/StoveControllerCPP/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/jacob/Dev/StoveController/StoveControllerCode/StoveControllerCPP/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/jacob/Dev/StoveController/StoveControllerCode/StoveControllerCPP/Inc" -I"/home/jacob/Dev/StoveController/StoveControllerCode/StoveControllerCPP/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/jacob/Dev/StoveController/StoveControllerCode/StoveControllerCPP/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


