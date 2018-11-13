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
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -x assembler-with-cpp -DDEBUG -DSTM32L433xx -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/CMSIS/Include" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


