################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SX1211Driver.c \
../Src/main.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/SX1211Driver.o \
./Src/main.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/SX1211Driver.d \
./Src/main.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32L433xx -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/CMSIS/Include" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


