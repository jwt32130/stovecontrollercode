################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.o \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.d \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d \
./Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.o: ../Middlewares/ST/STM32_USB_Device_Library/Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra  -g -DNDEBUG -DSTM32L433xx -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/CMSIS/Include" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/jacob/Dev/StoveController.git/StoveControllerCode/StoveControllerCPP/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


