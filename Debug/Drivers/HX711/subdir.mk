################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/HX711/hx711.c 

C_DEPS += \
./Drivers/HX711/hx711.d 

OBJS += \
./Drivers/HX711/hx711.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/HX711/%.o Drivers/HX711/%.su Drivers/HX711/%.cyclo: ../Drivers/HX711/%.c Drivers/HX711/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/HX711 -I"C:/stands/standKM/kmstand/modbus/include" -I"C:/stands/standKM/kmstand/modbus/rtu" -I"C:/stands/standKM/kmstand/modbus/BARE/port" -I"C:/stands/standKM/kmstand/modbus/ascii" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-HX711

clean-Drivers-2f-HX711:
	-$(RM) ./Drivers/HX711/hx711.cyclo ./Drivers/HX711/hx711.d ./Drivers/HX711/hx711.o ./Drivers/HX711/hx711.su

.PHONY: clean-Drivers-2f-HX711

