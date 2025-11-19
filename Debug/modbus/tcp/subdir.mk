################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/tcp/mbtcp.c 

C_DEPS += \
./modbus/tcp/mbtcp.d 

OBJS += \
./modbus/tcp/mbtcp.o 


# Each subdirectory must supply rules for building sources it contributes
modbus/tcp/%.o modbus/tcp/%.su modbus/tcp/%.cyclo: ../modbus/tcp/%.c modbus/tcp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/HX711 -I"C:/stands/standKM/kmstand/modbus/include" -I"C:/stands/standKM/kmstand/modbus/rtu" -I"C:/stands/standKM/kmstand/modbus/BARE/port" -I"C:/stands/standKM/kmstand/modbus/ascii" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-modbus-2f-tcp

clean-modbus-2f-tcp:
	-$(RM) ./modbus/tcp/mbtcp.cyclo ./modbus/tcp/mbtcp.d ./modbus/tcp/mbtcp.o ./modbus/tcp/mbtcp.su

.PHONY: clean-modbus-2f-tcp

