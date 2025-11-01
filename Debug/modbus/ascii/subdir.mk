################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/ascii/mbascii.c 

C_DEPS += \
./modbus/ascii/mbascii.d 

OBJS += \
./modbus/ascii/mbascii.o 


# Each subdirectory must supply rules for building sources it contributes
modbus/ascii/%.o modbus/ascii/%.su modbus/ascii/%.cyclo: ../modbus/ascii/%.c modbus/ascii/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/HX711 -I"C:/KMStand/kmstand/modbus/include" -I"C:/KMStand/kmstand/modbus/rtu" -I"C:/KMStand/kmstand/modbus/BARE/port" -I"C:/KMStand/kmstand/modbus/ascii" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-modbus-2f-ascii

clean-modbus-2f-ascii:
	-$(RM) ./modbus/ascii/mbascii.cyclo ./modbus/ascii/mbascii.d ./modbus/ascii/mbascii.o ./modbus/ascii/mbascii.su

.PHONY: clean-modbus-2f-ascii

