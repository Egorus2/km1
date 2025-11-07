################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/mb.c 

C_DEPS += \
./modbus/mb.d 

OBJS += \
./modbus/mb.o 


# Each subdirectory must supply rules for building sources it contributes
modbus/%.o modbus/%.su modbus/%.cyclo: ../modbus/%.c modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/HX711 -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/include" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/rtu" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/BARE/port" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/ascii" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-modbus

clean-modbus:
	-$(RM) ./modbus/mb.cyclo ./modbus/mb.d ./modbus/mb.o ./modbus/mb.su

.PHONY: clean-modbus

