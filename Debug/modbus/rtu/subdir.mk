################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/rtu/mbcrc.c \
../modbus/rtu/mbrtu.c 

C_DEPS += \
./modbus/rtu/mbcrc.d \
./modbus/rtu/mbrtu.d 

OBJS += \
./modbus/rtu/mbcrc.o \
./modbus/rtu/mbrtu.o 


# Each subdirectory must supply rules for building sources it contributes
modbus/rtu/%.o modbus/rtu/%.su modbus/rtu/%.cyclo: ../modbus/rtu/%.c modbus/rtu/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/HX711 -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/include" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/rtu" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/BARE/port" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/ascii" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-modbus-2f-rtu

clean-modbus-2f-rtu:
	-$(RM) ./modbus/rtu/mbcrc.cyclo ./modbus/rtu/mbcrc.d ./modbus/rtu/mbcrc.o ./modbus/rtu/mbcrc.su ./modbus/rtu/mbrtu.cyclo ./modbus/rtu/mbrtu.d ./modbus/rtu/mbrtu.o ./modbus/rtu/mbrtu.su

.PHONY: clean-modbus-2f-rtu

