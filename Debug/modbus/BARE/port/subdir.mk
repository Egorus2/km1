################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/BARE/port/portevent.c \
../modbus/BARE/port/portserial.c \
../modbus/BARE/port/porttimer.c 

C_DEPS += \
./modbus/BARE/port/portevent.d \
./modbus/BARE/port/portserial.d \
./modbus/BARE/port/porttimer.d 

OBJS += \
./modbus/BARE/port/portevent.o \
./modbus/BARE/port/portserial.o \
./modbus/BARE/port/porttimer.o 


# Each subdirectory must supply rules for building sources it contributes
modbus/BARE/port/%.o modbus/BARE/port/%.su modbus/BARE/port/%.cyclo: ../modbus/BARE/port/%.c modbus/BARE/port/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/HX711 -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/include" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/rtu" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/BARE/port" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/ascii" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-modbus-2f-BARE-2f-port

clean-modbus-2f-BARE-2f-port:
	-$(RM) ./modbus/BARE/port/portevent.cyclo ./modbus/BARE/port/portevent.d ./modbus/BARE/port/portevent.o ./modbus/BARE/port/portevent.su ./modbus/BARE/port/portserial.cyclo ./modbus/BARE/port/portserial.d ./modbus/BARE/port/portserial.o ./modbus/BARE/port/portserial.su ./modbus/BARE/port/porttimer.cyclo ./modbus/BARE/port/porttimer.d ./modbus/BARE/port/porttimer.o ./modbus/BARE/port/porttimer.su

.PHONY: clean-modbus-2f-BARE-2f-port

