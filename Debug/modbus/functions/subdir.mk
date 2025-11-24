################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/functions/mbfunccoils.c \
../modbus/functions/mbfuncdiag.c \
../modbus/functions/mbfuncdisc.c \
../modbus/functions/mbfuncholding.c \
../modbus/functions/mbfuncinput.c \
../modbus/functions/mbfuncother.c \
../modbus/functions/mbutils.c 

C_DEPS += \
./modbus/functions/mbfunccoils.d \
./modbus/functions/mbfuncdiag.d \
./modbus/functions/mbfuncdisc.d \
./modbus/functions/mbfuncholding.d \
./modbus/functions/mbfuncinput.d \
./modbus/functions/mbfuncother.d \
./modbus/functions/mbutils.d 

OBJS += \
./modbus/functions/mbfunccoils.o \
./modbus/functions/mbfuncdiag.o \
./modbus/functions/mbfuncdisc.o \
./modbus/functions/mbfuncholding.o \
./modbus/functions/mbfuncinput.o \
./modbus/functions/mbfuncother.o \
./modbus/functions/mbutils.o 


# Each subdirectory must supply rules for building sources it contributes
modbus/functions/%.o modbus/functions/%.su modbus/functions/%.cyclo: ../modbus/functions/%.c modbus/functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/HX711 -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/include" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/rtu" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/BARE/port" -I"C:/Users/AndreyChuev/Desktop/KMStand/kmstand/modbus/ascii" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-modbus-2f-functions

clean-modbus-2f-functions:
	-$(RM) ./modbus/functions/mbfunccoils.cyclo ./modbus/functions/mbfunccoils.d ./modbus/functions/mbfunccoils.o ./modbus/functions/mbfunccoils.su ./modbus/functions/mbfuncdiag.cyclo ./modbus/functions/mbfuncdiag.d ./modbus/functions/mbfuncdiag.o ./modbus/functions/mbfuncdiag.su ./modbus/functions/mbfuncdisc.cyclo ./modbus/functions/mbfuncdisc.d ./modbus/functions/mbfuncdisc.o ./modbus/functions/mbfuncdisc.su ./modbus/functions/mbfuncholding.cyclo ./modbus/functions/mbfuncholding.d ./modbus/functions/mbfuncholding.o ./modbus/functions/mbfuncholding.su ./modbus/functions/mbfuncinput.cyclo ./modbus/functions/mbfuncinput.d ./modbus/functions/mbfuncinput.o ./modbus/functions/mbfuncinput.su ./modbus/functions/mbfuncother.cyclo ./modbus/functions/mbfuncother.d ./modbus/functions/mbfuncother.o ./modbus/functions/mbfuncother.su ./modbus/functions/mbutils.cyclo ./modbus/functions/mbutils.d ./modbus/functions/mbutils.o ./modbus/functions/mbutils.su

.PHONY: clean-modbus-2f-functions

