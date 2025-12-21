################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RunnionProjectDrivers/src/PID.c \
../RunnionProjectDrivers/src/i3gd20.c \
../RunnionProjectDrivers/src/kalman.c \
../RunnionProjectDrivers/src/lsm303.c 

OBJS += \
./RunnionProjectDrivers/src/PID.o \
./RunnionProjectDrivers/src/i3gd20.o \
./RunnionProjectDrivers/src/kalman.o \
./RunnionProjectDrivers/src/lsm303.o 

C_DEPS += \
./RunnionProjectDrivers/src/PID.d \
./RunnionProjectDrivers/src/i3gd20.d \
./RunnionProjectDrivers/src/kalman.d \
./RunnionProjectDrivers/src/lsm303.d 


# Each subdirectory must supply rules for building sources it contributes
RunnionProjectDrivers/src/%.o RunnionProjectDrivers/src/%.su RunnionProjectDrivers/src/%.cyclo: ../RunnionProjectDrivers/src/%.c RunnionProjectDrivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc" -include"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc/i3gd20.h" -include"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc/kalman.h" -include"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc/lsm303.h" -include"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc/PID.h" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RunnionProjectDrivers-2f-src

clean-RunnionProjectDrivers-2f-src:
	-$(RM) ./RunnionProjectDrivers/src/PID.cyclo ./RunnionProjectDrivers/src/PID.d ./RunnionProjectDrivers/src/PID.o ./RunnionProjectDrivers/src/PID.su ./RunnionProjectDrivers/src/i3gd20.cyclo ./RunnionProjectDrivers/src/i3gd20.d ./RunnionProjectDrivers/src/i3gd20.o ./RunnionProjectDrivers/src/i3gd20.su ./RunnionProjectDrivers/src/kalman.cyclo ./RunnionProjectDrivers/src/kalman.d ./RunnionProjectDrivers/src/kalman.o ./RunnionProjectDrivers/src/kalman.su ./RunnionProjectDrivers/src/lsm303.cyclo ./RunnionProjectDrivers/src/lsm303.d ./RunnionProjectDrivers/src/lsm303.o ./RunnionProjectDrivers/src/lsm303.su

.PHONY: clean-RunnionProjectDrivers-2f-src

