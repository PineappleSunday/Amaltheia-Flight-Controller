################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FlightControl/AHRS.c \
../FlightControl/flight_logic.c \
../FlightControl/mixer.c \
../FlightControl/navigation.c \
../FlightControl/state.c 

OBJS += \
./FlightControl/AHRS.o \
./FlightControl/flight_logic.o \
./FlightControl/mixer.o \
./FlightControl/navigation.o \
./FlightControl/state.o 

C_DEPS += \
./FlightControl/AHRS.d \
./FlightControl/flight_logic.d \
./FlightControl/mixer.d \
./FlightControl/navigation.d \
./FlightControl/state.d 


# Each subdirectory must supply rules for building sources it contributes
FlightControl/%.o FlightControl/%.su FlightControl/%.cyclo: ../FlightControl/%.c FlightControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc" -I"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/FlightControl" -include"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc/i3gd20.h" -include"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc/kalman.h" -include"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc/lsm303.h" -include"G:/Electronics Projects/Flight Controller Quadcopter/Amaltheia Flight Controller/RunnionProjectDrivers/inc/PID.h" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FlightControl

clean-FlightControl:
	-$(RM) ./FlightControl/AHRS.cyclo ./FlightControl/AHRS.d ./FlightControl/AHRS.o ./FlightControl/AHRS.su ./FlightControl/flight_logic.cyclo ./FlightControl/flight_logic.d ./FlightControl/flight_logic.o ./FlightControl/flight_logic.su ./FlightControl/mixer.cyclo ./FlightControl/mixer.d ./FlightControl/mixer.o ./FlightControl/mixer.su ./FlightControl/navigation.cyclo ./FlightControl/navigation.d ./FlightControl/navigation.o ./FlightControl/navigation.su ./FlightControl/state.cyclo ./FlightControl/state.d ./FlightControl/state.o ./FlightControl/state.su

.PHONY: clean-FlightControl

