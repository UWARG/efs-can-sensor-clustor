################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing_main.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking_main.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_smart_presence.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_tank_level.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_touchless_button.c 

C_DEPS += \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing_main.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking_main.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_smart_presence.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_tank_level.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_touchless_button.d 

OBJS += \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing_main.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking_main.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_smart_presence.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_tank_level.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_touchless_button.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/%.o Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/%.su Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/%.cyclo: ../Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/%.c Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/rss/include" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/algorithms" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/integration" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-use_cases-2f-reference_apps

clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-use_cases-2f-reference_apps:
	-$(RM) ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing_main.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing_main.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing_main.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_breathing_main.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking_main.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking_main.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking_main.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_parking_main.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_smart_presence.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_smart_presence.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_smart_presence.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_smart_presence.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_tank_level.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_tank_level.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_tank_level.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_tank_level.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_touchless_button.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_touchless_button.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_touchless_button.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/reference_apps/ref_app_touchless_button.su

.PHONY: clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-use_cases-2f-reference_apps

