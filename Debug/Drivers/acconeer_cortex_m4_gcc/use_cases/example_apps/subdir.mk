################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection_main.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_surface_velocity.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration_main.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level.c \
../Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level_main.c 

C_DEPS += \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection_main.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_surface_velocity.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration_main.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level.d \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level_main.d 

OBJS += \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection_main.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_surface_velocity.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration_main.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level.o \
./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level_main.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/%.o Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/%.su Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/%.cyclo: ../Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/%.c Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/rss/include" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/algorithms" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/integration" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-use_cases-2f-example_apps

clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-use_cases-2f-example_apps:
	-$(RM) ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection_main.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection_main.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection_main.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_hand_motion_detection_main.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_surface_velocity.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_surface_velocity.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_surface_velocity.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_surface_velocity.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration_main.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration_main.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration_main.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_vibration_main.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level.su ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level_main.cyclo ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level_main.d ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level_main.o ./Drivers/acconeer_cortex_m4_gcc/use_cases/example_apps/example_waste_level_main.su

.PHONY: clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-use_cases-2f-example_apps

