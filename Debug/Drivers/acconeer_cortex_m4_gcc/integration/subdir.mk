################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/acconeer_cortex_m4_gcc/integration/acc_hal_integration_stm32cube_xe121_single_sensor.c \
../Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_log.c \
../Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_stm32.c 

C_DEPS += \
./Drivers/acconeer_cortex_m4_gcc/integration/acc_hal_integration_stm32cube_xe121_single_sensor.d \
./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_log.d \
./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_stm32.d 

OBJS += \
./Drivers/acconeer_cortex_m4_gcc/integration/acc_hal_integration_stm32cube_xe121_single_sensor.o \
./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_log.o \
./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/acconeer_cortex_m4_gcc/integration/%.o Drivers/acconeer_cortex_m4_gcc/integration/%.su Drivers/acconeer_cortex_m4_gcc/integration/%.cyclo: ../Drivers/acconeer_cortex_m4_gcc/integration/%.c Drivers/acconeer_cortex_m4_gcc/integration/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/rss/include" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/algorithms" -I"C:/Users/ronik/Programming/WARG/efs-can-sensor-clustor/Drivers/acconeer_cortex_m4_gcc/integration" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-integration

clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-integration:
	-$(RM) ./Drivers/acconeer_cortex_m4_gcc/integration/acc_hal_integration_stm32cube_xe121_single_sensor.cyclo ./Drivers/acconeer_cortex_m4_gcc/integration/acc_hal_integration_stm32cube_xe121_single_sensor.d ./Drivers/acconeer_cortex_m4_gcc/integration/acc_hal_integration_stm32cube_xe121_single_sensor.o ./Drivers/acconeer_cortex_m4_gcc/integration/acc_hal_integration_stm32cube_xe121_single_sensor.su ./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_log.cyclo ./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_log.d ./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_log.o ./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_log.su ./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_stm32.cyclo ./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_stm32.d ./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_stm32.o ./Drivers/acconeer_cortex_m4_gcc/integration/acc_integration_stm32.su

.PHONY: clean-Drivers-2f-acconeer_cortex_m4_gcc-2f-integration

