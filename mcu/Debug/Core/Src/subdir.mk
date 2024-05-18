################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/eth.c \
../Core/Src/gpio.c \
../Core/Src/hmc5883l.c \
../Core/Src/i2c.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/motion.c \
../Core/Src/mpu6050.c \
../Core/Src/pid.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/ultrasonic.c \
../Core/Src/usart.c \
../Core/Src/utilities.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/eth.o \
./Core/Src/gpio.o \
./Core/Src/hmc5883l.o \
./Core/Src/i2c.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/motion.o \
./Core/Src/mpu6050.o \
./Core/Src/pid.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/ultrasonic.o \
./Core/Src/usart.o \
./Core/Src/utilities.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/eth.d \
./Core/Src/gpio.d \
./Core/Src/hmc5883l.d \
./Core/Src/i2c.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/motion.d \
./Core/Src/mpu6050.d \
./Core/Src/pid.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/ultrasonic.d \
./Core/Src/usart.d \
./Core/Src/utilities.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/eth.d ./Core/Src/eth.o ./Core/Src/eth.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/hmc5883l.d ./Core/Src/hmc5883l.o ./Core/Src/hmc5883l.su ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motion.d ./Core/Src/motion.o ./Core/Src/motion.su ./Core/Src/mpu6050.d ./Core/Src/mpu6050.o ./Core/Src/mpu6050.su ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/ultrasonic.d ./Core/Src/ultrasonic.o ./Core/Src/ultrasonic.su ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/utilities.d ./Core/Src/utilities.o ./Core/Src/utilities.su

.PHONY: clean-Core-2f-Src

