################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Encoder.c \
../Src/GPIO.c \
../Src/PWM.c \
../Src/main.c \
../Src/spi.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/systick.c \
../Src/tim2.c \
../Src/uart2.c 

OBJS += \
./Src/Encoder.o \
./Src/GPIO.o \
./Src/PWM.o \
./Src/main.o \
./Src/spi.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/systick.o \
./Src/tim2.o \
./Src/uart2.o 

C_DEPS += \
./Src/Encoder.d \
./Src/GPIO.d \
./Src/PWM.d \
./Src/main.d \
./Src/spi.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/systick.d \
./Src/tim2.d \
./Src/uart2.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/chip_headers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/chip_headers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/Encoder.cyclo ./Src/Encoder.d ./Src/Encoder.o ./Src/Encoder.su ./Src/GPIO.cyclo ./Src/GPIO.d ./Src/GPIO.o ./Src/GPIO.su ./Src/PWM.cyclo ./Src/PWM.d ./Src/PWM.o ./Src/PWM.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/spi.cyclo ./Src/spi.d ./Src/spi.o ./Src/spi.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/systick.cyclo ./Src/systick.d ./Src/systick.o ./Src/systick.su ./Src/tim2.cyclo ./Src/tim2.d ./Src/tim2.o ./Src/tim2.su ./Src/uart2.cyclo ./Src/uart2.d ./Src/uart2.o ./Src/uart2.su

.PHONY: clean-Src

