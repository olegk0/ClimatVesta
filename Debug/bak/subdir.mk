################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bak/dht11.c \
../bak/lcd.c \
../bak/temp_18b20.c 

OBJS += \
./bak/dht11.o \
./bak/lcd.o \
./bak/temp_18b20.o 

C_DEPS += \
./bak/dht11.d \
./bak/lcd.d \
./bak/temp_18b20.d 


# Each subdirectory must supply rules for building sources it contributes
bak/%.o: ../bak/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=8000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


