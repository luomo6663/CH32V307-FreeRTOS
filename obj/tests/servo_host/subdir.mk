################################################################################
# MRS Version: 3.0.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tests/servo_host/test_config.c \
../tests/servo_host/test_console.c \
../tests/servo_host/test_servo.c 

C_DEPS += \
./tests/servo_host/test_config.d \
./tests/servo_host/test_console.d \
./tests/servo_host/test_servo.d 

OBJS += \
./tests/servo_host/test_config.o \
./tests/servo_host/test_console.o \
./tests/servo_host/test_servo.o 

DIR_OBJS += \
./tests/servo_host/*.o \

DIR_DEPS += \
./tests/servo_host/*.d \

DIR_EXPANDS += \
./tests/servo_host/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
tests/servo_host/%.o: ../tests/servo_host/%.c $(FLAGS_C_FILE)
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Debug" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Core" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/User" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Peripheral/inc" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/include" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/Common" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/GCC/RISC-V" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/MemMang" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

