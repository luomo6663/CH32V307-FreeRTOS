################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/control/chassis.c \
../User/control/encoder.c \
../User/control/mecanum.c \
../User/control/motor.c \
../User/control/pid.c \
../User/control/servo.c \
../User/control/stepper.c \
../User/control/wheel_control.c 

C_DEPS += \
./User/control/chassis.d \
./User/control/encoder.d \
./User/control/mecanum.d \
./User/control/motor.d \
./User/control/pid.d \
./User/control/servo.d \
./User/control/stepper.d \
./User/control/wheel_control.d 

OBJS += \
./User/control/chassis.o \
./User/control/encoder.o \
./User/control/mecanum.o \
./User/control/motor.o \
./User/control/pid.o \
./User/control/servo.o \
./User/control/stepper.o \
./User/control/wheel_control.o 

DIR_OBJS += \
./User/control/*.o \

DIR_DEPS += \
./User/control/*.d \

DIR_EXPANDS += \
./User/control/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/control/%.o: ../User/control/%.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Debug" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Core" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/User" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Peripheral/inc" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/include" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/Common" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/GCC/RISC-V" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/MemMang" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

