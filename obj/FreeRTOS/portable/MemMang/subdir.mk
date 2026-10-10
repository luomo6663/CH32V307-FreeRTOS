################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/portable/MemMang/heap_4.c 

C_DEPS += \
./FreeRTOS/portable/MemMang/heap_4.d 

OBJS += \
./FreeRTOS/portable/MemMang/heap_4.o 

DIR_OBJS += \
./FreeRTOS/portable/MemMang/*.o \

DIR_DEPS += \
./FreeRTOS/portable/MemMang/*.d \

DIR_EXPANDS += \
./FreeRTOS/portable/MemMang/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/portable/MemMang/%.o: ../FreeRTOS/portable/MemMang/%.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Debug" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Core" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/User" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/Peripheral/inc" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/include" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/Common" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/GCC/RISC-V" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions" -I"e:/MounRiver_Project/CH32V307-FreeRTOS/FreeRTOS/portable/MemMang" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

