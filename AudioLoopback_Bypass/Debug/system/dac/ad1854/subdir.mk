################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/dac/ad1854/adi_ad1854.c 

SRC_OBJS += \
./system/dac/ad1854/adi_ad1854.doj 

C_DEPS += \
./system/dac/ad1854/adi_ad1854.d 


# Each subdirectory must supply rules for building sources it contributes
system/dac/ad1854/%.doj: ../system/dac/ad1854/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore Blackfin C/C++ Compiler'
	ccblkfn.exe -c -file-attr ProjectName="AudioLoopback_Bypass" -proc ADSP-BF537 -flags-compiler --no_wrap_diagnostics -si-revision any -O -Ov100 -g -D_DEBUG -DCORE0 -I"C:\Analog Devices\CrossCore Embedded Studio 2.1.0\Blackfin\lib\src\libdsp" -I"D:\CrossCore\AudioLoopback_Bypass\system" -I"C:/Analog Devices/ADSP-BF537_EZKIT-Rel1.0.0/BF537_EZ-KIT_Lite/Blackfin/include" -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos -double-size-32 -decls-strong -no-cplbs -sdram -gnu-style-dependencies -MD -Mo "system/dac/ad1854/adi_ad1854.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


