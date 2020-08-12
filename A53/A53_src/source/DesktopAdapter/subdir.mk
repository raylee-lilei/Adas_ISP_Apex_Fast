################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../A53_src/source/DesktopAdapter/DesktopAdapter.cpp 

OBJS += \
./A53_src/source/DesktopAdapter/DesktopAdapter.o 

CPP_DEPS += \
./A53_src/source/DesktopAdapter/DesktopAdapter.d 


# Each subdirectory must supply rules for building sources it contributes
A53_src/source/DesktopAdapter/%.o: ../A53_src/source/DesktopAdapter/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C++ Compiler'
	aarch64-linux-gnu-g++ "@A53_src/source/DesktopAdapter/DesktopAdapter.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


