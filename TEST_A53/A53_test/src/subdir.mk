################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../A53_test/src/main_test.cpp 

OBJS += \
./A53_test/src/main_test.o 

CPP_DEPS += \
./A53_test/src/main_test.d 


# Each subdirectory must supply rules for building sources it contributes
A53_test/src/%.o: ../A53_test/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C++ Compiler'
	aarch64-linux-gnu-g++ "@A53_test/src/main_test.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


