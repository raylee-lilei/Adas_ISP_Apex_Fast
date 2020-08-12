################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../APU_gen/src/FAST9COLOR.cpp \
../APU_gen/src/FAST9COLOR__MKDBstub.cpp 

OBJS += \
./APU_gen/src/FAST9COLOR.o \
./APU_gen/src/FAST9COLOR__MKDBstub.o 

CPP_DEPS += \
./APU_gen/src/FAST9COLOR.d \
./APU_gen/src/FAST9COLOR__MKDBstub.d 


# Each subdirectory must supply rules for building sources it contributes
APU_gen/src/%.o: ../APU_gen/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: APU C++ Compiler'
	clang "@APU_gen/src/FAST9COLOR.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


