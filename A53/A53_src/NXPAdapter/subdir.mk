################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../A53_src/NXPAdapter/HogSvmDetect.cpp \
../A53_src/NXPAdapter/NXPAdapter.cpp \
../A53_src/NXPAdapter/NXPCamera.cpp \
../A53_src/NXPAdapter/NXPCameraNew.cpp \
../A53_src/NXPAdapter/SaveVideo.cpp 

OBJS += \
./A53_src/NXPAdapter/HogSvmDetect.o \
./A53_src/NXPAdapter/NXPAdapter.o \
./A53_src/NXPAdapter/NXPCamera.o \
./A53_src/NXPAdapter/NXPCameraNew.o \
./A53_src/NXPAdapter/SaveVideo.o 

CPP_DEPS += \
./A53_src/NXPAdapter/HogSvmDetect.d \
./A53_src/NXPAdapter/NXPAdapter.d \
./A53_src/NXPAdapter/NXPCamera.d \
./A53_src/NXPAdapter/NXPCameraNew.d \
./A53_src/NXPAdapter/SaveVideo.d 


# Each subdirectory must supply rules for building sources it contributes
A53_src/NXPAdapter/%.o: ../A53_src/NXPAdapter/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C++ Compiler'
	aarch64-linux-gnu-g++ "@A53_src/NXPAdapter/HogSvmDetect.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


