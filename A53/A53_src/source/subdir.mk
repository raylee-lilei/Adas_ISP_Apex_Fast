################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../A53_src/source/DSM.cpp \
../A53_src/source/FCW.cpp \
../A53_src/source/HogSvmDetect.cpp \
../A53_src/source/LDWS.cpp \
../A53_src/source/LDWS2.cpp \
../A53_src/source/MediaCapture.cpp \
../A53_src/source/Message.cpp \
../A53_src/source/Notify.cpp \
../A53_src/source/PeopleDetect.cpp \
../A53_src/source/Points.cpp \
../A53_src/source/Predict.cpp \
../A53_src/source/ShowImage.cpp \
../A53_src/source/TSR.cpp \
../A53_src/source/Util.cpp \
../A53_src/source/anyarg.cpp \
../A53_src/source/config.cpp 

OBJS += \
./A53_src/source/DSM.o \
./A53_src/source/FCW.o \
./A53_src/source/HogSvmDetect.o \
./A53_src/source/LDWS.o \
./A53_src/source/LDWS2.o \
./A53_src/source/MediaCapture.o \
./A53_src/source/Message.o \
./A53_src/source/Notify.o \
./A53_src/source/PeopleDetect.o \
./A53_src/source/Points.o \
./A53_src/source/Predict.o \
./A53_src/source/ShowImage.o \
./A53_src/source/TSR.o \
./A53_src/source/Util.o \
./A53_src/source/anyarg.o \
./A53_src/source/config.o 

CPP_DEPS += \
./A53_src/source/DSM.d \
./A53_src/source/FCW.d \
./A53_src/source/HogSvmDetect.d \
./A53_src/source/LDWS.d \
./A53_src/source/LDWS2.d \
./A53_src/source/MediaCapture.d \
./A53_src/source/Message.d \
./A53_src/source/Notify.d \
./A53_src/source/PeopleDetect.d \
./A53_src/source/Points.d \
./A53_src/source/Predict.d \
./A53_src/source/ShowImage.d \
./A53_src/source/TSR.d \
./A53_src/source/Util.d \
./A53_src/source/anyarg.d \
./A53_src/source/config.d 


# Each subdirectory must supply rules for building sources it contributes
A53_src/source/%.o: ../A53_src/source/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C++ Compiler'
	aarch64-linux-gnu-g++ "@A53_src/source/DSM.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


