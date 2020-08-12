################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
IPUS_SRCS += \
/home/lilei/NXP/S32DS.3.1/S32DS/software/VSDK_S32V2_RTM_1_3_0/s32v234_sdk/isp/kernels/generic/src/copy_1to1_ipus.ipus 

OIPUS += \
./ISP_kernels/copy_1to1_ipus.oipus 


# Each subdirectory must supply rules for building sources it contributes
ISP_kernels/copy_1to1_ipus.oipus: /home/lilei/NXP/S32DS.3.1/S32DS/software/VSDK_S32V2_RTM_1_3_0/s32v234_sdk/isp/kernels/generic/src/copy_1to1_ipus.ipus
	@echo 'Building file: $<'
	@echo 'Invoking: S32V IPUS Assembler'
	as-IPUS "@ISP_kernels/copy_1to1_ipus.args" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


