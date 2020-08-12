################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include A53_src/source/subdir.mk
-include A53_src/NXPAdapter/subdir.mk
-include A53_src/subdir.mk
-include A53_gen/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
Adas_ISP_Apex.siz \


# All Target
all:
	$(MAKE) --no-print-directory pre-build
	$(MAKE) --no-print-directory main-build

# Main-build Target
main-build: Adas_ISP_Apex.elf secondary-outputs

dependents:
	-cd /home/lilei/workspaceS32DS.3.1/Adas_ISP_Apex/APU && $(MAKE) all

# Tool invocations
Adas_ISP_Apex.elf: $(OBJS) $(USER_OBJS) /home/lilei/workspaceS32DS.3.1/Adas_ISP_Apex/APU/$(TARGETS)
	@echo 'Building target: $@'
	@echo 'Invoking: Standard S32DS C++ Linker'
	aarch64-linux-gnu-g++ -o "Adas_ISP_Apex.elf" "@Adas_ISP_Apex.args"  $(USER_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

Adas_ISP_Apex.siz: Adas_ISP_Apex.elf
	@echo 'Invoking: Standard S32DS Print Size'
	aarch64-linux-gnu-size --format=berkeley Adas_ISP_Apex.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES) ./*/*/*.o  ./*/*.o $(SECONDARY_SIZE) ./*/*/*.d  ./*/*/*.d  ./*/*.d  Adas_ISP_Apex.elf
	-@echo ' '

pre-build:
	-$(MAKE) check_loadh; rm -f ../A53_gen/src/sequencer_srec.c; $(MAKE) -f ../Project_Settings/Scripts/seq.mk  TD="/home/lilei/NXP/S32DS.3.1/eclipse/../S32DS/build_tools/gcc-6.3-arm32-eabi"  VD="/home/lilei/NXP/S32DS.3.1/eclipse/../S32DS/software/VSDK_S32V2_RTM_1_3_0/s32v234_sdk"
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents main-build pre-build
/home/lilei/workspaceS32DS.3.1/Adas_ISP_Apex/APU/$(TARGETS):

-include ../makefile.targets
