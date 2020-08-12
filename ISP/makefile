################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include ISP_kernels/subdir.mk
-include subdir.mk
-include objects.mk

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	$(MAKE) --no-print-directory pre-build
	$(MAKE) --no-print-directory main-build

# Main-build Target
main-build: Adas_ISP_Apex_ISP.elf

# Tool invocations
Adas_ISP_Apex_ISP.elf: /home/lilei/workspaceS32DS.3.1/Adas_ISP_Apex/Project_Settings/Linker_Files/S32V_ISP.ld $(OIPUS) $(OIPUV)
	@echo 'Building target: $@'
	@echo 'Invoking: S32V ISP C++ Linker'
	arm-none-eabi-ld -o "Adas_ISP_Apex_ISP.elf" "@Adas_ISP_Apex_ISP.args"  $(OIPUV)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OIPUV)$(OIPUS) Adas_ISP_Apex_ISP.elf
	-@echo ' '

pre-build:
	-rm -f ../A53_gen/inc/kmem_sym.h ../A53_gen/src/kmem.c ../ISP/*.elf
	-@echo ' '

post-build:
	-$(MAKE) -f ../Project_Settings/Scripts/isp_kmem.mk ProjName=Adas_ISP_Apex
	-@echo ' '

.PHONY: all clean dependents main-build pre-build post-build

-include ../makefile.targets
