################################################################################
# Automatically-generated file. Do not edit!
################################################################################

ECHO = @echo
RM := rm -rf
CD := cd

CURRENT_DIR := $(notdir $(shell pwd))
GRAPHS_DIR := APU_gen/src
ACF_DIR := graphs
VSDK_DIR := /home/lilei/NXP/S32DS.3.1/eclipse/../S32DS/software/VSDK_S32V2_RTM_1_3_0/s32v234_sdk

-include ../makefile.acf.init

GRAPHS := $(patsubst %_apu_process_desc_main.cpp,%, $(notdir $(wildcard ../$(ACF_DIR)/*_apu_process_desc_main.cpp)))

CX := g++
LD := g++
CXX_INCS += -I"$(VSDK_DIR)/include" -I"$(VSDK_DIR)/libs/apex/acf_offline/include" -I"../APU_kernels"
CXX_OPTS := -std=c++11 -D_GLIBCXX_USE_CXX11_ABI=0 -includeapu2_kernel_db_lib.h -Wall -c -fmessage-length=0 -MMD
LD_OPTS := -L"$(VSDK_DIR)/libs/apex/acf_offline/build-x86-gnu-linux-d"
ACF_LIBS := -lacf_offline
UP_CASE_GRAPHS := $(shell echo $(GRAPHS) | tr '[:lower:]' '[:upper:]')
ACF_MKDBSTUBS := $(foreach graph,$(UP_CASE_GRAPHS),../$(GRAPHS_DIR)/$(graph)__MKDBstub.cpp)
ACF_NATIVEKERNELS := $(foreach graph,$(UP_CASE_GRAPHS),../$(GRAPHS_DIR)/$(graph)_nativekernel.hpp)
ACF_CPPS := $(foreach graph,$(UP_CASE_GRAPHS),../$(GRAPHS_DIR)/$(graph).cpp)
ACF_HPPS := $(foreach graph,$(UP_CASE_GRAPHS),../$(GRAPHS_DIR)/$(graph).hpp)
ACF_TXTS := $(foreach graph,$(UP_CASE_GRAPHS),../$(GRAPHS_DIR)/$(graph)_analysis_log.txt)
ACF_TARGETS := $(foreach graph,$(GRAPHS),$(graph).exe)
ACF_TARGETS += $(ACF_MKDBSTUBS) $(ACF_NATIVEKERNELS) $(ACF_CPPS) $(ACF_HPPS) $(ACF_TXTS)

define acf_rules
ACF_OBJS += $(ACF_DIR)/$(1)_apu_process_desc_main.o
ACF_DEPS += $(ACF_DIR)/$(1)_apu_process_desc_main.d

$(ACF_DIR)/$(1)_apu_process_desc_main.o: ../$(ACF_DIR)/$(1)_apu_process_desc_main.cpp
	$(ECHO) ' '
	$(ECHO) 'Building file: ../$(ACF_DIR)/$(1)_apu_process_desc_main.cpp'
	$(ECHO) 'Invoking: Cross G++ Compiler'
	$(CX) $(CXX_OPTS) $(CXX_INCS) -o $(ACF_DIR)/$(1)_apu_process_desc_main.o ../$(ACF_DIR)/$(1)_apu_process_desc_main.cpp
	$(ECHO) 'Finished building: ../$(ACF_DIR)/$(1)_apu_process_desc_main.cpp'

$(1).exe: $(ACF_DIR)/$(1)_apu_process_desc_main.o
	$(ECHO) ' '
	$(ECHO) 'Building target: $(1).exe'
	$(ECHO) 'Invoking: Cross G++ Linker'
	$(LD) $(LD_OPTS) -o $(1).exe $(ACF_DIR)/$(1)_apu_process_desc_main.o $(ACF_LIBS)
	$(ECHO) 'Finished building target: $(1).exe'

../$(GRAPHS_DIR)/$(shell echo $(1) | tr '[:lower:]' '[:upper:]')__MKDBstub.cpp: $(1).exe
	$(ECHO) ' '
	$(ECHO) 'Executing ACF resolver: $(1)'
	$(CD) ../$(GRAPHS_DIR) && ../../$(CURRENT_DIR)/$(1).exe && $(CD) -

../$(GRAPHS_DIR)/$(shell echo $(1) | tr '[:lower:]' '[:upper:]')_nativekernel.hpp: $(1).exe
	$(ECHO) ' '
	$(ECHO) 'Executing ACF resolver: $(1)'
	$(CD) ../$(GRAPHS_DIR) && ../../$(CURRENT_DIR)/$(1).exe && $(CD) -

../$(GRAPHS_DIR)/$(shell echo $(1) | tr '[:lower:]' '[:upper:]').cpp: $(1).exe
	$(ECHO) ' '
	$(ECHO) 'Executing ACF resolver: $(1)'
	$(CD) ../$(GRAPHS_DIR) && ../../$(CURRENT_DIR)/$(1).exe && $(CD) -

../$(GRAPHS_DIR)/$(shell echo $(1) | tr '[:lower:]' '[:upper:]').hpp: $(1).exe
	$(ECHO) ' '
	$(ECHO) 'Executing ACF resolver: $(1)'
	$(CD) ../$(GRAPHS_DIR) && ../../$(CURRENT_DIR)/$(1).exe && $(CD) -

../$(GRAPHS_DIR)/$(shell echo $(1) | tr '[:lower:]' '[:upper:]')_analysis_log.txt: $(1).exe
	$(ECHO) ' '
	$(ECHO) 'Executing ACF resolver: $(1)'
	$(CD) ../$(GRAPHS_DIR) && ../../$(CURRENT_DIR)/$(1).exe && $(CD) -
endef

$(foreach graph,$(GRAPHS),$(eval $(call acf_rules,$(graph))))

-include $(ACF_DEPS)

all: $(ACF_TARGETS)
	$(ECHO) ' '

clean-acf-outputs:
	-$(RM) $(ACF_MKDBSTUBS) $(ACF_NATIVEKERNELS) $(ACF_CPPS) $(ACF_HPPS) $(ACF_TXTS)

clean:
	-$(RM) $(ACF_DEPS) $(ACF_OBJS) $(ACF_TARGETS)

.PHONY: all clean clean-acf-outputs
