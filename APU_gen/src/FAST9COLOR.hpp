#ifndef _ACF_PROCESS_APU_FAST9COLOR
#define _ACF_PROCESS_APU_FAST9COLOR

#include <acf_process_apu.h>
#include <apex_launcher.hpp>
#include <stdint.h>

#include <FAST9COLOR_APU_LOAD.h> //APU load associated with this process

//SCENARIO LIST*************************************************
static acf_scenario_buffer_data gScenarioBufferData0_FAST9COLOR[] = {{8, 4, 7, 0, {0, 0, 0, 0}}, {8, 4, 4, 0, {1, 1, 2, 2}}, {8, 4, 4, 1, {3, 3, 4, 4}}, {8, 4, 4, 2, {1, 1, 2, 2}}, {8, 4, 4, 3, {2, 2, 2, 2}}, {8, 4, 4, 4, {2, 2, 2, 2}}, {8, 4, 1, 5, {0, 0, 0, 0}}, {8, 4, 1, 5, {0, 0, 0, 0}}, {8, 4, 2, 5, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}, {1, 1, 1, 0, {0, 0, 0, 0}}};

static acf_scenario_kernel_data gScenarioKernelData0_FAST9COLOR[] = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {5, 0}, {5, 0}};

static acf_scenario gScenarioArray_FAST9COLOR[] = {
{0, 0, 2192, 32, 5, gScenarioBufferData0_FAST9COLOR, 176, gScenarioKernelData0_FAST9COLOR, 32}
};

static acf_scenario_list gScenarioList_FAST9COLOR = {
1, //number of scenarios
gScenarioArray_FAST9COLOR};
//**************************************************************

class FAST9COLOR : public ACF_Process_APU
{

public:
   FAST9COLOR(int32_t apex_id = 0) : ACF_Process_APU(apex_id)
   {} 

   int32_t Initialize()
   {
      int32_t lRetVal = 0;

      if (!Initialized()) //initialization steps that only need to occur once
      {
         SetProcessIdentifier("FAST9COLOR");

         lRetVal = NonParametricInitialization();
         if (lRetVal != 0) return lRetVal;

         lRetVal |= mpApexTask->SetupHw("FAST9COLOR",
                                        FAST9COLOR_LOAD_SEGMENTS,
                                        FAST9COLOR_LOAD_PMEM, FAST9COLOR_LOAD_PMEM_SIZE,
                                        FAST9COLOR_LOAD_DMEM, FAST9COLOR_LOAD_DMEM_SIZE,
                                        0, 0); //assuming _LOAD_CMEM does not exist
         SetApuStdioInfo(FAST9COLOR_APEX_LOG_BUFFER, FAST9COLOR_APEX_LOG_SIZE);

         if (lRetVal != 0) return lRetVal;

         FlagSpatialDep();

         //***NOTE: the order in which the following ports are added is meaningful; do not change!

         AddPort("Input", icp::DATATYPE_08U, 3, 1, 8, 4, 0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 0);
         AddPort("Threshold", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 9, 0, 0, 0, 0, 0);
         AddPort("MarkColorChannel", icp::DATATYPE_08U, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 10, 0, 0, 0, 0, 0);
         AddPort("Output", icp::DATATYPE_08U, 3, 1, 8, 4, 1, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0);

         CfgScenarios(&gScenarioList_FAST9COLOR);

         FlagAsInitialized();
      }

      lRetVal = SelectApuConfiguration(mApuCfg, mApexId); //by default mApuCfg = ACF_APU_CFG__APEX0_DEFAULT and mApexId = 0

      return lRetVal;
   }
};

#endif //_ACF_PROCESS_APU_FAST9COLOR
