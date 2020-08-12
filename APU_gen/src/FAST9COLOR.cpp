#include <icp_datatype.h>
#include <acf_common.h>
#include <acf_utilities.h>
#include <acf_hwutilities.h>
#include <acf_memmgmt.h>
#include <linked_list_utilities.h> //for linked list entry sizes
#include <mcdma_hal.h>
#include <mce_hal.h>
#include <seq_hal.h>
#include <stio_hal.h>
#include <acf_cbmanager.hpp>
#include <acf_odprodpattern.hpp>
#include <util.h>

//global values that can be used by advanced kernels to perform init/reduction operations
extern int16_t gsACF_TILE_WIDTH_IN_CHUNKS;
extern int16_t gsACF_FIRST_CUID;
extern int16_t gsACF_NUM_INPUT_ITERATIONS;
extern int16_t gsACF_CU_ARRAY_WIDTH;

//profiling stuff
#if ((1 == ACF_SIM_NATIVE) || (1 == ACF_SIM_ISS))
#define RESET_PERF_REG //empty
#define QUERY_PERF_REG 0
#else
#include <apex_642cl_map.h>
#include <apex_642cl_dmem_fifo_map_type.h>
#define RESET_PERF_REG (*((volatile unsigned int *)(APEX_642CL_DMEM_FIFO_0_PERFORMANCE)) = 0)
#define QUERY_PERF_REG (*((volatile unsigned int *)(APEX_642CL_DMEM_FIFO_0_PERFORMANCE)))
#endif
volatile static struct ApuPerfInfoStruct gPerfInfo;
static int32_t gProbeStart    = 0;
static int32_t gProbeA        = 0;
static int32_t gProbeB        = 0;

//kernel function signatures
void apu_rgb_to_grayscale(kernel_io_desc, kernel_io_desc);
void apu_gauss_3x3(kernel_io_desc, kernel_io_desc);
void apu_fast9(kernel_io_desc, kernel_io_desc, kernel_io_desc);
void apu_nms(kernel_io_desc, kernel_io_desc);
void apu_dilate_diamond(kernel_io_desc, kernel_io_desc);
void apu_dilate_diamond(kernel_io_desc, kernel_io_desc);
void apu_saturate_nonzero(kernel_io_desc, kernel_io_desc);
void apu_mark_color_channel(kernel_io_desc, kernel_io_desc, kernel_io_desc, kernel_io_desc);

//global input info for DMA+STREAM from XMEM to CMEM (populated/initialized by the host)
static acf_io_desc gAcfInputDescArray[1]; //[0]=Input 

//global input info for static input DMA xfer (populated/initialized by the host)
static acf_io_desc gAcfStaticInputDescArray[2]; //[0]=Threshold [1]=MarkColorChannel 

//global output info for STREAM+DMA from CMEM to XMEM (populated/initialized by the host)
static acf_io_desc gAcfOutputDescArray[1]; //[0]=Output 

//global linked list memory allocation (must be 128-bit aligned)
const int gInputLinkedListMemSize = LLE_SIZE_2D_DMA_XMEM_CMEM*1 + LLE_SIZE_2D_DMA_MEM_MEM*0;
DECLARE_BYTE_ALIGNED(static int, gAcfInputLinkedListMem[gInputLinkedListMemSize*2], 16); //*2 for double buffering 
const int gOutputLinkedListMemSize = LLE_SIZE_2D_DMA_XMEM_CMEM*1 + LLE_SIZE_2D_DMA_MEM_MEM*0;
DECLARE_BYTE_ALIGNED(static int, gAcfOutputLinkedListMem[gOutputLinkedListMemSize*2], 16); //*2 for double buffering 

//allocate global array of local memory descriptors
static ACF_LocalMemDesc gLocalMemArray[11];

//allocate memory for linked list entry database(s)
static acf_llentry_desc gInputLinkedListEntryDatabase[1];
static acf_llentry_desc gOutputLinkedListEntryDatabase[1];

//allocate global array of advanced transfer info that is associated (by index) with gLocalMemArray
static vec16u gvBytesPushedIntoFifoArray[1];  //not used
static uint32_t* gPtrToFinalFifoSizeArray[1]; //not used

//the physical DMEM base address from the DMA's perspective (i.e. the host/fpga perspective)
extern int gsDmemBaseAddrForDma;

//allocate global array of kernel_io_desc
static kernel_io_desc gAcfKernelIoDescArray[19];

//circular buffer index tracking (NOTE: circular buffer index tracking related to kernels is currently tied to kernel_io_desc)
static int16_t gCbIdxArray_pad[11]; //tied to padding

//grouping of IOs into logical transfers (managing transfers with multiple DMA)
static int8_t gNumLogicalTransfersCmemIn = 0;
static int8_t gNumLogicalTransfersDmemIn = 0;
static int8_t gNumLogicalTransfersCmemOut = 0;
static int8_t gNumLogicalTransfersDmemOut = 0;
static int8_t gNumLogicalTransfersCmemIn_MCE = 0;
static struct transfer gLogicalTransfersCmemIn[1];
static struct transfer* gLogicalTransfersDmemIn = 0;
static struct transfer gLogicalTransfersCmemOut[1];
static struct transfer* gLogicalTransfersDmemOut = 0;
static struct transfer* gLogicalTransfersCmemIn_MCE = 0;

volatile static int gNumInputIterations = 0;
volatile static int gInputIter          = 0;
volatile static int gProcIter           = 0;
volatile static int gInputIterMce       = 0;

int FAST9COLOR(int* const lpParams, int const lNumParams)
{
   RESET_PERF_REG;

   gProbeStart = QUERY_PERF_REG;

   int lRetVal = ACF_SUCCESS;

   //---------------------------
   //unpack parameters from host
   //---------------------------
   int lNumInputIterations          = lpParams[0];
   int lInputTileWidthInChunks      = lpParams[1];
   int lCoreDataCuOffset            = lpParams[2];
   int lOutputTileWidthInChunks     = lpParams[3];
   int lNumOutputIterations         = lpParams[4];
   int lOutputSkip                  = lpParams[5]; //number of output tiles to skip (related to ROI processing)
   int lLastInputTileWidthInChunks  = lpParams[6];
   int lLastOutputTileWidthInChunks = lpParams[7];
   gsDmemBaseAddrForDma             = lpParams[8];
   int lCmemHeapBaseAddr            = lpParams[9];
   int lCmemHeapSizeInbytes         = lpParams[10];
   int lDmemHeapBaseAddr            = lpParams[11];
   int lDmemHeapSizeInBytes         = lpParams[12];
   int lClover389Flag               = lpParams[13];
   gsACF_CU_ARRAY_WIDTH             = lpParams[14];
   gNumInputIterations              = lNumInputIterations; //for debugging

   //only make last tile width adjustment if it differs from previous tiles
   int lLastInputTileWidthInChunks_tmp = (lLastInputTileWidthInChunks == lInputTileWidthInChunks) ? -1 : lLastInputTileWidthInChunks;
   int lLastOutputTileWidthInChunks_tmp = (lLastOutputTileWidthInChunks == lOutputTileWidthInChunks) ? -1 : lLastOutputTileWidthInChunks;

   acf_res_info* lpResInfo = (acf_res_info*)(&lpParams[16]); //HW resources
   int lApexId = lpResInfo->apexId;

   memcpy_int(&lpParams[32], (int*)gAcfInputDescArray, 1*14);
   memcpy_int(&lpParams[46], (int*)gAcfStaticInputDescArray, 2*14);
   memcpy_int(&lpParams[74], (int*)gAcfOutputDescArray, 1*14);
   int lScenarioIterAdjust = lpParams[88];
   acf_scenario_buffer_data* lpScenarioBufferData = (acf_scenario_buffer_data*)&lpParams[89];
   acf_scenario_kernel_data* lpScenarioKernelData = (acf_scenario_kernel_data*)&lpParams[133];

   //***********************************************
   //initialization---------------------------------
   //***********************************************

   //sequencer is required for generating the 'final' interrupt that indicates to the host that execution has completed
   do
   {
   ACF_InitSeqRegPtr(lApexId); //initialize global static pointer to sequencer ONCE for use by subsequent ACF_ seq hal calls
   seq_hal_enable_seq_apu(lApexId, lpResInfo->apuId, lpResInfo->procIdSeqRes);
   seq_hal_set_rsc_counter(lApexId, lpResInfo->zeroSeqRes_Done, ((1 << 16) - 1)); //initialize to -1

   gsACF_TILE_WIDTH_IN_CHUNKS = lInputTileWidthInChunks;
   gsACF_FIRST_CUID           = lCoreDataCuOffset;
   gsACF_NUM_INPUT_ITERATIONS = lNumInputIterations;

   int lApuCuOffset = 0; //0 if APU0, 32 if APU1 (this is for the stream dma 'start cu' field
   if (lpResInfo->apuId == 1) lApuCuOffset = 32;

   if (ACF_OrgLogXfer_CmemOnly(lpResInfo->numDmaChIn, lpResInfo->numStreamInChForDma, 1, &gNumLogicalTransfersCmemIn, gLogicalTransfersCmemIn) != 0)
   {
      lRetVal = ACF_HW_RESOURCE_ERROR;
      break;//goto EARLY_EXIT;
   }

   if (ACF_OrgLogXfer_CmemOnly(lpResInfo->numDmaChOut, lpResInfo->numStreamOutChForDma, 1, &gNumLogicalTransfersCmemOut, gLogicalTransfersCmemOut) != 0)
   {
      lRetVal = ACF_HW_RESOURCE_ERROR;
      break;//goto EARLY_EXIT;
   }

   InitVecHeap((vec08s*)(uint32_t)lCmemHeapBaseAddr, lCmemHeapSizeInbytes);
   InitSclHeap((int8_t*)(uint32_t)lDmemHeapBaseAddr, lDmemHeapSizeInBytes);

   //calculate padding offset info (relevant for vector inputs where input size is not a nicely divisible by chunk width)
   int lPadOffsetRefChunkWidth = lpScenarioBufferData[0].chunkWidth;
   int lPadOffsetRefChunkHeight = lpScenarioBufferData[0].chunkHeight;
   int lTmp = gAcfInputDescArray[0].nd_size.width % lPadOffsetRefChunkWidth;
   int lPadOffsetR = (lTmp == 0) ? (0)  : (lPadOffsetRefChunkWidth - lTmp);
   lTmp = gAcfInputDescArray[0].nd_size.height % lPadOffsetRefChunkHeight;
   int lPadOffsetB = (lTmp == 0) ? (0)  : (lPadOffsetRefChunkHeight - lTmp);

   for (int i=0; i<11; i++)
      gCbIdxArray_pad[i] = 0;

   //initialize array of local memory descriptors
   int lLmError = 0;

   //Memory associated with the following ports:
   //Producer: graph port 'Input'
   //Consumer kernel 'rgb_to_grayscale_0' port 'INPUT_0'
   //Consumer kernel 'mark_color_channel_0' port 'INPUT_IMAGE'
   lLmError += LmInit(&gLocalMemArray[0], &lpScenarioBufferData[0], icp::DATATYPE_08U, 3, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: kernel 'rgb_to_grayscale_0' port 'OUTPUT_0'
   //Consumer kernel 'gauss_3x3_0' port 'INPUT_0'
   lLmError += LmInit(&gLocalMemArray[1], &lpScenarioBufferData[1], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: kernel 'gauss_3x3_0' port 'OUTPUT_0'
   //Consumer kernel 'fast9_0' port 'IN_Img'
   lLmError += LmInit(&gLocalMemArray[2], &lpScenarioBufferData[2], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: kernel 'fast9_0' port 'OUT_Img'
   //Consumer kernel 'nms_0' port 'INPUT_0'
   lLmError += LmInit(&gLocalMemArray[3], &lpScenarioBufferData[3], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: kernel 'nms_0' port 'OUTPUT_0'
   //Consumer kernel 'dilate_diamond_0' port 'INPUT_0'
   lLmError += LmInit(&gLocalMemArray[4], &lpScenarioBufferData[4], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: kernel 'dilate_diamond_0' port 'OUTPUT_0'
   //Consumer kernel 'dilate_diamond_1' port 'INPUT_0'
   lLmError += LmInit(&gLocalMemArray[5], &lpScenarioBufferData[5], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: kernel 'dilate_diamond_1' port 'OUTPUT_0'
   //Consumer kernel 'saturate_nonzero_0' port 'INPUT_0'
   lLmError += LmInit(&gLocalMemArray[6], &lpScenarioBufferData[6], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: kernel 'saturate_nonzero_0' port 'OUTPUT_0'
   //Consumer kernel 'mark_color_channel_0' port 'INPUT_MARKER'
   lLmError += LmInit(&gLocalMemArray[7], &lpScenarioBufferData[7], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: kernel 'mark_color_channel_0' port 'OUTPUT'
   //Consumer: graph port 'Output'
   lLmError += LmInit(&gLocalMemArray[8], &lpScenarioBufferData[8], icp::DATATYPE_08U, 3, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 1, 0);

   //Memory associated with the following ports:
   //Producer: graph port 'Threshold'
   //Consumer kernel 'fast9_0' port 'IN_Thr'
   lLmError += LmInit(&gLocalMemArray[9], &lpScenarioBufferData[9], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 0, 0);

   //Memory associated with the following ports:
   //Producer: graph port 'MarkColorChannel'
   //Consumer kernel 'mark_color_channel_0' port 'INPUT_CHANNEL_INDEX'
   lLmError += LmInit(&gLocalMemArray[10], &lpScenarioBufferData[10], icp::DATATYPE_08U, 1, 1, -1, lPadOffsetR, lPadOffsetRefChunkWidth, lPadOffsetB, lPadOffsetRefChunkHeight, 0, 0);

   if (lLmError != 0)
   {
      lRetVal = ACF_MEMORY_ALLOCATION_ERROR;
      break;//goto EARLY_EXIT;
   }

#if (1 == ACF_SIM_NATIVE)
   SimLmInitAdjustment(gLocalMemArray, 11, lCmemHeapBaseAddr);
#endif

   //initialize global array of kernel_io_desc
   IoDescInit(&gAcfKernelIoDescArray[0], &gLocalMemArray[0], 0); //associated with kernel apu_rgb_to_grayscale port INPUT_0
   IoDescInit(&gAcfKernelIoDescArray[1], &gLocalMemArray[1], 1); //associated with kernel apu_rgb_to_grayscale port OUTPUT_0
   IoDescInit(&gAcfKernelIoDescArray[4], &gLocalMemArray[2], 2); //associated with kernel apu_fast9 port IN_Img
   IoDescInit(&gAcfKernelIoDescArray[5], &gLocalMemArray[9], 9); //associated with kernel apu_fast9 port IN_Thr
   IoDescInit(&gAcfKernelIoDescArray[6], &gLocalMemArray[3], 3); //associated with kernel apu_fast9 port OUT_Img
   IoDescInit(&gAcfKernelIoDescArray[15], &gLocalMemArray[0], 0); //associated with kernel apu_mark_color_channel port INPUT_IMAGE
   IoDescInit(&gAcfKernelIoDescArray[16], &gLocalMemArray[7], 7); //associated with kernel apu_mark_color_channel port INPUT_MARKER
   IoDescInit(&gAcfKernelIoDescArray[17], &gLocalMemArray[10], 10); //associated with kernel apu_mark_color_channel port INPUT_CHANNEL_INDEX
   IoDescInit(&gAcfKernelIoDescArray[18], &gLocalMemArray[8], 8); //associated with kernel apu_mark_color_channel port OUTPUT
   IoDescInit(&gAcfKernelIoDescArray[2], &gLocalMemArray[1], 1); //associated with kernel apu_gauss_3x3 port INPUT_0
   IoDescInit(&gAcfKernelIoDescArray[3], &gLocalMemArray[2], 2); //associated with kernel apu_gauss_3x3 port OUTPUT_0
   IoDescInit(&gAcfKernelIoDescArray[7], &gLocalMemArray[3], 3); //associated with kernel apu_nms port INPUT_0
   IoDescInit(&gAcfKernelIoDescArray[8], &gLocalMemArray[4], 4); //associated with kernel apu_nms port OUTPUT_0
   IoDescInit(&gAcfKernelIoDescArray[9], &gLocalMemArray[4], 4); //associated with kernel apu_dilate_diamond port INPUT_0
   IoDescInit(&gAcfKernelIoDescArray[10], &gLocalMemArray[5], 5); //associated with kernel apu_dilate_diamond port OUTPUT_0
   IoDescInit(&gAcfKernelIoDescArray[11], &gLocalMemArray[5], 5); //associated with kernel apu_dilate_diamond port INPUT_0
   IoDescInit(&gAcfKernelIoDescArray[12], &gLocalMemArray[6], 6); //associated with kernel apu_dilate_diamond port OUTPUT_0
   IoDescInit(&gAcfKernelIoDescArray[13], &gLocalMemArray[6], 6); //associated with kernel apu_saturate_nonzero port INPUT_0
   IoDescInit(&gAcfKernelIoDescArray[14], &gLocalMemArray[7], 7); //associated with kernel apu_saturate_nonzero port OUTPUT_0

   //allocate and initialize linked list buffer arrays (each LL is double buffered)
   int* lInputLLBufArray[2]  = {&gAcfInputLinkedListMem[0],  &gAcfInputLinkedListMem[gInputLinkedListMemSize]};
   int* lOutputLLBufArray[2] = {&gAcfOutputLinkedListMem[0], &gAcfOutputLinkedListMem[gOutputLinkedListMemSize]};

   //allocate and initialize linked list database entries (associates each ll entry with a lm desc and an io desc)
   gInputLinkedListEntryDatabase[0].pIoDesc = &gAcfInputDescArray[0];
   gInputLinkedListEntryDatabase[0].pLmDesc = &gLocalMemArray[0];
   gInputLinkedListEntryDatabase[0].size = LLE_SIZE_2D_DMA_XMEM_CMEM;
   gInputLinkedListEntryDatabase[0].pEntry[0] = lInputLLBufArray[0] + 0*LLE_SIZE_2D_DMA_XMEM_CMEM;
   gInputLinkedListEntryDatabase[0].pEntry[1] = lInputLLBufArray[1] + 0*LLE_SIZE_2D_DMA_XMEM_CMEM;
   gInputLinkedListEntryDatabase[0]._framework_cbindex = 0;

   gOutputLinkedListEntryDatabase[0].pIoDesc = &gAcfOutputDescArray[0];
   gOutputLinkedListEntryDatabase[0].pLmDesc = &gLocalMemArray[8];
   gOutputLinkedListEntryDatabase[0].size = LLE_SIZE_2D_DMA_XMEM_CMEM;
   gOutputLinkedListEntryDatabase[0].pEntry[0] = lOutputLLBufArray[0] + 0*LLE_SIZE_2D_DMA_XMEM_CMEM;
   gOutputLinkedListEntryDatabase[0].pEntry[1] = lOutputLLBufArray[1] + 0*LLE_SIZE_2D_DMA_XMEM_CMEM;
   gOutputLinkedListEntryDatabase[0]._framework_cbindex = lOutputSkip%gLocalMemArray[8].numBuffers;

   //perform DMA transfers associated with STATIC region initialization 
   ACF_XFER_DMEM_XMEM(lApexId, gAcfStaticInputDescArray[0], gLocalMemArray[9], lpResInfo->dmaChIn[0], 1, lClover389Flag); //associated with port Threshold
   ACF_XFER_DMEM_XMEM(lApexId, gAcfStaticInputDescArray[1], gLocalMemArray[10], lpResInfo->dmaChIn[0], 1, lClover389Flag); //associated with port MarkColorChannel

   //-----------------------
   //initialize linked lists
   //-----------------------

   int lLlOffset = 0;

   lLlOffset = 0;
   for (int i=0; i<1; i++)
   {
      ACF_LLE_2DTransferConfig_XMEM_CMEM(
                lInputLLBufArray[0] + lLlOffset,                        //linked list entry address
                gInputLinkedListEntryDatabase[i].pLmDesc->e0,           //e0
                gInputLinkedListEntryDatabase[i].pIoDesc->physAddr,     //source address
                RetStreamInPeriphId(lpResInfo->streamInChForDma[0]),    //destination address (***dummy - will be reconfigured)
                gInputLinkedListEntryDatabase[i].pIoDesc->nd_span.height, //span
                gInputLinkedListEntryDatabase[i].pIoDesc->nd_size.width,//width
                lInputTileWidthInChunks,                                //input tile width in chunks
                gInputLinkedListEntryDatabase[i].pLmDesc->chunkWidth,   //chunk width
                gInputLinkedListEntryDatabase[i].pLmDesc->bufferHeight, //chunk height
                gInputLinkedListEntryDatabase[i].pLmDesc->chunkSpan,    //chunk span
                0xBEEF,                                                 //cmem address (***dummy - will be reconfigured)
                lApuCuOffset);                                          //start CU
      lLlOffset += LLE_SIZE_2D_DMA_XMEM_CMEM;
   }
   memcpy_int(lInputLLBufArray[0], lInputLLBufArray[1], gInputLinkedListMemSize);

   lLlOffset = 0;
   for (int i=0; i<1; i++)
   {
      ACF_LLE_2DTransferConfig_XMEM_CMEM(
                lOutputLLBufArray[0] + lLlOffset,                        //linked list entry address
                gOutputLinkedListEntryDatabase[i].pLmDesc->e0,           //e0
                RetStreamOutPeriphId(lpResInfo->streamOutChForDma[0]),   //source address (***dummy - will be reconfigured)
                gOutputLinkedListEntryDatabase[i].pIoDesc->physAddr,     //destination address
                gOutputLinkedListEntryDatabase[i].pIoDesc->nd_span.height, //span
                gOutputLinkedListEntryDatabase[i].pIoDesc->nd_size.width,//width
                lOutputTileWidthInChunks,                                //output tile width in chunks
                gOutputLinkedListEntryDatabase[i].pLmDesc->chunkWidth,   //chunk width
                gOutputLinkedListEntryDatabase[i].pLmDesc->chunkHeight,  //chunk height
                gOutputLinkedListEntryDatabase[i].pLmDesc->chunkSpan,    //chunk span
                0xBEEF,                                                  //cmem address (***dummy - will be reconfigured)
                lApuCuOffset + lCoreDataCuOffset);                       //start CU
      lLlOffset += LLE_SIZE_2D_DMA_XMEM_CMEM;
   }
   memcpy_int(lOutputLLBufArray[0], lOutputLLBufArray[1], gOutputLinkedListMemSize);

   ConfigureLogicalTransfersDma(lApexId, gNumLogicalTransfersCmemIn, gLogicalTransfersCmemIn, gInputLinkedListEntryDatabase,
                                &lpResInfo->dmaChIn[0], &lpResInfo->streamInChForDma[0], 0);
   ConfigureLogicalTransfersDma(lApexId, gNumLogicalTransfersCmemOut, gLogicalTransfersCmemOut, gOutputLinkedListEntryDatabase,
                                &lpResInfo->dmaChOut[0], &lpResInfo->streamOutChForDma[0], 1);
   //local variables
   int lInputIter = 0;
   int lInputLLBufIndex = 0;
   int lInputTransferArray[1];
   int lInputDmaProcDecFlag = 1;
   int lOutputLLBufIndex = 0;
   int lOutputTransferArray[1];
   int lProcessingProcDecFlag = 1;
   int lProcIter = 0;
   int lLaunchDelay = 0;
   int lOutputDelay = 0;
   int lWriteDelay = 0;
   int lNumIterations = lNumInputIterations + lScenarioIterAdjust;
   int lLastFlag = 0;

   //----------------------------------------------------------
   //initialize sequencer resources
   //----------------------------------------------------------

   int lFixSeqResIdx = 0; //used to move through the lFixedSeqRes array 
   int lSEQ_RESOURCE_DMA_INPUT_DATABUF_AVAIL_DMA = lpResInfo->fixedSeqRes[lFixSeqResIdx++]; //incremented by ACP when processing has completed for an input iteration

   //-----dma + stream in-----
   for (int i=0; i<gNumLogicalTransfersCmemIn; i++)
   {
      gLogicalTransfersCmemIn[i].seqResDescAvailDma = lpResInfo->fixedSeqRes[lFixSeqResIdx++];
      gLogicalTransfersCmemIn[i].seqResBufAvailApu = lpResInfo->fixedSeqRes[lFixSeqResIdx++];
   }

   //-----stream out + dma-----
   for (int i=0; i<gNumLogicalTransfersCmemOut; i++)
   {
      gLogicalTransfersCmemOut[i].seqResDescAvailDma = lpResInfo->fixedSeqRes[lFixSeqResIdx++];
      gLogicalTransfersCmemOut[i].seqResBufAvailApu = lpResInfo->fixedSeqRes[lFixSeqResIdx++];
   }

   if (lFixSeqResIdx > lpResInfo->numFixedSeqRes)
   {
      lRetVal = ACF_HW_RESOURCE_ERROR;
      break;//goto EARLY_EXIT;
   }

   seq_hal_set_rsc_counter(lApexId, lSEQ_RESOURCE_DMA_INPUT_DATABUF_AVAIL_DMA, 2);

   //-----dma + stream in-----
   for (int i=0; i<gNumLogicalTransfersCmemIn; i++)
   {
      seq_hal_set_dma_dec(lApexId, lpResInfo->dmaChIn[i], (1ul << gLogicalTransfersCmemIn[i].seqResDescAvailDma));
      seq_hal_set_dma_inc(lApexId, lpResInfo->dmaChIn[i], (1ul << gLogicalTransfersCmemIn[i].seqResBufAvailApu));
      seq_hal_set_rsc_counter(lApexId, gLogicalTransfersCmemIn[i].seqResDescAvailDma, 0);
      seq_hal_set_rsc_counter(lApexId, gLogicalTransfersCmemIn[i].seqResBufAvailApu,   0);
   }

   //-----stream out + dma-----
   for (int i=0; i<gNumLogicalTransfersCmemOut; i++)
   {
      seq_hal_set_dma_dec(lApexId, lpResInfo->dmaChOut[i], (1ul << gLogicalTransfersCmemOut[i].seqResDescAvailDma));
      seq_hal_set_dma_inc(lApexId, lpResInfo->dmaChOut[i], (1ul << gLogicalTransfersCmemOut[i].seqResBufAvailApu));
      seq_hal_set_rsc_counter(lApexId, gLogicalTransfersCmemOut[i].seqResDescAvailDma, 0);
      seq_hal_set_rsc_counter(lApexId, gLogicalTransfersCmemOut[i].seqResBufAvailApu, 2);
   }

   int lScbMask = 0;
   //-----dma + stream in-----
   for (int i=0; i<gNumLogicalTransfersCmemIn; i++)
   {
      lScbMask |= (1ul << lpResInfo->dmaChIn[i]);
   }
   //-----stream out + dma-----
   for (int i=0; i<gNumLogicalTransfersCmemOut; i++)
   {
      lScbMask |= (1ul << lpResInfo->dmaChOut[i]);
   }
   seq_hal_scb_enable_apu(lApexId, lpResInfo->apuId, lScbMask);

   //ensure interrupts associated with procid lpResInfo->procIntSeqRes[0] (dma in), lpResInfo->procIntSeqRes[1] (mce in), and lpResInfo->procIntSeqRes[2] (processing & dma out)
   //are cleared and unmasked
   int lProcIntMask = ((1<<lpResInfo->procIntSeqRes[0]) | (1<<lpResInfo->procIntSeqRes[1]) | (1<<lpResInfo->procIntSeqRes[2]));
   seq_hal_clear_interrupts(lApexId, lProcIntMask);
   seq_hal_mask_interrupts_apu(lApexId, lpResInfo->apuId, ~lProcIntMask);

   gProbeA = QUERY_PERF_REG; //probe for end of 'initialization' section
   gPerfInfo.init = gProbeA - gProbeStart;

   //----------------------------------------------------------
   //main execution loop
   //----------------------------------------------------------

   while (lProcIter < lNumIterations || lInputIter < lNumIterations)
   {
      //interrupt associated with DMA input
      if ((lInputIter < lNumIterations) && (1 == lInputDmaProcDecFlag))
      {
         ACF_seq_hal_proc_dec_rscmask(lpResInfo->procIntSeqRes[0], 1<<lSEQ_RESOURCE_DMA_INPUT_DATABUF_AVAIL_DMA);
         lInputDmaProcDecFlag = 0;
      }

      //no 'iterative' MCE input ...

      //interrupt associated with processing and output
      if ((lProcIter < lNumIterations) && (1 == lProcessingProcDecFlag))
      {
         int lProcDecMask = 0;
         for (int i=0; i<gNumLogicalTransfersCmemIn; i++)
            lProcDecMask |= (1<<(gLogicalTransfersCmemIn[i].seqResBufAvailApu));
         for (int i=0; i<gNumLogicalTransfersCmemOut; i++)
            lProcDecMask |= (1<<(gLogicalTransfersCmemOut[i].seqResBufAvailApu));
         ACF_seq_hal_proc_dec_rscmask(lpResInfo->procIntSeqRes[2], lProcDecMask);
         lProcessingProcDecFlag = 0;
      }

      gProbeA = QUERY_PERF_REG; //probe for idle start

      ACF_WAIT(32768); //this will 'wait' until an interrupt occurs (or the timer runs out)
#if ((1 == ACF_SIM_NATIVE) || (1 == ACF_SIM_ISS))
      AdvanceSimulation(lApexId);
#endif

      gProbeB = QUERY_PERF_REG; //probe for idle end
      gPerfInfo.idle += gProbeB - gProbeA;

      uint32_t isrval = ACF_seq_hal_query_interrupts() & lProcIntMask; //capture state of relevant interrupts

      if (isrval) //if any of the relevant interrupts have occurred
      {
         ACF_seq_hal_clear_interrupts(isrval); //clear the interrupts that occurred

         //**********************************************
         //input (DMA)
         //**********************************************

         if (isrval & (0x1<<lpResInfo->procIntSeqRes[0])) //input DMA
         {
            lInputDmaProcDecFlag = 1;

            //determine input DMA transfers that will take place during iteration 'lInputIter'
            lLaunchDelay = lpScenarioBufferData[0].writeDelay; //input dma launch delay
            lInputTransferArray[0] = (lInputIter >= lLaunchDelay) && (lInputIter < (lLaunchDelay + lNumInputIterations)) ? (lInputIter - lLaunchDelay) : -1; //input transfer associated with port 'Input'
            lLastFlag = (lInputIter == lNumIterations-1) ? 1 : 0;

            //Generate DMA input linked list
            int* lpLinkedList = lInputLLBufArray[lInputLLBufIndex];

            ACF_AppendLLE_XMEM_CMEM_IN(lpLinkedList, lInputTransferArray, 1, gInputLinkedListEntryDatabase, lNumInputIterations, lLastInputTileWidthInChunks_tmp, LLE_SIZE_2D_DMA_XMEM_CMEM);
            lpLinkedList += 24;

            for (int i=0; i<gNumLogicalTransfersCmemIn; i++)
            {
               ACF_LleSetLast(gLogicalTransfersCmemIn[i].lastEntryPtr[lInputLLBufIndex], lLastFlag);
               ACF_seq_hal_proc_inc_rscmask(lpResInfo->procIdSeqRes, 1<<gLogicalTransfersCmemIn[i].seqResDescAvailDma); //kick transfer
            }

            lInputLLBufIndex = ((0 == lInputLLBufIndex) ? 1 : 0); //toggle 
            lInputIter ++;
            gInputIter = lInputIter; //for debugging
         }

         //**********************************************
         //processing and output
         //**********************************************

         if (isrval & (0x1<<lpResInfo->procIntSeqRes[2])) //processing and output
         {
            lProcessingProcDecFlag = 1;

            //<processing> -------------------------------------------------------------------

            gProbeA = QUERY_PERF_REG; //probe for processing start

            //apu_rgb_to_grayscale
            lLaunchDelay = lpScenarioKernelData[0].executionStartDelay;
            lOutputDelay = lpScenarioKernelData[0].extraIterToSatisfyOd;
            if (lProcIter >= lLaunchDelay && lProcIter < (lNumInputIterations + lLaunchDelay + lOutputDelay))
            {
               int16_t lLocalIter = lProcIter - lLaunchDelay;
               ACF_UpdateTileFlags(lLocalIter, lNumInputIterations);
               apu_rgb_to_grayscale(
                  *_RETIODESC_VEC08(&gLocalMemArray[0], &gAcfKernelIoDescArray[0]),
                  *_RETIODESC_VEC08(&gLocalMemArray[1], &gAcfKernelIoDescArray[1]));
            }

            //padding and circular buffer emulation for gLocalMemArray[1]
            lWriteDelay = lpScenarioBufferData[1].writeDelay;
            if (lProcIter >= lWriteDelay && lProcIter < (lWriteDelay + lNumInputIterations))
            {
               ACF_PAD_08_e0xis1_LequaltoR(&gLocalMemArray[1], gCbIdxArray_pad[1], lProcIter - lWriteDelay, lNumInputIterations, lInputTileWidthInChunks, 1, 1, 2, 2, 0, gLocalMemArray[1].padOffsetB, 0, gLocalMemArray[1].padOffsetR, gsACF_CU_ARRAY_WIDTH);
               ACF_COPYBUF(&gLocalMemArray[1], gCbIdxArray_pad[1], lProcIter - lWriteDelay, lNumInputIterations, 1, 1, 2);
               CbIndexIncrement(&gCbIdxArray_pad[1], gLocalMemArray[1].numBuffers);
            }

            //apu_gauss_3x3
            lLaunchDelay = lpScenarioKernelData[1].executionStartDelay;
            lOutputDelay = lpScenarioKernelData[1].extraIterToSatisfyOd;
            if (lProcIter >= lLaunchDelay && lProcIter < (lNumInputIterations + lLaunchDelay + lOutputDelay))
            {
               int16_t lLocalIter = lProcIter - lLaunchDelay;
               ACF_UpdateTileFlags(lLocalIter, lNumInputIterations);
               apu_gauss_3x3(
                  *_RETIODESC_VEC08(&gLocalMemArray[1], &gAcfKernelIoDescArray[2]),
                  *_RETIODESC_VEC08(&gLocalMemArray[2], &gAcfKernelIoDescArray[3]));
            }

            //padding and circular buffer emulation for gLocalMemArray[2]
            lWriteDelay = lpScenarioBufferData[2].writeDelay;
            if (lProcIter >= lWriteDelay && lProcIter < (lWriteDelay + lNumInputIterations))
            {
               ACF_PAD_08_e0xis1_LequaltoR(&gLocalMemArray[2], gCbIdxArray_pad[2], lProcIter - lWriteDelay, lNumInputIterations, lInputTileWidthInChunks, 3, 3, 4, 4, 0, gLocalMemArray[2].padOffsetB, 0, gLocalMemArray[2].padOffsetR, gsACF_CU_ARRAY_WIDTH);
               ACF_COPYBUF(&gLocalMemArray[2], gCbIdxArray_pad[2], lProcIter - lWriteDelay, lNumInputIterations, 3, 3, 4);
               CbIndexIncrement(&gCbIdxArray_pad[2], gLocalMemArray[2].numBuffers);
            }

            //apu_fast9
            lLaunchDelay = lpScenarioKernelData[2].executionStartDelay;
            lOutputDelay = lpScenarioKernelData[2].extraIterToSatisfyOd;
            if (lProcIter >= lLaunchDelay && lProcIter < (lNumInputIterations + lLaunchDelay + lOutputDelay))
            {
               int16_t lLocalIter = lProcIter - lLaunchDelay;
               ACF_UpdateTileFlags(lLocalIter, lNumInputIterations);
               apu_fast9(
                  *_RETIODESC_VEC08(&gLocalMemArray[2], &gAcfKernelIoDescArray[4]),
                  *_RETIODESC_VEC08(&gLocalMemArray[3], &gAcfKernelIoDescArray[6]),
                  *_RETIODESC_SCL(&gLocalMemArray[9], &gAcfKernelIoDescArray[5]));
            }

            //padding and circular buffer emulation for gLocalMemArray[3]
            lWriteDelay = lpScenarioBufferData[3].writeDelay;
            if (lProcIter >= lWriteDelay && lProcIter < (lWriteDelay + lNumInputIterations))
            {
               ACF_PAD_08_e0xis1_LequaltoR(&gLocalMemArray[3], gCbIdxArray_pad[3], lProcIter - lWriteDelay, lNumInputIterations, lInputTileWidthInChunks, 1, 1, 2, 2, 0, gLocalMemArray[3].padOffsetB, 0, gLocalMemArray[3].padOffsetR, gsACF_CU_ARRAY_WIDTH);
               ACF_COPYBUF(&gLocalMemArray[3], gCbIdxArray_pad[3], lProcIter - lWriteDelay, lNumInputIterations, 1, 1, 2);
               CbIndexIncrement(&gCbIdxArray_pad[3], gLocalMemArray[3].numBuffers);
            }

            //apu_nms
            lLaunchDelay = lpScenarioKernelData[3].executionStartDelay;
            lOutputDelay = lpScenarioKernelData[3].extraIterToSatisfyOd;
            if (lProcIter >= lLaunchDelay && lProcIter < (lNumInputIterations + lLaunchDelay + lOutputDelay))
            {
               int16_t lLocalIter = lProcIter - lLaunchDelay;
               ACF_UpdateTileFlags(lLocalIter, lNumInputIterations);
               apu_nms(
                  *_RETIODESC_VEC08(&gLocalMemArray[3], &gAcfKernelIoDescArray[7]),
                  *_RETIODESC_VEC08(&gLocalMemArray[4], &gAcfKernelIoDescArray[8]));
            }

            //padding and circular buffer emulation for gLocalMemArray[4]
            lWriteDelay = lpScenarioBufferData[4].writeDelay;
            if (lProcIter >= lWriteDelay && lProcIter < (lWriteDelay + lNumInputIterations))
            {
               ACF_PAD_08_e0xis1_LequaltoR(&gLocalMemArray[4], gCbIdxArray_pad[4], lProcIter - lWriteDelay, lNumInputIterations, lInputTileWidthInChunks, 2, 2, 2, 2, 0, gLocalMemArray[4].padOffsetB, 0, gLocalMemArray[4].padOffsetR, gsACF_CU_ARRAY_WIDTH);
               ACF_COPYBUF(&gLocalMemArray[4], gCbIdxArray_pad[4], lProcIter - lWriteDelay, lNumInputIterations, 2, 2, 2);
               CbIndexIncrement(&gCbIdxArray_pad[4], gLocalMemArray[4].numBuffers);
            }

            //apu_dilate_diamond
            lLaunchDelay = lpScenarioKernelData[4].executionStartDelay;
            lOutputDelay = lpScenarioKernelData[4].extraIterToSatisfyOd;
            if (lProcIter >= lLaunchDelay && lProcIter < (lNumInputIterations + lLaunchDelay + lOutputDelay))
            {
               int16_t lLocalIter = lProcIter - lLaunchDelay;
               ACF_UpdateTileFlags(lLocalIter, lNumInputIterations);
               apu_dilate_diamond(
                  *_RETIODESC_VEC08(&gLocalMemArray[4], &gAcfKernelIoDescArray[9]),
                  *_RETIODESC_VEC08(&gLocalMemArray[5], &gAcfKernelIoDescArray[10]));
            }

            //padding and circular buffer emulation for gLocalMemArray[5]
            lWriteDelay = lpScenarioBufferData[5].writeDelay;
            if (lProcIter >= lWriteDelay && lProcIter < (lWriteDelay + lNumInputIterations))
            {
               ACF_PAD_08_e0xis1_LequaltoR(&gLocalMemArray[5], gCbIdxArray_pad[5], lProcIter - lWriteDelay, lNumInputIterations, lInputTileWidthInChunks, 2, 2, 2, 2, 0, gLocalMemArray[5].padOffsetB, 0, gLocalMemArray[5].padOffsetR, gsACF_CU_ARRAY_WIDTH);
               ACF_COPYBUF(&gLocalMemArray[5], gCbIdxArray_pad[5], lProcIter - lWriteDelay, lNumInputIterations, 2, 2, 2);
               CbIndexIncrement(&gCbIdxArray_pad[5], gLocalMemArray[5].numBuffers);
            }

            //apu_dilate_diamond
            lLaunchDelay = lpScenarioKernelData[5].executionStartDelay;
            lOutputDelay = lpScenarioKernelData[5].extraIterToSatisfyOd;
            if (lProcIter >= lLaunchDelay && lProcIter < (lNumInputIterations + lLaunchDelay + lOutputDelay))
            {
               int16_t lLocalIter = lProcIter - lLaunchDelay;
               ACF_UpdateTileFlags(lLocalIter, lNumInputIterations);
               apu_dilate_diamond(
                  *_RETIODESC_VEC08(&gLocalMemArray[5], &gAcfKernelIoDescArray[11]),
                  *_RETIODESC_VEC08(&gLocalMemArray[6], &gAcfKernelIoDescArray[12]));
            }

            //apu_saturate_nonzero
            lLaunchDelay = lpScenarioKernelData[6].executionStartDelay;
            lOutputDelay = lpScenarioKernelData[6].extraIterToSatisfyOd;
            if (lProcIter >= lLaunchDelay && lProcIter < (lNumInputIterations + lLaunchDelay + lOutputDelay))
            {
               int16_t lLocalIter = lProcIter - lLaunchDelay;
               ACF_UpdateTileFlags(lLocalIter, lNumInputIterations);
               apu_saturate_nonzero(
                  *_RETIODESC_VEC08(&gLocalMemArray[6], &gAcfKernelIoDescArray[13]),
                  *_RETIODESC_VEC08(&gLocalMemArray[7], &gAcfKernelIoDescArray[14]));
            }

            //apu_mark_color_channel
            lLaunchDelay = lpScenarioKernelData[7].executionStartDelay;
            lOutputDelay = lpScenarioKernelData[7].extraIterToSatisfyOd;
            if (lProcIter >= lLaunchDelay && lProcIter < (lNumInputIterations + lLaunchDelay + lOutputDelay))
            {
               int16_t lLocalIter = lProcIter - lLaunchDelay;
               ACF_UpdateTileFlags(lLocalIter, lNumInputIterations);
               apu_mark_color_channel(
                  *_RETIODESC_VEC08(&gLocalMemArray[0], &gAcfKernelIoDescArray[15]),
                  *_RETIODESC_VEC08(&gLocalMemArray[7], &gAcfKernelIoDescArray[16]),
                  *_RETIODESC_SCL(&gLocalMemArray[10], &gAcfKernelIoDescArray[17]),
                  *_RETIODESC_VEC08(&gLocalMemArray[8], &gAcfKernelIoDescArray[18]));
            }

            gProbeB = QUERY_PERF_REG; //probe for processing end
            gPerfInfo.processing += gProbeB - gProbeA;

            //</processing> ------------------------------------------------------------------

            ACF_seq_hal_proc_inc_rscmask(lpResInfo->procIdSeqRes, 1<<lSEQ_RESOURCE_DMA_INPUT_DATABUF_AVAIL_DMA);

            //<output> -----------------------------------------------------------------------

            //setup and kick output transfer associated with iteration 'lProcIter'; if there isn't one, increment resources to keep the loop going

            //determine output transfers that should be scheduled during iteration 'lProcIter' (if any)
            lLaunchDelay = lpScenarioBufferData[8].writeDelay; //dma launch delay
            lOutputTransferArray[0] = (lProcIter >= (lLaunchDelay + lOutputSkip) && lProcIter < (lLaunchDelay + lOutputSkip + lNumOutputIterations)) ? (lProcIter - lLaunchDelay) : -1; //output transfer associated with port 'Output'
            lLastFlag = (lProcIter == lNumIterations-1) ? 1 : 0;

            //Generate DMA output linked list
            int* lpLinkedList = lOutputLLBufArray[lOutputLLBufIndex];

            ACF_AppendLLE_XMEM_CMEM_OUT(lpLinkedList, lOutputTransferArray, 1, gOutputLinkedListEntryDatabase, -lOutputSkip, lNumOutputIterations, lLastOutputTileWidthInChunks_tmp, LLE_SIZE_2D_DMA_XMEM_CMEM);
            lpLinkedList += 24;

            for (int i=0; i<gNumLogicalTransfersCmemOut; i++)
            {
               ACF_LleSetLast(gLogicalTransfersCmemOut[i].lastEntryPtr[lOutputLLBufIndex], lLastFlag);
               ACF_seq_hal_proc_inc_rscmask(lpResInfo->procIdSeqRes, 1<<gLogicalTransfersCmemOut[i].seqResDescAvailDma); //kick transfer
            }

            lOutputLLBufIndex = ((0 == lOutputLLBufIndex) ? 1 : 0); //toggle 


            //</output> ----------------------------------------------------------------------

            lProcIter ++;
            gProcIter = lProcIter; //for debugging
         }
      }
   }

   //----------------------------------------------------------
   //wait for output transfers to complete and clean up
   //----------------------------------------------------------

   //wait for all output transfers to complete
   int lProcDecMask = 0;
   for (int i=0; i<gNumLogicalTransfersCmemOut; i++)
      lProcDecMask |= (1<<(gLogicalTransfersCmemOut[i].seqResBufAvailApu));
   ACF_seq_hal_proc_dec_rscmask(lpResInfo->procIntSeqRes[0], lProcDecMask);
   while(seq_hal_query_interrupt(lApexId, lpResInfo->procIntSeqRes[0]) == 0)
   {
      gProbeA = QUERY_PERF_REG; //probe for idle start

      ACF_WAIT(32768);
#if ((1 == ACF_SIM_NATIVE) || (1 == ACF_SIM_ISS))
      AdvanceSimulation(lApexId);
#endif

      gProbeB = QUERY_PERF_REG; //probe for idle end
      gPerfInfo.idle += gProbeB - gProbeA;
   }
   seq_hal_clear_interrupt(lApexId, lpResInfo->procIntSeqRes[0]);
   ACF_seq_hal_proc_dec_rscmask(lpResInfo->procIntSeqRes[0], lProcDecMask);
   while(seq_hal_query_interrupt(lApexId, lpResInfo->procIntSeqRes[0]) == 0)
   {
      gProbeA = QUERY_PERF_REG; //probe for idle start

      ACF_WAIT(32768);
#if ((1 == ACF_SIM_NATIVE) || (1 == ACF_SIM_ISS))
      AdvanceSimulation(lApexId);
#endif

      gProbeB = QUERY_PERF_REG; //probe for idle end
      gPerfInfo.idle += gProbeB - gProbeA;
   }
   seq_hal_clear_interrupt(lApexId, lpResInfo->procIntSeqRes[0]);

   seq_hal_scb_disable_apu(lApexId, lpResInfo->apuId, lScbMask);

   for (int i=0; i<gNumLogicalTransfersCmemIn; i++)
   {
      mcdma_hal_interruptclear_ch_done(lApexId, lpResInfo->dmaChIn[i]);
      stio_hal_interruptclear_sti_done(lApexId, lpResInfo->streamInChForDma[i]);
   }
   for (int i=0; i<gNumLogicalTransfersCmemOut; i++)
   {
      mcdma_hal_interruptclear_ch_done(lApexId, lpResInfo->dmaChOut[i]);
      stio_hal_interruptclear_sto_done(lApexId, lpResInfo->streamOutChForDma[i]);
   }
   seq_hal_mask_interrupts_apu(lApexId, lpResInfo->apuId, lProcIntMask);


   }
   while(false);

   //signal interrupt to indicate to host that the program has completed
   ACF_seq_hal_proc_inc_rscmask(lpResInfo->procIdSeqRes, 1<<lpResInfo->zeroSeqRes_Done);
   seq_hal_disable_seq_apu(lApexId, lpResInfo->apuId, lpResInfo->procIdSeqRes);

   gProbeA = QUERY_PERF_REG; //probe for total
   gPerfInfo.total = gProbeA - gProbeStart;

   return lRetVal;
}

#if (1 == ACF_SIM_NATIVE)
//create a wrapper for acf native sim use (otherwise function pointer name clashes with class name)
int FAST9COLOR_wrapper(void* lpParams)
{
   return FAST9COLOR((int* const)lpParams, 0); //number of params is not used
}
#endif

