#ifndef _ACF_PROCESS_APU_FAST9COLOR_NATIVE_KERNEL_WRAPPER
#define _ACF_PROCESS_APU_FAST9COLOR_NATIVE_KERNEL_WRAPPER

#include <FAST9COLOR.hpp>

//structure encapsulating the information ACF requires for each I/O
typedef struct _acf_io_info {                                      
    int32_t      pBuf;   //CL framework will initialize this with a pointer to global memory (based on associated cl_mem entity)
    int32_t      width;  //width in unit of elements
    int32_t      height; //height in unit of elements
    int32_t      span;   //(2D only) the number of bytes required to jump from one line of bytes in memory to the 'next' line of bytes in memory
    icp::DATATYPE elementDataType; //element data type (icp::DATATYPE_08U, icp::DATATYPE_16S, etc.
    int32_t      elementDimX;     //element X dimension
    int32_t      elementDimY;     //element Y dimension
} acf_io_info;

/******************************************************
pArgs must point to an array with the following format:

acf_io_info MYKERNEL_args[4];

where:
   [0] corresponds to input 'Input'
   [1] corresponds to input 'Threshold'
   [2] corresponds to input 'MarkColorChannel'
   [3] corresponds to output 'Output'

and 'pArgs = (void*)MYKERNEL_args;'
******************************************************/

static void FAST9COLOR_nativekernel(void* pArgs)
{
   acf_io_info* lpIo = 0;

   //Input
   ACF_DataDesc Input;
   lpIo = (acf_io_info*)((int32_t)pArgs+0*sizeof(acf_io_info));
   Input.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //Threshold
   ACF_DataDesc Threshold;
   lpIo = (acf_io_info*)((int32_t)pArgs+1*sizeof(acf_io_info));
   Threshold.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //MarkColorChannel
   ACF_DataDesc MarkColorChannel;
   lpIo = (acf_io_info*)((int32_t)pArgs+2*sizeof(acf_io_info));
   MarkColorChannel.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   //Output
   ACF_DataDesc Output;
   lpIo = (acf_io_info*)((int32_t)pArgs+3*sizeof(acf_io_info));
   Output.Init2D(0, (void*)lpIo->pBuf, lpIo->nd_size.width, lpIo->nd_size.height, lpIo->nd_span.height, lpIo->elementDataType, lpIo->elementDimX, lpIo->elementDimY);

   FAST9COLOR lProcess;
   lProcess.Initialize();
   lProcess.ConnectIO("Input", Input);
   lProcess.ConnectIO("Threshold", Threshold);
   lProcess.ConnectIO("MarkColorChannel", MarkColorChannel);
   lProcess.ConnectIO("Output", Output);
   lProcess.Start();
   lProcess.Wait();
}

#endif //_ACF_PROCESS_APU_FAST9COLOR_NATIVE_KERNEL_WRAPPER
