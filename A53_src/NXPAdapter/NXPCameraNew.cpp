/*
 * NXPCameraNew.cpp
 *
 *  Created on: 2019年12月10日
 *      Author: lilei
 */

#ifndef __STANDALONE__
#include <signal.h>
#endif // #ifdef __STANDALONE__

#include <string.h>

#ifdef __STANDALONE__
#include "frame_output_dcu.h"
#define CHNL_CNT io::IO_DATA_CH2
#else // #ifdef __STANDALONE__
#include "frame_output_v234fb.h"
#define CHNL_CNT io::IO_DATA_CH2
#endif // else from #ifdef __STANDALONE__

//ISP model
#include "oal.h"
#include "vdb_log.h"
#include "sdi.hpp"
#include "isp_camera_test_c.h"

#include "../include/Adapter.h"

// add for openCV
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
//mat
#include <umat_mat.hpp>

//***************************************************************************
// constants
//***************************************************************************

// Possible to set input resolution (must be supported by the DCU)
#define WIDTH           1280 ///< width of DDR buffer in pixels
#define HEIGHT          720 ///< height of DDR buffer in pixels
#define DDR_BUFFER_CNT  3    ///< number of DDR buffers per ISP stream

//***************************************************************************

#define STREAM_CYCLE_FRM_CNT 150 ///< number of frames until stream switche
#define STREAM_CNT             5 ///< total number of availabe camear streams

#define FRM_TIME_MSR 300 ///< number of frames to measure the time and fps

//***************************************************************************
// cameara path
//***************************************************************************
static int curr_cam = 0;

#ifdef __STANDALONE__
//extern SEQ_Buf_t  producerless_buffer_1;
extern "C" {
	unsigned long get_uptime_microS(void);
}

#define GETTIME(time)   (*(time)=get_uptime_microS())
#else // ifdef __STANDALONE__
#define GETTIME(time) \
	  { \
	  struct timeval lTime; gettimeofday(&lTime,0); \
	  *time=(lTime.tv_sec*1000000+lTime.tv_usec);   \
	  }
#endif // else from #ifdef __STANDALONE__

//***************************************************************************
// types
//***************************************************************************
struct AppContext {
	sdi_grabber *mpGrabber;      ///< pointer to grabber instance
	sdi_FdmaIO *mpFdma;         ///< pointer to fdma object

	// ** event counters and flags **
	uint8_t mKbKey;
	uint8_t mKbMode;    // 0=off, 'k'= knee point, r=redgain, b=bluegain
	bool mError;            ///< to signal ISP problems
	uint32_t mFrmDoneCnt;       ///< number of frames done events
	uint32_t mFrmCnt;           ///< number of frames grabbed by the app
};
// struct AppContext

/************************************************************************/
/** User defined call-back function for Sequencer event handling.
 *
 * \param  aEventType defines Sequencer event type
 * \param  apUserVal  pointer to any user defined object
 ************************************************************************/
static void SeqEventCallBack(uint32_t aEventType, void* apUserVal);

/************************************************************************/
/** Prepare everything before executing the main functionality .
 *
 * \param arContext structure capturing the context of the application
 *
 * \return 0 if all ok, <0 otherwise
 ************************************************************************/
static int32_t Prepare(AppContext &arContext);

/************************************************************************/
/** Initial setup of application context.
 *
 * \param arContext structure capturing the context of the application
 ************************************************************************/
static void ContextInit(AppContext &arContext);

/************************************************************************/
/** Prepares required libraries.
 *
 * \param arContext structure capturing the context of the application
 *
 * \return 0 if all ok, != 0 otherwise
 ************************************************************************/
static int32_t LibsPrepare(AppContext &arContext);

/************************************************************************/
/** Prepares DDR buffers.
 *
 * \param arContext structure capturing the context of the application
 *
 * \return 0 if all ok, != 0 otherwise
 ************************************************************************/
static int32_t DdrBuffersPrepare(AppContext &arContext);

/************************************************************************/
/** Execute main functionality of the application.
 *
 * \param arContext structure capturing the context of the application
 *
 * \return 0 if all ok, <0 otherwise
 ************************************************************************/
static int32_t Run(AppContext &arContext);

/************************************************************************/
/** Cleanup all resources before application end.
 *
 * \param arContext structure capturing the context of the application
 *
 * \return 0 if all ok, <0 otherwise
 ************************************************************************/
static int32_t Cleanup(AppContext &arContext);

#ifndef __STANDALONE__
/************************************************************************/
/** SIGINT handler.
 *
 * \param  aSigNo
 ************************************************************************/
static void SigintHandler(int);

/************************************************************************/
/** SIGINT handler.
 *
 * \param  aSigNo
 *
 * \return SEQ_LIB_SUCCESS if all ok
 *         SEQ_LIB_FAILURE if failed
 ************************************************************************/
static int32_t SigintSetup(void);

//***************************************************************************

static bool sStop = false; ///< to signal Ctrl+c from command line

#endif // #ifndef __STANDALONE__

vsdk::SMat gFrameCAM;  // current frame from CAM

void frame_save(int i) {
	FILE *lpSavefile = 0;
	//int j;
	unsigned char *data;
	int stride_byte, lines;
	char path[100] = "data/test", p[12];
	stride_byte = 2560;
	lines = 800;
	sprintf(p, "%d", i);
	strcpy(path, "data/AVM_");
	strcat(path, p);
	lpSavefile = fopen(path, "w+");

	if (!lpSavefile) {
		printf("Error: could not open save file %s\n", lpSavefile);
		return;
	}
	//gFrameCAM=lFrame[lFdmaChannel].mUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	//data = (unsigned char*)gFrameCAM.data;
	OAL_MemoryInvalidate(data, stride_byte * lines);
	fwrite(data, stride_byte, lines, lpSavefile);
	fclose(lpSavefile);
	return;

}

static AppContext lContext;
extern int g_cameraIndex;

int NXPPrepare() {
	if (g_cameraIndex == 9) {
		return 0;
	}

	if (SigintSetup() != SEQ_LIB_SUCCESS) {
		VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
		return -1;
	}

	if (Prepare(lContext) != 0) {
		cout << "NXP Prepare error!" << endl;
		return -1;
	}

	if (lContext.mpGrabber->Start() != LIB_SUCCESS) {
		printf("Failed to start the grabber.\n");
		return -1;
	}

	return 0;
}

int NXPCleanup() {
	Cleanup(lContext);
	return 0;
}

void NXPCapture(int dev, cv::Mat& dst) {
	if (sStop) {
		dst.release();
		return;
	}

	SDI_Frame lpFrame[STREAM_CNT];

	for (int i = 0; i < STREAM_CNT; i++) {
		int j = 0;
		for (; j < 100; ++j) {
			//printf("FramePop:%d begin\n",j);
			lpFrame[i] = lContext.mpGrabber->FramePop(i);
			if (j >= 1) {
				printf("获取视频有延迟：%d 秒\n", j);
				if (j >= 1) {
					cv::Mat image = cv::Mat::zeros(720, 1280, CV_8UC3);

					{
						image = cv::Mat::zeros(720, 1280, CV_8UC3);
						std::string text = "    Unable to capture camera video:"
								+ to_string(j) + std::string("'s    ");
						int font_face = cv::FONT_HERSHEY_COMPLEX;
						double font_scale = 2;
						int thickness = 2;
						int baseline;
						//获取文本框的长宽
						cv::Size text_size = cv::getTextSize(text, font_face,
								font_scale, thickness, &baseline);

						//将文本框居中绘制
						cv::Point origin;
						origin.x = image.cols / 2 - text_size.width / 2;
						origin.y = image.rows / 2 + text_size.height / 2;
						cv::putText(image, text, origin, font_face, font_scale,
								cv::Scalar(0, 255, 255), thickness, 6, 0);
					}
					Adapter::instance()->showImage("dd", image, 0);
				}
			}

			if (!lpFrame[i].mUMat.empty()) {
				break;
			} // if pop failed

		} // second for

		if (lpFrame[i].mUMat.empty()) {
			printf("Failed to grab image number %u\n", lContext.mFrmCnt);
			lContext.mError = true;
			dst.release();
			return; //temp
		} // if pop failed
	} // for all channels

#if 0
	dev = (dev < 0 || dev >3) ? 0 : dev;
	vsdk::Mat vMat = lpFrame[dev].mUMat.getMat(OAL_USAGE_CACHED);
	dst = ((cv::Mat)vMat).clone();
#endif

	for (int i = 0; i < STREAM_CNT; i++) {
		if (lContext.mpGrabber->FramePush(lpFrame[i]) != LIB_SUCCESS) {
			printf("Failed to push image number %u\n", lContext.mFrmCnt);
			lContext.mError = true;
			break;
		} // if push failed
	}

	dev = (dev < 0 || dev > 3) ? 0 : dev;
	vsdk::Mat vMat = lpFrame[dev].mUMat.getMat(OAL_USAGE_CACHED);
	cv::Mat flipMat;
	cv::flip((cv::Mat)vMat, flipMat, 1);
	dst = (flipMat.clone());

	return;
}

int main3(int, char **) {
	int lRet = 0;
	// AppContext lContext;
	//*** process command line parameters ***
	//*** process command line parameters ***
	printf(
			"\n**************************************************************\n"
					"** Omnivision Ov10635 quad demo using Maxim Ser/Des HW setup\n"
					"** Description:\n"
					"**  o Maxim 9286 deserializer board with 4xOmnivision Ov10635\n"
					"**    cameras each with 9271 serializer (on MipiCsi_0) expected as\n"
					"**    image input.\n"
					"**  o ISP converts YUV422 10bit piexel data provided by the sensor\n"
					"**    to YUV422 8bit pixels and stores single camera images into\n"
					"**    separate DDR buffers.\n"
					"**  o Resulting YUV 1280x800 image are displayed live using DCU.\n"
					"**    150 frames (5s) for each camera stream are displayed before\n"
					"**    switching to a different stream.\n"
					"**\n"
					"** Usage:\n"
					"**  o no cmd line parameters available.\n"
					"**\n"
					"**************************************************************\n\n");
#ifndef __STANDALONE__
	fflush(stdout);
	sleep(1);
#endif // ifndef __STANDALONE__

	if (Prepare(lContext) == 0) {
		if (Run(lContext) != 0) {
			printf("Demo execution failed.\n");
			lRet = -1;
			return lRet;
		} // if Run() failed
	} // if Prepare() ok
	else {
		printf("Demo failed in preparation phase.\n");
		lRet = -1;
	} // else from if Prepare() ok

	if (Cleanup(lContext) != 0) {
		printf("Demo failed in cleanup phase.\n");
		lRet = -1;
	} // if cleanup failed

	return lRet;
} // main()

static int32_t Prepare(AppContext &arContext) {
	// init app context
	ContextInit(arContext);
	// enable LIBS
	if (LibsPrepare(arContext) != 0) {
		printf("Failed to prepare libraries.\n");
		return -1;
	} // if failed to configure decoder
	  // enable OAL

#ifndef __STANDALONE__
#endif // #ifndef __STANDALONE__

	if (DdrBuffersPrepare(arContext) != 0) {
		printf("Failed to prepare DDR buffers.\n");
		return -1;
	} // if fialed to prepare DDR buffers

	// *** prestart grabber ***
	if (arContext.mpGrabber->PreStart() != LIB_SUCCESS) {
		printf("Failed to prestart the grabber.\n");
		return -1;
	} // if PreStart() failed

	if (arContext.mpGrabber->SeqEventCallBackInstall(&SeqEventCallBack,
			&arContext) != LIB_SUCCESS) {
		printf("Failed to install Sequencer event callback.\n");
		return -1;
	} // if callback setup failed

	return 0;
} // Prepare()

static void ContextInit(AppContext &arContext) {
	arContext.mpGrabber = NULL;
	arContext.mpFdma = NULL;
	arContext.mError = false;
	arContext.mFrmCnt = 0;
	arContext.mFrmDoneCnt = 0;
} // ContextInit()

static int32_t LibsPrepare(AppContext &arContext) {
	// *** Initialize SDI ***
	if (sdi::Initialize(0) != LIB_SUCCESS) {
		printf("Failed to initialzie SDI.\n");
		return -1;
	} // if failed to initialize SDI
	  // create grabber
	arContext.mpGrabber = new (sdi_grabber);
	if (arContext.mpGrabber == NULL) {
		printf("Failed to create sdi grabber.\n");
		return -1;
	} // if failed to create grabber
	if (arContext.mpGrabber->ProcessSet(gpGraph, &gGraphMetadata)
			!= LIB_SUCCESS) {
		printf("Failed to set ISP graph to grabber.\n");
		return -1;
	} // if ISP graph not set
	  // get IOs
	arContext.mpFdma = (sdi_FdmaIO*) arContext.mpGrabber->IoGet(
			SEQ_OTHRIX_FDMA);
	if (arContext.mpFdma == NULL) {
		printf("Failed to get FDMA object.\n");
		return -1;
	} // if no FDMA object
	return 0;
} // LibsPrepare(AppContext &arContext)

static int32_t DdrBuffersPrepare(AppContext &arContext) {
	// *** 4x YUV full frame buffer array ***
	// modify DDR frame geometry to fit display output

	SDI_ImageDescriptor lFrmDesc;
	lFrmDesc = SDI_ImageDescriptor(WIDTH, HEIGHT, YUV422Stream_UYVY);

	if (arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FastDMA_Out, lFrmDesc)
			!= LIB_SUCCESS) {
		printf("Failed to set image descriptor 0.\n");
		return -1;
	} // if frame descriptor setup failed

	if (arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FastDMA_Out1, lFrmDesc)
			!= LIB_SUCCESS) {
		printf("Failed to set image descriptor 1.\n");
		return -1;
	} // if frame descriptor setup failed

	if (arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FastDMA_Out2, lFrmDesc)
			!= LIB_SUCCESS) {
		printf("Failed to set image descriptor 2.\n");
		return -1;
	} // if frame descriptor setup failed

	if (arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FastDMA_Out3, lFrmDesc)
			!= LIB_SUCCESS) {
		printf("Failed to set image descriptor 3.\n");
		return -1;
	} // if frame descriptor setup failed
#if 1
	if (arContext.mpFdma->DdrBufferDescSet(FDMA_IX_FDMA_0, lFrmDesc)
			!= LIB_SUCCESS) {
		printf("Failed to set image descriptor 4.\n");
		return -1;
	} // if frame descriptor setup failed
#endif
	// allocate DDR buffers
	if (arContext.mpFdma->DdrBuffersAlloc(DDR_BUFFER_CNT) != LIB_SUCCESS) {
		printf("Failed to allocate DDR buffers.\n");
		return -1;
	} // if ddr buffers not allocated

	return 0;
} // DdrBuffersPrepare(AppContext &arContext)

static int KeyDown() {
	struct timeval lTv;
	fd_set lFdSet;
	lTv.tv_sec = 0;
	lTv.tv_usec = 50;
	FD_ZERO(&lFdSet);
	FD_SET(STDIN_FILENO, &lFdSet);
	select(STDIN_FILENO + 1, &lFdSet, NULL, NULL, &lTv);
	return FD_ISSET(STDIN_FILENO, &lFdSet);
} // KeyDown()

static int GetCharNonBlock() {
	int lChar = EOF;
	usleep(1);
	if (KeyDown()) {
		lChar = fgetc(stdin);
	} // if Key pressed
	return lChar;
} // KeyDown()

static char GetChar() {
#ifdef __STANDALONE__
	return sci_0_testchar();
#else // #ifdef __STANDALONE__
	int lChar = GetCharNonBlock();
	return (char) ((lChar < 0) ? 0 : lChar & 0xff);
#endif // else from #ifdef __STANDALONE__
} // GetChar()

static void KbAction(AppContext &arContext) {
	uint8_t lKey = arContext.mKbKey;
	if (lKey == 'h') {   // help
		printf("keyboard help:\n"
				"h .........help\n"
				"D .........toggle display on off\n"
				"p .........change logging print mode\n"
				"i .........enter IIC command mode\n"
				"  aaaavv ..enter IIC command to 0xaaaa with value 0xvv\n"
				"  ret .....write to IIC\n"
				"  r .......read from IIC\n"
				"< .........toggle bit shift\n"
				"k .........select exposure mode (0/1/2/3)\n"
				"  0 .......12bit combined\n"
				"  1 .......very short\n"
				"  2 .......short\n"
				"  3 .......long\n"
				"x .........enter display xoffset mode (+/-)\n"
				"E .........toggle endian swap\n"
				"O .........toggle output lut\n"
				"g .........toggle channel gain\n"

		);
		return;
	} // if help requested

	if (lKey == 'k') {
		printf("Mode = exposure mode selection (0/1/2/3/4)\n");
		arContext.mKbMode = 'k';
		return;
	} // if k pressed

	if (arContext.mKbMode == 'k') {
		if (lKey == '0') { // show combined
			// MAXIM_CAM_RegWrite(CSI_IDX_CAM, 0x3119, 0x04);
			curr_cam = 0;
			printf("select camera 0 \n");
			return;
		} // if key 0

		if (lKey == '1') {
			//MAXIM_CAM_RegWrite(CSI_IDX_CAM,0x3119, 0x47);
			curr_cam = 1;
			printf("select camera  1 \n");

			return;
		} // if key 1
		if (lKey == '2') {
			//MAXIM_CAM_RegWrite(CSI_IDX_CAM,0x3119, 0x46);
			curr_cam = 2;
			printf("select camera  2 \n");
			return;
		} // if key 2
		if (lKey == '3') {
			//MAXIM_CAM_RegWrite(CSI_IDX_CAM,0x3119, 0x45);
			curr_cam = 3;
			printf("select camera  3 \n");
			return;
		} // // if key 3
		if (lKey == '4') {
			//MAXIM_CAM_RegWrite(CSI_IDX_CAM,0x3119, 0x45);
			curr_cam = 4;
			printf("select camera  4 \n");
			return;
		} // // if key 3

		//arContext.mKbI2c=0;
	} // if mode k
}

void KeyboardInputProcess(AppContext &arContext) {
	// *** get keyboard input ***
	arContext.mKbKey = GetChar();
	//printf("char %c\n", gKbKey);
	if (arContext.mKbKey) {
		KbAction(arContext);
	}
	arContext.mKbKey = 0;
	// output frame
	// arContext.mDcu.PutFrame(arContext.mDisplayBuffer);
} // if display enabled

static int32_t Run(AppContext &arContext) {
//*** Init DCU Output ***
#ifdef __STANDALONE__
	io::FrameOutputDCU lDcuOutput(
			WIDTH,
			HEIGHT,
			io::IO_DATA_DEPTH_08,
			CHNL_CNT,
			DCU_BPP_YCbCr422);
#else // #ifdef __STANDALONE__
	// setup Ctrl+C handler
	if (SigintSetup() != SEQ_LIB_SUCCESS) {
		VDB_LOG_ERROR("Failed to register Ctrl+C signal handler.");
		return -1;
	}

	printf("Press Ctrl+C to terminate the demo.\n");
	io::FrameOutputV234Fb lDcuOutput(
	WIDTH,
	HEIGHT, io::IO_DATA_DEPTH_08, io::IO_DATA_CH3, DCU_BPP_YCbCr422);
#endif // else from #ifdef __STANDALONE__
	unsigned long lTimeStart = 0, lTimeEnd = 0, lTimeDiff = 0;
	// *** start grabbing ***
	GETTIME(&lTimeStart);
	if (arContext.mpGrabber->Start() != LIB_SUCCESS) {
		printf("Failed to start the grabber.\n");
		return -1;
	} // if Start() failed

	uint32_t lActiveStreamIndex = 0;
	int write_count = 0;
	SDI_Frame lpFrame[STREAM_CNT];
	// *** grabbing/processing loop ***
	for (;;) {
		// pop all
		for (int i = 0; i < STREAM_CNT; i++) {
			lpFrame[i] = arContext.mpGrabber->FramePop(i);
			if (lpFrame[i].mUMat.empty()) {
				printf("[Frame %d] Failed to grab image number %u\n", i,
						arContext.mFrmCnt);
				arContext.mError = true;
				return -1; //temp
			} // if pop failed
			  // printf("[Frame %d] grab image number %u\n", i, arContext.mFrmCnt);

#ifdef CAPTURE_OUTBUF

			if(write_count<308 && write_count >=300)
			{
				printf("framcount= %u \n",arContext.mFrmCnt);
				gFrameCAM=lpFrame[i].mUMat.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
				//frame_save((write_count-300)*4+i);
			}
#endif

		} // for all channels

		write_count++;
		// *** act on keyboard input and display ***
		KeyboardInputProcess(arContext);
		if (((++arContext.mFrmCnt) % STREAM_CYCLE_FRM_CNT) == 0) {
			++lActiveStreamIndex;
			lActiveStreamIndex = lActiveStreamIndex % STREAM_CNT;
		} // if stream to be switched

		// lDcuOutput.PutFrame(lpFrame[lActiveStreamIndex].mUMat);
		//lDcuOutput.PutFrame(lpFrame[0].mUMat);

		if (curr_cam >= STREAM_CNT)
			lDcuOutput.PutFrame(lpFrame[0].mUMat);
		else
			lDcuOutput.PutFrame(lpFrame[curr_cam].mUMat);
		for (int i = 0; i < STREAM_CNT; i++) {

			if (arContext.mpGrabber->FramePush(lpFrame[i]) != LIB_SUCCESS) {
				printf("Failed to push image number %u\n", arContext.mFrmCnt);
				arContext.mError = true;
				break;
			} // if push failed
		}

		if ((arContext.mFrmCnt % FRM_TIME_MSR) == 0) {
			GETTIME(&lTimeEnd);
			lTimeDiff = lTimeEnd - lTimeStart;
			lTimeStart = lTimeEnd;

			/*printf("%u frames took %lu usec (%5.2ffps)\n",
			 FRM_TIME_MSR,
			 lTimeDiff,
			 (FRM_TIME_MSR*1000000.0)/((float)lTimeDiff));*/
		} // if time should be measured

#ifndef __STANDALONE__
		if (sStop) {
			break; // break if Ctrl+C pressed
		} // if Ctrl+C
#endif //#ifndef __STANDALONE__

	} // for ever
	return 0;
} // Run()

static int32_t Cleanup(AppContext &arContext) {
	int32_t lRet = 0;

	if (arContext.mpGrabber != NULL) {
		if (arContext.mpGrabber->Stop()) {
			printf("Failed to stop the grabber.\n");
			lRet = -1;
		} // if grabber stop failed

		if (arContext.mpGrabber->Release()) {
			printf("Failed to release grabber resources.\n");
			lRet = -1;
		} // if grabber resources not released

		delete (arContext.mpGrabber);
		arContext.mpGrabber = NULL;
	} // if grabber exists

#ifdef __STANDALONE__
	for(;;);  // *** don't return ***
#endif // #ifdef __STANDALONE__

	if (sdi::Close(0) != LIB_SUCCESS) {
		printf("Failed to terminate use of SDI.\n");
		lRet = -1;
	} // if SDI use termination failed

	return lRet;
} // Cleanup()
//***************************************************************************

static void SeqEventCallBack(uint32_t aEventType, void* apUserVal) {
	AppContext *lpAppContext = (AppContext*) apUserVal;

	if (lpAppContext) {
		if (aEventType == SEQ_MSG_TYPE_FRAMEDONE) {
			// printf("Frame done message arrived #%u.\n",
			//    lpAppContext->mFrmDoneCnt++);
		} // if frame done arrived
	} // if user pointer is NULL
} // SeqEventCallBack()

#ifndef __STANDALONE__
static void SigintHandler(int) {
	sStop = true;
} // SigintHandler()

int32_t SigintSetup() {
	static int32_t lRet = SEQ_LIB_SUCCESS;

	// prepare internal signal handler
	struct sigaction lSa;
	memset(&lSa, 0, sizeof(lSa));
	lSa.sa_handler = SigintHandler;

	if (sigaction(SIGINT, &lSa, NULL) != 0) {
		VDB_LOG_ERROR("Failed to register signal handler.\n");
		lRet = SEQ_LIB_FAILURE;
	} // if signal not registered

	return lRet;
} // SigintSetup()

#endif // #ifndef __STANDALONE__

