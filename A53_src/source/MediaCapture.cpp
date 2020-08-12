//
// Created by wangxiaobao on 18-8-23.
//

#include "../include/MediaCapture.h"
#include "../include/LDWS.h"
#include "../include/Notify.h"
#include "../include/ShowImage.h"
#include "../include/FCW.h"
#include "../include/config.h"
#include "../include/Adapter.h"
#include "../include/Predict.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>

#include<FAST9COLOR.hpp>
#include<umat.hpp>
//#include<sumat.hpp>
#include<string>

//#include "NXPAdapter.h"
#include "frame_output_v234fb.h"
#ifdef APEX2_EMULATE
#include "apu_lib.hpp"
#include "apu_extras.hpp"
#include "acf_lib.hpp"
using namespace APEX2;
#else
#include "apex.h"
#endif

#include "common_time_measure.h"
#include "fast9color_process_type.h"
#include "apu_fast9color_process_controller.hpp"
#include "fast9color_graph_names.h"
using namespace cv;
using namespace std;
#define THREAD_IS_STOPED 0
#define THREAD_IS_RUN    1

#define WIDTH 256
#define HEIGHT 256

MediaCapture * MediaCapture::inst = NULL;

cv::Mat MediaCapture::processMatLdws;
cv::Mat MediaCapture::processMatTsr;
cv::Mat MediaCapture::processMatFcw;

/**
 * LDWS的处理线程
 * @param args
 * @return
 */
void * MediaCapture::LdwsThread(void* args) {
	if (!Config::instance()->isSyncRun) {
		pthread_detach(pthread_self());
	}

	MediaCapture *mediaCapture = (MediaCapture *) args;

	//cout << "L0000" << endl;
	try {
		mediaCapture->getLdwsInstance()->process(processMatLdws);
	} catch (...) {
	}

	mediaCapture->ldwsProcessed(*mediaCapture->getLdwsInstance());

	//cout << "L1" << endl;
	if (!Config::instance()->isSyncRun) {
		pthread_exit(0);
	}

	return NULL;
}

/**
 * TSR的处理线程
 * @param args
 * @return
 */
void *MediaCapture::TsrThread(void *args) {
	if (!Config::instance()->isSyncRun) {
		pthread_detach(pthread_self());
	}
	MediaCapture *mediaCapture = (MediaCapture *) args;

	TSR tsr(mediaCapture, processMatTsr);
	try {
		tsr.process();
	} catch (...) {
	}

	mediaCapture->tsrProcessed(tsr);
	if (!Config::instance()->isSyncRun) {
		pthread_exit(0);
	}

	return NULL;
}

/**
 * TSR的处理线程
 * @param args
 * @return
 */
void *MediaCapture::FcwThread(void *args) {
	if (!Config::instance()->isSyncRun) {
		pthread_detach(pthread_self());
	}
	MediaCapture *mediaCapture = (MediaCapture *) args;

	//cout << "F0" << endl;
	FCW fcw(mediaCapture, processMatFcw);
	try {
		fcw.process();
	} catch (...) {
	}

	mediaCapture->fcwProcessed(fcw);
	//cout << "F1" << endl;
	if (!Config::instance()->isSyncRun) {
		pthread_exit(0);
	}
	return NULL;
}

MediaCapture * MediaCapture::instance() {
	return MediaCapture::inst;
}

LDWS* MediaCapture::getLdwsInstance() {
	if (NULL == pmLdws) {
		pmLdws = new LDWS(this);
	}
	return pmLdws;
}

MediaCapture::MediaCapture() {
	pmLdws = NULL;
	isLdwsThreadIsRun = THREAD_IS_STOPED;
	isTsrThreadIsRun = THREAD_IS_STOPED;
	isFcwThreadIsRun = THREAD_IS_STOPED;

	isARMLocalCamera = false;

	MediaCapture::inst = this;
}

MediaCapture::~MediaCapture() {
	videoCapture.release();
	processMatLdws.release();
	processMatTsr.release();
	processMatFcw.release();
}

bool MediaCapture::open(int devId) {
#ifdef ARM
	isARMLocalCamera = true;
	this->devId = devId;
	return true;
#endif
	videoCapture.open(devId);
	return videoCapture.isOpened();
}

bool MediaCapture::open(const string &fileName) {
	videoCapture.open(fileName);
	return videoCapture.isOpened();
}

/**
 * 启动LDWS线程
 * @param mat
 */
void MediaCapture::startLdwsThread(cv::Mat&mat) {
	if (!Config::instance()->isLdwEnable)
		return;
	if (Config::instance()->isSyncRun) {
		processMatLdws = mat.clone();
		LdwsThread((void *) this);
		return;
	}
	if (THREAD_IS_STOPED == isLdwsThreadIsRun) {
		isLdwsThreadIsRun = THREAD_IS_RUN;
		if (!processMatLdws.empty()) {
			processMatLdws.release();
		}
		processMatLdws = mat.clone();
		int result = pthread_create(&ldwsThreadId, NULL, LdwsThread,
				(void *) this);
		if (0 != result) {
			cout << "--------------start ldws error:--------------"
					<< (int) result << endl;
			isLdwsThreadIsRun = THREAD_IS_STOPED;
		}
	}
}

/**
 * 启动TSR线程
 * @param mat
 */
void MediaCapture::startTsrThread(cv::Mat&mat) {
	if (!Config::instance()->isTsrEnable)
		return;
	if (Config::instance()->isSyncRun) {
		processMatTsr = mat.clone();
		TsrThread((void *) this);
		return;
	}
	if ( THREAD_IS_STOPED == isTsrThreadIsRun) {
		isTsrThreadIsRun = THREAD_IS_RUN;
		if (!processMatTsr.empty()) {
			processMatTsr.release();
		}
		processMatTsr = mat.clone();
		int result = pthread_create(&tsrThreadId, NULL, TsrThread,
				(void *) this);
		if (result != 0) {
			cout << "--------------start tsr error:--------------"
					<< (int) result << endl;
			isTsrThreadIsRun = THREAD_IS_STOPED;
		}
	}
}

/**
 * 启动FCW线程
 * @param mat
 */
void MediaCapture::startFcwThread(cv::Mat&mat) {
	if (!Config::instance()->isFcwEnable)
		return;
	if (Config::instance()->isSyncRun) {
		processMatFcw = mat.clone();
		FcwThread((void *) this);
		return;
	}
	if ( THREAD_IS_STOPED == isFcwThreadIsRun) {
		isFcwThreadIsRun = THREAD_IS_RUN;
		if (!processMatFcw.empty()) {
			processMatFcw.release();
		}
		processMatFcw = mat.clone();
		int result = pthread_create(&fcwThreadId, NULL, FcwThread,
				(void *) this);
		if (result != 0) {
			cout << "--------------start Thread Error--------------"
					<< (int) result << endl;
			isFcwThreadIsRun = THREAD_IS_STOPED;
		}
	}
}

//APEX 角点检测
void MediaCapture::ApexFast(cv::Mat&mat) {
	cout << ".......fast start......." << endl;
	int lRetVal = 0;
	uint8_t threshold = 50;
	uint8_t markColorChannel = 1;
	FAST9COLOR_PI process;
	vsdk::SUMat dataThreshold;
	vsdk::SUMat dataMarkColorChannel;
	vsdk::SUMat dataOut;
	APEX_Init();

	dataThreshold = vsdk::SUMat(1, 1, VSDK_CV_8UC1);      // threshold parameter
	dataMarkColorChannel = vsdk::SUMat(1, 1, VSDK_CV_8UC1); // mark color channel parameter
	dataOut = vsdk::SUMat(HEIGHT, WIDTH, VSDK_CV_8UC3); // data out buffer

	// Set up the parameters
	dataThreshold.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED).at<uint8_t>(0u) =
			threshold;
	dataMarkColorChannel.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED).at<
			uint8_t>(0u) = markColorChannel;

	lRetVal |= process.Initialize();
	lRetVal |= process.ConnectIO(GR_THRESHOLD_IN, dataThreshold);
	lRetVal |= process.ConnectIO(GR_MARKCOLORCHANNEL_IN, dataMarkColorChannel);
	lRetVal |= process.ConnectIO(GR_OUTPUT_OUT, dataOut);

	// Need to connect because of buffer change
	//输入——>Apex适配  output_umat_fast
	cout << "input row:\n" << mat.rows << "input col:\n" << mat.cols << endl;
	cv::resize(mat, mat, cv::Size(256, 256));
	cout << "inputResize row:\n" << mat.rows << "inputResize col:\n" << mat.cols
			<< endl;
	static cv::UMat imageUmat(cv::USAGE_ALLOCATE_HOST_MEMORY);
	mat.copyTo(imageUmat);
	vsdk::UMat output_umat_fast(imageUmat);
	cout << "inputApex row:" << output_umat_fast.rows << "inputApex col:"
			<< output_umat_fast.cols << endl;
	lRetVal |= process.ConnectIO(GR_INPUT_IN, output_umat_fast);

	int ApuRuntimeStart = FSL_Ticks();

	// execute on APEX
	lRetVal |= process.Start();
	lRetVal |= process.Wait();

	int ApuRuntimeStop = FSL_Ticks();

	printf("Frame %.6f sec.\n",
			(float) FSL_TicksToSeconds(ApuRuntimeStop - ApuRuntimeStart));

	if (0 != lRetVal) {
		printf("Program Ended Error 0x%X [ERROR]\n", lRetVal);
	}

	int widthV = 800;	//800;
	int heightV = 480;	//480;
	uint32_t aFormatV = 0;
	cv::Mat dstUpdate;

	//dataOut-> cv::mat
	vsdk::Mat vMatUpdate = dataOut.getMat(OAL_USAGE_CACHED);
	dstUpdate = ((cv::Mat) vMatUpdate).clone();

	if (dstUpdate.rows != heightV || dstUpdate.cols != widthV) {
		cv::resize(dstUpdate, mat, cv::Size(widthV, heightV));
	}

	static io::FrameOutputV234Fb outputFrame(widthV, heightV,
			io::IO_DATA_DEPTH_08, io::IO_DATA_CH3, aFormatV);
	//outputFrame.PutFrame(dataOut);
#if 1
	static cv::UMat imageUUmat(cv::USAGE_ALLOCATE_HOST_MEMORY);
	mat.copyTo(imageUUmat);
	//vsdk::UMat output_umat(show.getUMat(cv::ACCESS_READ));
	vsdk::UMat output_umat(imageUUmat);
	outputFrame.PutFrame(output_umat);
#endif

}

void MediaCapture::loop() {
	double frameRate = 0.1;
	if ((!isARMLocalCamera) && (!videoCapture.isOpened())) {
		cout << "media is not opened\n";
		return;
	} else {
		long totalFrameNumber = videoCapture.get(CV_CAP_PROP_FRAME_COUNT);
		printf("%s:%ld\n", "视频帧数", totalFrameNumber);
		frameRate = videoCapture.get(CV_CAP_PROP_FPS);
		Config::instance()->videoFps =
				frameRate > 0 ? (int) frameRate : Config::instance()->videoFps;
		frameRate = frameRate > 0 ? frameRate : 0.1;
		printf("%s:%d FPS\n", "帧率", (int) frameRate);
		printf("%s:%.2f秒=%.2f分\n", "播放时长", totalFrameNumber / frameRate,
				totalFrameNumber / frameRate / 60);
		printf("从第%d秒开始播放到第%d秒结束\n", (int) Config::instance()->jumpToSeconds,
				(int) (Config::instance()->jumpEndSeconds > 0 ?
						Config::instance()->jumpEndSeconds :
						totalFrameNumber / frameRate));
		cout << "-----------------------------" << endl;

		videoCapture.set(CV_CAP_PROP_POS_FRAMES,
				Config::instance()->jumpToSeconds * frameRate);
	}

	cv::Mat mat;

	Predict::instance();

	long playCount = 0;
	int playAllCount = Config::instance()->jumpEndSeconds;
	if (playAllCount > 0) {
		playAllCount = (Config::instance()->jumpEndSeconds
				- Config::instance()->jumpToSeconds) * frameRate;
	}
	int saveCount = 0;
	for (;;) {
		if (!isARMLocalCamera) {
			videoCapture >> mat;
		} else {
			Adapter::instance()->capature(this->devId, mat);
		}

		if (mat.empty()) {
			cout << "process done" << endl;
			break;
		}

		if (!isARMLocalCamera && playAllCount > 0 && playCount > playAllCount) {
			cout << "play over" << endl;
			break;
		}

		playCount++;

		if (mat.cols > 1280) {
			cv::resize(mat, mat, cv::Size(1280, 720));
		}

		if (mat.type() == CV_8UC2) {
			//cv::cvtColor( mat, mat, cv::COLOR_YUV2RGB_UYVY );
			cv::cvtColor(mat, mat, cv::COLOR_YUV2BGR_UYVY);
		}

		//startLdwsThread(mat);

		//startTsrThread(mat);

		//startFcwThread(mat);

		//APEX 角点检测
		ApexFast(mat);

		//ShowImage::showImage("capture", mat, 30);
//        if (!Config::instance()->isNoDrawDetectResult) {
//            this->getLdwsInstance()->drawResult(mat);
//
//            FCW::drawResult(mat);
//
//            TSR::drawResult(mat);
//
//            Notify::drawResult(mat);
//
//            if (saveCount++ > 10 && saveCount < 60){
//            	if (saveCount % 10 == 0){
//            		char path[100]={0};
//            		sprintf(path,"./save/%d.jpeg",saveCount);
//            		cv::imwrite(path,mat);
//            	}
//            }
//            if (saveCount > 105){
//            	saveCount = 105;
//            }
//            Adapter::instance()->saveVideo(Config::instance()->saveVideoPath, mat);
//            ShowImage::showImage("capture", mat, 30);
//        }
	}

//    for(int i = 0; i < 15; ++i){
//        if((isLdwsThreadIsRun == THREAD_IS_STOPED || !Config::instance()->isLdwEnable)
//            && (isTsrThreadIsRun == THREAD_IS_STOPED || !Config::instance()->isTsrEnable)
//            && (THREAD_IS_STOPED == isFcwThreadIsRun || !Config::instance()->isFcwEnable)) break;
//        else sleep(2);
//    }
}

/**
 * LDWS处理完成
 * @param ldws
 */
void MediaCapture::ldwsProcessed(LDWS& ldws) {

#if 0
	if (ldws.rightShift) {
		Notify::notify(SHIFT_RIGHT);
	} else if (ldws.leftShift) {
		Notify::notify(SHIFT_LEFT);
	} else {
		isLdwsThreadIsRun = THREAD_IS_STOPED;
		return;
	}
#endif

	mDsm.setShiftState(ldws.isLeftShift, ldws.isRightShift);
	isLdwsThreadIsRun = THREAD_IS_STOPED;
}

/**
 * TSR处理完成
 * @param tsr
 */
void MediaCapture::tsrProcessed(TSR& tsr) {
	isTsrThreadIsRun = THREAD_IS_STOPED;
}

/**
 * FCW处理完成
 * @param fcw
 */
void MediaCapture::fcwProcessed(FCW& fcw) {
	isFcwThreadIsRun = THREAD_IS_STOPED;
}

void MediaCapture::syncShow(const char* title, cv::Mat& mat) {
	if (Config::instance()->isSyncRun) {
		ShowImage::showImage(title, mat);
	}
}
