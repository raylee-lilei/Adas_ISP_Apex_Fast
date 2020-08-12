/*
 * SaveVideo.cpp
 *
 *  Created on: 2019年1月29日
 *      Author: wangxiaobao
 */
#include "SaveVideo.h"
#include "utils_h264encoder.h"

using namespace std;

SaveVideo::SaveVideo(std::string savePath) {
	// TODO Auto-generated constructor stub
	mEncodeSaveBuff = NULL;
	mSaveFileHandle = NULL;
	if (savePath.length() > 0){
		mSavePath = savePath;
		cout << "savePath:" << mSavePath <<  endl;
		init();
	}
}

SaveVideo::~SaveVideo() {
	// TODO Auto-generated destructor stub
	if (mSaveFileHandle != NULL){
		fclose(mSaveFileHandle);
	}
}

#define STREAM_WIDTH 1280
#define STREAM_HEIGHT 720
#define STREAM_LINES_NUMBER 16

void onH264encodingDone(unsigned char *result, int final_length, long encoding_time_us)
{
   printf("Encode Frame Done : %lu us\n", encoding_time_us);
}
void SaveVideo::init(){

	if (mSavePath.empty()){
		return;
	}

	//mSaveFileHandle = fopen("myh264.h264","wb");
	mSaveFileHandle = fopen(mSavePath.c_str(),"wb");
	if (mSaveFileHandle == NULL){
		cout << "mSaveFileHandle == NULL \n";
	}

    mEncodeSaveBuff = (unsigned char *)malloc(STREAM_WIDTH* STREAM_HEIGHT * 3);

    H264Encoder::H264Encoder_setup(STREAM_WIDTH, STREAM_HEIGHT, STREAM_LINES_NUMBER, YUV420p/*RGB*/);
    //H264Encoder::H264Encoder_setup(STREAM_WIDTH, STREAM_HEIGHT, STREAM_LINES_NUMBER, RGB);
    H264Encoder::getInstance()->Initialize();
    H264Encoder::getInstance()->setOnEncoderDoneListener(&onH264encodingDone);
}

void SaveVideo::save(cv::Mat& mat){
	if (mSaveFileHandle == NULL){
		return;
	}
	//第二个参数是YUV420的格式
	cv::Mat save;
	cv::cvtColor(mat, save, cv::COLOR_BGR2YUV_I420);
    H264Encoder::getInstance()->EncoderFrame(mEncodeSaveBuff,(unsigned char*)save.data,true);
    int final_length = H264Encoder::getInstance()->encoded_data_length;
    if (final_length >= STREAM_WIDTH* STREAM_HEIGHT * 3){
    	std::cout << "**SaveVideo::save length over!**" << std::endl;
    }
    fwrite(mEncodeSaveBuff,1,final_length,mSaveFileHandle);
}

