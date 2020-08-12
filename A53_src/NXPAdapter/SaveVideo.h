/*
 * SaveVideo.h
 *
 *  Created on: 2019年1月29日
 *      Author: wangxiaobao
 */
#ifndef SAVEVIDEO_H_
#define SAVEVIDEO_H_

#include <opencv2/opencv.hpp>
#include <list>
#include <map>
#include <vector>
#include <set>
#include <unistd.h>

class SaveVideo {
public:
	SaveVideo(std::string savePath);
	virtual ~SaveVideo();
	void init();
	void save(cv::Mat& mat);

	unsigned char* mEncodeSaveBuff;
	FILE* mSaveFileHandle;
	std::string mSavePath;
};

#endif /* SAVEVIDEO_H_ */
