/*
 * NXPAdapter.h
 *
 *  Created on: 2018年10月24日
 *      Author: wangxiaobao
 */

#ifndef NXPADAPTER_H_
#define NXPADAPTER_H_
#include "../include/Adapter.h"
#include "apexcv_pro_canny.h"
#include "apexcv_base_image_filters.h"
#include "apexcv_pro_histogram_equalization.h"
#include "apexcv_pro_lbp.h"
#include "SaveVideo.h"
#include "apexcv_pro_hog.h"
#include "umat.hpp"


class NXPAdapter : virtual public Adapter{
private:
	apexcv::Canny *mCanny;
	apexcv::GaussianFilter* mGaussian;
	apexcv::HistogramEqualization *mHistogramEqualization;
	apexcv::ColorConverter* mColorConverter;
	apexcv::Lbp *mLbp;
	apexcv::Hog *mHog;
	SaveVideo *mSaveVideo;
public:
	static void toUMat(cv::Mat& src, vsdk::UMat& dst);
	static void toCMat(vsdk::UMat&src, cv::Mat& dst);

	static NXPAdapter* inst;
	ErrorCode initialize();
    ErrorCode canny(cv::Mat& src, cv::Mat& dst,double min,double max);
    void showImage(const char *title, cv::Mat &mat,int waitTime);
    void gaussianFilter(cv::Mat& src, cv::Mat&dst,int windowSize);
    void equalizeHist(cv::Mat& src, cv::Mat& dst);
    void lbpTrain(cv::Mat& src,int count,int width,int height,cv::Mat& model);
    void lbpPredict(cv::Mat& model,int modelCount,cv::Mat& src,cv::Mat& result);
    void Rgb2Gray(cv::Mat&src,cv::Mat&dst);
    void capature(int dev,cv::Mat& mat);
	NXPAdapter();
	virtual ~NXPAdapter();
	void saveVideo(std::string path,cv::Mat& mat);
	void foundCar(cv::Mat&img,std::vector<cv::Rect> &found);
};

#endif /* NXPADAPTER_H_ */
