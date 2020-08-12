/*
 * NXPAdapter.cpp
 *
 *  Created on: 2018年10月24日
 *      Author: wangxiaobao
 */
#include "NXPAdapter.h"
#include "../include/ErrorCode.h"
#include "frame_output_v234fb.h"
//#include "apu_gen.h"

#ifdef APEX2_EMULATE
  #include "apu_lib.hpp"
  #include "apu_extras.hpp"
  #include "acf_lib.hpp"
  using namespace APEX2;
#else
  #include "apex.h"
#endif

#include "common_time_measure.h"
#include "../include/HogSvmDetect.h"
#include "../include/path.h"
//#define NOT_USE_APEXCV

#define APEX_CORE_0 0
#define APEX_CORE_1 1

#define CHNL_CNT io::IO_DATA_CH3

#define RETURN_ON_ERR(title,code) if (code!=0){cout << title << " error,code:"<<(int)code << endl;}
#define CONVERT_TICKS_TO_US(_TICKS) ((int)(1000000.0*FSL_TicksToSeconds(_TICKS)))

NXPAdapter* NXPAdapter::inst = NULL;

Adapter* Adapter::instance(){
	if (NXPAdapter::inst == NULL){
		NXPAdapter::inst = new NXPAdapter();
	}

	return NXPAdapter::inst;
}

NXPAdapter::NXPAdapter() {
	// TODO Auto-generated constructor stub
	mSaveVideo = NULL;
	mHog = NULL;
#if 1
	mCanny = NULL;
	mGaussian = NULL;
	mHistogramEqualization = NULL;
	mLbp = NULL;
#endif
}

NXPAdapter::~NXPAdapter() {
#if 1
	// TODO Auto-generated destructor stub
	delete mCanny;
	mCanny = NULL;

	delete mGaussian;
	mGaussian = NULL;

	delete mHistogramEqualization;
	mHistogramEqualization = NULL;

	delete mLbp;
	mLbp = NULL;
#endif
}

void NXPAdapter::saveVideo(std::string path,cv::Mat& mat){
	if (path.empty()){
		return;
	}

	if (mSaveVideo == NULL){
		mSaveVideo = new SaveVideo(path);
	}

	mSaveVideo->save(mat);
}

extern int NXPPrepare();
extern int NXPCleanup();
ErrorCode NXPAdapter::initialize(){

    //ACF_Init();
	 APEX_Init();

    //APU_CALL();

    int code =NXPPrepare();
    if (code != 0){
    	cout << "NXP Prepare error" << endl;
    	NXPCleanup();
    	return code;
    }

    return ERROR_SUCCESS;
}

extern void NXPCapture(int dev,cv::Mat& dst);
extern void readFromCamera(cv::Mat& result);
extern void writeToCamera(cv::Mat &mat);
int g_dev = 0;
void NXPAdapter::capature(int dev,cv::Mat& mat){
	g_dev = dev;
	if (g_dev == 9){
		cout << "NXPAdapter::capature\n";
		readFromCamera(mat);
		return;
	}
	//NXPPrepare();
	//cout << "NXPAdapter::cap_begin\n";
	NXPCapture(dev,mat);
	//cout << "NXPAdapter::cap_end\n";
}
/**
 * cv::Mat转vsdk::UMat
 * @param src
 * @param dst
 * @return
 */
void NXPAdapter::toUMat(cv::Mat& src, vsdk::UMat& dst){
	//dst = src.getUMat(cv::ACCESS_RW);
	//src.copyTo(dst);

	static cv::UMat image(cv::USAGE_ALLOCATE_HOST_MEMORY);
	src.copyTo(image);
	dst = image;
}

/**
 * vsdk::UMat转cv::Mat
 * @param src
 * @param dst
 * @return
 */
void NXPAdapter::toCMat(vsdk::UMat&src, cv::Mat& dst){
	vsdk::Mat vMat = src.getMat(OAL_USAGE_CACHED);
	dst = ((cv::Mat)vMat).clone();
}

/**
 * Canny检测
 * @param src
 * @param dst
 * @param min
 * @param max
 * @return
 */
ErrorCode NXPAdapter::canny(cv::Mat& src, cv::Mat& dst,double min,double max){
#ifdef NOT_USE_APEXCV
	cv::Canny(src,dst,min,max);
	return ERROR_SUCCESS;
#else
	vsdk::UMat uMat;
	toUMat(src,uMat);

	int result = 0;
	if (NULL == mCanny){
		mCanny = new apexcv::Canny();
		result = mCanny->Initialize(uMat, uMat, src.cols, src.rows);
		mCanny->SetThresholds(min, max);

	} else {
		result = mCanny->ReconnectIO(uMat, uMat);
		mCanny->SetThresholds(min,max);
	}
	if (result != 0) {
		cout << "canny init error:" << (int)result << endl;
		delete mCanny;
		mCanny = NULL;
		cv::Canny(src,dst,min,max);
		return ERROR_SUCCESS;
	}

	result = mCanny->Process();
	if (result != 0) {
		cout << "canny Process error:" << (int)result << endl;
		delete mCanny;
		mCanny = NULL;
		cv::Canny(src,dst,min,max);
		return ERROR_SUCCESS;
	}

	toCMat(uMat,dst);
	return ERROR_SUCCESS;
#endif
}

/**
 * 显示图像
 * @param title
 * @param mat
 * @param waitTime
 */
extern int g_debugShow;
void NXPAdapter::showImage(const char *title, cv::Mat &mat,int waitTime){
	if (!g_debugShow){
		return;
	}
#if 0
	if (g_dev == 9){
		writeToCamera(mat);
		return;
	}
#endif
	int width=800;//800;
	int height=480;//480;
	uint32_t aFormat = 0;

    if (mat.rows != height || mat.cols != width) {
    	cv::resize(mat, mat, cv::Size(width,height));
    }

    static io::FrameOutputV234Fb outputFrame(width, height, io::IO_DATA_DEPTH_08, io::IO_DATA_CH3,aFormat);

#if 0
	vsdk::UMat output_umat = vsdk::UMat(height, width, CV_8UC3);
	{
	  cv::Mat    output_mat = output_umat.getMat(vsdk::ACCESS_WRITE | OAL_USAGE_CACHED);
	  memset(output_mat.data, 0, height*width*3);

	  mat.copyTo(output_mat);
	}

	outputFrame.PutFrame(output_umat);
#endif

#if 1
	static cv::UMat image(cv::USAGE_ALLOCATE_HOST_MEMORY);
	mat.copyTo(image);
    //vsdk::UMat output_umat(show.getUMat(cv::ACCESS_READ));
	vsdk::UMat output_umat(image);
    outputFrame.PutFrame(output_umat);
#endif
}

/**
 * 高斯滤波
 * @param src
 * @param dst
 * @param windowSize
 */
void NXPAdapter::gaussianFilter(cv::Mat& src, cv::Mat&dst,int windowSize){
#ifdef NOT_USE_APEXCV
	cv::GaussianBlur(src,dst,cv::Size(windowSize, windowSize), 0, 0);
#else
	if (src.type() != CV_8UC1) {
		cv::GaussianBlur(src,dst,cv::Size(windowSize, windowSize), 0, 0);
		return;
	}

	windowSize = (windowSize < 5) ? 3 : 5;

	vsdk::UMat umat,dstMat;
	toUMat(src,umat);
	toUMat(src,dstMat);
	int result;
	if (NULL == mGaussian){
		mGaussian = new apexcv::GaussianFilter();
		result = mGaussian->Initialize(umat, windowSize, dstMat);
	} else {
		result = mGaussian->ReconnectIO(umat, umat);
	}

	if (result != 0)
	{
		cout << "mGaussian->ReconnectIO error:" << (int)result << endl;
		cv::GaussianBlur(src,dst,cv::Size(windowSize, windowSize), 0, 0);
		delete mGaussian;
		mGaussian = NULL;
		return;
	}
	//mGaussian->SelectApexCore(APEX_CORE_0);
	result = mGaussian->Process();
	if (result != 0)
	{
		cout << " mGaussian->Process error:" << (int)result << endl;
		cv::GaussianBlur(src,dst,cv::Size(windowSize, windowSize), 0, 0);
		delete mGaussian;
		mGaussian = NULL;
		return;
	}

	toCMat(dstMat,dst);

	return;
#endif
}

/**
 * 图像均衡
 * @param src
 * @param dst
 */
void NXPAdapter::equalizeHist(cv::Mat& src, cv::Mat& dst){
#ifdef NOT_USE_APEXCV
	cv::equalizeHist(src,dst);;
#else
	vsdk::UMat srcUMat,dstUMat;
	toUMat(src,srcUMat);
	toUMat(dst,dstUMat);

	int result = 0;
	if (mHistogramEqualization == NULL) {
		mHistogramEqualization = new apexcv::HistogramEqualization();
		result = mHistogramEqualization->Initialize(srcUMat, dstUMat);
	} else {
		result = mHistogramEqualization->ReconnectIO(srcUMat, dstUMat);
	}

	if (result != 0) {
		cout << "mHistogramEqualization->ReconnectIO:"<< (int)result << endl;
		delete mHistogramEqualization;
		mHistogramEqualization = NULL;
		cv::equalizeHist(src,dst);;
		return;
	}

	result = mHistogramEqualization->Process();
	if (result != 0) {
		cout << "mHistogramEqualization->Process:"<< (int)result << endl;
		delete mHistogramEqualization;
		mHistogramEqualization = NULL;
		cv::equalizeHist(src,dst);
		return;
	}

	toCMat(dstUMat,dst);
	return;
#endif
}

void NXPAdapter::Rgb2Gray(cv::Mat&src,cv::Mat&dst){
#ifdef NOT_USE_APEXCV
	cv::cvtColor(src,dst,cv::COLOR_RGB2GRAY);
#else
	if (src.cols > 256) {
		cv::cvtColor(src,dst,cv::COLOR_RGB2GRAY);
		return;
	}
	vsdk::UMat srcUMat,dstUMat(src.rows,src.cols,(VSDK_CV_8UC1));
	toUMat(src,srcUMat);

	int result = 0;
	if (NULL == mColorConverter){
		mColorConverter = new apexcv::ColorConverter();
		result = mColorConverter->Initialize(srcUMat, apexcv::ColorConverter::eRGB888_TO_GREY, dstUMat);
	} else {
		result = mColorConverter->ReconnectIO(srcUMat, dstUMat);
	}

	if (result != 0){
		cout << "gray Error:" << (int)result << endl;
		cv::cvtColor(src,dst,cv::COLOR_RGB2GRAY);
		delete mColorConverter;
		mColorConverter = NULL;
		return;
	}

	result = mColorConverter->Process();
	if (result != 0){
		cout << "gray Process error:" << (int)result << endl;
		cv::cvtColor(src,dst,cv::COLOR_RGB2GRAY);
		delete mColorConverter;
		mColorConverter = NULL;
		return;
	}

	toCMat(dstUMat,dst);
#endif
}

/**
 * LBP训练
 * @param src
 * @param count
 * @param width
 * @param height
 * @param model
 */
void NXPAdapter::lbpTrain(cv::Mat& src,int count,int width,int height,cv::Mat& model){
	return;
}

/**
 * LBP筛选
 * @param model
 * @param modelCount
 * @param src
 * @param result
 */
void NXPAdapter::lbpPredict(cv::Mat& model,int modelCount,cv::Mat& src,cv::Mat& result){

}

int getEnv(const char*name){
	char* buffer = getenv (name);
	  if (buffer!=NULL){
		  return atoi(buffer);
	  }
	  return -1;
}

static std::vector<cv::Rect> mergeRect(std::vector<cv::Rect>& in){
	cout << "mergeRect in:" << in.size() << endl;
	std::vector<cv::Rect> result;
	for (int i = 0; i < (int)in.size(); ++i){
		cv::Rect& rect = in[i];
		int j = 0;
		for (; j < (int)result.size(); ++j){
			cv::Rect& temp = result[j];
			int x = 0;
			int y = 0;

			x = rect.x;y = rect.y;
			if  (x > temp.x && x < (temp.width + temp.x) && y > temp.y && y < (temp.y + temp.height)){
				break;
			}

			x = rect.x+rect.width;y = rect.y;
			if  (x > temp.x && x < (temp.width + temp.x) && y > temp.y && y < (temp.y + temp.height)){
				break;
			}

			x = rect.x;y = rect.y + rect.height;
			if  (x > temp.x && x < (temp.width + temp.x) && y > temp.y && y < (temp.y + temp.height)){
				break;
			}

			x = rect.x+rect.width;y = rect.y+rect.height;
			if  (x > temp.x && x < (temp.width + temp.x) && y > temp.y && y < (temp.y + temp.height)){
				break;
			}
		}
		if (j == (int)result.size()){
			result.push_back(rect);
		} else {
			cv::Rect& temp = result[j];
			int x = temp.x > rect.x ? rect.x : temp.x;
			int y = temp.y > rect.y ? rect.y : temp.y;

			int w = temp.x + temp.width  - x;
			int h = temp.y + temp.height - y;

			int w1 = rect.x + rect.width - x;
			int h1 = rect.y + rect.height -y;

			w = w > w1 ? w : w1;
			h = h > h1 ? h : h1;

			temp.x = x;
			temp.y = y;
			temp.width = w;
			temp.height = h;
		}
	}

	cout << "mergeRect out:" << result.size() << endl;
	return result;
}


size_t GetHogDescriptorSize(const apexcv::Hog::Config& hogConfig)
{
  return (hogConfig.mDetWinWidth * hogConfig.mDetWinHeight * hogConfig.mHistogramBins)/
         (hogConfig.mBlockWidth*hogConfig.mBlockHeight);
}

cv::Rect g_foundCarRect = cv::Rect(0,0,0,0);
void NXPAdapter::foundCar(cv::Mat&img,std::vector<cv::Rect> &found){

#ifdef NOT_USE_APEXCV
	static HogSvmDetect hog;
	uint64_t lStart    = FSL_Ticks();
	hog.foundCar(img, found);
	uint64_t lStop    = FSL_Ticks();
	uint64_t time = CONVERT_TICKS_TO_US(lStop - lStart);
	printf("hog.foundCar run time:%lld us\n",time);
	return;
#else
	vsdk::SUMat aDst;
	vsdk::SUMat aSrc;
	vsdk::UMat aSrcUmat;
	found.clear();
	//toUMat(img,aSrc);

    cout << "img.rows:" << img.rows << endl;
    cout << "img.cols:" << img.cols << endl;

	aSrcUmat = img.getUMat(cv::ACCESS_RW);
	aSrc = aSrcUmat;

	int env = getEnv("HHSQ_HOLD");
	int HHSQ_HOLD = env == -1 ? -800 : env;

	env = getEnv("HHSQ_DW");
	int HHSQ_DW = env == -1 ? 64 : env;

	env = getEnv("HHSQ_MERGE");
	int HHSQ_MERGE = env == -1 ? 3 : env;

	const char* svmPath = getenv ("HHSQ_PATH");
	if (svmPath == NULL){
		svmPath = PATH_HOG_SVM_CAR;
	}

	cout << "svmPath: " << svmPath << endl;
#if 1
	cout <<  "HHSQ_HOLD:"  << (int)HHSQ_HOLD << endl;
	cout <<  "HHSQ_DW:"  << (int)HHSQ_DW << endl;
	cout <<  "HHSQ_MERGE:"  << (int)HHSQ_MERGE << endl;
#endif
	std::map<int,cv::Rect> mapFound;
	std::vector<cv::Rect> rectList;
	APEXCV_LIB_RESULT errCode;
	if (mHog == NULL) {

	    mHog = new apexcv::Hog();

	    apexcv::Hog::Config config;
	    config.mDetWinWidth = 64;
	    config.mDetWinHeight = 64;

	    mHog->SetConfig(config);
	    mHog->GetConfig(config);

	    cout << "config.mDetWinWidth:" << config.mDetWinWidth << endl;
	    cout << "config.mDetWinHeight:" << config.mDetWinHeight << endl;
	    cout << "config.mBlockWidth:" << config.mBlockWidth << endl;
	    cout << "config.mStrideWidth:" << config.mStrideWidth << endl;
	    cout << "config.mStrideHeight:" << config.mStrideHeight << endl;
	    cout << "config.mHistogramBins:" << config.mHistogramBins << endl;
	    cout << "aSrc.rows:" << aSrc.rows << endl;
	    cout << "aSrc.cols:" << aSrc.cols << endl;
	    cout << "aSrc.type():" << aSrc.type() << endl;

		const size_t svmSize = GetHogDescriptorSize(config);

	    FILE *fp = fopen(svmPath,"rb");
	    vector<float> svmData;
	    float value;
	    while(fscanf(fp,"%f\n",&value) != EOF){
	        svmData.push_back(value);
	    }

	    cout << "svmData.size():" << svmData.size() << endl;

	    static vsdk::SUMat mSvmDouble = vsdk::SUMat(1,svmData.size(), VSDK_CV_64FC1);
	    vsdk::SMat svmMat     = mSvmDouble.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	    double*   pSvmDouble = (double*)svmMat.data;
	    for (int i = 0; (i < (int)svmData.size()) ; ++i){
	    	pSvmDouble[i] = svmData[i];
	    }

	    cout << "mSvmDouble.rows:" << mSvmDouble.rows << endl;
	    cout << "mSvmDouble.cols:" << mSvmDouble.cols << endl;

	    errCode = mHog->Initialize(aSrc, mSvmDouble, aDst);
	    RETURN_ON_ERR("mHog->Initialize",errCode);
	} else {
		errCode = mHog->ReconnectIO(aSrc,aDst);
		RETURN_ON_ERR("mHog->ReconnectIO",errCode);
	}

	uint64_t lStart    = FSL_Ticks();
	errCode = mHog->Process();
	uint64_t lStop    = FSL_Ticks();

	uint64_t time = CONVERT_TICKS_TO_US(lStop - lStart);
	//printf("mHog->Process run time:%lld us\n",time);

	RETURN_ON_ERR("mHog->Process",errCode);

	vsdk::Mat lHogScores_mat  = aDst.getMat(vsdk::ACCESS_READ | OAL_USAGE_CACHED);
	//const int16_t* const cpcHogScoresS16 = (const int16_t* const)(((uintptr_t)lHogScores_mat.data));

	vsdk::SMat cDst = aDst.getMat(OAL_USAGE_CACHED);

	//cout << "cDst rows:" << (int)cDst.rows << " cols:" << (int)cDst.cols << " type:" << (int)cDst.type() <<endl;
	//cout << cDst;
	int value16 = -32767;
	int x = 0;
	int y = 0;
	cv::Mat mask = cv::Mat::zeros(cDst.rows,cDst.cols,CV_8UC1);

	cout << "cDst.rows:" << cDst.rows << endl;
	cout << "cDst.cols:" << cDst.cols << endl;
	cout << "svmPath 5" << endl;
	for (int i = 0; i < cDst.rows; ++i) {
		for (int j = 0; j < cDst.cols;++j){
			short int value = cDst.at<short int>(i,j);
			if (value > 0){
				//cout <<  "found row:" << (int)i << "  col:" << (int)j << " value:" << (short int)value << endl;
			}
			if (value > value16){
				value16 = value;
				y = i * 4;
				x = j * 4;
			}
			if  (value > HHSQ_HOLD){
				mapFound[value] = cv::Rect(j*4,i*4,64,64);
				rectList.push_back(cv::Rect(j*4,i*4,HHSQ_DW,HHSQ_DW));
				//cout << "rectList i:" << i << " j:" << j << " v:" << value << endl;
				mask.at<uchar>(i,j) = 255;
			}
		}
	}

	std::vector<cv::Rect> mrect = rectList;//mergeRect(rectList);

	//mrect.reserve(mrect.size());
	//mrect = mergeRect(mrect);

	x =  (int)rectList.size();
	//cout << "b->rectList.size:" << (int)rectList.size() << endl;
	vector<int> weights;
	//cv::groupRectangles(rectList,weights,HHSQ_MERGE);

	printf("run time:%lld us, max:%d list b:%d a:%d msize:%d\n",time,value16,x,(int)rectList.size(),(int)mrect.size());
#if 0
	for (int i = 0; i < mrect.size(); ++i){
		cout << "rect x:" << mrect[i].x << "  y:" << mrect[i].y << " w:" << mrect[i].width << " h:" << mrect[i].height << endl;
	}
#endif
	found = mrect;

	return;
#endif
}


#if 0

void NXPAdapter::foundCar(cv::Mat&img,std::vector<cv::Rect> &found){

#ifdef NOT_USE_APEXCV
	static HogSvmDetect hog;
	uint64_t lStart    = FSL_Ticks();
	hog.foundCar(img, found);
	uint64_t lStop    = FSL_Ticks();
	uint64_t time = CONVERT_TICKS_TO_US(lStop - lStart);
	printf("hog.foundCar run time:%lld us\n",time);
	return;
#else
	vsdk::SUMat aDst;
	vsdk::UMat aSrc;
	found.clear();
	toUMat(img,aSrc);

	int env = getEnv("HHSQ_HOLD");
	int HHSQ_HOLD = env == -1 ? -800 : env;

	env = getEnv("HHSQ_DW");
	int HHSQ_DW = env == -1 ? 64 : env;

	env = getEnv("HHSQ_MERGE");
	int HHSQ_MERGE = env == -1 ? 3 : env;

	const char* svmPath = getenv ("HHSQ_PATH");
	if (svmPath == NULL){
		svmPath = PATH_HOG_SVM_CAR;
	}

	cout << "svmPath: " << svmPath << endl;
#if 0
	cout <<  "HHSQ_HOLD:"  << (int)HHSQ_HOLD << endl;
	cout <<  "HHSQ_DW:"  << (int)HHSQ_DW << endl;
	cout <<  "HHSQ_MERGE:"  << (int)HHSQ_MERGE << endl;
#endif
	std::map<int,cv::Rect> mapFound;
	std::vector<cv::Rect> rectList;
	APEXCV_LIB_RESULT errCode;
	if (mHog == NULL) {

	    mHog = new apexcv::Hog();

	    apexcv::Hog::Config config;
	    config.mSVMTransformMode = apexcv::Hog::SVMTransformMode::NONE;
	    config.mDetWinWidth = 64;
	    config.mDetWinHeight = 64;

	    cout << "config.mDetWinWidth:" << config.mDetWinWidth << endl;
	    cout << "config.mDetWinHeight:" << config.mDetWinHeight << endl;
	    cout << "config.mBlockWidth:" << config.mBlockWidth << endl;
	    cout << "config.mStrideWidth:" << config.mStrideWidth << endl;
	    cout << "config.mStrideHeight:" << config.mStrideHeight << endl;
	    cout << "config.mHistogramBins:" << config.mHistogramBins << endl;
	    cout << "aSrc.rows:" << aSrc.rows << endl;
	    cout << "aSrc.cols:" << aSrc.cols << endl;
	    cout << "aSrc.type():" << aSrc.type() << endl;

	    mHog->SetConfig(config);

	    mHog->GetConfig(config);

		const size_t svmSize = GetHogDescriptorSize(config);

	    FILE *fp = fopen(svmPath,"rb");
	    vector<float> svmData;
	    float value;
	    while(fscanf(fp,"%f\n",&value) != EOF){
	        svmData.push_back(value);
	    }

	    static vsdk::SUMat mSvmDouble = vsdk::SUMat(1,svmSize + 1, VSDK_CV_64FC1);
	    vsdk::SMat svmMat     = mSvmDouble.getMat(vsdk::ACCESS_RW | OAL_USAGE_CACHED);
	    double*   pSvmDouble = (double*)svmMat.data;
	    for (int i = 0; i < (int)svmData.size(); ++i){
	    	pSvmDouble[i] = svmData[i];
	    }

	    cout << "mSvmDouble.rows:" << mSvmDouble.rows << endl;
	    cout << "mSvmDouble.cols:" << mSvmDouble.cols << endl;
#if 0
	    errCode = mHog->Initialize(aSrc, aDst, apexcv::Hog::HogType::DETECT);
	    RETURN_ON_ERR("mHog->Initialize",errCode);

	    errCode = mHog->SetSVM(mSvmDouble);
	    RETURN_ON_ERR("mHog->SetSVM",errCode);
#endif
	    errCode = mHog->Initialize(aSrc, mSvmDouble, aDst);
	    RETURN_ON_ERR("mHog->Initialize",errCode);
	} else {
		errCode = mHog->ReconnectIO(aSrc,aDst);
		RETURN_ON_ERR("mHog->ReconnectIO",errCode);
	}

	uint64_t lStart    = FSL_Ticks();
	errCode = mHog->Process();
	uint64_t lStop    = FSL_Ticks();

	uint64_t time = CONVERT_TICKS_TO_US(lStop - lStart);
	//printf("mHog->Process run time:%lld us\n",time);

	RETURN_ON_ERR("mHog->Process",errCode);

	vsdk::Mat lHogScores_mat  = aDst.getMat(vsdk::ACCESS_READ | OAL_USAGE_CACHED);
	//const int16_t* const cpcHogScoresS16 = (const int16_t* const)(((uintptr_t)lHogScores_mat.data));
	cv::Mat cDst;
	toCMat(aDst,cDst);
	//cout << "cDst rows:" << (int)cDst.rows << " cols:" << (int)cDst.cols << " type:" << (int)cDst.type() <<endl;
	//cout << cDst;
	int value16 = -32767;
	int x = 0;
	int y = 0;
	cv::Mat mask = cv::Mat::zeros(cDst.rows,cDst.cols,CV_8UC1);
	for (int i = 0; i < cDst.rows; ++i) {
		for (int j = 0; j < cDst.cols;++j){
			short int value = cDst.at<short int>(i,j);
			if (value > 0){
				//cout <<  "found row:" << (int)i << "  col:" << (int)j << " value:" << (short int)value << endl;
			}
			if (value > value16){
				value16 = value;
				y = i * 4;
				x = j * 4;
			}
			if  (value > HHSQ_HOLD){
				mapFound[value] = cv::Rect(j*4,i*4,64,64);
				rectList.push_back(cv::Rect(j*4,i*4,HHSQ_DW,HHSQ_DW));
				mask.at<uchar>(i,j) = 255;
			}
		}
	}
	//cout <<  "found max value16:" << (int)value16 << endl;
	//cout << mask << endl;
#if 0
	if (x != 0 || y != 0){
		g_foundCarRect.x = x;
		g_foundCarRect.y = y;
		g_foundCarRect.width = 64;
		g_foundCarRect.height = 64;

		char path[255] = {0};
		sprintf(path,"%lld.jpg",lStop);
		//cv::imwrite(path, mask);
	}
#endif
	std::vector<cv::Rect> mrect = mergeRect(rectList);
	mrect.reserve(mrect.size());
	mrect = mergeRect(mrect);

	x =  (int)rectList.size();
	//cout << "b->rectList.size:" << (int)rectList.size() << endl;
	vector<int> weights;
	cv::groupRectangles(rectList,weights,HHSQ_MERGE);

	printf("run time:%lld us, max:%d list b:%d a:%d msize:%d\n",time,value16,x,(int)rectList.size(),(int)mrect.size());

	found = mrect;

	return;
#endif
}

#endif


