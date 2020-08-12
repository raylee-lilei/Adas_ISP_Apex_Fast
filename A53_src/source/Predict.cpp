//
// Created by wangxiaobao on 18-8-30.
//

#include <dirent.h>
#include "../include/Predict.h"
#include <unistd.h>
#include "../include/define.h"
#include "../include/path.h"

using namespace std;
using namespace cv;

Predict * Predict::ins = NULL;

//#define TRAIN_IMAGE_SIZE 64,64
#define TRAIN_IMAGE_SIZE 32,32

static std::vector<std::string> trainInformation;

Predict::Predict()
{
    trainInformation.push_back(PATH_SVM_TRAIN_DIR);
    trainInformation.push_back(PATH_TSR_SVM_FILE);
    trainInformation.push_back(PATH_SVM_TRAIN_DIR2);
    trainInformation.push_back(PATH_TSR_SVM_FILE2);
    for (int i = 0; i < trainInformation.size()/2; ++i) {
        //svm = cv::ml::SVM::create();
        svms.push_back(cv::ml::SVM::create());
    }
}

Predict* Predict::instance()
{
    if (NULL == Predict::ins) {
        Predict::ins = new Predict();
        Predict::ins->init();
    }

    return Predict::ins;
}

/**
 * 训练分类器
 */
void Predict::train() {
    for (int i = 0; i < trainInformation.size()/2; ++i) {
        Mat data, labels;   //特征矩阵
        int l = 0;

        const char*trainPath = trainInformation[i*2].c_str();
        const char*trainFile = trainInformation[i*2 + 1].c_str();

        DIR *dir = opendir(trainPath);
        struct dirent *ptr;
        while ((ptr = readdir(dir)) != NULL) {

            if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) {
                continue;
            }

            char path[255] = {0};
            l = atoi(ptr->d_name);
            sprintf(path, "%s%s", trainPath, ptr->d_name);
            Mat SrcImage = imread(path);
            Mat gray;
            cv::cvtColor(SrcImage, gray, cv::COLOR_RGB2GRAY);
            cv::GaussianBlur(gray,gray,cv::Size(3, 3), 0, 0);//率波
            //gray = 255 - gray;
            Mat out;
            cv::resize(gray, out, Size(TRAIN_IMAGE_SIZE));
            cv::threshold(out, out, 0, 255, CV_THRESH_OTSU);
            out.convertTo(out, CV_32F);
            out = out.reshape(0, 1);
            data.push_back(out);
            labels.push_back(l);
            cout << "label:" << l << " " << path << endl;
            pics[l] = std::string(path);
            std::cout << "label:" << l << " path:" << path << endl;
        }

        data.convertTo(data, CV_32F); //uchar型转换为cv_32f

        int samplesNum = data.rows;
        Mat trainData, trainLabels;
        trainData = data(Range(0, samplesNum), Range::all());
        trainLabels = labels(Range(0, samplesNum), Range::all());

        svms[i]->setType(cv::ml::SVM::Types::C_SVC);

        //linear
        //svms[i]->setKernel(cv::ml::SVM::LINEAR);

        svms[i]->setKernel(cv::ml::SVM::POLY);
        svms[i]->setDegree(1.0);

        /*rbf
        svms[i]->setKernel(cv::ml::SVM::RBF);
        svms[i]->setDegree(0.1);
        svms[i]->setGamma(0.1);
        svms[i]->setC(1);
        svms[i]->setCoef0(0.1);
        svms[i]->setNu(0.1);
        svms[i]->setP(0.1);
         */

        svms[i]->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 0.000001));
        svms[i]->train(trainData, cv::ml::ROW_SAMPLE, trainLabels);

        svms[i]->save(trainFile);
    }

    cout << "tsr train over\n";
}

void Predict::init()
{
    bool needTrain = false;
    for (int i = 0; i < trainInformation.size()/2; ++i) {
        const char*fileName = trainInformation[i*2 + 1].c_str();
        if ((access(fileName, F_OK)) != -1) {
            printf("文件 %s 存在.\n",fileName);
            svms[i]= Algorithm::load<cv::ml::SVM>(fileName);
        } else{
            needTrain = true;
            break;
        }
    }

    if (!needTrain){
        cout << "加载完成" << endl;
        return;
    }

    train();
}

int Predict::offset() {
    return 100;
}

int Predict::realSpeed(int speed) {
    return speed /  offset();
}

/**
 * 识别数字
 * @param mat
 * @return
 */
int Predict::predict(cv::Mat& mat) {
    Mat out;
    cv::resize(mat, out, Size(TRAIN_IMAGE_SIZE));
    cv::threshold(out, out, 0, 255, CV_THRESH_OTSU);
    out.convertTo(out, CV_32F);
    out = out.reshape(0, 1);
    Mat re;
    int result = -1;
    cout << endl;
    for (int i = 0; i < trainInformation.size()/2; ++i) {
        float response = svms[i]->predict(out, re);

        if (response != 0) {
            return -1;
        }

        //cout << "response1:" << response << " " << re << " " << (int)re.at<float>(0,0) << endl;

        int result2 = (int) re.at<float>(0, 0);
#if 0
        if (result2 / 100 > 10) {
            cout <<(int)i <<  ":response---->" << result2 << endl;
        }
#endif
        if (result2 / 100 > 120){
            return -1;
        }

        if (result == -1) {
            result = result2;
            continue;
        }

        if (result/ 100 != result2 / 100){
            result = -1;
        }
    }

    if (result > 0) result = result / 100 * 100;

    return result;// / 100;

}
