/*
 * HogSvmDetect.cpp
 *
 *  Created on: 2019年2月18日
 *      Author: wangxiaobao
 */
#if 0
#include "HogSvmDetect.h"

#include "../include/define.h"
#include <fstream>

using namespace std;
using namespace cv;
using namespace cv::ml;

void Train(string path)
{
    ////////////////////////////////读入训练样本图片路径和类别///////////////////////////////////////////////////
    //图像路径和类别
    vector<string> imagePath;
    int numberOfLine = 0;
    string buffer;
    ifstream trainingData(path + "/train.txt");

    while (!trainingData.eof())
    {
        getline(trainingData, buffer);
        if (!buffer.empty())
        {
            ++numberOfLine;
			//读取图像路径
			imagePath.push_back(buffer);
        }
    }

    //关闭文件
    trainingData.close();


    ////////////////////////////////获取样本的HOG特征///////////////////////////////////////////////////
    //样本特征向量矩阵
    int numberOfSample = numberOfLine ;
    Mat featureVectorOfSample;//矩阵中每行为一个样本
    //样本的类别
    Mat classOfSample;

    Mat convertedImage;
    Mat trainImage;

    // 计算HOG特征
    HOGDescriptor hog(cvSize(128, 128), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);
    for (vector<string>::size_type i = 0; i <= imagePath.size() - 1; ++i)
    {
        //读入图片
        Mat src = imread(imagePath[i], -1);
        if (src.empty())
        {
            cout << "can not load the image:" << imagePath[i] << endl;
            continue;
        }

        // 归一化
        resize(src, trainImage, Size(128, 128));

        // 提取HOG特征

        vector<float> descriptors;

        hog.compute(trainImage, descriptors);//这里可以设置检测窗口步长，如果图片大小超过64×128，可以设置winStride

        vector<float>::size_type descriptorDim = descriptors.size();
        if (0 == i){
        	featureVectorOfSample = Mat::zeros(numberOfSample, descriptorDim, CV_32FC1);
        	classOfSample = Mat::zeros(numberOfSample, 1, CV_32FC1);
        }

        //保存到特征向量矩阵中
        for (vector<float>::size_type j = 0; j < descriptorDim; ++j)
        {
            featureVectorOfSample.at<float>(i, j) = descriptors[j];
        }

        //保存类别到类别矩阵
        //!!注意类别类型一定要是int 类型的
        classOfSample.at<int>(i, 0) = i;
    }


    ///////////////////////////////////使用SVM分类器训练///////////////////////////////////////////////////
    //设置参数，注意Ptr的使用
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);//注意必须使用线性SVM进行训练，因为HogDescriptor检测函数只支持线性检测！！！
    svm->setTermCriteria(TermCriteria(CV_TERMCRIT_ITER, 1000, FLT_EPSILON));

    //使用SVM学习
    svm->train(featureVectorOfSample, ROW_SAMPLE, classOfSample);

    //保存分类器(里面包括了SVM的参数，支持向量,α和rho)
    svm->save(path+"/classifier.xml");

    /*
    SVM训练完成后得到的XML文件里面，有一个数组，叫做support vector，还有一个数组，叫做alpha,有一个浮点数，叫做rho;
    将alpha矩阵同support vector相乘，注意，alpha*supportVector,将得到一个行向量，将该向量前面乘以-1。之后，再该行向量的最后添加一个元素rho。
    如此，变得到了一个分类器，利用该分类器，直接替换opencv中行人检测默认的那个分类器（cv::HOGDescriptor::setSVMDetector()），
    */
    //获取支持向量机：矩阵默认是CV_32F
    Mat supportVector = svm->getSupportVectors();//

    //获取alpha和rho
    Mat alpha;//每个支持向量对应的参数α(拉格朗日乘子)，默认alpha是float64的
    Mat svIndex;//支持向量所在的索引
    float rho = svm->getDecisionFunction(0, alpha, svIndex);

    //转换类型:这里一定要注意，需要转换为32的
    Mat alpha2;
    alpha.convertTo(alpha2, CV_32FC1);

    int descriptorDim = svm->getVarCount();

    //结果矩阵，两个矩阵相乘
    Mat result(1, descriptorDim, CV_32FC1);
    result = alpha2*supportVector;

    //乘以-1，这里为什么会乘以-1？
    //注意因为svm.predict使用的是alpha*sv*another-rho，如果为负的话则认为是正样本，在HOG的检测函数中，使用rho+alpha*sv*another(another为-1)
    for (int i = 0; i < descriptorDim; ++i)
        result.at<float>(0, i) *= -1;

    //将分类器保存到文件，便于HOG识别
    //这个才是真正的判别函数的参数(ω)，HOG可以直接使用该参数进行识别
    FILE *fp = fopen((path + "hog_svm.bin").c_str(), "wb");
    for (int i = 0; i<descriptorDim; i++)
    {
        fprintf(fp, "%f \n", result.at<float>(0,i));
    }
    fprintf(fp, "%f", rho);

    fclose(fp);

}

#if 0
class MySVM: public CvSVM{
public:
    double * get_alpha_data()
    {
        return this->decision_func->alpha;
    }
    double  get_rho_data()
    {
        return this->decision_func->rho;
    }
};
#endif
void train2(){
#if 0
    MySVM SVM;
    int descriptorDim;

    string buffer;
    string trainImg;
    vector<string> posSamples;
    vector<string> negSamples;
    vector<string> testSamples;
    int posSampleNum;
    int negSampleNum;
    int testSampleNum;
    string basePath = "";//相对路径之前加上基地址，如果训练样本中是相对地址，则都加上基地址
    double rho;

#if 1
        ifstream fInPos("D:\\DataSet\\CarFaceDataSet\\PositiveSample.txt");//读取正样本
        ifstream fInNeg("D:\\DataSet\\CarFaceDataSet\\NegtiveSample.txt");//读取负样本

        while (fInPos)//讲正样本读入imgPathList中
        {
            if(getline(fInPos, buffer))
                posSamples.push_back(basePath + buffer);
        }
        posSampleNum = posSamples.size();
        fInPos.close();

        while(fInNeg)//读取负样本
        {
            if (getline(fInNeg, buffer))
                negSamples.push_back(basePath + buffer);
        }
        negSampleNum = negSamples.size();
        fInNeg.close();

        Mat sampleFeatureMat;//样本特征向量矩阵
        Mat sampleLabelMat;//样本标签

        HOGDescriptor * hog = new HOGDescriptor (cvSize(128, 128), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);
        vector<float> descriptor;

        for(int i = 0 ; i < posSampleNum; i++)// 处理正样本
        {
            Mat inputImg = imread(posSamples[i]);
            cout<<"processing "<<i<<"/"<<posSampleNum<<" "<<posSamples[i]<<endl;
            Size dsize = Size(128,128);
            Mat trainImg = Mat(dsize, CV_32S);
            resize(inputImg, trainImg, dsize);

            hog->compute(trainImg, descriptor, Size(8, 8));
            descriptorDim = descriptor.size();

            if(i == 0)//首次特殊处理根据检测到的维数确定特征矩阵的尺寸
            {
                sampleFeatureMat = Mat::zeros(posSampleNum + negSampleNum, descriptorDim, CV_32FC1);
                sampleLabelMat = Mat::zeros(posSampleNum + negSampleNum, 1, CV_32FC1);
            }

            for(int j = 0; j < descriptorDim; j++)//将特征向量复制到矩阵中
            {
                sampleFeatureMat.at<float>(i, j) = descriptor[j];
            }

            sampleLabelMat.at<float>(i, 0) = 1;
        }

        cout<<"extract posSampleFeature done"<<endl;

        for(int i = 0 ; i < negSampleNum; i++)//处理负样本
        {
            Mat inputImg = imread(negSamples[i]);
            cout<<"processing "<<i<<"/"<<negSampleNum<<" "<<negSamples[i]<<endl;
            Size dsize = Size(128,128);
            Mat trainImg = Mat(dsize, CV_32S);
            resize(inputImg, trainImg, dsize);
            hog->compute(trainImg, descriptor, Size(8,8));
            descriptorDim = descriptor.size();

            for(int j = 0; j < descriptorDim; j++)//将特征向量复制到矩阵中
            {
                sampleFeatureMat.at<float>(posSampleNum + i, j) = descriptor[j];
            }

            sampleLabelMat.at<float>(posSampleNum + i, 0) = -1;
        }

        cout<<"extract negSampleFeature done"<<endl;

        //此处先预留hard example 训练后再添加

        ofstream foutFeature("SampleFeatureMat.txt");//保存特征向量文件
        for(int i = 0; i <  posSampleNum + negSampleNum; i++)
        {
            for(int j = 0; j < descriptorDim; j++)
            {
                foutFeature<<sampleFeatureMat.at<float>(i, j)<<" ";
            }
            foutFeature<<"\n";
        }
        foutFeature.close();
        cout<<"output posSample and negSample Feature done"<<endl;

        CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER, 1000, FLT_EPSILON);
        CvSVMParams params(CvSVM::C_SVC, CvSVM::LINEAR, 0, 1, 0, 0.01, 0, 0, 0, criteria);  //这里一定要注意，LINEAR代表的是线性核，RBF代表的是高斯核，如果要用opencv自带的detector必须用线性核，如果自己写，或者只是判断是否为车脸的2分类问题则可以用RBF，在此应用环境中线性核的性能还是不错的
        cout<<"SVM Training Start..."<<endl;
        SVM.train_auto(sampleFeatureMat, sampleLabelMat, Mat(), Mat(), params);
        SVM.save("SVM_Model.xml");
        cout<<"SVM Training Complete"<<endl;
#endif

#ifndef TRAIN
        SVM.load("SVM_Model.xml");//加载模型文件
#endif
    descriptorDim = SVM.get_var_count();
    int supportVectorNum = SVM.get_support_vector_count();
    cout<<"support vector num: "<< supportVectorNum <<endl;

    Mat alphaMat = Mat::zeros(1, supportVectorNum, CV_32FC1);
    Mat supportVectorMat = Mat::zeros(supportVectorNum, descriptorDim, CV_32FC1);
    Mat resultMat = Mat::zeros(1, descriptorDim, CV_32FC1);

    for (int i = 0; i < supportVectorNum; i++)//复制支持向量矩阵
    {
        const float * pSupportVectorData = SVM.get_support_vector(i);
        for(int j = 0 ;j < descriptorDim; j++)
        {
            supportVectorMat.at<float>(i,j) = pSupportVectorData[j];
        }
    }

    double *pAlphaData = SVM.get_alpha_data();
    for (int i = 0; i < supportVectorNum; i++)//复制函数中的alpha 记住决策公式Y= wx+b
    {
        alphaMat.at<float>(0, i) = pAlphaData[i];
    }

    resultMat = -1 * alphaMat * supportVectorMat; //alphaMat就是权重向量

    //cout<<resultMat;

    cout<<"描述子维数 "<<descriptorDim<<endl;
    vector<float> myDetector;
    for (int i = 0 ;i < descriptorDim; i++)
    {
        myDetector.push_back(resultMat.at<float>(0, i));
    }

    rho = SVM.get_rho_data();
    myDetector.push_back(rho);
    cout<<"检测子维数 "<<myDetector.size()<<endl;

    HOGDescriptor myHOG (Size(128, 128), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    myHOG.setSVMDetector(myDetector);//设置检测子

    //保存检测子
    int minusNum = 0;
    int posNum = 0;

    ofstream foutDetector("HogDetectorForCarFace.txt");
    for (int i = 0 ;i < myDetector.size(); i++)
    {
        foutDetector<<myDetector[i]<<" ";
        //cout<<myDetector[i]<<" ";
    }

    //cout<<endl<<"posNum "<<posNum<<endl;
    //cout<<endl<<"minusNum "<<minusNum<<endl;
    foutDetector.close();
    //test part
    ifstream fInTest("D:\\DataSet\\CarFaceDataSet\\testSample.txt");
    while (fInTest)
    {
        if(getline(fInTest, buffer))
        {
            testSamples.push_back(basePath + buffer);
        }
    }
    testSampleNum = testSamples.size();
    fInTest.close();

    for (int i = 0; i < testSamples.size(); i++)
    {
        Mat testImg = imread(testSamples[i]);
        Size dsize = Size(320, 240);
        Mat testImgNorm (dsize, CV_32S);
        resize(testImg, testImgNorm, dsize);

        vector<Rect> found, foundFiltered;
        cout<<"MultiScale detect "<<endl;
        myHOG.detectMultiScale(testImgNorm, found, 0, Size(8,8), Size(0,0), 1.05, 2);
        cout<<"Detected Rect Num"<< found.size()<<endl;

        for (int i = 0; i < found.size(); i++)//查看是否有嵌套的矩形框
        {
            Rect r = found[i];
            int j = 0;
            for (; j < found.size(); j++)
            {
                if ( i != j && (r & found[j]) == r)
                {
                    break;
                }
            }
            if(j == found.size())
                foundFiltered.push_back(r);
        }
        for( int i = 0; i < foundFiltered.size(); i++)//画出矩形框
        {
            Rect r = foundFiltered[i];
            rectangle(testImgNorm, r.tl(), r.br(), Scalar(0,255,0), 1);
        }

        imshow("test",testImgNorm);
        waitKey();
    }
#endif
}

HogSvmDetect::HogSvmDetect() {
	// TODO Auto-generated constructor stub

}

HogSvmDetect::~HogSvmDetect() {
	// TODO Auto-generated destructor stub
}

#endif
