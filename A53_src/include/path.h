//
// Created by wangxiaobao on 18-9-5.
//

#ifndef ADASPROJECT_PATH_H
#define ADASPROJECT_PATH_H

#ifndef _PATH_RESOURCE
#define _PATH_RESOURCE ".."
#endif

//#define PATH_IMAGE_LANE_DEPARTURE "/media/wangxiaobao/n2/00-code/adas/images/timg.jpeg"
#define PATH_IMAGE_LANE_LEFT        _PATH_RESOURCE "/images/shift_left.jpeg"
#define PATH_IMAGE_LANE_RIGHT       _PATH_RESOURCE "/images/shift_right.jpeg"
#define PATH_IMAGE_DSM              _PATH_RESOURCE "/images/dsm.jpeg"
#define PATH_CLASSIFIER_CAR         _PATH_RESOURCE "/train/car.xml"
//#define PATH_CLASSIFIER_CAR         _PATH_RESOURCE "/train/carlilei.xml"
#define PATH_TSR_SVM_FILE           _PATH_RESOURCE "/train/tsr/main.svm.xml"
#define PATH_SVM_TRAIN_DIR          _PATH_RESOURCE "/train/tsr/main/"
#define PATH_TSR_SVM_FILE2          _PATH_RESOURCE "/train/tsr/slave.svm.xml"
#define PATH_SVM_TRAIN_DIR2         _PATH_RESOURCE "/train/tsr/slave/"
#define PATH_DEFAULT_VIDEO          _PATH_RESOURCE "/testVideo/out720_cut.mp4"
#define PATH_IMAGE_FCW_ONE          _PATH_RESOURCE "/images/fcw_one.jpeg"
#define PATH_IMAGE_FCW_TWO          _PATH_RESOURCE "/images/fcw_two.jpeg"
//#define PATH_HOG_SVM_CAR            _PATH_RESOURCE "/train/hog_svm_x64.bin"
#define PATH_HOG_SVM_CAR            _PATH_RESOURCE "/train/hog_svm_x64lilei.bin"
//#define PATH_HOG_SVM_CAR            _PATH_RESOURCE "/train/hog_svm_x48.bin"


#endif //ADASPROJECT_PATH_H
