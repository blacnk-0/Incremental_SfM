//
// Created by GING on 2018-12-15.
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <string>
#include <vector>

#include "Definitions.h"

using namespace cv;
using namespace std;



#ifndef INCREMENTAL_SFM_COMPUTEFEATURES_H
#define INCREMENTAL_SFM_COMPUTEFEATURES_H

void Compute_SIFT_Feature_Single(const string & in_sImgPath,vector<KeyPoint> & out_KeyPoints,MAT_DESC & out_Descs);


void Compute_SIFT_Features_All(const MAP_IMGS & in_images,
        MAP_IMGS & out_valid_images,
        MAP_KEYPOINTS & out_all_keypoints ,
        MAP_DESCS & out_all_descs);

void Convert_MapKps_to_VecKps(const MAP_KEYPOINT & in_MapKps,vector<KeyPoint> out_kps);


#endif //INCREMENTAL_SFM_COMPUTEFEATURES_H
