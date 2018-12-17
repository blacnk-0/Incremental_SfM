//
// Created by GING on 2018-12-17.
//

#include "ComputeFeatures.h"


void Compute_SIFT_Feature_Single(const string in_sImgPath,vector<KeyPoint> & out_KeyPoints,MAT_DESC & out_Descs)
{
    Mat image=imread(in_sImgPath);
    Ptr<Feature2D> sift_detector = xfeatures2d::SIFT::create();

    sift_detector->detectAndCompute(image,noArray(),out_KeyPoints,out_Descs);

}


void Compute_SIFT_Features_All(const MAP_IMGS in_images,MAP_IMGS & out_valid_images, MAP_KEYPOINTS & out_all_keypoints ,MAP_DESCS & out_all_descs)
{
    vector<KeyPoint> kps;
    MAT_DESC descs;
    for(const auto & iter:in_images)
    {
        uint32_t dImgID=iter.first;
        const string sImgName=iter.second;

        Compute_SIFT_Feature_Single(sImgName,kps,descs);

        if(kps.size()>10)
        {
            out_valid_images[dImgID]=sImgName;

            MAP_KEYPOINT map_kps;
            for(int i=0;i<kps.size();++i)
            {
                map_kps[i]=kps[i];
            }
            out_all_keypoints[dImgID]=map_kps;
            out_all_descs[dImgID]=descs;
        }
    }

}

void Convert_MapKps_to_VecKps(const MAP_KEYPOINT & in_MapKps,vector<KeyPoint> out_kps)
{
    for(const auto & iter:in_MapKps)
    {
        out_kps.push_back(iter.second);
    }
}