//
// Created by GING on 2018-12-19.
//

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>

#include "Definitions.h"
#include "TrackHelper.h"

#ifndef INCREMENTAL_SFM_BUNDLEADJUSTMENT_H
#define INCREMENTAL_SFM_BUNDLEADJUSTMENT_H

using namespace ceres;

struct ReprojectionCost
{
    ReprojectionCost(cv::Point2d & _observation):observation(_observation)
    {
    }

    template <typename T>
    bool operator()(const T * const intrinsic,const T * const extrinsic, const T * const points3D,T * residuals) const {
        //initial the address of rotation and translation
        const T * rotation=extrinsic;
        const T * translation=&extrinsic[3];

        //Apply camera rotation
        T position_after_projection[3];
        ceres::AngleAxisRotatePoint(rotation,points3D,position_after_projection);

        //Apply camera translation
        position_after_projection[0]+=translation[0];
        position_after_projection[1]+=translation[1];
        position_after_projection[2]+=translation[2];

        //Camera world coordinate as homogenous coordinate
        //divide by scale factor
        const T x=position_after_projection[0]/position_after_projection[2];
        const T y=position_after_projection[1]/position_after_projection[2];

        //Extract camera intrinsics
        const T fx=intrinsic[0];
        const T fy=intrinsic[1];
        const T cx=intrinsic[2];
        const T cy=intrinsic[3];

        //Apply intrinsic to homogenous coordinates
        //homogenous coordinate (3D world coordinate as homo-coord) --> homogenous coordinate( image plane u,v )
        const T u=fx*x+cx;
        const T v=fy*y+cy;

        //Calculate residuals
        residuals[0]=u-T(observation.x);
        residuals[1]=v-T(observation.y);

        return true;
    }

    //coordinate of feature point in image plane
    cv::Point2d observation;
};

//do bundle adjustment for current scene
void BundleAdjustment(cv::Mat & in_intrinsic,
        std::map<int,cv::Mat> & in_extrinsics,
        MAP_POINT3D & in_map_point3d,
        MAP_TRACKS & in_all_tracks,
        MAP_KEYPOINTS & in_all_kps,
        std::vector<cv::Point3d> & in_structure,
        std::set<int> & in_reconstructed_imgs);

//help update intrinsic ,extrinsic(rotation and translation)
void Update_Intrinsic_Extrinsic(std::map<int,cv::Mat> & in_extrinsics_map,
        cv::Mat & in_intrinsic_Mat,
        cv::Mat & out_K,
        std::map<int,cv::Mat> & out_rotations_map,
        std::map<int,cv::Mat> & out_translations_map);


#endif //INCREMENTAL_SFM_BUNDLEADJUSTMENT_H
