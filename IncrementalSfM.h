//
// Created by GING on 2018-12-15.
//

#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include "ComputeFeatures.h"
#include "Definitions.h"
#include <set>
#include "flat_pair_map.h"
#include "TrackHelper.h"

#ifndef INCREMENTAL_SFM_INCREMENTALSFM_H
#define INCREMENTAL_SFM_INCREMENTALSFM_H

//choose initial pair with most matches
bool Choose_Initial_Pair(const MAP_MATCHES & in_map_matches,std::pair<int,int> & out_ImageID);

//Rescontruct Initial Pair
//initial out_structure
//initial out_rotation
//initial out_translation
void Reconstruct_Initial_Pair(
        std::pair<int,int> & in_initialPair,
        cv::Mat & in_K,cv::Mat & in_R,cv::Mat & in_T, std::vector<cv::Point2f> & in_p1,std::vector<cv::Point2f> & in_p2,
        VEC_MATCHES & in_initial_matches,
        MAP_TRACKS & in_all_tracks,
        std::vector<cv::Point3f> & out_structure,
        std::map<int,cv::Mat> & out_rotations,
        std::map<int,cv::Mat> & out_translations,
        MAP_POINT3D & out_point3d_correspondence);

//First Camera [I|0] , Second Camera [R|T]
bool Find_Transform_Initial(cv::Mat & in_K,std::vector<cv::Point2f> &in_p1, std::vector<cv::Point2f> &in_p2, cv::Mat &out_R, cv::Mat &out_T);

//Find Next Image Prepare for Next Round Reconstruction
bool Find_Next_Image(std::set<int> in_remaing_imageID,std::set<int> in_reconstructured_track_ID,MAP_TRACKS in_all_tracks,int & out_next_imageID);

//Use Next Image to Update Scene
//in_matches --  all matches
//in keypoints -- all keypoints
//1.find reconstructed image with most matches
//2.get correspondence 3d points
//3.solve PnP problem
//4.find correspondence feature points
//5.triangulate
//6.update scene
void Incremental_Process(
        cv::Mat & in_K,
        int in_ProcessImgID,
        std::vector<int> & in_reconstructed_images,
        const MAP_TRACKS & in_all_tracks,
        MAP_MATCHES & in_matches,
        MAP_KEYPOINTS & in_keypoints,
        MAP_COLORS & in_all_colors,
        std::vector<std::vector<int>> & out_corresponds,
        std::vector<cv::Point3f> & out_structure,
        std::map<int,cv::Mat> & out_rotations,
        std::map<int,cv::Mat> & out_translations,
        std::set<int> & out_remaining_images,
        std::set<int> & out_recons_trackID,
        std::vector<cv::Vec3b> & out_colors,
        MAP_POINT3D & out_point3d_correspondence);

//main SfM and save structure to yml
//Maintain List:
//1.structure -- 3D Points
//2.correspondence -- [ImageID][FeatureID]-[3DPointIndex in structure]
//3.remaining_images -- ImageID
//4.reconsturctured_images -- ImageID
//5.out_rotations -- Mat
//6.out_translations -- Mat
void Main_SfM(cv::Mat & in_K,MAP_IMGS & in_images,MAP_TRACKS & in_tracks,MAP_MATCHES & in_matches,
        MAP_KEYPOINTS & in_keypoints,
        MAP_COLORS & in_colors,
        std::map<int,cv::Mat> out_rotations,
        std::map<int,cv::Mat> out_translations);



#endif //INCREMENTAL_SFM_INCREMENTALSFM_H
