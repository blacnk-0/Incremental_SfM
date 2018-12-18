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
bool Choose_Initial_Pair(const MAP_MATCHES & in_map_matches,pair<int,int> & out_ImageID);

//Rescontruct Initial Pair
//initial out_structure
//initial out_rotation
//initial out_translation
void Reconstruct_Initial_Pair(
        pair<int,int> & in_initialPair,
        Mat & in_K,Mat & in_R,Mat & in_T, vector<Point2f> & in_p1,vector<Point2f> & in_p2,vector<Point3f> & out_structure,
        map<int,Mat> & out_rotations,
        map<int,Mat> & out_translations);

//First Camera [I|0] , Second Camera [R|T]
bool Find_Transform_Initial(Mat & in_K,vector<Point2f> &in_p1, vector<Point2f> &in_p2, Mat &out_R, Mat &out_T);

//Find Next Image Prepare for Next Round Reconstruction
bool Find_Next_Image(set<int> in_remaing_imageID,set<int> in_reconstructured_track_ID,MAP_TRACKS in_all_tracks,int & out_next_imageID);

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
        Mat & in_K,
        int in_ProcessImgID,
        vector<int> & in_reconstructed_images,
        const MAP_TRACKS & in_all_tracks,
        MAP_MATCHES & in_matches,
        MAP_KEYPOINTS & in_keypoints,
        MAP_COLORS & in_all_colors,
        vector<vector<int>> & out_corresponds,
        vector<Point3f> & out_structure,
        map<int,Mat> & out_rotations,
        map<int,Mat> & out_translations,
        set<int> & out_remaining_images,
        set<int> & out_recons_trackID,
        vector<Vec3b> & out_colors);


//main SfM and save structure to yml
//Maintain List:
//1.structure -- 3D Points
//2.correspondence -- [ImageID][FeatureID]-[3DPointIndex in structure]
//3.remaining_images -- ImageID
//4.reconsturctured_images -- ImageID
//5.out_rotations -- Mat
//6.out_translations -- Mat
void Main_SfM(Mat & in_K,MAP_IMGS & in_images,MAP_TRACKS & in_tracks,MAP_MATCHES & in_matches,
        MAP_KEYPOINTS & in_keypoints,
        MAP_COLORS & in_colors,
        map<int,Mat> out_rotations,
        map<int,Mat> out_translations);



#endif //INCREMENTAL_SFM_INCREMENTALSFM_H
