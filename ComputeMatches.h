//
// Created by GING on 2018-12-15.
//

#include <iterator>
#include <opencv2/calib3d/calib3d.hpp>
#include "Definitions.h"

#ifndef INCREMENTAL_SFM_COMPUTEMATCHES_H
#define INCREMENTAL_SFM_COMPUTEMATCHES_H


//Compute Single Matches
void Compute_Matches_Single(const Mat & in_query,const Mat & in_train, vector<DMatch> & out_matches);

//Filter Using Essential Matrix
bool Geometry_Verfication(const Mat & in_K, vector<Point2f> & in_p1,vector<Point2f> & in_p2, Mat & out_mask);


//Compute All Matches including geometric filter
//out_MapAllMatches should be empty
void Compute_Matches_All(Mat K,const MAP_DESCS & in_all_descs, MAP_KEYPOINTS & in_all_kps,MAP_MATCHES & out_MapAllMatches);

//for test
//output DMatch for test
void Compute_Matches_All(Mat K,const MAP_DESCS & in_all_descs, MAP_KEYPOINTS & in_all_kps,MAP_MATCHES & out_MapAllMatches,
        map<pair<int,int>,vector<DMatch>> & out_MapAllMatches_DMatch);


#endif //INCREMENTAL_SFM_COMPUTEMATCHES_H
