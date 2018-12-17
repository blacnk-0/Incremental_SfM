//
// Created by GING on 2018-12-15.
//

#include <map>
#include <string>
#include <vector>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

#ifndef INCREMENTAL_SFM_DEFINITIONS_H
#define INCREMENTAL_SFM_DEFINITIONS_H

// MAP_IMGS = < ImageID, ImagePath >
using MAP_IMGS=map<int ,string>;

//MAP_KEYPOINT = <Feature ID , Feature>
using MAP_KEYPOINT=map<int ,KeyPoint>;

//MAP_KEYPOINTS= <Image ID, KeyPoints>
using MAP_KEYPOINTS=map<int ,MAP_KEYPOINT >;

//MAP LOC
using MAP_KPLOCATION=map<int , Point2f>;

//MAP LOCS=map imageID to kpLoc
using MAP_KPLOCATIONS=map<int,MAP_KPLOCATION >;

//
using MAT_DESC=Mat;

//
using MAP_DESCS=map< int,MAT_DESC >;


//VECTOR MATCHES= vector< pair<Feature ID, Feature ID> >
using VEC_MATCHES=vector<pair<int  , int > >;

//MAP MATCHES =map< pair<Image ID, Image ID> , VEC_MATCHES >
using MAP_MATCHES = map< pair<int,int> , VEC_MATCHES >;

//MAP <Image ID, Feat ID>
using MAP_TRACK=map<int,int>;

//MAP <TrackID, Track>
using MAP_TRACKS = map<int,MAP_TRACK> ;

#endif //INCREMENTAL_SFM_DEFINITIONS_H
