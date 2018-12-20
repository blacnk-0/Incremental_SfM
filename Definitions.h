//
// Created by GING on 2018-12-15.
//

#include <map>
#include <string>
#include <vector>
#include <opencv2/features2d/features2d.hpp>


#ifndef INCREMENTAL_SFM_DEFINITIONS_H
#define INCREMENTAL_SFM_DEFINITIONS_H

// MAP_IMGS = < ImageID, ImagePath >
using MAP_IMGS=std::map<int ,std::string>;

//MAP_KEYPOINT = <Feature ID , Feature>
using MAP_KEYPOINT=std::map<int ,cv::KeyPoint>;

//MAP_KEYPOINTS= <Image ID, KeyPoints>
using MAP_KEYPOINTS=std::map<int ,MAP_KEYPOINT >;

//MAP LOC----no need to use
using MAP_KPLOCATION=std::map<int , cv::Point2f>;

//MAP LOCS=map imageID to kpLoc----no need to use
using MAP_KPLOCATIONS=std::map<int,MAP_KPLOCATION >;

//Matrix Descriptor
using MAT_DESC=cv::Mat;

//MAP <ImageID,Descriptors>
using MAP_DESCS=std::map< int,MAT_DESC >;


//VECTOR MATCHES= vector< pair<Feature ID, Feature ID> >
using VEC_MATCHES=std::vector<std::pair<int  , int > >;

//MAP MATCHES =map< pair<Image ID, Image ID> , VEC_MATCHES >
using MAP_MATCHES = std::map< std::pair<int,int> , VEC_MATCHES >;

//MAP <Image ID, Feat ID>
using MAP_TRACK=std::map<int,int>;

//MAP <TrackID, Track>
using MAP_TRACKS = std::map<int,MAP_TRACK> ;

//MAP <FeattureID,Color>
using MAP_COLOR=std::map<int,cv::Vec3b>;

//MAP <ImageID,Color>
using MAP_COLORS=std::map<int,MAP_COLOR>;

//MAP <structure 3D Point Index,track ID>
using MAP_POINT3D=std::map<int,int>;

//MAP <structure 3D Point Index,extrinsic ID>
using MAP_EXTRINSIC=std::map<int,int>;

#endif //INCREMENTAL_SFM_DEFINITIONS_H
