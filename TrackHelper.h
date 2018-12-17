//
// Created by GING on 2018-12-16.
//

#include <set>
#include <algorithm>
#include "Definitions.h"

using namespace std;

#ifndef INCREMENTAL_SFM_TRACKHELPER_H
#define INCREMENTAL_SFM_TRACKHELPER_H

void FindTrack_with_ImageID(const int & in_ImageID,const MAP_TRACKS & in_all_tracks,set<int> & out_trackID);

void FindTrack_with_ImagePair(const pair<int,int> & in_imagePair,const MAP_TRACKS & in_all_tracks,set<int> & out_trackID);

//only find one possible track
int FindTrack_with_ImageIDandFeatID(const int & in_imgID,const int & in_featID,const MAP_TRACKS & in_all_tracks);

#endif //INCREMENTAL_SFM_TRACKHELPER_H
