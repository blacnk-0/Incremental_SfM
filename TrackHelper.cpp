//
// Created by GING on 2018-12-17.
//

#include "TrackHelper.h"


void FindTrack_with_ImageID(const int & in_ImageID,const MAP_TRACKS & in_all_tracks,set<int> & out_trackID)
{
    for(const auto & track:in_all_tracks)
    {
        if(track.second.count(in_ImageID)!=0)
        {
            int d_TrackID=track.first;
            out_trackID.emplace(d_TrackID);
        }
    }
}

void FindTrack_with_ImagePair(const pair<int,int> & in_imagePair,const MAP_TRACKS & in_all_tracks,set<int> & out_trackID)
{
    int I=in_imagePair.first;
    int J=in_imagePair.second;

    set<int> tracks_I,tracks_J;
    FindTrack_with_ImageID(I,in_all_tracks,tracks_I);
    FindTrack_with_ImageID(J,in_all_tracks,tracks_J);

    std::set_intersection(tracks_I.begin(),tracks_I.end(),tracks_J.begin(),tracks_J.end(),inserter(out_trackID,out_trackID.begin()));
}

//only find one possible track
int FindTrack_with_ImageIDandFeatID(const int & in_imgID,const int & in_featID,const MAP_TRACKS & in_all_tracks)
{
    set<int> track_with_image;

    bool res{false};
    int out_trackID{-1};
    for(const auto & track:in_all_tracks)
    {
        if(track.second.count(in_imgID)!=0)
        {
            for(const auto & node : track.second)
            {
                if(node.second==in_featID)
                {
                    out_trackID=track.first;
                    res=true;
                }
                if(res)
                {
                    return out_trackID;
                }
            }
        }
    }

    return out_trackID;
}

