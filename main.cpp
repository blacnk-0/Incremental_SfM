#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <string>
#include <map>
#include <set>

#include "ComputeFeatures.h"
#include "Definitions.h"
#include "ComputeMatches.h"
#include "ComputeTracks.h"
#include "TrackHelper.h"
#include "IncrementalSfM.h"
#include "flat_pair_map.h"


using namespace std;
using namespace cv;

int main() {




//
//    int count1{1};
//    int count2{4};
//    double count1_div_count2=(double)count1/(double)count2;
//    cout<<count1_div_count2<<endl;

//    vector<int>::size_type s;
//    int i=1;
//    s=i;
//    cout<<s<<endl;

//    VEC_MATCHES ab,bc,ad,ac;
//    //initial ab
//    ab.push_back(make_pair(0,0));
//    ab.push_back(make_pair(1,1));
//    ab.push_back(make_pair(2,3));
//    //initial bc
//    bc.push_back(make_pair(0,0));
//    bc.push_back(make_pair(1,6));
//    bc.push_back(make_pair(3,2));
//    bc.push_back(make_pair(3,8));
//    bc.push_back(make_pair(5,6));
//    //initial ad
//    ad.push_back(make_pair(1,2));
//    ad.push_back(make_pair(1,3));
//    ad.push_back(make_pair(3,4));
//    //initial ac
//    ac.push_back(make_pair(1,5));
//    ac.push_back(make_pair(2,7));
//    ac.push_back(make_pair(5,9));
//    ac.push_back(make_pair(3,7));
//
//    MAP_MATCHES map_matches;
//    map_matches[make_pair(0,1)]=ab;
//    map_matches[make_pair(1,2)]=bc;
//    map_matches[make_pair(0,3)]=ad;
//    map_matches[make_pair(0,2)]=ac;
//
//    pair<int,int> out_images_id;
//    Choose_Initial_Pair(map_matches,out_images_id);
//    cout<<"Choosen image pair:"<<out_images_id.first<<"  "<<out_images_id.second<<endl;


//
//    MAP_TRACKS map_tracks;
//    UnionFind uf_tree;
//    flat_pair_map<pair<int,int> ,int> map_node_to_index;
//    Compute_Tracks(uf_tree,map_matches,map_node_to_index);
//    Filter_Tracks(uf_tree,map_node_to_index,map_tracks);
//
//    for(const auto & iter:uf_tree.m_cc_size)
//    {
//        cout<<iter<<endl;
//    }
//
//    cout<<endl;
//    for(const auto & track:map_tracks)
//    {
//        cout<<"track "<<track.first<<" :"<<endl;
//        for(const auto & node:track.second)
//        {
//            cout<<node.first<<"  "<<node.second<<endl;
//        }
//        cout<<endl;
//    }
//
//
//    MAP_TRACKS map_tracks1;
//    MAP_TRACK track0,track1,track2;
//    track0[3]=10,track0[1]=2,track0[2]=12; //track1
//    track1[3]=1,track1[2]=5;               //track2
//    track2[1]=34,track2[2]=17;             //track3
//
//    map_tracks1[0]=track0;
//    map_tracks1[1]=track1;
//    map_tracks1[2]=track2;
//
//    set<int> reconstructed_track_id;
//    reconstructed_track_id.insert(0);
//    reconstructed_track_id.insert(1);
//
//    set<int> remaining_image_id;
//    remaining_image_id.insert(3);
//    remaining_image_id.insert(1);
//    remaining_image_id.insert(2);
//
//    int out_image;
//    Find_Next_Image(remaining_image_id,reconstructed_track_id,map_tracks1,out_image);



    //
//    set<int> set_track_id;
//    int id=FindTrack_with_ImageIDandFeatID(0,2,map_tracks1);
//    cout<<id<<endl;


    vector<KeyPoint> kps;
    Mat desc;

    string img0="/Users/xujun/CLionProjects/Incremental_SfM/Images/0.jpg";
    string img1="/Users/xujun/CLionProjects/Incremental_SfM/Images/1.jpg";
    string img2="/Users/xujun/CLionProjects/Incremental_SfM/Images/2.jpg";

    Mat m_img0,m_img1,m_img2;
    m_img0=imread(img0);
    m_img1=imread(img1);
    m_img2=imread(img2);

    MAP_IMGS images;
    images[0]=img0;
    images[1]=img1;
    images[2]=img2;

    // 2881.252 ,    0     , 1416.0
    //    0     , 2881.252 , 1064.0
    //    0     ,    0     ,   1
    Mat K(Matx33d(
            2881.252 , 0 , 1416.0 ,
            0 , 2881.252 , 1064.0 ,
            0 , 0 , 1
            ));


    MAP_IMGS valid_images;
    MAP_KEYPOINTS all_kps;
    MAP_DESCS all_descs;

    MAP_MATCHES map_matches;
    map<pair<int,int>,vector<DMatch>> map_allMatches_DMatch;

    Compute_SIFT_Features_All(images,valid_images,all_kps,all_descs);

    Compute_Matches_All(K,all_descs,all_kps,map_matches,map_allMatches_DMatch);

    vector<KeyPoint> kps_one,kps_two;
    for(const auto & iter:all_kps[0])
    {
        kps_one.push_back(iter.second);
    }

    for(const auto & iter:all_kps[1])
    {
        kps_two.push_back(iter.second);
    }

    Mat out_image;
    drawMatches(m_img0,kps_one,m_img1,kps_two,map_allMatches_DMatch[make_pair(0,1)],out_image);
    imshow("o",out_image);
    waitKey();


//    cout<<all_kps.size()<<endl;
//
//    Mat gd_image=imread(img1);
//    Ptr<Feature2D> sift_detector=xfeatures2d::SIFT::create();
//
//    vector<KeyPoint> gd_kps;
//    Mat gd_desc;
//    sift_detector->detectAndCompute(gd_image,noArray(),gd_kps,gd_desc);
//
//    vector<KeyPoint> test_kps;
//    Mat test_desc=all_descs[2];
//    for(const auto & iter:all_kps[2])
//    {
//        test_kps.push_back(iter.second);
//    }
//
//    vector<DMatch> match;
//    Compute_Matches_Single(test_desc,gd_desc,match);
//
//    Mat imgMatch;
//    drawMatches(m_img1,test_kps,m_img1,gd_kps,match,imgMatch);
//    imshow("o",imgMatch);
//    waitKey();


//    string img0="/Users/xujun/CLionProjects/Incremental_SfM/Images/0.jpg";
//    string img1="/Users/xujun/CLionProjects/Incremental_SfM/Images/1.jpg";
//
//    Mat image=imread(img0);
//    Mat image1=imread(img1);
//    Ptr<Feature2D> sift_detector = xfeatures2d::SIFT::create();
//
//    vector<KeyPoint> kps,kps1;
//    Mat descs,desc1;
//    sift_detector->detectAndCompute(image,noArray(),kps,descs);
//    sift_detector->detectAndCompute(image1,noArray(),kps1,desc1);
//
//    vector<vector<DMatch>> knn_matches;
//    BFMatcher matcher(NORM_L2);
//
//    Mat d1=desc1.row(0);
//    Mat d2=descs.row(32820);
//
//    cout<<d1<<endl;
//    cout<<d2<<endl;
//
//    cout<<desc1.size()<<"  "<<descs.size()<<endl;
//    cout<<d1.size()<<"  "<<d2.size()<<endl;
//
//    cout<<kps1.size()<<endl;
//    cout<<kps.size()<<endl;
//
//
//    matcher.knnMatch(desc1,descs,knn_matches,2);




    return 0;
}