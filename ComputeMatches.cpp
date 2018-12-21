//
// Created by GING on 2018-12-18.
//
#include <iostream>

#include "ComputeMatches.h"

//Compute Single Matches
void Compute_Matches_Single(const cv::Mat & in_query,const cv::Mat & in_train, std::vector<cv::DMatch> & out_matches)
{
    std::vector<std::vector<cv::DMatch>> knn_matches;
    cv::BFMatcher bfMatcher(cv::NORM_L2);
    bfMatcher.knnMatch(in_query,in_train,knn_matches,2);

    float min_dist=FLT_MAX;
    for(int i=0;i<knn_matches.size();++i)
    {
        //ratio test
        if(knn_matches[i][0].distance>0.6*knn_matches[i][1].distance)
        {
            continue;
        }

        float temp_dist=knn_matches[i][0].distance;
        min_dist=min_dist<temp_dist?min_dist:temp_dist;
    }


    out_matches.clear();
    for(int i=0;i<knn_matches.size();++i)
    {
        if(knn_matches[i][0].distance > 0.6* knn_matches[i][1].distance || knn_matches[i][0].distance > 5* std::max(min_dist,15.0f))
        {
            continue;
        }

        out_matches.push_back(knn_matches[i][0]);
    }
}

//Filter Using Essential Matrix
bool Geometry_Verfication(const cv::Mat & in_K, std::vector<cv::Point2f> & in_p1,std::vector<cv::Point2f> & in_p2, cv::Mat & out_mask)
{
    double focal_length=0.5*(in_K.at<double>(0)+in_K.at<double>(4));
    cv::Point2f principle_point(in_K.at<double>(2),in_K.at<double>(5));

    cv::Mat E=cv::findEssentialMat(in_p1,in_p2,focal_length,principle_point,cv::RANSAC,0.999,1.0,out_mask);

    if(E.empty())
    {
        return false;
    }

    double inlier_count=countNonZero(out_mask);

    //too little inliers
    //inliers less than 0.5
    if(inlier_count<=15 || (inlier_count/double(in_p1.size())) <0.5)
    {
        return false;
    }

    return true;
}

//Compute All Matches including geometric filter
//out_MapAllMatches should be empty
void Compute_Matches_All(cv::Mat K,const MAP_DESCS & in_all_descs, MAP_KEYPOINTS & in_all_kps,MAP_MATCHES & out_MapAllMatches)
{
    for(const auto & iter:in_all_descs)
    {
        for(const auto & iter1:in_all_descs)
        {
            uint32_t d_ImageID1=iter.first;
            uint32_t d_ImageID2=iter1.first;

            //match pair < smaller_imageID , bigger_imageID >
            if(d_ImageID1>=d_ImageID2)
            {
                continue;
            }

            cv::Mat query=iter.second;
            cv::Mat train=iter1.second;

            std::vector<cv::DMatch> matches;
            Compute_Matches_Single(query,train,matches);

            MAP_KEYPOINT map_keypoint1,map_keypoint2;
            map_keypoint1=in_all_kps[d_ImageID1];
            map_keypoint2=in_all_kps[d_ImageID2];


            //matched feats
            //vec_feat no need
            std::vector<std::pair<int,cv::Point2f>> vec_feats1,vec_feats2;
            std::vector<cv::Point2f> vec_feats1_noID,vec_feats2_noID;
            for(int i=0;i<matches.size();++i)
            {
                vec_feats1_noID.push_back(map_keypoint1[matches[i].queryIdx].pt);
                vec_feats2_noID.push_back(map_keypoint2[matches[i].trainIdx].pt);
                vec_feats1.push_back(std::make_pair(matches[i].queryIdx , map_keypoint1[matches[i].queryIdx].pt));
                vec_feats2.push_back( std::make_pair( matches[i].trainIdx , map_keypoint2[matches[i].queryIdx].pt ) );
            }

            cv::Mat mask;
            if(!Geometry_Verfication(K,vec_feats1_noID,vec_feats2_noID,mask))
            {
                std::cout<<"Cannot estimate Essential Matrix between "<<d_ImageID1<< " and "<<d_ImageID2<<std::endl;
                continue;
            }

            VEC_MATCHES vec_matches;
            for(int i=0;i<mask.rows;++i)
            {
                if(mask.at<uchar>(i) >0 )
                {
                    vec_matches.push_back(std::make_pair(matches[i].queryIdx,matches[i].trainIdx));
                }
            }

            std::pair<int,int> pair_Img=std::make_pair(d_ImageID1,d_ImageID2);
            out_MapAllMatches[pair_Img]=vec_matches;

        }
    }
}


//for test
//output DMatch for test
void Compute_Matches_All(cv::Mat K,const MAP_DESCS & in_all_descs, MAP_KEYPOINTS & in_all_kps,MAP_MATCHES & out_MapAllMatches,
                         std::map<std::pair<int,int>,std::vector<cv::DMatch>> & out_MapAllMatches_DMatch)
{
    for(const auto & iter:in_all_descs)
    {
        for(const auto & iter1:in_all_descs)
        {
            uint32_t d_ImageID1=iter.first;
            uint32_t d_ImageID2=iter1.first;

            //match pair < smaller_imageID , bigger_imageID >
            if(d_ImageID1>=d_ImageID2)
            {
                continue;
            }

            cv::Mat query=iter.second;
            cv::Mat train=iter1.second;

            std::vector<cv::DMatch> matches;
            Compute_Matches_Single(query,train,matches);


            MAP_KEYPOINT map_keypoint1,map_keypoint2;
            map_keypoint1=in_all_kps[d_ImageID1];
            map_keypoint2=in_all_kps[d_ImageID2];


            //matched feats
            //vec_feat no need
            std::vector<std::pair<int,cv::Point2f>> vec_feats1,vec_feats2;
            std::vector<cv::Point2f> vec_feats1_noID,vec_feats2_noID;
            for(int i=0;i<matches.size();++i)
            {
                vec_feats1_noID.push_back(map_keypoint1[matches[i].queryIdx].pt);
                vec_feats2_noID.push_back(map_keypoint2[matches[i].trainIdx].pt);
                vec_feats1.push_back(std::make_pair(matches[i].queryIdx , map_keypoint1[matches[i].queryIdx].pt));
                vec_feats2.push_back( std::make_pair( matches[i].trainIdx , map_keypoint2[matches[i].queryIdx].pt ) );
            }

            cv::Mat mask;
            if(!Geometry_Verfication(K,vec_feats1_noID,vec_feats2_noID,mask))
            {
                std::cout<<"Cannot estimate Essential Matrix between "<<d_ImageID1<< " and "<<d_ImageID2<<std::endl;
                continue;
            }

            VEC_MATCHES vec_matches;
            for(int i=0;i<mask.rows;++i)
            {
                if(mask.at<uchar>(i) >0 )
                {
                    vec_matches.push_back(std::make_pair(matches[i].queryIdx,matches[i].trainIdx));
                }
            }

            std::pair<int,int> pair_Img=std::make_pair(d_ImageID1,d_ImageID2);
            out_MapAllMatches[pair_Img]=vec_matches;
            out_MapAllMatches_DMatch[pair_Img]=matches;

        }
    }
}
