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

#include "StringSplit.h"
#include "tinydir.h"




int main(int argc,char ** argv) {


    std::string s_images_dir;
    s_images_dir="/Users/xujun/CLionProjects/Incremental_SfM/Images";

    std::vector<std::string> vec_files_name;
    tinydir_dir dir;
    tinydir_open(&dir,s_images_dir.c_str());

    while (dir.has_next)
    {
        tinydir_file file;
        tinydir_readfile(&dir, &file);
        if (!file.is_dir)
        {
            vec_files_name.push_back(file.path);
        }
        tinydir_next(&dir);
    }
    tinydir_close(&dir);

    //Filter files
    std::vector<std::string> vec_images_name;
    for(const auto & file_name:vec_files_name)
    {
        std::vector<std::string> file_name_after_split=split_by_character(file_name,'.');

        std::string s_ext_name=file_name_after_split.back();
        if(s_ext_name=="jpg" || s_ext_name=="JPG" || s_ext_name=="jpeg" || s_ext_name=="png" )
        {
            vec_images_name.push_back(file_name);
        }
    }

    MAP_IMGS images;

    // 2881.252 ,    0     , 1416.0
    //    0     , 2881.252 , 1064.0
    //    0     ,    0     ,   1
    cv::Mat K(cv::Matx33d(
            2881.252 , 0 , 1416.0 ,
            0 , 2881.252 , 1064.0 ,
            0 , 0 , 1
            ));

    MAP_MATCHES out;
    MAP_DESCS map_descs;
    MAP_KEYPOINTS map_keypoints;

    Compute_Matches_All(K,map_descs,map_keypoints,out);

    std::cout<<"out size:"<<out.size()<<std::endl;
    for(const auto matches:out)
    {
        std::cout<<matches.second.size()<<std::endl;
    }


    MAP_IMGS valid_images;
    MAP_KEYPOINTS all_kps;
    MAP_DESCS all_descs;

    MAP_MATCHES map_matches;
    MAP_COLORS map_colors;

    Compute_SIFT_Features_All_Colors(images,valid_images,all_kps,all_descs,map_colors);

    Compute_Matches_All(K,all_descs,all_kps,map_matches);


    UnionFind uf_tree;
    flat_pair_map<std::pair<int,int>,int> map_node_to_index;
    MAP_TRACKS map_tracks;
    Compute_Tracks(uf_tree,map_matches,map_node_to_index);
    Filter_Tracks(uf_tree,map_node_to_index,map_tracks);

    std::map<int,cv::Mat> out_rotation,out_translation;
    Main_SfM(K,valid_images,map_tracks,map_matches,all_kps,map_colors,out_rotation,out_translation);


    return 0;
}