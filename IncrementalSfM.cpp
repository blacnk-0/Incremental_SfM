//
// Created by GING on 2018-12-17.
//

#include "IncrementalSfM.h"
#include "BundleAdjustment.h"
#include <vector>


//choose initial pair with most matches
bool Choose_Initial_Pair(const MAP_MATCHES & in_map_matches,std::pair<int,int> & out_ImageID)
{
    std::vector<std::pair<int  , int >>::size_type max{0};
    std::pair<int,int> p_ImagePair;

    for(const auto & elem:in_map_matches)
    {
        if(elem.second.size()>max)
        {
            max=elem.second.size();
            p_ImagePair=elem.first;
        }
    }

    if(max<10)
    {
        return false;
    }

    out_ImageID=p_ImagePair;
    return true;
}

//Rescontruct Initial Pair
//initial out_structure
//initial out_rotation
//initial out_translation
void Reconstruct_Initial_Pair(
        std::pair<int,int> & in_initialPair,
        cv::Mat & in_K,cv::Mat & in_R,cv::Mat & in_T, std::vector<cv::Point2f> & in_p1,std::vector<cv::Point2f> & in_p2,
        VEC_MATCHES & in_initial_matches,
        MAP_TRACKS & in_all_tracks,
        std::vector<cv::Point3d> & out_structure,
        std::map<int,cv::Mat> & out_rotations,
        std::map<int,cv::Mat> & out_translations,
        MAP_POINT3D & out_point3d_correspondence,
        std::vector<std::vector<int>> & out_correspond_ImgID_FeatID_and_3DPt,
        std::vector<cv::Vec3b> & out_colors,
        MAP_COLORS & in_all_colors)
{
    //Projection Matrix [R|T] of the initial two cameras
    cv::Mat projection1(3,4,CV_32FC1);
    cv::Mat projection2(3,4,CV_32FC1);


    //first camera pose [I|0]
    projection1(cv::Range(0,3),cv::Range(0,3))=cv::Mat::eye(3,3,CV_32FC1);
    projection1.col(3)=cv::Mat::zeros(3,1,CV_32FC1);

    in_R.convertTo(projection2(cv::Range(0,3),cv::Range(0,3)),CV_32FC1);
    in_T.convertTo(projection2.col(3),CV_32FC1);

    cv::Mat R0=cv::Mat::eye(3,3,CV_64FC1);
    cv::Mat T0=cv::Mat::zeros(3,1,CV_64FC1);

    //store [ImageID, Matrix ]
    std::pair<int,cv::Mat> R0_pair=std::make_pair(in_initialPair.first,R0);
    std::pair<int,cv::Mat> T0_pair=std::make_pair(in_initialPair.first,T0);
    std::pair<int,cv::Mat> R1_pair=std::make_pair(in_initialPair.second,in_R);
    std::pair<int,cv::Mat> T1_pair=std::make_pair(in_initialPair.second,in_T);

    //Initial out_rotation and out_translation
    out_rotations={R0_pair,R1_pair};
    out_translations={T0_pair,T1_pair};


    cv::Mat intrinsic;
    in_K.convertTo(intrinsic,CV_32FC1);

    projection1=intrinsic*projection1;
    projection2=intrinsic*projection2;

    cv::Mat mat_structure;
    triangulatePoints(projection1,projection2,in_p1,in_p2,mat_structure);

    out_structure.clear();
    //Safe to convert ,mat_structure.cols is always positive
    std::vector<cv::Point3f>::size_type mat_structure_cols_size=mat_structure.cols;
    out_structure.reserve(mat_structure_cols_size);

    int first_image=in_initialPair.first;
    int second_image=in_initialPair.second;

    //Safe to use int, mat_structure.cols is type int
    //initial structure and [Point3D,TrackID] correspondence
    //count is the id of 3D point
    MAP_POINT3D ::size_type count{out_point3d_correspondence.size()}; //always initial count like this (0)
    for(int i=0;i<mat_structure.cols;++i)
    {
        //Only feature in track can do bundle adjustment
        if(FindTrack_with_ImageIDandFeatID(first_image,in_initial_matches[i].first,in_all_tracks)==-1)
        {
            continue;
        }

        //update correspondence and structure
        int trackID=FindTrack_with_ImageIDandFeatID(first_image,in_initial_matches[i].first,in_all_tracks);
        out_point3d_correspondence[count]=trackID;

        out_correspond_ImgID_FeatID_and_3DPt[first_image][in_initial_matches[i].first]=count;
        out_correspond_ImgID_FeatID_and_3DPt[second_image][in_initial_matches[i].second]=count;

        out_colors.push_back(in_all_colors[first_image][in_initial_matches[i].first]);

        cv::Mat_<float> col=mat_structure.col(i);
        col/=col(3);
        out_structure.push_back(cv::Point3f(col(0),col(1),col(2)));
        ++count;
    }
}

//First Camera [I|0] , Second Camera [R|T]
bool Find_Transform_Initial(cv::Mat & in_K,std::vector<cv::Point2f> &in_p1, std::vector<cv::Point2f> &in_p2, cv::Mat &out_R, cv::Mat &out_T) {
    double focal_length = 0.5 * (in_K.at<double>(0) + in_K.at<double>(4));
    cv::Point2f principle_point(in_K.at<double>(2), in_K.at<double>(5));

    cv::Mat mask;
    cv::Mat E = findEssentialMat(in_p1, in_p2, focal_length, principle_point, cv::RANSAC, 0.999, 1.0, mask);

    int count1 = countNonZero(mask);
    int count2 = recoverPose(E, in_p1, in_p2, out_R, out_T, focal_length, principle_point, mask);
    double count1_div_count2= (double)count1/(double)count2;
    //ratio of points in front of camera
    if ( count1_div_count2 < 0.7) {
        return false;
    }

    return true;
}

//Find Next Image Prepare for Next Round Reconstruction
//Select image has largest common tracks with current reconstructed tracks
//If there are multi image with the same number of common tracks, this function choose the smallest image id
bool Find_Next_Image(std::set<int> in_remaing_imageID,std::set<int> in_reconstructured_track_ID,MAP_TRACKS in_all_tracks,int & out_next_imageID)
{
    std::set<int>::size_type d_max{0};
    int d_imageID{-1};

    for(const int & imageID:in_remaing_imageID)
    {
        std::set<int> tracks_with_imageID;
        FindTrack_with_ImageID(imageID,in_all_tracks,tracks_with_imageID);

        std::set<int> intersection;
        std::set_intersection(in_reconstructured_track_ID.begin(),in_reconstructured_track_ID.end(),
                              tracks_with_imageID.begin(),tracks_with_imageID.end(),std::inserter(intersection,intersection.begin()));

        if(intersection.size()>d_max)
        {
            d_imageID=imageID;
            d_max=intersection.size();
        }
    }

    if(d_max<5)
    {
        return false;
    }

    out_next_imageID=d_imageID;
    return true;
}

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
        std::set<int> & in_reconstructed_images,
        MAP_TRACKS & in_all_tracks,
        MAP_MATCHES & in_matches,
        MAP_KEYPOINTS & in_keypoints,
        MAP_COLORS & in_all_colors,
        std::vector<std::vector<int>> & out_corresponds,
        std::vector<cv::Point3d> & out_structure,
        std::map<int,cv::Mat> & out_rotations,
        std::map<int,cv::Mat> & out_translations,
        std::set<int> & out_remaining_images,
        std::set<int> & out_recons_trackID,
        std::vector<cv::Vec3b> & out_colors,
        MAP_POINT3D & out_point3d_correspondence)
{
    //Find reconstructed image matches best with current process image
    std::pair<int,int> best_match_pair(-1,-1);
    VEC_MATCHES::size_type d_matchNumbers{0};
//    for(vector<int>::size_type i=0;i<in_reconstructed_images.size();++i)
      for(const auto & current_imageID:in_reconstructed_images)
      {
          //ensure I < J
          int I = std::min(in_ProcessImgID, current_imageID);
          int J = std::max(in_ProcessImgID, current_imageID);
          //ensure current_pair I < J
          std::pair<int, int> current_pair(I, J);
          const auto &matches = in_matches[current_pair];
          if (matches.size() > d_matchNumbers) {
              d_matchNumbers = matches.size();
              best_match_pair = current_pair;
          }
      }

    //Get Feature Points and Correspondence 3D Points
    //to solve PnP Problem
    VEC_MATCHES vec_matches=in_matches[best_match_pair];
    std::vector<cv::Point3f> points_3D;
    std::vector<cv::Point2f> featurePoints;
    //if 2D-3D correspondences is too little, may have problem to solve PnP Problem
    //if this happens, 2D-3D correspondences should get from all reconstructured images, use FindTrack_with_ImagePair
    //or brute force
    for(VEC_MATCHES::size_type i=0;i<vec_matches.size();++i)
    {
        //query is not in_ProcessingImgID
        //train is in_ProcessingImgID
        int queryID,trainID;
        int queryImgID,trainImgID;
        if(best_match_pair.first==in_ProcessImgID)
        {
            queryID=vec_matches[i].second;
            trainID=vec_matches[i].first;
            queryImgID=best_match_pair.second;
            trainImgID=best_match_pair.first;
        }
        else
        {
            queryID=vec_matches[i].first;
            trainID=vec_matches[i].second;
            queryImgID=best_match_pair.first;
            trainImgID=best_match_pair.second;
        }

        if(out_corresponds[queryImgID][queryID]!=-1)
        {
            //get 3D points from query
            int point_3d_index=out_corresponds[queryImgID][queryID];
            points_3D.push_back(out_structure[point_3d_index]);
            //get correspondence feature point from train
            featurePoints.push_back(in_keypoints[trainImgID][trainID].pt);
        }

    }

    //for test
    if(points_3D.size()<30)
    {
        std::cout<<"Not enough points to solve PnP problems.\n";
    }


    //may set a thresholf for the size of points_3D
    cv::Mat array_R;
    cv::Mat mat_R,mat_T;
    solvePnPRansac(points_3D,featurePoints,in_K,cv::noArray(),array_R,mat_T);

    Rodrigues(array_R,mat_R);

    out_rotations[in_ProcessImgID]=mat_R;
    out_translations[in_ProcessImgID]=mat_T;

    //find feature correspondence to triangulate
    //p1 is in_Process feat point
    std::vector<cv::Point2f> p1,p2;
    for(VEC_MATCHES::size_type i=0;i<vec_matches.size();++i)
    {
        if(best_match_pair.first==in_ProcessImgID)
        {
            p1.push_back(in_keypoints[best_match_pair.first][vec_matches[i].first].pt);
            p2.push_back(in_keypoints[best_match_pair.second][vec_matches[i].second].pt);
        }
        else
        {
            p1.push_back(in_keypoints[best_match_pair.second][vec_matches[i].second].pt);
            p2.push_back(in_keypoints[best_match_pair.first][vec_matches[i].first].pt);
        }
        //Colors should not be updated here
//        Vec3b color=in_all_colors[best_match_pair.first][vec_matches[i].first];
//        out_colors.push_back(color);
    }

    //projection1 is in_processing
    std::vector<cv::Point3f> new_structure;
    cv::Mat mat_new_structure;
    cv::Mat projection1(3,4,CV_32FC1);
    cv::Mat projection2(3,4,CV_32FC1);

    mat_R.convertTo(projection1(cv::Range(0, 3), cv::Range(0, 3)), CV_32FC1);
    mat_T.convertTo(projection1.col(3), CV_32FC1);

    int anotherImageID{0};
    if(best_match_pair.first==in_ProcessImgID)
    {
        anotherImageID=best_match_pair.second;
    } else{
        anotherImageID=best_match_pair.first;
    }

    cv::Mat anotherR,anotherT;
    anotherR=out_rotations[anotherImageID];
    anotherT=out_translations[anotherImageID];

    anotherR.convertTo(projection2(cv::Range(0, 3), cv::Range(0, 3)), CV_32FC1);
    anotherT.convertTo(projection2.col(3), CV_32FC1);

    cv::Mat intrinsic;
    in_K.convertTo(intrinsic,CV_32FC1);

    //multiply intrinsic matrix
    projection1=intrinsic*projection1;
    projection2=intrinsic*projection2;

    triangulatePoints(projection1,projection2,p1,p2,mat_new_structure);

    //convert mat structure to vector<Point3f>
    for(int i=0;i<mat_new_structure.cols;++i)
    {
        cv::Mat_<float> col = mat_new_structure.col(i);
        col /= col(3);
        new_structure.push_back(cv::Point3f(col(0), col(1), col(2)));
    }

    //update reconstructed images ,remaining images
    in_reconstructed_images.emplace(in_ProcessImgID);
    out_remaining_images.erase(in_ProcessImgID);

    //update structure and correspndence and track
    //update point3d correspondence and extrinsic correspondence

    for(VEC_MATCHES::size_type i=0;i<vec_matches.size();++i)
    {
        //query means already reconstructed
        int queryImgID,trainImgID;
        int queryFeatID,trainFeatID;
        if(best_match_pair.first==in_ProcessImgID)
        {
            queryImgID=best_match_pair.second;
            queryFeatID=vec_matches[i].second;
            trainImgID=best_match_pair.first;
            trainFeatID=vec_matches[i].first;
        } else{
            queryImgID=best_match_pair.first;
            queryFeatID=vec_matches[i].first;
            trainImgID=best_match_pair.second;
            trainFeatID=vec_matches[i].second;
        }

        //update structure and correspondence and tracks
        //3D point doesn't exist
        if(out_corresponds[queryImgID][queryFeatID]==-1)
        {
            //Find track use queryImgID and queryFeatID
            //Update all <imageID,featID> has been reconstructed or equals to trainImgID in track to correspondence list
            //if(new_trackID == -1) only for test

            int new_trackID=FindTrack_with_ImageIDandFeatID(queryImgID,queryFeatID,in_all_tracks);
            //for test
            if(new_trackID==-1)
            {
                std::cout<<"Missing hit in FindTrack_with_ImageIDandFeatID in Incremental_Process, unstable match found"<<std::endl;
                continue;
            }
            out_recons_trackID.emplace(new_trackID);

            //update structure and color
            //note:out_structure.size() should equal to out_point3d_correspondence.size() and out_extrinsic_correspondence.size()
            out_structure.push_back(new_structure[i]);
            out_colors.push_back(in_all_colors[queryImgID][queryFeatID]);

            out_point3d_correspondence[out_point3d_correspondence.size()]=new_trackID;

            for(const auto & img_feat_pair:in_all_tracks[new_trackID])
            {
                int imgID=img_feat_pair.first;
                int featID=img_feat_pair.second;

                //if image id is in_processing_image or already reconstructed
                if(imgID==trainImgID || in_reconstructed_images.count(imgID))
                {
                    out_corresponds[imgID][featID]=int(out_structure.size())-1;
                }

            }

//            //update out_structure
//            out_structure.push_back(new_structure[i]);
//            //update query correspondence
//            out_corresponds[queryImgID][queryFeatID]=int(out_structure.size())-1;
//            //update train correspondence
//            out_corresponds[trainImgID][trainFeatID]=int(out_structure.size())-1;
//
//            //may have problem
//            //update track
//            int new_trackID=FindTrack_with_ImageIDandFeatID(trainImgID,trainFeatID,in_all_tracks);
//            if(new_trackID==-1)
//            {
//                cout<<"Error in FindTrack_with_ImageIDandFeatID in Incremental_Process"<<endl;
//                return;
//            }
//            out_recons_trackID.emplace(new_trackID);
        } else{
            //update correspondence
            out_corresponds[trainImgID][trainFeatID]=out_corresponds[queryImgID][queryFeatID];
            //3D point already exists, no need to update track
        }
    }
}


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
              std::map<int,cv::Mat> out_translations)
{

    std::set<int> remaining_images;
    for(const auto & img:in_images)
    {
        remaining_images.emplace(img.first);
    }


    std::pair<int,int> initial_pair;
    //return initial_pair which I < J
    if(!Choose_Initial_Pair(in_matches,initial_pair))
    {
        std::cout<<"Choose Initial Pair Failed\n";
        return;
    }

    //make sure initial_Pair first < second
    int pair_first=initial_pair.first;
    int pair_second=initial_pair.second;
    if(pair_first>pair_second)
    {
        initial_pair.first=pair_second;
        initial_pair.second=pair_first;
    }

    std::set<int> reconstructed_imgs;
    reconstructed_imgs.emplace(initial_pair.first);
    reconstructed_imgs.emplace(initial_pair.second);

    //initial pair matches
    //make sure initial_pair first < second
    VEC_MATCHES initial_matches=in_matches[initial_pair];
    std::vector<cv::Point2f> vec_kpLocation1;
    std::vector<cv::Point2f> vec_kpLocation2;
    std::vector<cv::Vec3b> colors;

    if(initial_matches.empty())
    {
        std::cout<<"Initial_matches empty\n";
        return;
    }

    for(const auto & feat_pair:initial_matches)
    {

        int I=initial_pair.first;
        int J=initial_pair.second;
        int _i=feat_pair.first;
        int _j=feat_pair.second;
        vec_kpLocation1.push_back(in_keypoints[I][_i].pt);
        vec_kpLocation2.push_back(in_keypoints[J][_j].pt);
        colors.push_back(in_colors[I][_i]);
    }

    cv::Mat R;
    cv::Mat T;
    if(!Find_Transform_Initial(in_K,vec_kpLocation1,vec_kpLocation2,R,T))
    {
        std::cout<<"FInd Transform with initial pair Failed\n";
        return;
    }

    MAP_POINT3D map_point3D;


    //Correspondence between [ImageID,FeatureID] and 3D Point
    std::vector<std::vector<int>> correspond_ImgID_FeatID_and_3DPt;
    //Initialize
    correspond_ImgID_FeatID_and_3DPt.resize(in_images.size());
    for(std::vector<std::vector<int>>::size_type i=0;i<correspond_ImgID_FeatID_and_3DPt.size();++i)
    {
        //initialize to -1 which means no correspondence
        correspond_ImgID_FeatID_and_3DPt[i].resize(in_keypoints[i].size(),-1);
    }

    //Scene Structure
    std::vector<cv::Point3d> structure;
    Reconstruct_Initial_Pair(initial_pair,in_K,R,T,vec_kpLocation1,vec_kpLocation2,initial_matches,in_tracks,
            structure,out_rotations,out_translations,map_point3D,correspond_ImgID_FeatID_and_3DPt,colors,in_colors);

    //prepare for bundle adjustment
    cv::Mat intrinsic(cv::Matx41d(in_K.at<double>(0, 0), in_K.at<double>(1, 1), in_K.at<double>(0, 2), in_K.at<double>(1, 2)));
    std::map<int,cv::Mat> extrinsics;
    for(const auto & para:out_rotations)
    {
        cv::Mat extrinsic(6,1,CV_64FC1);
        cv::Mat rotation_compressed;

        int img_id=para.first;

        Rodrigues(para.second,rotation_compressed);

        rotation_compressed.copyTo(extrinsic.rowRange(0,3));
        out_translations[img_id].copyTo(extrinsic.rowRange(3,6));

        extrinsics[img_id]=extrinsic;
    }


    //do bundle adjustment
    BundleAdjustment(intrinsic,extrinsics,map_point3D,in_tracks,in_keypoints,structure,reconstructed_imgs);


    remaining_images.erase(initial_pair.first);
    remaining_images.erase(initial_pair.second);

    std::set<int> reconstructured_track_ID;
    FindTrack_with_ImagePair(initial_pair,in_tracks,reconstructured_track_ID);

    int d_NextImageID{-1};
    while(Find_Next_Image(remaining_images,reconstructured_track_ID,in_tracks,d_NextImageID))
    {
        Incremental_Process(in_K,
                d_NextImageID,
                reconstructed_imgs,
                in_tracks,
                in_matches,
                in_keypoints,
                in_colors,
                correspond_ImgID_FeatID_and_3DPt,
                structure,
                out_rotations,
                out_translations,
                remaining_images,
                reconstructured_track_ID,
                colors,
                map_point3D);

        //prepare for BA
        for(const auto & para:out_rotations)
        {
            cv::Mat extrinsic(6,1,CV_64FC1);
            cv::Mat rotation_compressed;
            Rodrigues(para.second,rotation_compressed);

            int img_id=para.first;

            rotation_compressed.copyTo(extrinsic.rowRange(0,3));
            out_translations[img_id].copyTo(extrinsic.rowRange(3,6));

            extrinsics[img_id]=extrinsic;
        }

        BundleAdjustment(intrinsic,extrinsics,map_point3D,in_tracks,in_keypoints,structure,reconstructed_imgs);
    }

    //save to file
    int n = (int)out_rotations.size();


    //write to yml file
    cv::FileStorage fs("structure.yml", cv::FileStorage::WRITE);
    fs << "Camera Count" << n;
    fs << "Point Count" << (int)structure.size();

    fs << "Rotations" << "[";
    for (size_t i = 0; i < n; ++i)
    {
        fs << out_rotations[i];
    }
    fs << "]";

    fs << "Motions" << "[";
    for (size_t i = 0; i < n; ++i)
    {
        fs << out_translations[i];
    }
    fs << "]";

    fs << "Points" << "[";
    for (size_t i = 0; i < structure.size(); ++i)
    {
        fs << structure[i];
    }
    fs << "]";

    fs << "Colors" << "[";
    for (size_t i = 0; i < colors.size(); ++i)
    {
        fs << colors[i];
    }
    fs << "]";

    fs.release();

    return;
}
