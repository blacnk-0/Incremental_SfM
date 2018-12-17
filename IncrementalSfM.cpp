//
// Created by GING on 2018-12-17.
//

#include "IncrementalSfM.h"


//choose initial pair with most matches
bool Choose_Initial_Pair(const MAP_MATCHES & in_map_matches,pair<int,int> & out_ImageID)
{
    int max{-1};
    pair<int,int> p_ImagePair;

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
        pair<int,int> & in_initialPair,
        Mat & in_K,Mat & in_R,Mat & in_T, vector<Point2f> & in_p1,vector<Point2f> & in_p2,vector<Point3f> & out_structure,
        map<int,Mat> & out_rotations,
        map<int,Mat> & out_translations)
{
    //Projection Matrix [R|T] of the initial two cameras
    Mat projection1(3,4,CV_32FC1);
    Mat projection2(3,4,CV_32FC1);


    //first camera pose [I|0]
    projection1(Range(0,3),Range(0,3))=Mat::eye(3,3,CV_32FC1);
    projection1.col(3)=Mat::zeros(3,1,CV_32FC1);

    in_R.convertTo(projection2(Range(0,3),Range(0,3)),CV_32FC1);
    in_T.convertTo(projection2.col(3),CV_32FC1);

    Mat R0=Mat::eye(3,3,CV_64FC1);
    Mat T0=Mat::zeros(3,1,CV_64FC1);

    //store [ImageID, Matrix ]
    pair<int,Mat> R0_pair=make_pair(in_initialPair.first,R0);
    pair<int,Mat> T0_pair=make_pair(in_initialPair.first,T0);
    pair<int,Mat> R1_pair=make_pair(in_initialPair.second,in_R);
    pair<int,Mat> T1_pair=make_pair(in_initialPair.second,in_T);

    out_rotations={R0_pair,R1_pair};
    out_translations={T0_pair,T1_pair};

    Mat intrinsic;
    in_K.convertTo(intrinsic,CV_32FC1);

    projection1=intrinsic*projection1;
    projection2=intrinsic*projection2;

    Mat mat_structure;
    triangulatePoints(projection1,projection2,in_p1,in_p2,mat_structure);

    out_structure.clear();
    out_structure.reserve(mat_structure.cols);

    for(int i=0;i<mat_structure.cols;++i)
    {
        Mat_<float> col=mat_structure.col(i);
        col/=col(3);
        out_structure.push_back(Point3f(col(0),col(1),col(2)));
    }
}

//First Camera [I|0] , Second Camera [R|T]
bool Find_Transform_Initial(Mat & in_K,vector<Point2f> &in_p1, vector<Point2f> &in_p2, Mat &out_R, Mat &out_T) {
    double focal_length = 0.5 * (in_K.at<double>(0) + in_K.at<double>(4));
    Point2f principle_point(in_K.at<double>(2), in_K.at<double>(5));

    Mat mask;
    Mat E = findEssentialMat(in_p1, in_p2, focal_length, principle_point, RANSAC, 0.999, 1.0, mask);

    int count1 = countNonZero(mask);
    int count2 = recoverPose(E, in_p1, in_p2, out_R, out_T, focal_length, principle_point, mask);
    if ((double) count2 / (double) count1 < 0.7) {
        return false;
    }

    return true;
}

//Find Next Image Prepare for Next Round Reconstruction
bool Find_Next_Image(set<int> in_remaing_imageID,set<int> in_reconstructured_track_ID,MAP_TRACKS in_all_tracks,int & out_next_imageID)
{
    int d_max{-1};
    int d_imageID{-1};

    for(const int & imageID:in_remaing_imageID)
    {
        set<int> tracks_with_imageID;
        FindTrack_with_ImageID(imageID,in_all_tracks,tracks_with_imageID);

        set<int> intersection;
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
        Mat & in_K,
        int in_ProcessImgID,
        vector<int> & in_reconstructed_images,
        const MAP_TRACKS & in_all_tracks,
        MAP_MATCHES & in_matches,
        MAP_KEYPOINTS & in_keypoints,
        vector<vector<int>> & out_corresponds,
        vector<Point3f> & out_structure,
        map<int,Mat> & out_rotations,
        map<int,Mat> & out_translations,
        set<int> & out_remaining_images,
        set<int> & out_recons_trackID)
{
    //Find reconstructed image matches best with current process image
    pair<int,int> best_match_pair(-1,-1);
    int d_matchNumbers{-1};
    for(int i=0;i<in_reconstructed_images.size();++i)
    {
        //ensure I < J
        int I=std::min(in_ProcessImgID,in_reconstructed_images[i]);
        int J=std::max(in_ProcessImgID,in_reconstructed_images[i]);
        //ensure current_pair I < J
        pair<int,int> current_pair(I,J);
        const auto & matches=in_matches[current_pair];
        if(matches.size()>d_matchNumbers)
        {
            d_matchNumbers=matches.size();
            best_match_pair=current_pair;
        }
    }

    //Get Feature Points and Correspondence 3D Points
    //to solve PnP Problem
    VEC_MATCHES vec_matches=in_matches[best_match_pair];
    vector<Point3f> points_3D;
    vector<Point2f> featurePoints;
    for(int i=0;i<vec_matches.size();++i)
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
            int point_3d_index=out_corresponds[queryImgID][queryID];
            points_3D.push_back(out_structure[point_3d_index]);
            featurePoints.push_back(in_keypoints[trainImgID][trainID].pt);
        }

    }

    Mat array_R;
    Mat mat_R,mat_T;
    solvePnPRansac(points_3D,featurePoints,in_K,noArray(),array_R,mat_T);

    Rodrigues(array_R,mat_R);

    out_rotations[in_ProcessImgID]=mat_R;
    out_translations[in_ProcessImgID]=mat_T;

    //find feature correspondence to triangulate
    //p1 is in_Process feat point
    vector<Point2f> p1,p2;
    for(int i=0;i<vec_matches.size();++i)
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
    }

    //projection1 is in_processing
    vector<Point3f> new_structure;
    Mat mat_new_structure;
    Mat projection1(3,4,CV_32FC1);
    Mat projection2(3,4,CV_32FC1);

    mat_R.convertTo(projection1(Range(0, 3), Range(0, 3)), CV_32FC1);
    mat_T.convertTo(projection1.col(3), CV_32FC1);

    int anotherImageID{0};
    if(best_match_pair.first==in_ProcessImgID)
    {
        anotherImageID=best_match_pair.second;
    } else{
        anotherImageID=best_match_pair.first;
    }

    Mat anotherR,anotherT;
    anotherR=out_rotations[anotherImageID];
    anotherT=out_translations[anotherImageID];

    anotherR.convertTo(projection2(Range(0, 3), Range(0, 3)), CV_32FC1);
    anotherT.convertTo(projection2.col(3), CV_32FC1);

    triangulatePoints(projection1,projection2,p1,p2,mat_new_structure);

    //convert mat structure to vector<Point3f>
    for(int i=0;i<mat_new_structure.cols;++i)
    {
        Mat_<float> col = mat_new_structure.col(i);
        col /= col(3);
        new_structure.push_back(Point3f(col(0), col(1), col(2)));
    }

    //update reconstructed images ,remaining images
    in_reconstructed_images.push_back(in_ProcessImgID);
    out_remaining_images.erase(in_ProcessImgID);

    //update structure and correspndence and track
    for(int i=0;i<vec_matches.size();++i)
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
        if(out_corresponds[queryImgID][queryFeatID]!=-1)
        {
            //update out_structure
            out_structure.push_back(new_structure[i]);
            //update query correspondence
            out_corresponds[queryImgID][queryFeatID]=out_structure.size()-1;
            //update train correspondence
            out_corresponds[trainImgID][trainFeatID]=out_structure.size()-1;

            //may have problem
            int new_trackID=FindTrack_with_ImageIDandFeatID(trainImgID,trainFeatID,in_all_tracks);
            if(new_trackID==-1)
            {
                cout<<"Error in FindTrack_with_ImageIDandFeatID in Incremental_Process"<<endl;
                return;
            }
            out_recons_trackID.emplace(new_trackID);
        } else{
            //update correspondence
            out_corresponds[trainImgID][trainFeatID]=out_corresponds[queryImgID][queryFeatID];
            //no need to update track
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
void Main_SfM(Mat & in_K,MAP_IMGS & in_images,MAP_TRACKS & in_tracks,MAP_MATCHES & in_matches,MAP_KEYPOINTS & in_keypoints,
              map<int,Mat> out_rotations,
              map<int,Mat> out_translations)
{

    set<int> remaining_images;
    for(const auto & img:in_images)
    {
        remaining_images.emplace(img.first);
    }

    //problem !!
    pair<int,int> initial_pair;
    if(!Choose_Initial_Pair(in_matches,initial_pair))
    {
        cout<<"Choose Initial Pair Failed\n";
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

    vector<int> reconstructed_imgs;
    reconstructed_imgs.push_back(initial_pair.first);
    reconstructed_imgs.push_back(initial_pair.second);

    //initial pair matches
    //make sure initial_pair first < second
    VEC_MATCHES initial_matches=in_matches[initial_pair];
    vector<Point2f> vec_kpLocation1;
    vector<Point2f> vec_kpLocation2;

    if(initial_matches.empty())
    {
        cout<<"Initial_matches empty\n";
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
    }

    Mat R;
    Mat T;
    if(!Find_Transform_Initial(in_K,vec_kpLocation1,vec_kpLocation2,R,T))
    {
        cout<<"FInd Transform with initial pair Failed\n";
        return;
    }

    //Scene Structure
    vector<Point3f> structure;
    Reconstruct_Initial_Pair(initial_pair,in_K,R,T,vec_kpLocation1,vec_kpLocation2,structure,out_rotations,out_translations);

    //Correspondence between [ImageID,FeatureID] and 3D Point
    vector<vector<int>> correspond_ImgID_FeatID_and_3DPt;
    //Initialize
    correspond_ImgID_FeatID_and_3DPt.resize(in_images.size());
    for(int i=0;i<correspond_ImgID_FeatID_and_3DPt.size();++i)
    {
        correspond_ImgID_FeatID_and_3DPt[i].resize(in_keypoints[i].size(),-1);
    }

    int count{0};
    for(const auto & feat_pair:initial_matches)
    {
        int I=initial_pair.first;
        int J=initial_pair.second;
        int _i=feat_pair.first;
        int _j=feat_pair.second;

        correspond_ImgID_FeatID_and_3DPt[I][_i]=count;
        correspond_ImgID_FeatID_and_3DPt[J][_j]=count;
        ++count;
    }

    remaining_images.erase(initial_pair.first);
    remaining_images.erase(initial_pair.second);

    set<int> reconstructured_track_ID;
    FindTrack_with_ImagePair(initial_pair,in_tracks,reconstructured_track_ID);

    int d_NextImageID{-1};
    while(Find_Next_Image(remaining_images,reconstructured_track_ID,in_tracks,d_NextImageID))
    {
        Incremental_Process(in_K,d_NextImageID,reconstructed_imgs,in_tracks,in_matches,in_keypoints,correspond_ImgID_FeatID_and_3DPt,structure,
                            out_rotations,out_translations,remaining_images,reconstructured_track_ID);
    }

    //save to file
    int n = (int)out_rotations.size();


    //write to yml file
    FileStorage fs("3Dstructure.yml", FileStorage::WRITE);
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

//    fs << "Colors" << "[";
//    for (size_t i = 0; i < colors.size(); ++i)
//    {
//        fs << colors[i];
//    }
//    fs << "]";

    fs.release();

    return;
}
