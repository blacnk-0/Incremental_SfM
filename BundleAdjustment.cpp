//
// Created by GING on 2018-12-19.
//

#include "BundleAdjustment.h"

//need update first camera
//morning change follow this log
//first camera may not be 0
//in_extrinsics is a map,you should not take it in a counter for loop

void BundleAdjustment(cv::Mat & in_intrinsic,
                      std::map<int,cv::Mat> & in_extrinsics,
                      MAP_POINT3D & in_map_point3d,
                      MAP_TRACKS & in_all_tracks,
                      MAP_KEYPOINTS & in_all_kps,
                      std::vector<cv::Point3d> & in_structure,
                      std::set<int> & in_reconstructed_imgs,
                      int firstImageID_int)
{
    ceres::Problem problem;

    //load extrinsics
    //rotation and translation
    for(const auto & extrinsic_imageID_pair:in_extrinsics)
    {
        cv::Mat extrinsic_Mat=extrinsic_imageID_pair.second;
        problem.AddParameterBlock(extrinsic_Mat.ptr<double>(),6);
    }

    //fix the first camera
    problem.SetParameterBlockConstant(in_extrinsics[firstImageID_int].ptr<double>());

    //load intrinsics
    //fx,fy,cx,cy
    problem.AddParameterBlock(in_intrinsic.ptr<double>(),4);

    //create loss function--huber loss
    ceres::LossFunction * huber_loss=new ceres::HuberLoss(4.0);

    //load data and parameters for BA
    for(std::vector<cv::Point3d>::size_type i=0;i<in_structure.size();++i)
    {
        //get track ID and extrinsic id
        int track_id=in_map_point3d[i];

        MAP_TRACK track=in_all_tracks[track_id];

        for(const auto img_feat_pair:track )
        {
            int img_id=img_feat_pair.first;
            int feat_id=img_feat_pair.second;

            //do BA only for reconstructed images
            if(!in_reconstructed_imgs.count(img_id))
            {
                continue;
            }

            //observed feature point in image
            cv::Point2d observed=in_all_kps[img_id][feat_id].pt;

            //Initial reprojection cost function
            ceres::CostFunction *cost_function=new ceres::AutoDiffCostFunction<ReprojectionCost,2,4,6,3>(new ReprojectionCost(observed));

            problem.AddResidualBlock(cost_function,huber_loss,
                    in_intrinsic.ptr<double>(),
                    in_extrinsics[img_id].ptr<double>(),
                    &(in_structure[i].x));
        }
    }

    //Set BA options
    ceres::Solver::Options ceres_config_options;
    ceres_config_options.minimizer_progress_to_stdout= false;
    ceres_config_options.logging_type=ceres::SILENT;
    ceres_config_options.num_threads=1;
    ceres_config_options.preconditioner_type=ceres::JACOBI;
    ceres_config_options.linear_solver_type=ceres::SPARSE_SCHUR;
    ceres_config_options.sparse_linear_algebra_library_type=ceres::EIGEN_SPARSE;

    //Solve BA
    ceres::Solver::Summary BA_Summary;
    ceres::Solve(ceres_config_options,&problem,&BA_Summary);

    if(!BA_Summary.IsSolutionUsable())
    {
        std::cout<<"Bundle Adjustment Failed in"<<in_extrinsics.size()<<" Round\n";
    } else{
        std::cout << std::endl
                  <<"Rount "<<in_extrinsics.size()
                  <<" Bundle Adjustment statistics (approximated RMSE):\n"
                  <<"#views: " << in_extrinsics.size() << "\n"
                  <<"#residuals: " << BA_Summary.num_residuals << "\n"
                  <<"Initial RMSE: " << std::sqrt(BA_Summary.initial_cost / BA_Summary.num_residuals) << "\n"
                  <<"Final RMSE: " << std::sqrt(BA_Summary.final_cost / BA_Summary.num_residuals) << "\n"
                  <<"Time (s): " << BA_Summary.total_time_in_seconds << "\n"
                  << std::endl;
    }


}

void Update_Intrinsic_Extrinsic(std::map<int,cv::Mat> & in_extrinsics_map,
                                cv::Mat & in_intrinsic_Mat,
                                cv::Mat & out_K,
                                std::map<int,cv::Mat> & out_rotations_map,
                                std::map<int,cv::Mat> & out_translations_map)
{
    //update fx , fy , cx , cy
    out_K.at<double>(0,0)=in_intrinsic_Mat.at<double>(0,0);
    out_K.at<double>(1,1)=in_intrinsic_Mat.at<double>(1,0);
    out_K.at<double>(0,2)=in_intrinsic_Mat.at<double>(2,0);
    out_K.at<double>(1,2)=in_intrinsic_Mat.at<double>(3,0);

    //update rotations and translations
    for(const auto & id_extrinsic_pair:in_extrinsics_map)
    {
        //get image ID
        int imgID_int=id_extrinsic_pair.first;

        //get extrinsic
        cv::Mat extrinsic=id_extrinsic_pair.second;

        //update rotation
        cv::Mat rotation_vector=extrinsic.rowRange(0,3);
        cv::Mat rotation_matrix;
        cv::Rodrigues(rotation_vector,rotation_matrix);
        out_rotations_map[imgID_int]=rotation_matrix;

        //update translation
        cv::Mat translation_vector=extrinsic.rowRange(3,6);
        out_translations_map[imgID_int]=translation_vector;
    }
}

