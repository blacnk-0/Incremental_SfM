#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <string>
#include <map>
#include <set>

//#include "ComputeFeatures.h"
//#include "Definitions.h"
//#include "ComputeMatches.h"
//#include "ComputeTracks.h"
//#include "TrackHelper.h"
//#include "IncrementalSfM.h"
//#include "flat_pair_map.h"


using namespace std;
using namespace cv;

int main() {

//    vector<KeyPoint> kps;
//    Mat desc;
//
//    string img0="/Users/xujun/CLionProjects/Incremental_SfM/Images/0.jpg";
//    Compute_SIFT_Feature_Single(img0,kps,desc);
//
//    Mat img=imread(img0);
//    Mat res;
//    drawKeypoints(img,kps,res);
//    imshow("p",res);
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


    set<int> test;
    test.emplace(1);
    test.emplace(2);

    set<int> test2;
    test2.emplace(1);
    set<int> test3;

    set_intersection(test.begin(),test.end(),test2.begin(),test2.end(),inserter(test3,test3.begin()));

    cout<<test3.count(1)<<endl;


    return 0;
}