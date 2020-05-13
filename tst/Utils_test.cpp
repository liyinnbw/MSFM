#include "gtest/gtest.h"
#include <core/Utils.h>
#include <core/SFMPipeline.h>
#include <core/DetectorExtractor.h>
#include "TestGlobals.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;
TEST(Utils, KeyPoints2Points) {
    SFMPipeline sfm("", vector<string>());
    Mat img1 = imread(programExecutionPath+"/data/11.JPG",cv::IMREAD_GRAYSCALE);
    vector<KeyPoint> kpts1;
    vector<Point2f> pts1;
	DetectorExtractor &deAlgo = DetectorExtractor::GetInstance();
	deAlgo.detectORB(img1, kpts1);
    Utils::KeyPoints2Points(kpts1, pts1);

    // test
    EXPECT_EQ(pts1.size(), kpts1.size());
    bool allequal = true;
    for(size_t i=0; i<pts1.size(); i++){
        if(pts1[i]!=kpts1[i].pt){
            allequal=false;
            break;
        }
    }
    EXPECT_EQ(allequal,true);
}

TEST(Utils, Matches2Points) {
    SFMPipeline sfm("", vector<string>());
    Mat img1 = imread(programExecutionPath+"/data/11.JPG",cv::IMREAD_GRAYSCALE);
    Mat img2 = imread(programExecutionPath+"/data/16.JPG",cv::IMREAD_GRAYSCALE);

    vector<KeyPoint> kpts1, kpts2;
    vector<Point2f> pts1, pts2;
    Mat decs1,decs2;
	DetectorExtractor &deAlgo = DetectorExtractor::GetInstance();
	deAlgo.detectORB(img1, kpts1);
    deAlgo.detectORB(img2, kpts2);
    deAlgo.computeORB(img1, kpts1, decs1);
    deAlgo.computeORB(img2, kpts2, decs2);
    vector<DMatch> matches12;
    sfm.matchFeatures(decs1, decs2, matches12);

    // Utils::KeyPoints2Points(kpts1, pts1);
    // Utils::KeyPoints2Points(kpts2, pts2);

    Utils::Matches2Points(kpts1, kpts2, matches12, pts1, pts2);
    
    // test
    EXPECT_EQ(pts1.size(), matches12.size());
    EXPECT_EQ(pts2.size(), matches12.size());
    bool allequal = true;
    for(size_t i=0; i<pts1.size(); i++){
        if(pts1[i]!=kpts1[matches12[i].queryIdx].pt){
            allequal=false;
            break;
        }
        if(pts2[i]!=kpts2[matches12[i].trainIdx].pt){
            allequal=false;
            break;
        }
    }
    EXPECT_EQ(allequal,true);


    vector<Point2f> ptso1, ptso2;
    Utils::KeyPoints2Points(kpts1, ptso1);
    Utils::KeyPoints2Points(kpts2, ptso2);

    Utils::Matches2Points(ptso1, ptso2, matches12, pts1, pts2);
    // test
    EXPECT_EQ(pts1.size(), matches12.size());
    EXPECT_EQ(pts2.size(), matches12.size());
    allequal = true;
    for(size_t i=0; i<pts1.size(); i++){
        if(pts1[i]!=kpts1[matches12[i].queryIdx].pt){
            allequal=false;
            break;
        }
        if(pts2[i]!=kpts2[matches12[i].trainIdx].pt){
            allequal=false;
            break;
        }
    }
    EXPECT_EQ(allequal,true);

}

TEST(Utils, Matches2Indices) {
    SFMPipeline sfm("", vector<string>());
    Mat img1 = imread(programExecutionPath+"/data/11.JPG",cv::IMREAD_GRAYSCALE);
    Mat img2 = imread(programExecutionPath+"/data/16.JPG",cv::IMREAD_GRAYSCALE);

    vector<KeyPoint> kpts1, kpts2;
    Mat decs1,decs2;
	DetectorExtractor &deAlgo = DetectorExtractor::GetInstance();
	deAlgo.detectORB(img1, kpts1);
    deAlgo.detectORB(img2, kpts2);
    deAlgo.computeORB(img1, kpts1, decs1);
    deAlgo.computeORB(img2, kpts2, decs2);
    vector<DMatch> matches12;
    sfm.matchFeatures(decs1, decs2, matches12);

    vector<int> idxs1, idxs2;
    Utils::Matches2Indices(matches12, idxs1, idxs2);

    //test
    EXPECT_EQ(idxs1.size(), matches12.size());
    EXPECT_EQ(idxs2.size(), matches12.size());
    bool allequal = true;
    for(size_t i=0; i<matches12.size(); i++){
        if(idxs1[i]!=matches12[i].queryIdx){
            allequal=false;
            break;
        }
        if(idxs2[i]!=matches12[i].trainIdx){
            allequal=false;
            break;
        }
    }
    EXPECT_EQ(allequal,true);
}

TEST(Utils, transformPoints) {
    vector<Point3f> xyzs = {Point3f(0,0,0), Point3f(1,0,0), Point3f(0,1,0), Point3f(0,0,1)};
    vector<Point3f> result = {Point3f(1,1,1), Point3f(2,1,1), Point3f(1,2,1), Point3f(1,1,2)};
    Mat transfMat = Mat::zeros (3, 4, CV_64F);
    transfMat.at<double>(0,0) = transfMat.at<double>(1,1) = transfMat.at<double>(2,2) = 1;
    transfMat.at<double>(0,3) = transfMat.at<double>(1,3) = transfMat.at<double>(2,3) = 1;
    cout<<transfMat<<endl;
    Utils::transformPoints(transfMat, xyzs);
    EXPECT_EQ(result, xyzs);

    xyzs = {Point3f(0,0,0), Point3f(1,0,0), Point3f(0,1,0), Point3f(0,0,1)};
    result = {Point3f(1,1,1), Point3f(1,2,1), Point3f(0,1,1), Point3f(1,1,2)};
    transfMat = Mat::zeros (3, 4, CV_64F);
    transfMat.at<double>(0,3) = transfMat.at<double>(1,3) = transfMat.at<double>(2,3) = 1;
    transfMat.at<double>(0,0) = transfMat.at<double>(1,1) = 0;
    transfMat.at<double>(2,2) = 1;
    transfMat.at<double>(0,1) = -1;
    transfMat.at<double>(1,0) = 1;
    cout<<transfMat<<endl;
    Utils::transformPoints(transfMat, xyzs);
    EXPECT_EQ(result, xyzs);

    xyzs = {Point3f(0,0,0), Point3f(1,0,0), Point3f(0,1,0), Point3f(0,0,1)};
    result = {Point3f(1,1,1), Point3f(1,1,0), Point3f(1,2,1), Point3f(2,1,1)};
    transfMat = Mat::zeros (3, 4, CV_64F);
    transfMat.at<double>(0,3) = transfMat.at<double>(1,3) = transfMat.at<double>(2,3) = 1;
    transfMat.at<double>(0,0) = transfMat.at<double>(2,2) = 0;
    transfMat.at<double>(1,1) = 1;
    transfMat.at<double>(0,2) = 1;
    transfMat.at<double>(2,0) = -1;
    cout<<transfMat<<endl;
    Utils::transformPoints(transfMat, xyzs);
    EXPECT_EQ(result, xyzs);

    xyzs = {Point3f(0,0,0), Point3f(1,0,0), Point3f(0,1,0), Point3f(0,0,1)};
    result = {Point3f(1,1,1), Point3f(2,1,1), Point3f(1,1,2), Point3f(1,0,1)};
    transfMat = Mat::zeros (3, 4, CV_64F);
    transfMat.at<double>(0,3) = transfMat.at<double>(1,3) = transfMat.at<double>(2,3) = 1;
    transfMat.at<double>(1,1) = transfMat.at<double>(2,2) = 0;
    transfMat.at<double>(0,0) = 1;
    transfMat.at<double>(1,2) = -1;
    transfMat.at<double>(2,1) = 1;
    cout<<transfMat<<endl;
    Utils::transformPoints(transfMat, xyzs);
    EXPECT_EQ(result, xyzs);


}
