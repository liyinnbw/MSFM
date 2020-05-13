#include "gtest/gtest.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <core/SFMPipeline.h>
#include <core/DetectorExtractor.h>
#include <core/Utils.h>
#include "TestGlobals.h"

using namespace cv;
using namespace std;
TEST(SFMPipeline, FindHomographyInliers) {
    
    SFMPipeline sfm("", vector<string>());
    vector<Point2f> pts1,pts2;
    // at leat 4 points to estimate homography
    pts1.emplace_back(0,0);
    pts1.emplace_back(0,1);
    pts1.emplace_back(1,0);
    pts1.emplace_back(1,1);

    // transform these 4 points using an arbitrary homography
    transform(pts1, pts2, Matx23f(1,0,0, 0,-1,0));

    // additional two pairs of points which violates homography
    pts1.emplace_back(3,4);
    pts2.emplace_back(3,4);

    // test
    EXPECT_EQ(4.0/5, sfm.FindHomographyInliers(pts1,pts2));
    
}

TEST(SFMPipeline, checkMatchesBasePair) {
    
    SFMPipeline sfm("", vector<string>());
    Mat img1 = imread(programExecutionPath+"/data/11.JPG",cv::IMREAD_GRAYSCALE);
    Mat img2 = imread(programExecutionPath+"/data/16.JPG",cv::IMREAD_GRAYSCALE);
    Mat img3 = imread(programExecutionPath+"/data/31.JPG",cv::IMREAD_GRAYSCALE);
    vector<KeyPoint> kpts1, kpts2, kpts3;
    vector<Point2f> pts1, pts2, pts3;
    Mat decs1,decs2,decs3;
	DetectorExtractor &deAlgo = DetectorExtractor::GetInstance();
	deAlgo.detectORB(img1, kpts1);
    deAlgo.detectORB(img2, kpts2);
    deAlgo.detectORB(img3, kpts3);
    deAlgo.computeORB(img1, kpts1, decs1);
    deAlgo.computeORB(img2, kpts2, decs2);
    deAlgo.computeORB(img3, kpts3, decs3);
    vector<DMatch> matches12, matches13;
    sfm.matchFeatures(decs1, decs2, matches12);
    sfm.matchFeatures(decs1, decs3, matches13);

    Utils::KeyPoints2Points(kpts1, pts1);
    Utils::KeyPoints2Points(kpts2, pts2);
    Utils::KeyPoints2Points(kpts3, pts3);
    
    // test
    EXPECT_EQ(false, sfm.checkMatchesBasePair(kpts1,kpts2,matches12));
    EXPECT_EQ(true, sfm.checkMatchesBasePair(kpts1,kpts3,matches13));
    EXPECT_EQ(false, sfm.checkMatchesBasePair(pts1,pts2,matches12));
    EXPECT_EQ(true, sfm.checkMatchesBasePair(pts1,pts3,matches13));

}