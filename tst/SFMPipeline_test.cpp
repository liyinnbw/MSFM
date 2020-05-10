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
