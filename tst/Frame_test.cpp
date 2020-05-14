#include "gtest/gtest.h"
#include <core/Utils.h>
#include <core/SFMPipeline.h>
#include <core/DetectorExtractor.h>
#include <core/datastructs/Frame.h>
#include "TestGlobals.h"
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;
TEST(Frame, init) {
    Frame::Ptr frame = make_shared<Frame>(0, programExecutionPath+"/data", "11.JPG");
    frame->init();

    // test
    EXPECT_EQ(frame->kpts.size(), frame->decs.rows);
    EXPECT_NE(frame->kpts.size(), 0);
}

TEST(Frame, sortFeatures) {
    
    Frame::Ptr frame = make_shared<Frame>(0, programExecutionPath+"/data", "11.JPG");
    frame->init();

    // test
    bool allcorrect = true;
    const vector<KeyPoint> &kpts = frame->kpts;
    // for(KeyPoint const&kpt : kpts){
    //     cout<<kpt.pt.y<<","<<kpt.pt.x<<endl;
    // }
    for(size_t i=1; i<kpts.size(); i++){
        if(kpts[i].pt.y<kpts[i-1].pt.y){
            allcorrect = false;
            cout<<"y violated"<<endl;
            break;
        }else if (kpts[i].pt.y == kpts[i-1].pt.y && kpts[i].pt.x<kpts[i-1].pt.x){
            allcorrect = false;
            cout<<"x violated"<<endl;
            break;
        }
    }
    EXPECT_EQ(allcorrect, true);
}

TEST(Frame, makeLUT) {
    Frame::Ptr frame = make_shared<Frame>(0, programExecutionPath+"/data", "11.JPG");
    frame->init();

    // test
    const vector<KeyPoint> &kpts = frame->kpts;
    const vector<int> &kptLUT = frame->kptLUT;
    EXPECT_EQ(kptLUT.back(), kpts.size());

    bool allcorrect = true;
    for(size_t y=0; y<kptLUT.size()-1; y++){
        size_t feIdxStart = kptLUT[y];
        size_t feIdxEnd = kptLUT[y+1];
        
        for(size_t i=feIdxStart+1; i<feIdxEnd; i++){
            if(kpts[i-1].pt.y!=y || kpts[i-1].pt.y!=kpts[i].pt.y){
                cout<<"y violated"<<endl;
                allcorrect=false;
                break;
            }
            if(kpts[i-1].pt.x>kpts[i].pt.x){
                cout<<"x violated"<<endl;
                allcorrect=false;
                break;
            }
        }
        if(!allcorrect){
            break;
        }
    }
    EXPECT_EQ(allcorrect, true);
}
