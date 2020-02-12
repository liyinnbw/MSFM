/*
 * Params.cpp
 *
 *  Created on: Sep 17, 2010
 *      Author: asl
 */

#include "Params.h"
#include "opencv2/opencv.hpp"
#include "iostream"
void PtamParamsConfig::readVarParams(){

	cv::FileStorage fs("/home/yoyo/workspace/CamTrack/Params/FixParams.yaml", cv::FileStorage::READ);
	std::cout<<"file storage opened: "<<fs.isOpened()<<std::endl;
	fs["gui"]>>gui;
	fs["ImageSizeX"]>>ImageSizeX;
	fs["ImageSizeY"]>>ImageSizeY;
	fs["ARBuffer_width"]>>ARBuffer_width;
	fs["ARBuffer_height"]>>ARBuffer_height;
	fs["WiggleScale"]>>WiggleScale;
	fs["BundleMEstimator"]>>BundleMEstimator;
	fs["TrackerMEstimator"]>>TrackerMEstimator;
	fs["MinTukeySigma"]>>MinTukeySigma;
	fs["CandidateMinSTScore"]>>CandidateMinSTScore;
	fs["Calibrator_BlurSigma"]>>Calibrator_BlurSigma;
	fs["Calibrator_MeanGate"]>>Calibrator_MeanGate;
	fs["Calibrator_MinCornersForGrabbedImage"]>>Calibrator_MinCornersForGrabbedImage;
	fs["Calibrator_Optimize"]>>Calibrator_Optimize;
	fs["Calibrator_Show"]>>Calibrator_Show;
	fs["Calibrator_NoDistortion"]>>Calibrator_NoDistortion;
	fs["CameraCalibrator_MaxStepDistFraction"]>>CameraCalibrator_MaxStepDistFraction;
	fs["CameraCalibrator_CornerPatchSize"]>>CameraCalibrator_CornerPatchSize;
	fs["GLWindowMenu_Enable"]>>GLWindowMenu_Enable;
	fs["GLWindowMenu_mgvnMenuItemWidth"]>>GLWindowMenu_mgvnMenuItemWidth;
	fs["GLWindowMenu_mgvnMenuTextOffset"]>>GLWindowMenu_mgvnMenuTextOffset;
	fs["InitLevel"]>>InitLevel;
	fs["parent_frame"]>>parent_frame;
	fs["MaxStereoInitLoops"]>>MaxStereoInitLoops;
	fs["MaxFoV"]>>MaxFoV;


	fs["Cam_fx"]>>Cam_fx;
	fs["Cam_fy"]>>Cam_fy;
	fs["Cam_cx"]>>Cam_cx;
	fs["Cam_cy"]>>Cam_cy;
	fs["Cam_s"]>>Cam_s;

	fs["RotationEstimatorBlur"]>>RotationEstimatorBlur;
	fs["UseRotationEstimator"]>>UseRotationEstimator;
	fs["AutoInit"]>>AutoInit;
	fs["AutoInitPixel"]>>AutoInitPixel;
	fs["MiniPatchMaxSSD"]>>MiniPatchMaxSSD;
	fs["CoarseMin"]>>CoarseMin;
	fs["CoarseMax"]>>CoarseMax;
	fs["CoarseRange"]>>CoarseRange;
	fs["CoarseSubPixIts"]>>CoarseSubPixIts;
	fs["DisableCoarse"]>>DisableCoarse;
	fs["CoarseMinVelocity"]>>CoarseMinVelocity;
	fs["MaxPatchesPerFrame"]>>MaxPatchesPerFrame;
	fs["TrackingQualityFoundPixels"]>>TrackingQualityFoundPixels;
	fs["TrackingQualityGood"]>>TrackingQualityGood;
	fs["TrackingQualityLost"]>>TrackingQualityLost;
	fs["MotionModelUseIMU"]>>MotionModelUseIMU;
	fs["BundleDebugMessages"]>>BundleDebugMessages;
	fs["UpdateSquaredConvergenceLimit"]>>UpdateSquaredConvergenceLimit;
	fs["MaxIterations"]>>MaxIterations;
	fs["BundleMethod"]>>BundleMethod;
	fs["NoLevelZeroMapPoints"]>>NoLevelZeroMapPoints;
	fs["CheckInitMapVar"]>>CheckInitMapVar;
	fs["MaxKF"]>>MaxKF;
	fs["UseKFPixelDist"]>>UseKFPixelDist;
	fs["MaxKFDistWiggleMult"]>>MaxKFDistWiggleMult;
	fs["PlaneAlignerRansacs"]>>PlaneAlignerRansacs;
	fs["FASTMethod"]>>FASTMethod;
	fs["Thres_lvl0"]>>Thres_lvl0;
	fs["Thres_lvl1"]>>Thres_lvl1;
	fs["Thres_lvl2"]>>Thres_lvl2;
	fs["Thres_lvl3"]>>Thres_lvl3;
	fs["AdaptiveThrs"]>>AdaptiveThrs;
	fs["AdaptiveThrsMult"]>>AdaptiveThrsMult;
	fs["RelocMaxScore"]>>RelocMaxScore;
	fs.release();

}
void FixParams::readFixParams()
{

	cv::FileStorage fs("/home/yoyo/workspace/CamTrack/Params/FixParams.yaml", cv::FileStorage::READ);
	std::cout<<"file storage opened: "<<fs.isOpened()<<std::endl;
	fs["gui"]>>gui;
	fs["ImageSizeX"]>>ImageSizeX;
	fs["ImageSizeY"]>>ImageSizeY;
	fs["ARBuffer_width"]>>ARBuffer_width;
	fs["ARBuffer_height"]>>ARBuffer_height;
	fs["WiggleScale"]>>WiggleScale;
	fs["BundleMEstimator"]>>BundleMEstimator;
	fs["TrackerMEstimator"]>>TrackerMEstimator;
	fs["MinTukeySigma"]>>MinTukeySigma;
	fs["CandidateMinSTScore"]>>CandidateMinSTScore;
	fs["Calibrator_BlurSigma"]>>Calibrator_BlurSigma;
	fs["Calibrator_MeanGate"]>>Calibrator_MeanGate;
	fs["Calibrator_MinCornersForGrabbedImage"]>>Calibrator_MinCornersForGrabbedImage;
	fs["Calibrator_Optimize"]>>Calibrator_Optimize;
	fs["Calibrator_Show"]>>Calibrator_Show;
	fs["Calibrator_NoDistortion"]>>Calibrator_NoDistortion;
	fs["CameraCalibrator_MaxStepDistFraction"]>>CameraCalibrator_MaxStepDistFraction;
	fs["CameraCalibrator_CornerPatchSize"]>>CameraCalibrator_CornerPatchSize;
	fs["GLWindowMenu_Enable"]>>GLWindowMenu_Enable;
	fs["GLWindowMenu_mgvnMenuItemWidth"]>>GLWindowMenu_mgvnMenuItemWidth;
	fs["GLWindowMenu_mgvnMenuTextOffset"]>>GLWindowMenu_mgvnMenuTextOffset;
	fs["InitLevel"]>>InitLevel;
	fs["parent_frame"]>>parent_frame;
	fs["MaxStereoInitLoops"]>>MaxStereoInitLoops;
	fs["MaxFoV"]>>MaxFoV;


	fs["Cam_fx"]>>Cam_fx;
	fs["Cam_fy"]>>Cam_fy;
	fs["Cam_cx"]>>Cam_cx;
	fs["Cam_cy"]>>Cam_cy;
	fs["Cam_s"]>>Cam_s;

	fs["RotationEstimatorBlur"]>>RotationEstimatorBlur;
	fs["UseRotationEstimator"]>>UseRotationEstimator;
	fs["AutoInit"]>>AutoInit;
	fs["AutoInitPixel"]>>AutoInitPixel;
	fs["MiniPatchMaxSSD"]>>MiniPatchMaxSSD;
	fs["CoarseMin"]>>CoarseMin;
	fs["CoarseMax"]>>CoarseMax;
	fs["CoarseRange"]>>CoarseRange;
	fs["CoarseSubPixIts"]>>CoarseSubPixIts;
	fs["DisableCoarse"]>>DisableCoarse;
	fs["CoarseMinVelocity"]>>CoarseMinVelocity;
	fs["MaxPatchesPerFrame"]>>MaxPatchesPerFrame;
	fs["TrackingQualityFoundPixels"]>>TrackingQualityFoundPixels;
	fs["TrackingQualityGood"]>>TrackingQualityGood;
	fs["TrackingQualityLost"]>>TrackingQualityLost;
	fs["MotionModelUseIMU"]>>MotionModelUseIMU;
	fs["BundleDebugMessages"]>>BundleDebugMessages;
	fs["UpdateSquaredConvergenceLimit"]>>UpdateSquaredConvergenceLimit;
	fs["MaxIterations"]>>MaxIterations;
	fs["BundleMethod"]>>BundleMethod;
	fs["NoLevelZeroMapPoints"]>>NoLevelZeroMapPoints;
	fs["CheckInitMapVar"]>>CheckInitMapVar;
	fs["MaxKF"]>>MaxKF;
	fs["UseKFPixelDist"]>>UseKFPixelDist;
	fs["MaxKFDistWiggleMult"]>>MaxKFDistWiggleMult;
	fs["PlaneAlignerRansacs"]>>PlaneAlignerRansacs;
	fs["FASTMethod"]>>FASTMethod;
	fs["Thres_lvl0"]>>Thres_lvl0;
	fs["Thres_lvl1"]>>Thres_lvl1;
	fs["Thres_lvl2"]>>Thres_lvl2;
	fs["Thres_lvl3"]>>Thres_lvl3;
	fs["AdaptiveThrs"]>>AdaptiveThrs;
	fs["AdaptiveThrsMult"]>>AdaptiveThrsMult;
	fs["RelocMaxScore"]>>RelocMaxScore;
	fs.release();

}
//;
PtamParameters* PtamParameters::inst_ = NULL;
