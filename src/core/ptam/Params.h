/*
 * Params.h
 *
 *  Created on: Jul 21, 2010
 *      Author: sweiss
 */

#ifndef PARAMS_H_
#define PARAMS_H_

//#include <PtamParamsConfig.h>
#include <string>

//#include <ros/ros.h>
//#include <XmlRpcValue.h>

//#include <dynamic_reconfigure/server.h>
//#include <Params.h>

//typedef dynamic_reconfigure::Server<ptam::PtamParamsConfig> PtamParamsReconfigureServer;
//typedef ptam::PtamParamsConfig VarParams;

class FixParams
{
public:
  int ImageSizeX;
  int ImageSizeY;
  int ARBuffer_width;
  int ARBuffer_height;
  double WiggleScale;
  std::string BundleMEstimator;
  std::string TrackerMEstimator;
  double MinTukeySigma;
  int CandidateMinSTScore;
  double Cam_fx;
  double Cam_fy;
  double Cam_cx;
  double Cam_cy;
  double Cam_s;
  double Calibrator_BlurSigma;
  double Calibrator_MeanGate;
  int Calibrator_MinCornersForGrabbedImage;
  bool Calibrator_Optimize;
  bool Calibrator_Show;
  bool Calibrator_NoDistortion;
  double CameraCalibrator_MaxStepDistFraction;
  int CameraCalibrator_CornerPatchSize;
  bool GLWindowMenu_Enable;
  int GLWindowMenu_mgvnMenuItemWidth;
  int GLWindowMenu_mgvnMenuTextOffset;
  int InitLevel;
  std::string parent_frame;
  bool gui;
  int MaxStereoInitLoops;
  double MaxFoV;

  //from PatamParamsConfig
  double RotationEstimatorBlur; //min 0.0, max 10.0, default 0.75
  int UseRotationEstimator;		//0 or 1, default 1
  int AutoInit;					//0 or 1, default 0
  int AutoInitPixel;			//min 1, max 100, default 20
  int MiniPatchMaxSSD;			//min 0, max 10000000, default 100000
  double CoarseMin;				//min 1.0, max 100.0, default 20.0
  double CoarseMax;				//min 1.0, max 100.0, default 60.0
  double CoarseRange;			//min 1.0, max 100.0, default 30.0
  double CoarseSubPixIts;		//min 1.0, max 100.0, default 8.0
  int DisableCoarse;			//0 or 1, default 0
  double CoarseMinVelocity;		//min 0.0, max 1.0, default 0.006
  int MaxPatchesPerFrame;		//min 10, max 1000, default 500
  int TrackingQualityFoundPixels; //min 1, max 1000, default 100
  double TrackingQualityGood;	//min 0.0, max 1.0, default 0.3
  double TrackingQualityLost;	//min 0.0, max 1.0, default 0.13
  bool	MotionModelUseIMU;		//default False
  bool BundleDebugMessages;		//default False
  double UpdateSquaredConvergenceLimit; //min 0.0, max 1.0, default 1e-06
  int MaxIterations;			//min 1, max 100, default 20
  std::string BundleMethod;		//LOCAL, LOCAL_GLOBAL, GLOBAL
  bool NoLevelZeroMapPoints;	//default False
  bool CheckInitMapVar;			//default False
  int MaxKF;					//<2 means keep all KF. default 0
  bool UseKFPixelDist;			//default True
  double MaxKFDistWiggleMult;	//min 0.1, max 10.0, default 3.0
  int PlaneAlignerRansacs;		//min 0, max 1000, default 100
  std::string FASTMethod;		//FAST9, FAST10,FAST9_nonmax, AGAST12d, OAST16,  default FAST9_nonmax
  int Thres_lvl0;				//min 0, max 255, default 10
  int Thres_lvl1;				//min 0, max 255, default 15
  int Thres_lvl2;				//min 0, max 255, default 15
  int Thres_lvl3;				//min 0, max 255, default 10
  bool AdaptiveThrs;			//default False
  double AdaptiveThrsMult;		//min 0.5, max 20.0, default 5.0
  int RelocMaxScore;			//min 0, max 90000000 (7 0s), default 9000000 (6 0s)
  void readFixParams();
};
class PtamParamsConfig{
public:
  int ImageSizeX;
  int ImageSizeY;
  int ARBuffer_width;
  int ARBuffer_height;
  double WiggleScale;
  std::string BundleMEstimator;
  std::string TrackerMEstimator;
  double MinTukeySigma;
  int CandidateMinSTScore;
  double Cam_fx;
  double Cam_fy;
  double Cam_cx;
  double Cam_cy;
  double Cam_s;
  double Calibrator_BlurSigma;
  double Calibrator_MeanGate;
  int Calibrator_MinCornersForGrabbedImage;
  bool Calibrator_Optimize;
  bool Calibrator_Show;
  bool Calibrator_NoDistortion;
  double CameraCalibrator_MaxStepDistFraction;
  int CameraCalibrator_CornerPatchSize;
  bool GLWindowMenu_Enable;
  int GLWindowMenu_mgvnMenuItemWidth;
  int GLWindowMenu_mgvnMenuTextOffset;
  int InitLevel;
  std::string parent_frame;
  bool gui;
  int MaxStereoInitLoops;
  double MaxFoV;

  //from PatamParamsConfig
  double RotationEstimatorBlur; //min 0.0, max 10.0, default 0.75
  int UseRotationEstimator;		//0 or 1, default 1
  int AutoInit;					//0 or 1, default 0
  int AutoInitPixel;			//min 1, max 100, default 20
  int MiniPatchMaxSSD;			//min 0, max 10000000, default 100000
  double CoarseMin;				//min 1.0, max 100.0, default 20.0
  double CoarseMax;				//min 1.0, max 100.0, default 60.0
  double CoarseRange;			//min 1.0, max 100.0, default 30.0
  double CoarseSubPixIts;		//min 1.0, max 100.0, default 8.0
  int DisableCoarse;			//0 or 1, default 0
  double CoarseMinVelocity;		//min 0.0, max 1.0, default 0.006
  int MaxPatchesPerFrame;		//min 10, max 1000, default 500
  int TrackingQualityFoundPixels; //min 1, max 1000, default 100
  double TrackingQualityGood;	//min 0.0, max 1.0, default 0.3
  double TrackingQualityLost;	//min 0.0, max 1.0, default 0.13
  bool	MotionModelUseIMU;		//default False
  bool BundleDebugMessages;		//default False
  double UpdateSquaredConvergenceLimit; //min 0.0, max 1.0, default 1e-06
  int MaxIterations;			//min 1, max 100, default 20
  std::string BundleMethod;		//LOCAL, LOCAL_GLOBAL, GLOBAL
  bool NoLevelZeroMapPoints;	//default False
  bool CheckInitMapVar;			//default False
  int MaxKF;					//<2 means keep all KF. default 0
  bool UseKFPixelDist;			//default True
  double MaxKFDistWiggleMult;	//min 0.1, max 10.0, default 3.0
  int PlaneAlignerRansacs;		//min 0, max 1000, default 100
  std::string FASTMethod;		//FAST9, FAST10,FAST9_nonmax, AGAST12d, OAST16,  default FAST9_nonmax
  int Thres_lvl0;				//min 0, max 255, default 10
  int Thres_lvl1;				//min 0, max 255, default 15
  int Thres_lvl2;				//min 0, max 255, default 15
  int Thres_lvl3;				//min 0, max 255, default 10
  bool AdaptiveThrs;			//default False
  double AdaptiveThrsMult;		//min 0.5, max 20.0, default 5.0
  int RelocMaxScore;			//min 0, max 90000000 (7 0s), default 9000000 (6 0s)
  void readVarParams();
};
class PtamParameters{
private:
  FixParams mFixParams;
  PtamParamsConfig mVarParams;


  //PtamParamsReconfigureServer *mpPtamParamsReconfigureServer;

  //void ptamParamsConfig(PtamParamsConfig & config, uint32_t level){
  //  mVarParams = config;
  //};

  static PtamParameters* inst_;

  PtamParameters()
  {
    //mpPtamParamsReconfigureServer = new PtamParamsReconfigureServer(ros::NodeHandle("~"));
    //PtamParamsReconfigureServer::CallbackType PtamParamCall = boost::bind(&PtamParameters::ptamParamsConfig, this, _1, _2);
    //mpPtamParamsReconfigureServer->setCallback(PtamParamCall);

    mFixParams.readFixParams();
    mVarParams.readVarParams();
  }
  ~PtamParameters(){
    delete inst_;
    inst_ = NULL;
  }
public:

  static const PtamParamsConfig& varparams(){
    if(!inst_){
      inst_ = new PtamParameters;
    }
    return inst_->mVarParams;
  }

  static const FixParams& fixparams(){
    if(!inst_){
      inst_ = new PtamParameters;
    }
    return inst_->mFixParams;
  }
};

#endif /* PARAMS_H_ */
