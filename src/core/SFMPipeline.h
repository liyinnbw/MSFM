/*
* API for 3D reconstruction using sequential images
*/

#ifndef SFMPIPELINE_H_
#define SFMPIPELINE_H_


#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/Eigen>
// #include <opengv/relative_pose/CentralRelativeAdapter.hpp>

#include "datastructs/Frame.h"
#include "datastructs/LandMark.h"
#include "datastructs/Measurement.h"
class Data;

class SFMPipeline {
public:
	SFMPipeline(				const std::string 				&root,
								const std::vector<std::string> 	&paths);

	virtual ~SFMPipeline();

	void getNextPair(			int &imgIdx1,
								int &imgIdx2);

	double FindHomographyInliers(const std::vector<cv::Point2f> &pts1,
								const std::vector<cv::Point2f> 	&pts2);

	bool checkMatchesBasePair(	const std::vector<cv::Point2f>  &kpts1,
								const std::vector<cv::Point2f>  &kpts2,
								const std::vector<cv::DMatch> 	&matches);

	bool checkMatchesBasePair(	const std::vector<cv::KeyPoint> &kpts1,
								const std::vector<cv::KeyPoint> &kpts2,
								const std::vector<cv::DMatch> 	&matches);

	bool checkMatchesNextPair(	const int						imgIdx1,	//this will be new image(tryImg)
								const int						imgIdx2,	//this must be already added image (anchorImg)
								const std::vector<cv::KeyPoint> &kpts1,
								const std::vector<cv::KeyPoint> &kpts2,
								const std::vector<cv::DMatch> 	&matches);

	bool reconstructBasePair(	const int						imgIdx1,	//this will have identity camera projection
								const int						imgIdx2);

	bool reconstructBasePair(	Frame::Ptr 						&frame1,	//this will have identity camera projection
								Frame::Ptr						&frame2,
								const std::vector<cv::DMatch> 	&matches);

	// bool reconstructBasePair_gv(Frame::Ptr 						&frame1,	//this will have identity camera projection
	// 							Frame::Ptr						&frame2,
	// 							const std::vector<cv::DMatch> 	&matches);

	bool positionAndAddFrame(	Frame::Ptr						&frame);

	bool positionAndAddFrame(	const int 						imgIdx);

	std::vector<Measurement::Ptr>											//do not add frame to data.frames
	positionFrame		(		Frame::Ptr						&frame,
								const bool						&usePosePrior = false);

	std::vector<Measurement::Ptr>											//do not add frame to data.frames
	solvePnP			(		Frame::Ptr							&frame,
								const std::vector<Measurement::Ptr>	&unprunedMs,
								const double						error	= 8.0,
								const double 						confi 	= 0.99,
								const int							iter	= 1000);

	// std::vector<Measurement::Ptr>											//do not add frame to data.frames
	// solvePnP_gv			(		Frame::Ptr							&frame,
	// 							const std::vector<Measurement::Ptr>	&unprunedMs,
	// 							const double						thresh	= 1e-6,
	// 							const double 						confi 	= 0.99,
	// 							const int							iter	= 1000,
	// 							const bool 							useAll	= false);

	// std::vector<Measurement::Ptr>
	// optimizeCamPose_gv(			Frame::Ptr							&frame,
	// 							const std::vector<Measurement::Ptr>	&ms,
	// 							const double 						errorThresh,
	// 							const int							iter 		= 0,
	// 							const int							maxOctave 	= -1);

	//the opengv triangulate2 is lacking some checking, added in this version
	// Eigen::Vector3d triangulate3(const opengv::relative_pose::RelativeAdapterBase & adapter,
	// 	    					size_t 												index,
	// 							double												sigma,
	// 							bool& 												isValid,
	// 							bool& 												isParallel);

	void epipolarConstrainedMatchMask(	const int 				imgIdx1,	//from a point in img1
										const int				imgIdx2,	//to possible epipolar matches in img2
										cv::Mat					&mask);

	bool addMoreMeasures			( 	Frame::Ptr				&frame);

	void addMoreLandMarksAndMeasures(	Frame::Ptr				&frame1,	//both images must have pose available
										Frame::Ptr				&frame2);


	void addMoreLandMarksAndMeasures(	const int 				imgIdx1,	//both images must have pose available
										const int				imgIdx2);

	// void addMoreLandMarksAndMeasures_gv(const int 				imgIdx1,	//both images must have pose available
	// 									const int				imgIdx2);

	void matchFrames(			const Frame::Ptr				&frame1,
								const Frame::Ptr				&frame2,
								std::vector<cv::DMatch> 		&matches);

	void matchFeatures(			const cv::Mat 					&decs1,
								const cv::Mat					&decs2,
								std::vector<cv::DMatch> 		&matches,
								const cv::Mat 					&mask12 = cv::Mat(),
								const cv::Mat 					&mask21 = cv::Mat());

	// void matchByBOW(			const Frame::Ptr 				&frame1,
	// 							const Frame::Ptr 				&frame2,
	// 							std::vector<cv::DMatch> 		&matches,
	// 							const double					testRatio 				= 0.8,
	// 							const int						distThresh				= 256,
	// 							const bool 						onlyMeasured			= false);

	void matchByProjection(		const Frame::Ptr 				&frame1,
								const Frame::Ptr 				&frame2,
								const int						radius,
								std::vector<cv::DMatch> 		&matches,
								const bool						excludeAlreadyMeasured = false);

	std::vector<Measurement::Ptr>
	getMoreMeasuresByProjection(const Frame::Ptr 					&frame1,			//frame which you need more measures
								const Frame::Ptr 					&frame2				= nullptr,		//reference frame
								const std::vector<Measurement::Ptr> &existingMeasures 	= std::vector<Measurement::Ptr>(),
								const int							searchThresh 		= 7,
								const double						testRatio 			= 0.8,
								const int							distThresh			= 256,
								const int							maxMatches 			= 0,
								const bool							fixThresh			= false);

	std::vector<Measurement::Ptr>
	getMeasuresByProjection(	const Frame::Ptr 					&frame,
								const std::vector<Measurement::Ptr>	&otherFramesMs,		//these measures measure to different landmark
								const std::vector<Measurement::Ptr> &existingMeasures 	= std::vector<Measurement::Ptr>(),
								const int							searchThresh 		= 7,
								const double						testRatio 			= 0.8,
								const int							distThresh			= 256,
								const bool							fixThresh			= false);

	void triangulate(			const std::vector<cv::Point2f> 	&pts2D1,
								const std::vector<cv::Point2f> 	&pts2D2,
								const cv::Matx34d				&P1,
								const cv::Matx34d				&P2,
								std::vector<cv::Point3f> 		&pts3D,
								cv::Mat							&isGood);

	void pruneHighErrorMeasurements();

	void reprojectionErrorCheck(
								const std::vector<cv::Point3f> 	&pts3D,
								const std::vector<cv::Point2f> 	&pts2D,
								const cv::Matx34d				&P,
								float							&meanError,
								cv::Mat							&isGood);

	double computeMeanReprojectionError(	const std::vector<Measurement::Ptr> &in_ms = std::vector<Measurement::Ptr>());

	double computeReprojectionError(const Frame::Ptr 				&frame,
									const Eigen::Vector3d 			&pt3D,
									const	int						featureIdx);

	void cheiralityCheck(		const std::vector<cv::Point3f> 	&pts3D,
								const cv::Matx34d 				&P,
								const float						&minDist,
								const float						&maxDist,
								cv::Mat 						&isGood);

	void applyGlobalTransformation(const cv::Mat 				&transformation);

	void printDebug();

	void bundleAdjustment();

	void bundleAdjustmentLocal();

	void bundleAdjustmentLocal(	std::vector<Measurement::Ptr>	&ms);

	// void bundleAdjustmentLocalFixPoints(std::vector<Measurement::Ptr> &ms);

	void saveProject(			const std::string 				&fname);

	void loadProject(			const std::string				&fname);

	std::string						imgRoot;
	std::vector<std::string>		imgNames;
	Data							&data;



};

#endif /* SFMPIPELINE_H_ */
