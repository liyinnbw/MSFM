/*
 * SFMModules.h
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#ifndef SFMPIPELINE_H_
#define SFMPIPELINE_H_


#include <vector>
#include <string>
#include <opencv2/core/core.hpp>

#include "PtCloud.h"

class SFMPipeline {
public:
	SFMPipeline(				const std::string 				&root,
								const std::vector<std::string> 	&paths);

	virtual ~SFMPipeline();

	void getNextPair(			int &imgIdx1,
								int &imgIdx2);

	void getBestPairInRange(	const int start,
								const int end,
								int &imgIdx1,
								int &imgIdx2);

	void processBasePair();

	void processAddNewCamera();

	void processNext_bk();

	bool checkMatchesBasePair(	const std::vector<cv::KeyPoint> &kpts1,
								const std::vector<cv::KeyPoint> &kpts2,
								const std::vector<cv::DMatch> 	&matches);
	bool checkMatchesAddNewCamera(
								const int						imgIdx1,	//this will be new image(tryImg)
								const int						imgIdx2,	//this must be already added image (anchorImg)
								const std::vector<cv::KeyPoint> &kpts1,
								const std::vector<cv::KeyPoint> &kpts2,
								const std::vector<cv::DMatch> 	&matches);

	bool checkMatchesRetriangulateOld(
								const int						imgIdx1,	//this is newly added image(tryImg)
								const int						imgIdx2,	//this must be already added image (anchorImg)
								const std::vector<cv::KeyPoint> &kpts1,
								const std::vector<cv::KeyPoint> &kpts2,
								const std::vector<cv::DMatch> 	&matches);

	void reconstructBasePair(	const int						imgIdx1,	//this will have identity camera projection
								const int						imgIdx2,
								const std::vector<cv::KeyPoint> &kpts1,
								const std::vector<cv::KeyPoint> &kpts2,
								const cv::Mat					&decs1,
								const cv::Mat 					&decs2,
								const std::vector<cv::DMatch> 	&matches);



	void addNewCamera(			const int						imgIdx1,	//this will be new image (tryImg)
								const int						imgIdx2,	//this must be already added image (anchorImg)
								const std::vector<cv::KeyPoint> &kpts1,
								const std::vector<cv::KeyPoint> &kpts2,
								const cv::Mat					&decs1,
								const cv::Mat 					&decs2,
								const std::vector<cv::DMatch> 	&matches);

	void retriangulateOld(		const int						imgIdx1,	//this is the newly added image (tryImg)
								const int						imgIdx2,	//this must be already added image (anchorImg)
								const std::vector<cv::KeyPoint> &kpts1,
								const std::vector<cv::KeyPoint> &kpts2,
								const cv::Mat					&decs1,
								const cv::Mat 					&decs2,
								const std::vector<cv::DMatch> 	&matches);


	bool isGoodMatch(			const std::vector<cv::Point2f> 	&pts1,
								const std::vector<cv::Point2f> 	&pts2,
								const double					&thresh);

	double FindHomographyInliers(const std::vector<cv::Point2f> &pts1,
								const std::vector<cv::Point2f> 	&pts2);

	void matchFeatures(			const cv::Mat &decs1,
								const cv::Mat &decs2,
								std::vector<cv::DMatch> &matches);

	void findEandPruneMatches(	const std::vector<cv::Point2f> 	&pts1,
								const std::vector<cv::Point2f> 	&pts2,
								const std::vector<cv::DMatch> 	&matches,
								std::vector<cv::DMatch> 		&prunedMatches,
								cv::Mat &E);

	void separateMatches(		const int 						imgIdx,
								const int 						useTrainIdx,
								const std::vector<cv::DMatch> 	&matches,
								std::vector<cv::DMatch>			&matchesHas3D,
								std::vector<cv::DMatch>			&matchesNo3D);

	void pruneMatchesNo3D(		const int						imgIdx1,
								const int 						imgIdx2,
								const std::vector<cv::DMatch>	&matches,
								std::vector<cv::DMatch>			&prunedMatches);

	void triangulate(			const std::vector<cv::Point2f> 	&pts2D1,
								const std::vector<cv::Point2f> 	&pts2D2,
								const cv::Matx34d				&P1,
								const cv::Matx34d				&P2,
								std::vector<cv::Point3f> 		&pts3D,
								cv::Mat							&isGood);

	void getKptsAndDecs( 		const int 						imgIdx,
								std::vector<cv::KeyPoint> 		&kpts,
								cv::Mat							&decs);

	bool matchAndTriangulate(	const int 						tryIdx,
								const int 						anchorIdx); //match from tryIdx to anchorIdx

	void pruneHighReprojectionErrorPoints();

	void reprojectionErrorCheck(
								const std::vector<cv::Point3f> 	&pts3D,
								const std::vector<cv::Point2f> 	&pts2D,
								const cv::Matx34d				&P,
								float							&meanError,
								cv::Mat							&isGood);

	void computeMeanReprojectionError();

	void removeNearAndFar3DPoints();

	void cheiralityCheck(		const std::vector<cv::Point3f> 	&pts3D,
								const cv::Matx34d 				&P,
								const float						&minDist,
								const float						&maxDist,
								cv::Mat 						&isGood);

	void printDebug();

	void bundleAdjustment();
	void writePLY(std::string addOn);
	void readPLY(				const std::string 				&path,
								std::vector<cv::Point3f> 		&xyzs);
	bool isDone(){return done;}
	double getCamFocal();
	cv::Point2d getCamPrinciple();
	int getCloudSize(){return ptCloud.pt3Ds.size();}
	int getLastAddedImgIdx(){return lastAddedImgIdx;}

	PtCloud ptCloud; //contains more than just 3d cloud, see declaration
	int lastAddedImgIdx;

	bool done;
	double camFocal;
	cv::Point2d camPrinciple;
	cv::Mat camMat;
	cv::Mat distortionMat;

	//constants
	const static int 	MIN_MATCHES 		= 10;
	const static int 	MIN_3D4PNP 			= 6;
	const static int 	MIN_5PTALGO			= 6;
	const static int 	MIN_TRIANGULATE		= 1;
	const static double MATCH_RATIO 		= 0.9;
	const static int 	IMG_KEYPOINTS 		= 1000;
	const static double HINLIER_THRESH 		= 0.5;
	const static double HINLIER_THRESH2 	= 0.35;
	const static float 	REPROJERROR_THRESH 	= 4.0;
	const static float 	MIN_DIST_TO_CAM 	= 0.0;
	const static float 	MAX_DIST_TO_CAM		= 30.0;
	const static int 	PNP_MAX_ITERATION 	= 100;
	const static float 	PNP_MAX_ERROR   	= 8.0;
	const static double	PNP_CONFIDENCE   	= 0.99;
	const static float	MEAN_ERROR_THRESH	= 8.0;

};

#endif /* SFMPIPELINE_H_ */
