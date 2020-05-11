/*
* API for 3D reconstruction using sequential images
*/

#include <algorithm>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <set>
#include <ctime>
#include <cmath>
#include <list>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// #include <opengv/types.hpp>
// #include <opengv/relative_pose/methods.hpp>
// #include <opengv/absolute_pose/methods.hpp>
// #include <opengv/relative_pose/CentralRelativeAdapter.hpp>
// #include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
// #include <opengv/sac/Ransac.hpp>
// #include <opengv/triangulation/methods.hpp>
// #include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
// #include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <Eigen/Eigen>

#include "SFMPipeline.h"
#include "BAHandler.h"
#include "Utils.h"
#include "ProjectIO.h"
#include "Camera.h"
#include "datastructs/Data.h"

using namespace cv;
// using namespace opengv;
using namespace Eigen;
using namespace std;

//constants
const static int 	MIN_MATCHES 		= 10;
const static int 	MIN_3D4PNP 			= 6;
const static int 	MIN_POSE_OPTIMIZE 	= 3;
const static int 	MIN_5PTALGO			= 6;
const static int 	MIN_TRIANGULATE		= 1;
const static int 	MIN_BASE_TRIANGULATE= 10;
const static double MATCH_RATIO 		= 0.8;
const static int 	IMG_KEYPOINTS 		= 1000;
const static double HINLIER_THRESH 		= 0.5;
//const static double HINLIER_THRESH2 	= 0.35;
const static float 	REPROJERROR_THRESH 	= 4.0;
const static float 	MIN_DIST_TO_CAM 	= 0.0;
const static float 	MAX_DIST_TO_CAM		= 30.0;
//const static int 	PNP_MAX_ITERATION 	= 1000;
//const static float 	PNP_MAX_ERROR   	= 8.0;
//const static double	PNP_CONFIDENCE   	= 0.99;
const static float	MEAN_ERROR_THRESH	= 8.0;
const static int	EPIPOLAR_SEARCH_PIXEL_THRESH = 8;
const static int	EPIPOLAR_SEARCH_DEPTH_THRESH = 1000;

SFMPipeline::SFMPipeline(	const string			&root,
							const vector<string> 	&paths)
:imgRoot(root)
,imgNames(paths)
,data(Data::GetInstance())
{

}

SFMPipeline::~SFMPipeline() {

}

double SFMPipeline::FindHomographyInliers(const vector<Point2f> &pts1, const vector<Point2f> &pts2){

	// double minVal,maxVal;
	// minMaxIdx(pts1,&minVal,&maxVal);
	vector<uchar> status;
	Mat H = findHomography(pts1,pts2,status,RANSAC,3);//, 0.004 * maxVal); //threshold from Snavely07
	return countNonZero(status)*1.0/pts1.size(); //number of inliers
}

void SFMPipeline::getNextPair(	int &imgIdx1, 	//from
								int &imgIdx2)	//to
{
	imgIdx1 = -1;
	imgIdx2 = -1;

	if(data.countFrames() == 0){
		for(int anchorIdx=0; anchorIdx<imgNames.size()-1; anchorIdx++){
			for(int tryIdx=anchorIdx+1; tryIdx<imgNames.size(); tryIdx++){
				cout<<" trying ["<<anchorIdx<<"]->["<<tryIdx<<"]"<<endl;

				Frame::Ptr anchorFrame = make_shared<Frame>(anchorIdx,imgRoot,imgNames[anchorIdx]);
				Frame::Ptr tryFrame = make_shared<Frame>(tryIdx,imgRoot,imgNames[tryIdx]);

				//compute features & descriptors
				anchorFrame->init();
				tryFrame->init();

				//do feature matching
				vector<DMatch> 		matches;
				matchFrames(anchorFrame, tryFrame, matches);

				if(checkMatchesBasePair(anchorFrame->kpts,tryFrame->kpts,matches)){
					cout<<"total matches = "<<matches.size()<<endl;
					imgIdx1 = anchorIdx;	//from
					imgIdx2 = tryIdx;		//to
					return;
				}
			}
		}

	}else{
		Frame::Ptr anchorFrame = data.getFrames().back();
		int anchorIdx = anchorFrame->imgIdx;
		cout<<"last added frame idx = "<<anchorIdx<<endl;
		if(anchorIdx>=(int)(imgNames.size()-1)){
			cout<<"No more images to add"<<endl;
			return;
		}
		assert(anchorIdx>=0 && anchorIdx<imgNames.size()-1);

		for(int tryIdx = anchorIdx+1; tryIdx < imgNames.size(); tryIdx++){
			cout<<" trying ["<<tryIdx<<"]->["<<anchorIdx<<"]"<<endl;

			Frame::Ptr tryFrame = make_shared<Frame>(tryIdx,imgRoot,imgNames[tryIdx]);

			//compute features & descriptors
			tryFrame->init();

			//match features
			vector<DMatch> 		matches;
			matchFrames(tryFrame, anchorFrame, matches);

			if(checkMatchesNextPair(tryIdx, anchorIdx, tryFrame->kpts, anchorFrame->kpts, matches)){
				cout<<"total matches = "<<matches.size()<<endl;
				imgIdx1 = tryIdx;		//from
				imgIdx2 = anchorIdx;	//to
				return;
			}
		}
	}
	cout<<"no next pair found"<<endl;

}

bool SFMPipeline::checkMatchesBasePair(	const std::vector<cv::Point2f> 	&kpts1,
										const std::vector<cv::Point2f> 	&kpts2,
										const std::vector<cv::DMatch> 	&matches)
{
	if (kpts1.size()<MIN_5PTALGO || kpts2.size()<MIN_5PTALGO){
		cout<<" too few feature points, target: "<<MIN_5PTALGO<<endl;
		return false;
	}

	vector<Point2f> 	pts1, pts2;
	Mat 				E, R, rvec, t, inliers;
	int 				nonzeroCnt;

	Utils::Matches2Points(kpts1,kpts2,matches,pts1,pts2);

	//check homography
	double hInlier = FindHomographyInliers(pts1,pts2);
	if(hInlier > HINLIER_THRESH){
		cout<<" too much homography inliers: "<<hInlier<<" target: "<<HINLIER_THRESH<<endl;
		return false;
	}

	//check minimum matches for 5 point algo
	if(matches.size() < MIN_5PTALGO){
		cout<<" too few matches for essential mat: "<<matches.size()<<" target: "<<MIN_5PTALGO<<endl;
		return false;
	}
	return true;
}

bool SFMPipeline::checkMatchesBasePair(	const vector<KeyPoint> 	&kpts1,
										const vector<KeyPoint> 	&kpts2,
										const vector<DMatch> 	&matches)
{

	vector<Point2f> 	pts1, pts2;
	Mat 				E, R, rvec, t, inliers;
	int 				nonzeroCnt;

	Utils::Matches2Points(kpts1,kpts2,matches,pts1,pts2);

	//check homography
	double hInlier = FindHomographyInliers(pts1,pts2);
	if(hInlier > HINLIER_THRESH){
		cout<<" too much homography inliers: "<<hInlier<<" target: "<<HINLIER_THRESH<<endl;
		return false;
	}

	//check minimum matches for 5 point algo
	if(matches.size() < MIN_5PTALGO){
		cout<<" too few matches for essential mat: "<<matches.size()<<" target: "<<MIN_5PTALGO<<endl;
		return false;
	}
	/*
	//get camera intrinsics
	double f = Camera::GetInstance().getCamFocal();
	Point2d pp = Camera::GetInstance().getCamPrinciple();

	cout<<f<<endl;
	cout<<pp<<endl;
	cout<<"points used for essential mat "<<pts1.size()<<endl;
	//prune by fundamental mat
	E = findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);
	nonzeroCnt = countNonZero(inliers);

	cout<<E.rows<<endl;
	cout<<nonzeroCnt<<endl;

	//check if have sufficient matches left
	if(nonzeroCnt<MIN_MATCHES){
		cout<<" too few matches for recover pose: "<<nonzeroCnt<<" target: "<<MIN_MATCHES<<endl;
		return false;
	}

	cout<<"before recover pose"<<endl;

	//prune by recover pose
	//inliers is both input and output.
	//input is from previous step.
	//output is to prune those failed cheiralityCheck.
	recoverPose(E, pts1, pts2, R, t, f, pp);//, inliers);

	cout<<"after recover pose"<<endl;
	nonzeroCnt = countNonZero(inliers);

	if(nonzeroCnt<MIN_TRIANGULATE){
		cout<<" too few points for triangulation: "<<nonzeroCnt<<" target: "<<MIN_TRIANGULATE<<endl;
		return false;
	}

	cout<<"matches will be triangulated = "<<nonzeroCnt<<endl;*/
	return true;
}

bool SFMPipeline::checkMatchesNextPair(		const int				imgIdx1,	//new
											const int				imgIdx2,	//already added
											const vector<KeyPoint> 	&kpts1,
											const vector<KeyPoint> 	&kpts2,
											const vector<DMatch> 	&matches)
{
	if(data.getFrame(imgIdx1) || !data.getFrame(imgIdx2)){
		cout<<"MATCH CHECK FAILURE:first image must be not used and second image must be used"<<endl;
		return false;
	}

	vector<Point2f> pts1, pts2;
	Utils::Matches2Points(kpts1,kpts2,matches,pts2,pts1);

	//check homography
	double hInlier = FindHomographyInliers(pts1,pts2);
	if(hInlier > HINLIER_THRESH){
		cout<<" too much homography inliers: "<<hInlier<<" target: "<<HINLIER_THRESH<<endl;
		return false;
	}

	//check minimum matches for 5 point algo
	if(matches.size() < MIN_5PTALGO){
		cout<<" too few matches for essential mat: "<<matches.size()<<" target: "<<MIN_5PTALGO<<endl;
		return false;
	}

	return true;
}

//should only be called after checkMatchesBasePair
bool SFMPipeline::reconstructBasePair(	const int				imgIdx1,	//this will have identity camera projection
										const int				imgIdx2)
{
	clock_t				time;
	double				t_matching;

	time = clock();

	Frame::Ptr frame1 = make_shared<Frame>(imgIdx1, imgRoot, imgNames[imgIdx1]);
	frame1->fixed = true;	//frame 1 use as reference and pose not changed
	Frame::Ptr frame2 = make_shared<Frame>(imgIdx2, imgRoot, imgNames[imgIdx2]);

	frame1->init();
	frame2->init();

	vector<DMatch>		matches;
	matchFrames(frame1, frame2, matches);

	t_matching = double(clock()-time) / CLOCKS_PER_SEC;
	cout<<"Time(s) matching    = "<<t_matching<<endl;

	return reconstructBasePair(frame1, frame2, matches);
	//return reconstructBasePair_gv(frame1, frame2, matches);
}

bool SFMPipeline::reconstructBasePair(	Frame::Ptr 						&frame1,	//this will have identity camera projection
										Frame::Ptr						&frame2,
										const std::vector<cv::DMatch> 	&matches)
{
	//check if matches are sufficient
	if(matches.size() < MIN_5PTALGO){
		cout<<" too few matches for essential mat: "<<matches.size()<<" target: "<<MIN_5PTALGO<<endl;
		return false;
	}

	cout<<"initializing camera"<<endl;
	Mat tmp;
	if(frame1->img.empty() && frame2->img.empty()){
		tmp 	= imread(frame1->imgRoot+"/"+frame1->imgName,IMREAD_GRAYSCALE);
	}else if(frame1->img.empty()){
		tmp		= frame2->img;
	}else{
		tmp		= frame1->img;
	}
	Camera &camera = Camera::GetInstance();
	camera.w = tmp.cols;
	camera.h = tmp.rows;
	double foc = (camera.w>camera.h)? (double) camera.w: (double) camera.h;
	double ppx = camera.w/2.0;
	double ppy = camera.h/2.0;
	double camMatArr[9] = { foc, 	0.0, 		ppx,
							0.0, 	foc, 		ppy,
							0.0,	0.0,		1.0 };
	//assume no distorition
	double distortCoeffArr[5] 	= { 0, 0, 0, 0, 0 };
	camera.camMat 				= Mat(3, 3, CV_64F, camMatArr).clone();
	camera.distortionMat 		= Mat(1, 5, CV_64F, distortCoeffArr).clone();
	cout<<camera.camMat<<endl;
	cout<<camera.distortionMat<<endl;

	cout<<"(opencv) reconstruct base pair ["<<frame1->imgIdx<<"]"<<frame2->imgName<<" & "<<"["<<frame2->imgIdx<<"]"<<frame2->imgName<<endl;
	assert(!data.getFrame(frame1->imgIdx) && !data.getFrame(frame2->imgIdx));
	clock_t				time;
	double				t_poserecover;
	double				t_triangulate;
	double				t_postprocess;

	time = clock();

	vector<KeyPoint> 	&kpts1 = frame1->kpts;
	vector<KeyPoint> 	&kpts2 = frame2->kpts;

	vector<Point2f> 	pts1, pts2;
	Utils::Matches2Points(kpts1,kpts2,matches,pts1,pts2);

	double f = Camera::GetInstance().getCamFocal();
	Point2d pp = Camera::GetInstance().getCamPrinciple();
	Mat 				E, inliers;
	E = findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);

	//check if enough inliers
	int inlierCnt = countNonZero(inliers);
	if( inlierCnt < MIN_BASE_TRIANGULATE){
		cout<<" too few inliers for base triangulation: "<<inlierCnt<<" target: "<<MIN_BASE_TRIANGULATE<<endl;
		return false;
	}

	vector<DMatch> 		prunedMatches;
	for(int i=0; i<inliers.rows; i++){
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches.push_back(matches[i]);
		}
	}

	//inliers is both input and output.
	//input is from previous step.
	//Although output supposed to be those passed cheiralityCheck, 
	//experiment shows the inlier exclude too many good triangulations points in front of camera
	// so I'll avoid using the inlier to do futher pruning
	Mat					R,t;
	recoverPose(E, pts1, pts2, R, t, f, pp, inliers);

	//set camera pose, leave frame1's pose as default
	frame1->fixed = true;

	//its important that you specify rowmajor when mapping eigen to opencv, because eigen uses column major by default
	//if you directly map Matrix3d to opencv matx, the matrix will be transposed because Matrix3d is default column major
	//for eigen vectors, there is no distinction between row or column so Vector3d will do
	Map<Matrix<double, 3,3, RowMajor>> 	R_eigen((double *) R.data);	//no copy data
	Map<Vector3d> 						t_eigen((double *) t.data);	//no copy data
	frame2->rotation = Quaterniond(R_eigen);						//copy data
	frame2->rotation.normalize();
	frame2->position = -R_eigen.transpose()*t_eigen;				//copy data
	frame2->position.normalize();	//only for the base pair we normalize scale

	t_poserecover = double(clock()-time) / CLOCKS_PER_SEC;

	time = clock();

	//set projection mats
	Matx34d P1 = 	frame1->getCVTransform();
	Matx34d P2 =	frame2->getCVTransform();

	Utils::Matches2Points(kpts1,kpts2,prunedMatches,pts1,pts2);

	//inliers do both cheiralityCheck and reprojectionErrorCheck
	vector<Point3f>		pts3D;
	cout<<pts1.size()<<endl;
	triangulate(pts1,pts2,P1,P2,pts3D,inliers);
	assert(inliers.cols == prunedMatches.size());

	//tmp array to store triangulated points and measures.
	//do not immediately add to data cos we may not have enough triangulations
	vector<Measurement::Ptr> ms;
	vector<LandMark::Ptr>	lms;

	for(unsigned int i=0; i<inliers.cols; i++){	//type 0, size 1*n
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			Vector3d pt(pts3D[i].x, pts3D[i].y, pts3D[i].z);
			LandMark::Ptr lm = make_shared<LandMark>(pt);
			lms.push_back(lm);
			ms.push_back(make_shared<Measurement>(frame1, prunedMatches[i].queryIdx, lm));
			ms.push_back(make_shared<Measurement>(frame2, prunedMatches[i].trainIdx, lm));
		}
	}
	//check if enough triangulation results
	if( lms.size() < MIN_BASE_TRIANGULATE){
		cout<<" too few base triangulation: "<<lms.size()<<" target: "<<MIN_BASE_TRIANGULATE<<endl;
		return false;
	}

	t_triangulate = double(clock()-time) / CLOCKS_PER_SEC;

	time = clock();

	data.addFrame(frame1);
	data.addFrame(frame2);
	for(vector<LandMark::Ptr>::iterator it = lms.begin(); it!=lms.end(); ++it){
		data.addLandMark(*it);
	}
	for(vector<Measurement::Ptr>::iterator it = ms.begin(); it!=ms.end(); ++it){
		data.addMeasurement(*it);
	}

	t_postprocess = double(clock()-time) / CLOCKS_PER_SEC;

	cout<<"Time(s) recover pose = "<<t_poserecover<<endl;
	cout<<"Time(s) triangulate  = "<<t_triangulate<<endl;
	cout<<"Time(s) postprocess  = "<<t_postprocess<<endl;

	int cloudSize = data.countLandMarks();
	cout<<"\t cloud size = "<<cloudSize<<" ("<<cloudSize<<" new)"<<endl;
	return true;
}

// bool SFMPipeline::reconstructBasePair_gv(	Frame::Ptr 						&frame1,	//this will have identity camera projection
// 											Frame::Ptr						&frame2,
// 											const std::vector<cv::DMatch> 	&matches)
// {
// 	//check if matches are sufficient
// 	if(matches.size() < MIN_5PTALGO){
// 		cout<<" too few matches for essential mat: "<<matches.size()<<" target: "<<MIN_5PTALGO<<endl;
// 		return false;
// 	}else{
// 		cout<<matches.size()<<" matches"<<endl;
// 	}

// 	cout<<"initializing camera"<<endl;
// 	Mat tmp;
// 	if(frame1->img.empty() && frame2->img.empty()){
// 		tmp 	= imread(frame1->imgRoot+"/"+frame1->imgName,IMREAD_GRAYSCALE);
// 	}else if(frame1->img.empty()){
// 		tmp		= frame2->img;
// 	}else{
// 		tmp		= frame1->img;
// 	}
// 	Camera &camera = Camera::GetInstance();
// 	camera.w = tmp.cols;
// 	camera.h = tmp.rows;
// 	double foc = (camera.w>camera.h)? (double) camera.w: (double) camera.h;
// 	double ppx = camera.w/2.0;
// 	double ppy = camera.h/2.0;
// 	double camMatArr[9] = { foc, 	0.0, 		ppx,
// 							0.0, 	foc, 		ppy,
// 							0.0,	0.0,		1.0 };
// 	//assume no distorition
// 	double distortCoeffArr[5] 	= { 0, 0, 0, 0, 0 };
// 	camera.camMat 				= Mat(3, 3, CV_64F, camMatArr).clone();
// 	camera.distortionMat 		= Mat(1, 5, CV_64F, distortCoeffArr).clone();
// 	cout<<camera.camMat<<endl;
// 	cout<<camera.distortionMat<<endl;

// 	cout<<"(opengv) reconstruct base pair ["<<frame1->imgIdx<<"]"<<frame1->imgName<<" & "<<"["<<frame2->imgIdx<<"]"<<frame2->imgName<<endl;
// 	assert(!data.getFrame(frame1->imgIdx) && !data.getFrame(frame2->imgIdx));

// 	clock_t				time;
// 	double				t_poserecover;
// 	double				t_triangulate;
// 	double				t_postprocess;

// 	time = clock();

// 	bearingVectors_t 	bearingVectors1, bearingVectors2;
// 	vector<KeyPoint>	&kpts1 = frame1->kpts;
// 	vector<KeyPoint>	&kpts2 = frame2->kpts;
// 	Camera::GetInstance().getBearingVectors(kpts1, kpts2, matches, bearingVectors1, bearingVectors2);
	
// 	cout<<"bearing vector size ="<<bearingVectors1.size()<<endl;
// 	assert(bearingVectors1.size() == bearingVectors2.size());
// 	relative_pose::CentralRelativeAdapter adapter(bearingVectors1,bearingVectors2);
// 	std::shared_ptr<sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(
// 	  new sac_problems::relative_pose::CentralRelativePoseSacProblem(
// 	  adapter,
// 	  sac_problems::relative_pose::CentralRelativePoseSacProblem::EIGHTPT,//NISTER, //STEWENIUS, SEVENPT, or EIGHTPT
// 	  true)//use random seed
// 	);
	
// 	sac::Ransac<sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;
	
// 	ransac.sac_model_ = relposeproblem_ptr;
// 	ransac.threshold_ = 2.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
// 	ransac.max_iterations_ = 1000;
// 	int debugLvl 	  = 1;

// 	//compute relative pose
// 	cout<<"ok1"<<endl;
// 	ransac.computeModel(debugLvl);
// 	cout<<"ok2"<<endl;
// 	cout<<"probability:"<<ransac.probability_<<endl;

// 	vector<int> inliers = ransac.inliers_;
// 	//check if enough inliers
// 	if( inliers.size() < MIN_BASE_TRIANGULATE){
// 		cout<<" too few inliers for base triangulation: "<<inliers.size()<<" target: "<<MIN_BASE_TRIANGULATE<<endl;
// 		return false;
// 	}

// 	translation_t	 	position_recovered;	//translation from frame 1 to 2
// 	rotation_t		 	rotation_recovered;	//rotation from frame 1 to 2 XXX: not what the documentation said!!!
// 	position_recovered = ransac.model_coefficients_.col(3)/ransac.model_coefficients_.col(3).norm();	//normalized for the base pair
// 	rotation_recovered.col(0) = ransac.model_coefficients_.col(0);
// 	rotation_recovered.col(1) = ransac.model_coefficients_.col(1);
// 	rotation_recovered.col(2) = ransac.model_coefficients_.col(2);

// 	/*
// 	transformation_t nonlinear_transformation = relative_pose::optimize_nonlinear(adapter);

// 	position_recovered = nonlinear_transformation.col(3)/ nonlinear_transformation.col(3).norm();
// 	rotation_recovered.col(0) = nonlinear_transformation.col(0);
// 	rotation_recovered.col(1) = nonlinear_transformation.col(1);
// 	rotation_recovered.col(2) = nonlinear_transformation.col(2);

// 	adapter.setR12(rotation_recovered);
// 	adapter.sett12(position_recovered);*/

// 	//set frame pose, leave frame 1's pose to default
// 	frame1->fixed	 = true;
// 	frame2->position = position_recovered;
// 	frame2->rotation = Quaterniond(rotation_recovered).conjugate();	//set rotation back to no rotation
// 	frame2->rotation.normalize();	//always remember to normalize quaternion

// 	t_poserecover = double(clock()-time) / CLOCKS_PER_SEC;


// 	time = clock();
// 	//tmp array to store triangulated points and measures.
// 	//do not immediately add to data cos we may not have enough triangulations
// 	vector<LandMark::Ptr> 		lms;
// 	vector<Measurement::Ptr>	ms;

// 	adapter.setR12(rotation_recovered);	//set rotation from frame 1 to 2 XXX:not what the documentation said!!!
// 	adapter.sett12(position_recovered);	//set translation from frame 1 to 2
// 	double f			= Camera::GetInstance().getCamFocal();
// 	for(vector<int>::iterator it = inliers.begin(); it!=inliers.end(); ++it){
// 		bool valid, parallel;
// 		int featureIdx1 = matches[*it].queryIdx;
// 		int featureIdx2 = matches[*it].trainIdx;
// 		double kpt1Size	= frame1->kpts[featureIdx1].size;
// 		double kpt1std	= 0.8*kpt1Size/12.0;
// 		double ray1Sigma= sqrt(sqrt(2))*kpt1std/f;
// 		double kpt2Size	= frame2->kpts[featureIdx2].size;
// 		double kpt2std	= 0.8*kpt2Size/12.0;
// 		double ray2Sigma= sqrt(sqrt(2))*kpt2std/f;
// 		double raySigma = max(ray1Sigma, ray2Sigma);
// 		point_t point = triangulate3(adapter, *it, raySigma, valid, parallel);//triangulation::triangulate2( adapter, *it);

// 		if(valid && !parallel){
// 			//check reprojection error
// 			double error1	= computeReprojectionError(frame1, point, featureIdx1);
// 			double error2	= computeReprojectionError(frame2, point, featureIdx2);
// 			if(error1>REPROJERROR_THRESH || error2>REPROJERROR_THRESH){
// 				cout<<"== base pair triangulation large error, should pay attention to"<<endl;
// 				continue;
// 			}
// 		}else{
// 			cout<<"triangulation invalid"<<endl;
// 			continue;
// 		}
// 		LandMark::Ptr lm(new LandMark(point));
// 		lms.push_back(lm);

// 		Measurement::Ptr m1(new Measurement(frame1, matches[*it].queryIdx, lm));
// 		Measurement::Ptr m2(new Measurement(frame2, matches[*it].trainIdx, lm));
// 		ms.push_back(m1);
// 		ms.push_back(m2);

// 		/*if(	cheiralityCheck_gv(point,frame1->position, frame1->rotation) &&
// 			cheiralityCheck_gv(point,frame2->position, frame2->rotation)){

// 			LandMark::Ptr lm(new LandMark(point));
// 			lms.push_back(lm);

// 			Measurement::Ptr m1(new Measurement(frame1, matches[*it].queryIdx, lm));
// 			Measurement::Ptr m2(new Measurement(frame2, matches[*it].trainIdx, lm));
// 			ms.push_back(m1);
// 			ms.push_back(m2);
// 		}*/
// 	}
// 	//check if enough triangulation results
// 	if( lms.size() < MIN_BASE_TRIANGULATE){
// 		cout<<" too few base triangulation: "<<lms.size()<<" target: "<<MIN_BASE_TRIANGULATE<<endl;
// 		return false;
// 	}
// 	t_triangulate = double(clock()-time) / CLOCKS_PER_SEC;

// 	time = clock();

// 	//actually add data
// 	data.addFrame(frame1);
// 	data.addFrame(frame2);
// 	for(vector<LandMark::Ptr>::iterator it = lms.begin(); it!=lms.end(); ++it){
// 		data.addLandMark(*it);
// 	}
// 	for(vector<Measurement::Ptr>::iterator it = ms.begin(); it!=ms.end(); ++it){
// 		data.addMeasurement(*it);
// 	}

// 	t_postprocess = double(clock()-time) / CLOCKS_PER_SEC;

// 	cout<<"Time(s) recover pose = "<<t_poserecover<<endl;
// 	cout<<"Time(s) triangulate  = "<<t_triangulate<<endl;
// 	cout<<"Time(s) postprocess  = "<<t_postprocess<<endl;

// 	int cloudSize = data.countLandMarks();
// 	cout<<"\t cloud size = "<<cloudSize<<" ("<<cloudSize<<" new)"<<endl;

// 	return true;
// }


// Eigen::Vector3d SFMPipeline::triangulate3(	const opengv::relative_pose::RelativeAdapterBase & 	adapter,
// 		    								size_t 												index,
// 											double												sigma,
// 											bool& isValid,
// 											bool& isParallel)
// {
// 	isParallel = false; // This should be the default.
// 	// But parallel and invalid is not the same. Points at infinity are valid and parallel.
// 	isValid = false; // hopefully this will be reset to true.

// 	// stolen and adapted from the Kneip toolchain geometric_vision/include/geometric_vision/triangulation/impl/triangulation.hpp
// 	translation_t t12 = adapter.gett12();
// 	rotation_t R12 = adapter.getR12();
// 	bearingVector_t f1 = adapter.getBearingVector1(index);
// 	bearingVector_t f2 = adapter.getBearingVector2(index);

// 	bearingVector_t f2_unrotated = R12 * f2;
// 	Eigen::Vector2d b;
// 	b[0] = t12.dot(f1);
// 	b[1] = t12.dot(f2_unrotated);
// 	Eigen::Matrix2d A;
// 	A(0,0) = f1.dot(f1);
// 	A(1,0) = f1.dot(f2_unrotated);
// 	A(0,1) = -A(1,0);
// 	A(1,1) = -f2_unrotated.dot(f2_unrotated);

// 	if (A(1, 0) < 0.0) {
// 		A(1, 0) = -A(1, 0);
// 		A(0, 1) = -A(0, 1);
// 		// wrong viewing direction
// 		cout<<"wrong viewing direction"<<endl;
// 	};

// 	bool invertible;
// 	Eigen::Matrix2d A_inverse;
// 	A.computeInverseWithCheck(A_inverse, invertible, 1.0e-6);
// 	Eigen::Vector2d lambda = A_inverse * b;
// 	if (!invertible) {
// 		isParallel = true; // let's note this.
// 		// parallel. that's fine. but A is not invertible. so handle it separately.
// 		if ((f1.cross(f2_unrotated)).norm() < 6 * sigma){
// 		   isValid = true;  // check parallel
// 		}
// 		Vector4d homo((f1[0] + f2_unrotated[0]) / 2.0, (f1[1] + f2_unrotated[1]) / 2.0,
// 				(f1[2] + f2_unrotated[2]) / 2.0, 1e-3);
// 		Vector3d point(homo[0]/homo[3], homo[1]/homo[3], homo[2]/homo[3]);
// 		//return (Eigen::Vector4d((f1[0] + f2_unrotated[0]) / 2.0, (f1[1] + f2_unrotated[1]) / 2.0, (f1[2] + f2_unrotated[2]) / 2.0, 1e-3).normalized());
// 		return point;
// 	}

// 	Eigen::Vector3d xm = lambda[0] * f1;
// 	Eigen::Vector3d xn = t12 + lambda[1] * f2_unrotated;
// 	Eigen::Vector3d midpoint = (xm + xn) / 2.0;

// 	// check it
// 	Eigen::Vector3d error = midpoint - xm;
// 	Eigen::Vector3d diff = midpoint - ( 0.5 * t12);
// 	const double diff_sq = diff.dot(diff);
// 	const double chi2 = error.dot(error) * (1.0 / (diff_sq * sigma * sigma));

// 	isValid = true;
// 	if (chi2 > 9) {
// 		isValid = false;  // reject large chi2-errors
// 	}

// 	// flip if necessary
// 	if (diff.dot(f1) < 0) {
// 		midpoint = (0.5 * t12) - diff;
// 	}

// 	return midpoint;
// }

void SFMPipeline::epipolarConstrainedMatchMask(	const int 				imgIdx1,	//from a point in img1
												const int				imgIdx2,	//to possible epipolar matches in img2
												cv::Mat					&mask)
{
	Frame::Ptr frame1 = data.getFrame(imgIdx1);
	Frame::Ptr frame2 = data.getFrame(imgIdx2);
	assert(frame1 && frame2);

	//identify matched and unmatched keypoints between frame1 & 2
	vector<Measurement::Ptr> ms1 = data.getMeasurements(frame1);
	vector<unsigned char> matched1(frame1->kpts.size(), 0);	//0 for unmatched
	//vector<unsigned char> matched2(frame2->kpts.size(), 0);	//0 for unmatched
	int matchedCnt = 0;
	for(vector<Measurement::Ptr>::iterator it = ms1.begin(); it != ms1.end(); ++it){
		LandMark::Ptr lm 	= (*it)->landmark;
		Measurement::Ptr m2 = data.getMeasurement(lm, frame2);
		if(m2){
			matched1[(*it)->featureIdx] 	= 1;
			//matched2[m2->featureIdx]		= 1;
			matchedCnt++;
		}
	}

	cout<<"matchedCnt:"<<matchedCnt<<endl;

	//return empty mask when one of the frame has all available features matched to the other
	if(matchedCnt == frame1->kpts.size() || matchedCnt == frame2->kpts.size()){
		mask = Mat();
		return;
	}

	//use cloud depth range viewed by frame1 to determine epipolar search length
	//extend the search range 1.5 times and in front of frame 1 camera
	const vector<LandMark::Ptr> &lms = data.getLandMarks();
	double minZ = 1;
	double maxZ = 1;
	bool zInitialized = false;
	for(vector<LandMark::Ptr>::const_iterator it = lms.begin(); it!=lms.end(); ++it){
		Vector3d pt = frame1->rotation*((*it)->pt - frame1->position);
		if(!zInitialized){
			minZ = maxZ = pt[2];
			zInitialized = true;
		}else{
			if(pt[2]<minZ){
				minZ = pt[2];
			}else if(pt[2]>maxZ){
				maxZ = pt[2];
			}
		}
	}
	if(!zInitialized){
		minZ = 1;	//camera plane z=1
		maxZ = EPIPOLAR_SEARCH_DEPTH_THRESH;
	}
	double midZ = (maxZ+minZ)/2;
	double halfD= (maxZ-minZ)/2;
	if(halfD<EPIPOLAR_SEARCH_DEPTH_THRESH/2){
		halfD	= EPIPOLAR_SEARCH_DEPTH_THRESH/2;
	}
	minZ		= midZ-halfD*1.5;
	maxZ		= midZ+halfD*1.5;
	double f	= Camera::GetInstance().getCamFocal();
	int	imgW	= Camera::GetInstance().w;
	int	imgH	= Camera::GetInstance().h;

	if (minZ<1) minZ=1;


	int r		= frame1->decs.rows;
	int c		= frame2->decs.rows;
	mask		= Mat(r, c, CV_8UC1, Scalar(0));
	//unsigned char *maskData = mask.data;
	for(unsigned int i = 0; i<frame1->kpts.size(); i++){
		if(matched1[i]) continue;
		//find feature bearing vector
		Vector3d vec1 	= Camera::GetInstance().getBearingVector(frame1->kpts[i]);
		//find 3D line in frame1's coordinate
		Vector3d lStart1= vec1*minZ;
		Vector3d lEnd1	= vec1*maxZ;
		//find 3D line in world coordinate
		Vector3d lStart	= frame1->rotation.conjugate()*lStart1+frame1->position;
		Vector3d lEnd	= frame1->rotation.conjugate()*lEnd1+frame1->position;
		//find 3D line in frame2's coordinate.
		Vector3d lStart2= frame2->rotation*(lStart-frame2->position);
		Vector3d lEnd2	= frame2->rotation*(lEnd-frame2->position);
		//if the line segment is completely behind frame2 z=1
		if(lStart2[2]<1 && lEnd2[2]<1) continue;
		//if the line intersects frame2 z=1
		if(lStart2[2]<1 || lEnd2[2]<1)
		{
			Vector3d direction 	= lEnd2-lStart2;
			Vector3d intersection = lStart2+direction*((1.0-lStart2[2])/direction[2]);
			if(lStart2[2]<1) lStart2 = intersection;
			else lEnd2 = intersection;
		}

		//some precalculations to prevent duplicated calculation when calculating distance to epipolar line
		//for a 2D line defined by two points p1, p2, the distance of another point p0 on the plane to that line is:
		//abs((p2.y-p1.y)*p0.x - (p2.x-p1.x)*p0.y + p2.x*p1.y - p2.y*p1.x)/magnitude(p2-p1)
		Vector2d p1 	= Camera::GetInstance().project(lStart2);
		Vector2d p2 	= Camera::GetInstance().project(lEnd2);
		Vector2d diff 	= p2-p1;
		double grad		= diff[1]/diff[0];
		double grad_inv	= diff[0]/diff[1];
		//if p1 or p2 lies outside the image boundary, find the intersection points
		if(p1[0]<0){
			p1[1]		+= -p1[0]*grad;
			p1[0]		= 0;
		}
		if(p1[0]>=imgW){
			p1[1]		+= (imgW-1-p1[0])*grad;
			p1[0]		= imgW-1;
		}
		if(p1[1]<0){
			p1[0]		+= -p1[1]*grad_inv;
			p1[1]		= 0;
		}
		if(p1[1]>=imgH){
			p1[0]		+= (imgH-1-p1[1])*grad_inv;
			p1[1]		= imgH-1;
		}
		if(p2[0]<0){
			p2[1]		+= -p2[0]*grad;
			p2[0]		= 0;
		}
		if(p2[0]>=imgW){
			p2[1]		+= (imgW-1-p2[0])*grad;
			p2[0]		= imgW-1;
		}
		if(p2[1]<0){
			p2[0]		+= -p2[1]*grad_inv;
			p2[1]		= 0;
		}
		if(p2[1]>=imgH){
			p2[0]		+= (imgH-1-p2[1])*grad_inv;
			p2[1]		= imgH-1;
		}
		diff			= p2-p1;
		double diffNorm = diff.norm();
		double term3	= p2[0]*p1[1]-p2[1]*p1[0];
		//determine the image col range to search
		//extend the search range by distance threshold
		int xMin2, xMax2;
		if(p1[0]<=p2[0]){
			xMin2		= std::max(0,int(std::floor(double(p1[0]-EPIPOLAR_SEARCH_PIXEL_THRESH))));
			xMax2		= std::min(imgW-1, int(std::ceil(double(p2[0]+EPIPOLAR_SEARCH_PIXEL_THRESH))));
		}else{
			xMin2		= std::max(0,int(std::floor(double(p2[0]-EPIPOLAR_SEARCH_PIXEL_THRESH))));
			xMax2		= std::min(imgW-1, int(std::ceil(double(p1[0]+EPIPOLAR_SEARCH_PIXEL_THRESH))));
		}
		//determine the image row range to search
		//extend the search range by distance threshold
		int yMin2, yMax2;
		if(p1[1]<=p2[1]){
			yMin2		= std::max(0,int(std::floor(double(p1[1]-EPIPOLAR_SEARCH_PIXEL_THRESH))));
			yMax2		= std::min(imgH-1, int(std::ceil(double(p2[1]+EPIPOLAR_SEARCH_PIXEL_THRESH))));
		}else{
			yMin2		= std::max(0,int(std::floor(double(p2[1]-EPIPOLAR_SEARCH_PIXEL_THRESH))));
			yMax2		= std::min(imgH-1, int(std::ceil(double(p1[1]+EPIPOLAR_SEARCH_PIXEL_THRESH))));
		}
		//get feature search start and end idxs
		int feIdxStart	= frame2->kptLUT[yMin2];
		if(feIdxStart >= frame2->kpts.size()) continue;
		int feIdxEnd	= frame2->kptLUT[yMax2];
		if(feIdxEnd >= frame2->kpts.size()) feIdxEnd = frame2->kpts.size()-1;
		if(feIdxEnd<feIdxStart) continue;

		//create mask to be used for matching
		for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
			Point2f &pt	= frame2->kpts[j].pt;
			//skip if not between the horizontal range
			if(pt.x<xMin2 || pt.x>xMax2) continue;
			//add to candidate if along epipolar line
			double dist = std::fabs(double(diff[1]*pt.x - diff[0]*pt.y+term3))/diffNorm;
			if(dist<=EPIPOLAR_SEARCH_PIXEL_THRESH){
				mask.at<unsigned char>(i,j)=1;
			}
		}
	}
}

void SFMPipeline::addMoreLandMarksAndMeasures(	Frame::Ptr				&frame1,	//both images must have pose available
												Frame::Ptr				&frame2)	//order does not matter
{
	cout<<"add more landmarks between ["<<frame1->imgIdx<<"]"<<" & ["<<frame2->imgIdx<<"]"<<endl;
	clock_t				time;
	double				t_match;
	double				t_triangulate;
	double 				t_postprocess;

	vector<Measurement::Ptr> newMs;
	vector<LandMark::Ptr>	 newLms;

	time = clock();

	//robustly get good matches between two images{
	// cout<<">>>>>>>>>>>>>>>>>>>>>>"<<endl;
	//step1: get matches which are bidirectional and much better than second candidate match
	vector<DMatch> raw_matches;
	matchFeatures(frame1->decs, frame2->decs, raw_matches);
		//step1 (alternative): since we already have pose prior, we can constain our match along epipolar line
		//but this actually produce quite a lot of false matches, so abandoned, just document it here {
		// Mat mask12, mask21;
		// epipolarConstrainedMatchMask(frame1->imgIdx, frame2->imgIdx, mask12);
		// epipolarConstrainedMatchMask(frame2->imgIdx, frame1->imgIdx, mask21);
		// if(! (mask12.empty() || mask21.empty()) ){
		// 	matchFeatures(frame1->decs, frame2->decs, raw_matches, mask12, mask21);
		// }
		//}
	//step2: find essential matrix and prune matches by inliers
	vector<Point2f> 	pts1, pts2;
	Utils::Matches2Points(frame1->kpts,frame2->kpts,raw_matches,pts1,pts2);
	double f = Camera::GetInstance().getCamFocal();
	Point2d pp = Camera::GetInstance().getCamPrinciple();
	Mat 				E, inliers;
	E = findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);
	// cout<<"("<<inliers.rows<<','<<inliers.cols<<")"<<countNonZero(inliers)<<endl;
	vector<DMatch> 		matches;
	for(int i=0; i<inliers.rows; i++){
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			matches.push_back(raw_matches[i]);
		}
	}
	// cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
	//}

	vector<DMatch> matchesForTriangulation;

	//for matches that has one of the features already matched, add measurement,
	//else add to triangulation list
	for(vector<DMatch>::iterator it = matches.begin(); it!=matches.end(); ++it){
		Measurement::Ptr m1 = data.getMeasurement(frame1, it->queryIdx);
		Measurement::Ptr m2 = data.getMeasurement(frame2, it->trainIdx);
		if(m1 && m2){
			//both points are measured, ignore
		}else if(m1){
			//frame 1 feature has a measure, frame 2 feature has no measure
			if ( !(data.getMeasurement(m1->landmark, frame2)) ){
				//frame 1 feature's measured landmark has no other measure in frame 2
				m2.reset();
				m2 = make_shared<Measurement>(frame2, it->trainIdx, m1->landmark);
				newMs.push_back(m2);
			}
		}else if(m2){
			//frame 2 feature has a measure, frame 1 feature has no measure
			if ( !(data.getMeasurement(m2->landmark, frame1)) ){
				//frame 2 feature's measured landmark has no other measure in frame 1
				m1.reset();
				m1 = make_shared<Measurement>(frame1, it->queryIdx, m2->landmark);
				newMs.push_back(m1);
			}
			
		}else{
			matchesForTriangulation.push_back(*it);
		}
	}
	t_match = double(clock()-time) / CLOCKS_PER_SEC;

	cout<<"matches="<<matches.size()<<" for triangulation="<<matchesForTriangulation.size()<<" new measurements="<<newMs.size()<<endl;
	// if(matches.empty()){
	// 	cout<<"no more landmarks and measures to add between ["<<frame1->imgIdx<<"]"<<" & ["<<frame2->imgIdx<<"]"<<endl;
	// 	return;
	// }

	time = clock();
	if (!matchesForTriangulation.empty()){
		// vector<Point2f> 	pts1, pts2;
		Utils::Matches2Points(frame1->kpts, frame2->kpts, matchesForTriangulation, pts1, pts2);
		vector<Point3f>		pts3D;
		// Mat 				inliers;
		Matx34d	P1			= frame1->getCVTransform();
		Matx34d	P2			= frame2->getCVTransform();
		triangulate(pts1,pts2,P1,P2,pts3D,inliers);

		assert(pts1.size() == inliers.cols);
		assert(inliers.type() == 0);	//unsigned char 8U
		vector<LandMark::Ptr> lms;
		vector<Measurement::Ptr> ms;
		for(unsigned int i = 0; i<inliers.cols; i++){	//unlike essential mat, inliers are stored in columns rather than rows
			unsigned char val = inliers.at<unsigned char>(i);
			if(val){
				Vector3d pt(pts3D[i].x, pts3D[i].y, pts3D[i].z);
				LandMark::Ptr lm = make_shared<LandMark>(pt);
				newLms.push_back(lm);
				newMs.push_back(make_shared<Measurement>(frame1, matchesForTriangulation[i].queryIdx, lm));
				newMs.push_back(make_shared<Measurement>(frame2, matchesForTriangulation[i].trainIdx, lm));
			}
		}
	}

	t_triangulate = double(clock()-time) / CLOCKS_PER_SEC;

	time = clock();
	for(vector<LandMark::Ptr>::iterator it = newLms.begin(); it!=newLms.end(); ++it){
		data.addLandMark(*it);
	}
	for(vector<Measurement::Ptr>::iterator it = newMs.begin(); it!=newMs.end(); ++it){
		data.addMeasurement(*it);
	}
	t_postprocess = double(clock()-time) / CLOCKS_PER_SEC;

	cout<<"Time(s) match             = "<<t_match<<endl;
	cout<<"Time(s) triangulate       = "<<t_triangulate<<endl;
	cout<<"Time(s) postprocess       = "<<t_postprocess<<endl;

	cout<<"added "<<newLms.size()<<" landmarks "<<newMs.size()<<" measurements"<<endl;
}

//precondition: accurate pose already obtained for both input frames
void SFMPipeline::addMoreLandMarksAndMeasures(	const int 		imgIdx1,	//both images must have pose available
												const int		imgIdx2)	//order does not matter
{
	cout<<"add more landmarks between ["<<imgIdx1<<"]"<<imgNames[imgIdx1]<<" & ["<<imgIdx2<<"]"<<imgNames[imgIdx2]<<endl;
	Frame::Ptr frame1 = data.getFrame(imgIdx1);
	Frame::Ptr frame2 = data.getFrame(imgIdx2);
	assert(frame1 && frame2);

	addMoreLandMarksAndMeasures(frame1, frame2);
}

// //precondition: accurate pose already obtained for both input frames
// void SFMPipeline::addMoreLandMarksAndMeasures_gv(	const int 		imgIdx1,	//both images must have pose available
// 													const int		imgIdx2)	//order does not matter
// {
// 	cout<<"add more landmarks between ["<<imgIdx1<<"]"<<imgNames[imgIdx1]<<" & ["<<imgIdx2<<"]"<<imgNames[imgIdx2]<<endl;
// 	Frame::Ptr frame1 = data.getFrame(imgIdx1);
// 	Frame::Ptr frame2 = data.getFrame(imgIdx2);
// 	assert(frame1 && frame2);

// 	clock_t				time;
// 	double				t_epipolarSearch;
// 	double				t_match;
// 	double				t_triangulate;
// 	double 				t_postprocess;

// 	vector<Measurement::Ptr> newMs;
// 	vector<LandMark::Ptr>	 newLms;

// 	time = clock();
// 	Mat mask12, mask21;
// 	epipolarConstrainedMatchMask(imgIdx1, imgIdx2, mask12);
// 	epipolarConstrainedMatchMask(imgIdx2, imgIdx1, mask21);

// 	if(mask12.empty() || mask21.empty()){
// 		cout<<"no more landmarks and measures to add between ["<<imgIdx1<<"]"<<imgNames[imgIdx1]<<" & ["<<imgIdx2<<"]"<<imgNames[imgIdx2]<<endl;
// 		return;
// 	}
// 	t_epipolarSearch = double(clock()-time) / CLOCKS_PER_SEC;

// 	time = clock();
// 	//match features
// 	vector<DMatch> matches;
// 	matchFeatures(frame1->decs, frame2->decs, matches, mask12, mask21);

// 	vector<DMatch> matchesForTriangulation;

// 	//for matches that has one of the features already matched, add measurement,
// 	//else add to triangulation list
// 	for(vector<DMatch>::iterator it = matches.begin(); it!=matches.end(); ++it){
// 		Measurement::Ptr m1 = data.getMeasurement(frame1, it->queryIdx);
// 		Measurement::Ptr m2 = data.getMeasurement(frame2, it->trainIdx);
// 		if(!m1 && !m2){
// 			matchesForTriangulation.push_back(*it);
// 		}else if(m1 && m2){
// 			//both point is measured but not to the same landmark
// 			//ignore
// 		}else if(m1){
// 			double error = computeReprojectionError(frame2, m1->landmark->pt, it->trainIdx);
// 			if(error<=REPROJERROR_THRESH){
// 				m2.reset(new Measurement(frame2, it->trainIdx, m1->landmark));
// 				newMs.push_back(m2);
// 			}
// 		}else if(m2){
// 			double error = computeReprojectionError(frame1, m2->landmark->pt, it->queryIdx);
// 			if(error<=REPROJERROR_THRESH){
// 				m1.reset(new Measurement(frame1, it->queryIdx, m2->landmark));
// 				newMs.push_back(m1);
// 			}
// 		}
// 	}
// 	t_match = double(clock()-time) / CLOCKS_PER_SEC;

// 	cout<<"matches="<<matches.size()<<" for triangulation="<<matchesForTriangulation.size()<<" old measurements="<<newMs.size()<<endl;


// 	time = clock();
// 	//find relative pose using frame1 as identity
// 	rotation_t R1 		= frame1->rotation.toRotationMatrix();
// 	rotation_t R1_inv	= R1.transpose();
// 	rotation_t R2 		= frame2->rotation.toRotationMatrix();
// 	translation_t &t1 	= frame1->position;
// 	translation_t &t2 	= frame2->position;
// 	translation_t t 	= t2-t1;	//translation from frame1 to frame2
// 	rotation_t R		= R1_inv*R2;//rotation from frame 2 to frame 1
// 	rotation_t R_inv	= R.transpose();	//for some reason, opengv R12 is transposed

// 	double f			= Camera::GetInstance().getCamFocal();
// 	//add new points
// 	bearingVectors_t bearingVectors1, bearingVectors2;
// 	Camera::GetInstance().getBearingVectors(frame1->kpts, frame2->kpts, matchesForTriangulation, bearingVectors1, bearingVectors2);
// 	relative_pose::CentralRelativeAdapter adapter(bearingVectors1,bearingVectors2, t, R_inv);
// 	for(unsigned int i=0; i<bearingVectors1.size(); i++){
// 		bool valid, parallel;
// 		int featureIdx1 = matchesForTriangulation[i].queryIdx;
// 		int featureIdx2 = matchesForTriangulation[i].trainIdx;
// 		double kpt1Size	= frame1->kpts[featureIdx1].size;
// 		double kpt1std	= 0.8*kpt1Size/12.0;
// 		double ray1Sigma= sqrt(sqrt(2))*kpt1std/f;
// 		double kpt2Size	= frame2->kpts[featureIdx2].size;
// 		double kpt2std	= 0.8*kpt2Size/12.0;
// 		double ray2Sigma= sqrt(sqrt(2))*kpt2std/f;
// 		double raySigma = max(ray1Sigma, ray2Sigma);
// 		point_t point 	= triangulate3(adapter, i, raySigma, valid, parallel);//triangulation::triangulate2( adapter, i);
// 		//transform point to world coordinate
// 		point			= R1_inv*point+t1;
// 		if(valid && !parallel){
// 			//check reprojection error
// 			double error1	= computeReprojectionError(frame1, point, featureIdx1);
// 			double error2	= computeReprojectionError(frame2, point, featureIdx2);
// 			if(error1>REPROJERROR_THRESH || error2>REPROJERROR_THRESH){
// 				cout<<"triangulation has large error"<<endl;
// 				continue;
// 			}
// 		}else{
// 			cout<<"triangulation invalid"<<endl;
// 			continue;
// 		}

// 		LandMark::Ptr lm(new LandMark(point));
// 		newLms.push_back(lm);

// 		Measurement::Ptr m1(new Measurement(frame1, matchesForTriangulation[i].queryIdx, lm));
// 		Measurement::Ptr m2(new Measurement(frame2, matchesForTriangulation[i].trainIdx, lm));
// 		newMs.push_back(m1);
// 		newMs.push_back(m2);

// 		/*if(	cheiralityCheck_gv(point,frame1->position, frame1->rotation) &&
// 			cheiralityCheck_gv(point,frame2->position, frame2->rotation)){

// 			LandMark::Ptr lm(new LandMark(point));
// 			newLms.push_back(lm);

// 			Measurement::Ptr m1(new Measurement(frame1, matchesForTriangulation[i].queryIdx, lm));
// 			Measurement::Ptr m2(new Measurement(frame2, matchesForTriangulation[i].trainIdx, lm));
// 			newMs.push_back(m1);
// 			newMs.push_back(m2);
// 		}*/
// 	}

// 	t_triangulate = double(clock()-time) / CLOCKS_PER_SEC;

// 	time = clock();
// 	for(vector<LandMark::Ptr>::iterator it = newLms.begin(); it!=newLms.end(); ++it){
// 		data.addLandMark(*it);
// 	}
// 	for(vector<Measurement::Ptr>::iterator it = newMs.begin(); it!=newMs.end(); ++it){
// 		data.addMeasurement(*it);
// 	}
// 	t_postprocess = double(clock()-time) / CLOCKS_PER_SEC;

// 	cout<<"Time(s) epipolar search   = "<<t_epipolarSearch<<endl;
// 	cout<<"Time(s) match             = "<<t_match<<endl;
// 	cout<<"Time(s) triangulate       = "<<t_triangulate<<endl;
// 	cout<<"Time(s) postprocess       = "<<t_postprocess<<endl;

// 	cout<<"added "<<newLms.size()<<" landmarks "<<newMs.size()<<" measurements"<<endl;
// }

//precondition: a rough pose of the input frame already obtained
bool SFMPipeline::addMoreMeasures	( 	Frame::Ptr				&frame)
{
	int measuresBefore = data.countMeasurements();

	vector<Measurement::Ptr> newMs = positionFrame(frame,true);
	if(newMs.empty()){
		return false;
	}else{
		bundleAdjustmentLocal(newMs);
		for(vector<Measurement::Ptr>::iterator it = newMs.begin(); it!=newMs.end(); ++it){
			data.addMeasurement(*it);
		}
	}

	int measuresAfter = data.countMeasurements();
	assert(measuresAfter - measuresBefore == newMs.size());
	cout<<"\t new measurements = "<<measuresAfter<<" ("<<newMs.size()<<" new)"<<endl;

	return true;
}

std::vector<Measurement::Ptr>
SFMPipeline::positionFrame		(	Frame::Ptr				&frame,			//frame can be either added or not added to data
									const bool				&usePosePrior)
{
	clock_t				time;
	double				t_findNewMeasures;

	time = clock();

	//maps to keep track of the matches found and for update matches by distance comparison
	map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare> lm2match;
	map<int, LandMark::Ptr>	fe2lmk;

	bool frameExists = (data.getFrame(frame->imgIdx) != nullptr);

	if(usePosePrior){
		//have pose prior. used when tracking
		int MAX_UNMEASURED_POINTS 	= 1000;
		int SEARCH_RADIUS			= 10;
		//randomly pick at most max unmeasured points
		//data.shuffleLandMarks();

		Camera &camera = Camera::GetInstance();

		const vector<LandMark::Ptr> &lmks = data.getLandMarks();
		for(vector<LandMark::Ptr>::const_iterator it = lmks.begin(); it!=lmks.end(); ++it){
			assert(!((*it)->deleted));

			if(frameExists){
				//if frame was already added, check if the landmark was already measured
				Measurement::Ptr checkm = data.getMeasurement(*it,frame);
				if(checkm)	continue;
			}

			//project landmark to current frame
			Camera::ProjectionStatus s;
			Vector2d pos = camera.project(frame,(*it)->pt, s);
			//skip if landmark is behind camera plane z=1
			if(s == Camera::ProjectionStatus::Behind) continue;
			//skip if out of image given search radius
			if(camera.outofbound(pos, SEARCH_RADIUS)) continue;
			//get feature search range around pos
			int xMin				= pos[0]-SEARCH_RADIUS;
			int xMax				= pos[0]+SEARCH_RADIUS;
			int yMin				= pos[1]-SEARCH_RADIUS;
			int yMax				= pos[1]+SEARCH_RADIUS;
			if(xMin<0) xMin = 0;
			if(yMin<0) yMin = 0;
			if(xMax>=camera.w) xMax = camera.w-1;
			if(yMax>=camera.h) yMax = camera.h-1;
			int feIdxStart			= frame->kptLUT[yMin];
			if(feIdxStart >= frame->decs.rows) continue;
			int feIdxEnd			= frame->kptLUT[yMax];
			if(feIdxEnd >= frame->decs.rows) feIdxEnd = frame->decs.rows-1;
			if(feIdxEnd<feIdxStart) continue;
			//get landmark's measured feature descriptors, not many
			const vector<Measurement::Ptr> ms = data.getMeasurements((*it));
			Mat decs1;
			for(vector<Measurement::Ptr>::const_iterator jt = ms.begin(); jt!=ms.end(); ++jt){
				assert((*jt)->frame.get() != frame.get());
				decs1.push_back((*jt)->frame->decs.row((*jt)->featureIdx));
			}

			assert(!decs1.empty());
			//get current frame's feature descriptors in search range
			//important: need to record the original feature idxs
			vector<int> originalFeatureIdxs;
			Mat decs2;
			for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
				//skip if already measured. can happen when input frame was already added and measured to some landmark
				if(frame->measured[j]){
					//assert(data.getMeasurement(frame,j));
					continue;
				}
				//skip if not between the horizontal range
				Point2f &pt	= frame->kpts[j].pt;
				if(pt.x<xMin || pt.x>xMax) continue;
				decs2.push_back(frame->decs.row(j));
				originalFeatureIdxs.push_back(j);
			}
			if(decs2.empty()) continue;

			//match
			BFMatcher matcher(NORM_HAMMING,false);
			vector<vector<DMatch> > matches;
			matcher.knnMatch(decs1,decs2,matches,2);

			//ratio test
			for(vector<vector<DMatch> >::iterator jt = matches.begin(); jt != matches.end(); ++jt){
				if((*jt).size()==1 || (*jt)[0].distance/(*jt)[1].distance < MATCH_RATIO){
					int feIdx = originalFeatureIdxs[(*jt)[0].trainIdx];
					assert(!(data.getMeasurement(frame,feIdx)));
					//we swap train and query feature idx to be consistent with the other scenario
					DMatch match(feIdx, (*jt)[0].queryIdx, (*jt)[0].distance);
					assert(feIdx == match.queryIdx);
					map<int, LandMark::Ptr>::iterator ft = fe2lmk.find(feIdx);
					if(ft==fe2lmk.end()){
						//feature not measured
						map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mt = lm2match.find(*it);
						if(mt!=lm2match.end()){
							assert(mt->first.get() == (*it).get());
							//the landmark has been measured to another feature, happens when the landmarks sees more than 1 measure
							if(match.distance < mt->second.distance){
								int &existFeIdx = mt->second.queryIdx;
								fe2lmk.erase(existFeIdx);	//warning: this makes ft invalid
								mt->second = match;
								fe2lmk[feIdx] = mt->first;
							}
						}else{
							//the landmark has not been measured
							lm2match[*it] = match;	//warning: this makes mt invalid
							fe2lmk[feIdx] = *it;	//warning: this makes ft invalid
						}
					}else{
						//feature already measured to another landmark
						map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mt = lm2match.find(ft->second);
						map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mtNew = lm2match.find(*it);
						//by design the landmark must also hold the best match to this feature
						assert(mt!=lm2match.end());
						assert(mt->second.queryIdx == match.queryIdx);	//TODO: fix crash here
						if(match.distance < mt->second.distance){
							if(mtNew!=lm2match.end()){
								//the new landmark has been measured to another feature, happens when the landmarks sees more than 1 measure
								if(match.distance < mtNew->second.distance){
									//only replace the match if the two existing matches are weaker than new one
									int &existFeIdxNew 	= mtNew->second.queryIdx;
									fe2lmk.erase(existFeIdxNew);	//this does not invalidate ft, unless existFeIdxNew == feIdx
									lm2match.erase(mt);				//warning: this makes mt invalid, does not invalidate mtNew unless mtNew == mt
									lm2match[*it] 			= match;//to play safe, we do not reuse mtNew pointer
									fe2lmk[match.queryIdx] 	= *it;	//to play safe, we do not reuse ft pointer
								}
							}else{
								//the new landmark has not been measured to any other feature
								lm2match.erase(mt);		//warning: this makes mt invalid
								lm2match[*it] = match;
								ft->second = *it;
							}
						}
					}
				}
			}

			//assertion check
			for(map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator jt = lm2match.begin(); jt != lm2match.end(); ++jt){
				map<int, LandMark::Ptr>::iterator ft = fe2lmk.find(jt->second.queryIdx);
				assert(ft!=fe2lmk.end());
				assert(ft->second.get() == jt->first.get());
			}
			for(map<int, LandMark::Ptr>::iterator jt=fe2lmk.begin(); jt!=fe2lmk.end(); ++jt){
				map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mt = lm2match.find(jt->second);
				assert(mt!=lm2match.end());
				assert(mt->second.queryIdx == jt->first);
			}
		}
	}else{
		cout<<"match with all"<<endl;
		//no pose prior, try match with all frames. used when tracking is lost
		assert(!(data.getFrame(frame->imgIdx)));
		const vector<Frame::Ptr> &frames = data.getFrames();
		for(vector<Frame::Ptr>::const_iterator it = frames.begin(); it!=frames.end(); ++it){
			vector<DMatch> matches;
			matchFrames(frame, *it, matches);
			for(vector<DMatch>::iterator jt = matches.begin(); jt!=matches.end(); ++jt){
				int &newFeIdx 				= jt->queryIdx;
				Measurement::Ptr m 			= data.getMeasurement((*it),jt->trainIdx);
				assert(m && !(m->deleted));
				LandMark::Ptr &newLmk		= m->landmark;
				float &newDist				= jt->distance;

				map<int,LandMark::Ptr>::iterator ft = fe2lmk.find(newFeIdx);
				map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mt = lm2match.find(newLmk);

				if(ft != fe2lmk.end() && mt != lm2match.end()){
					//both the feature and landmark has been measured before, may be 1 measurement or 2 measurements
					const int &existFeIdx1			= ft->first;
					LandMark::Ptr &existLmk1		= ft->second;
					float	&existDist1				= lm2match[existLmk1].distance;
					int &existFeIdx2				= mt->second.queryIdx;
					const LandMark::Ptr &existLmk2	= mt->first;
					float 	&existDist2				= mt->second.distance;

					if(existFeIdx1 == existFeIdx2 && existLmk1.get() == existLmk2.get()){
						//the same measure
						assert(existDist1 == existDist2);
						if(newDist<existDist1){
							//stronger measure
							lm2match.erase(mt);
							fe2lmk.erase(ft);
							lm2match[newLmk]		= *jt;
							fe2lmk[newFeIdx]		= newLmk;
						}else{
							//do nothing
						}
					}else{
						//two completely separate measures
						//not possible to have same feature but different landmark or vise versa cos we ensured 1 feature 1 landmark match
						assert(existFeIdx1 != existFeIdx2 && existLmk1.get() != existLmk2.get());
						if(newDist<existDist1 && newDist<existDist2){
							//only replace existing ones if is stronger than both
							//erase exist1
							fe2lmk.erase(ft);
							lm2match.erase(existLmk1);
							//erase exist2
							lm2match.erase(mt);
							fe2lmk.erase(existFeIdx2);
							//add new
							lm2match[newLmk]		= *jt;
							fe2lmk[newFeIdx]		= newLmk;
						}else{
							//do nothing, even if stronger than one of the existing one
						}
					}
				}else if(ft != fe2lmk.end()){
					//feature has been measured before
					const int &existFeIdx			= ft->first;
					LandMark::Ptr &existLmk			= ft->second;
					float	&existDist				= lm2match[existLmk].distance;
					if(newDist<existDist){
						fe2lmk.erase(ft);
						lm2match.erase(existLmk);
						lm2match[newLmk]			= *jt;
						fe2lmk[newFeIdx]			= newLmk;
					}else{
						//do nothing
					}
				}else if(mt != lm2match.end()){
					//landmark has been measured before
					int &existFeIdx					= mt->second.queryIdx;
					const LandMark::Ptr &existLmk	= mt->first;
					float 	&existDist				= mt->second.distance;
					if(newDist<existDist){
						lm2match.erase(mt);
						fe2lmk.erase(existFeIdx);
						lm2match[newLmk]			= *jt;
						fe2lmk[newFeIdx]			= newLmk;
					}else{
						//do nothing
					}
				}else{
					//both have not been measured
					lm2match[newLmk]	= *jt;
					fe2lmk[newFeIdx]	= newLmk;
				}
			}
		}
	}

	std::vector<Measurement::Ptr> unprunedMs;
	if(frameExists){
		//combine existing measures with new measures for pnp
		vector<Measurement::Ptr> existingMs = data.getMeasurements(frame);

		int totalMsCnt = lm2match.size()+existingMs.size();
		//check minimum matches for pnp
		if(totalMsCnt<MIN_3D4PNP){
			cout<<"not enough measures for pnp :"<<totalMsCnt<<" ("<<MIN_3D4PNP<<" required)"<<endl;
			return unprunedMs;
		}else{
			cout<<"total measures for pnp found: "<<totalMsCnt<<" ("<<lm2match.size()<<" new)"<<endl;
		}

		unprunedMs.swap(existingMs);	//swap content of vectors without copying

		for(map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator it = lm2match.begin(); it!=lm2match.end(); ++it){
			unprunedMs.push_back(make_shared<Measurement>(frame,it->second.queryIdx, it->first));
		}

	}else{
		//check minimum matches for pnp
		if(lm2match.size()<MIN_3D4PNP){
			cout<<"not enough measures for pnp :"<<lm2match.size()<<" ("<<MIN_3D4PNP<<" required)"<<endl;
			return unprunedMs;
		}else{
			cout<<"total measures for pnp found: "<<lm2match.size()<<endl;
		}

		for(map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator it = lm2match.begin(); it!=lm2match.end(); ++it){
			unprunedMs.push_back(make_shared<Measurement>(frame,it->second.queryIdx, it->first));
		}
	}

	t_findNewMeasures = double(clock()-time) / CLOCKS_PER_SEC;
	cout<<"Time(s) find measure  = "<<t_findNewMeasures<<endl;

	vector<Measurement::Ptr> prunedMs = solvePnP(frame, unprunedMs);	//frame pose will be updated
	if(frameExists){
		vector<Measurement::Ptr> newMs;
		for(vector<Measurement::Ptr>::const_iterator it = prunedMs.begin(); it!=prunedMs.end(); ++it){
			if(!(data.getMeasurement((*it)->landmark,frame))){
				newMs.push_back(*it);
			}
		}
		return newMs;
	}else{
		return prunedMs;
	}

	//return solvePnP_gv(frame, unprunedMs,vector<DMatch>()); //frame pose will be updated

	//unprunedMs needs to be cleared but since its scope is within this function, it will be automatically cleared
}

vector<Measurement::Ptr>											//do not add frame to data.frames
SFMPipeline::solvePnP(		Frame::Ptr							&frame,
							const vector<Measurement::Ptr>		&unprunedMs,
							const double						error,
							const double 						confi,
							const int							iter)
{
	cout<<"(opencv) solve pnp"<<endl;
	vector<Measurement::Ptr> newMs;

	if(unprunedMs.size()<MIN_3D4PNP){
		return newMs;
	}

	clock_t				time;
	double				t_poserecover;
	double				t_postprocess;

	time = clock();

	vector<Point3f> pts3D;
	vector<Point2f> pts2D;
	pts3D.reserve(unprunedMs.size());
	pts2D.reserve(unprunedMs.size());
	for(vector<Measurement::Ptr>::const_iterator it = unprunedMs.begin(); it!=unprunedMs.end(); ++it){
		Vector3d &pt3D_eigen 	= (*it)->landmark->pt;
		pts3D.push_back(Point3f(pt3D_eigen[0], pt3D_eigen[1], pt3D_eigen[2]));
		assert((*it)->frame.get() == frame.get());
		pts2D.push_back(frame->kpts[(*it)->featureIdx].pt);
	}

	bool useExtrinsicGuess	= false;//true;
	int iterationsCount		= iter;
	float reprojectionError	= error;
	double confidence		= confi;
	int flags				= SOLVEPNP_ITERATIVE;
	Mat &camMat				= Camera::GetInstance().camMat;
	Mat &distortionMat 		= Camera::GetInstance().distortionMat;
	Mat	rvec, t, inliers;
	solvePnPRansac(pts3D,pts2D,camMat,distortionMat,rvec,t,useExtrinsicGuess,iterationsCount,reprojectionError,confidence,inliers,flags);

	//check minimum inliers
	if(inliers.rows<1){
		cout<<"not enough pnp inliers:"<<inliers.rows<<" (1 required)"<<endl;
		return newMs;
	}else{
		cout<<"inliers:"<<inliers.rows<<"/"<<pts3D.size()<<endl;
	}
	/*if(!inOutMatches.empty()){
		vector<DMatch> prunedMatches;
		for(int i=0; i<inliers.rows; i++){
			int val = (int)inliers.at<int>(i);	//XXX:the inlier is not mask but index, of type CV_SC1 signed int
			prunedMatches.push_back(inOutMatches[val]);
		}
		inOutMatches.swap(prunedMatches);
	}*/

	//get camera poses
	Mat R;
	Rodrigues(rvec,R);

	t_poserecover = double(clock()-time) / CLOCKS_PER_SEC;

	time = clock();

	//its important that you specify rowmajor when mapping eigen to opencv, because eigen uses column major by default
	//if you directly map Matrix3d to opencv matx, the matrix will be transposed because Matrix3d is default column major
	//for eigen vectors, there is no distinction between row or column so Vector3d will do
	Map<Matrix<double, 3,3, RowMajor>> 	R_eigen((double *) R.data);	//no copy data
	Map<Vector3d> 						t_eigen((double *) t.data);	//no copy data
	frame->rotation = Quaterniond(R_eigen);						//copy data
	frame->rotation.normalize();
	frame->position = -R_eigen.transpose()*t_eigen;				//copy data


	for(unsigned int i=0; i<inliers.rows; i++){
		int val = (int)inliers.at<int>(i);	//XXX:the inlier is not mask but index, of type CV_SC1 signed int
		newMs.push_back(unprunedMs[val]);
	}

	t_postprocess = double(clock()-time) / CLOCKS_PER_SEC;

	cout<<"Time(s) recover pose  = "<<t_poserecover<<endl;
	cout<<"Time(s) postprocess   = "<<t_postprocess<<endl;

	return newMs;
}

// std::vector<Measurement::Ptr>											//do not add frame to data.frames
// SFMPipeline::solvePnP_gv(	Frame::Ptr							&frame,
// 							const std::vector<Measurement::Ptr>	&unprunedMs,
// 							const double						thresh,
// 							const double 						confi,
// 							const int							iter,
// 							const bool 							useAll)
// {
// 	cout<<"(opengv) solve pnp"<<endl;
// 	vector<Measurement::Ptr> newMs;

// 	if(unprunedMs.size()<3){
// 		return newMs;
// 	}

// 	clock_t				time;
// 	double				t_poserecover;
// 	double				t_postprocess;

// 	time = clock();
// 	Camera &camera 		= Camera::GetInstance();
// 	bearingVectors_t bearingVecs;
// 	points_t mPoints;
// 	bearingVecs.reserve(unprunedMs.size());
// 	mPoints.reserve(unprunedMs.size());
// 	for(vector<Measurement::Ptr>::const_iterator it = unprunedMs.begin(); it!=unprunedMs.end(); ++it){
// 		assert((*it)->frame.get() == frame.get());
// 		bearingVecs.push_back(camera.getBearingVector(frame->kpts[(*it)->featureIdx]));
// 		mPoints.push_back((*it)->landmark->pt);
// 	}

// 	//create adaptor
// 	absolute_pose::CentralAbsoluteAdapter adapter(bearingVecs, mPoints);
// 	transformation_t transformation;

// 	if(useAll){
// 		cout<<"use all"<<endl;
// 		transformation_t epnp_transformation = absolute_pose::epnp(adapter);
// 		t_poserecover = double(clock()-time) / CLOCKS_PER_SEC;
// 	}else{
// 		cout<<"use ransac"<<endl;
// 		//create problem
// 		std::shared_ptr<sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
// 			new sac_problems::absolute_pose::AbsolutePoseSacProblem(
// 			adapter, sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP, //(KNEIP, GAO, or EPNP)
// 			true)
// 		);

// 		// create a Ransac object
// 		sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
// 		ransac.sac_model_ 		= absposeproblem_ptr;
// 		ransac.threshold_ 		= thresh;//2.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)))*100.0;
// 		ransac.max_iterations_ 	= iter;
// 		ransac.probability_		= confi;
// 		int debugLvl 	  		= 1;

// 		//Run ransac
// 		ransac.computeModel(debugLvl);
// 		cout<<"probability:"<<ransac.probability_<<endl;
// 		transformation = ransac.model_coefficients_;
// 		t_poserecover = double(clock()-time) / CLOCKS_PER_SEC;

// 		//check minimum inliers
// 		vector<int> inliers = ransac.inliers_;
// 		if(inliers.empty()){
// 			cout<<"not enough pnp inliers:"<<inliers.size()<<" (1 required)"<<endl;
// 			return newMs;
// 		}else{
// 			cout<<"inliers:"<<inliers.size()<<"/"<<mPoints.size()<<endl;
// 			for(vector<int>::iterator it = inliers.begin(); it!=inliers.end(); ++it){
// 				/*double error = computeReprojectionError(frame, unprunedMs[*it]->landmark->pt, unprunedMs[*it]->featureIdx);
// 				if(error>REPROJERROR_THRESH){
// 					cout<<"bad measurement"<<endl;
// 					continue;
// 				}*/
// 				newMs.push_back(unprunedMs[*it]);
// 			}
// 		}

// 		/*if(!inOutMatches.empty()){
// 			vector<DMatch> prunedMatches;
// 			for(vector<int>::iterator it = inliers.begin(); it!=inliers.end(); ++it){
// 				prunedMatches.push_back(inOutMatches[*it]);
// 			}
// 			inOutMatches.swap(prunedMatches);
// 		}*/

		
// 	}

// 	time = clock();
// 	translation_t	 	position_recovered;	//translation from world origin to frame 1
// 	rotation_t		 	rotation_recovered;	//rotation from world axis to frame 1 XXX: not like what the documentation said!!!

// 	position_recovered = transformation.col(3);	//do not normalize, not like the base pair
// 	rotation_recovered.col(0) = transformation.col(0);
// 	rotation_recovered.col(1) = transformation.col(1);
// 	rotation_recovered.col(2) = transformation.col(2);
	
// 	//set frame pose
// 	frame->position = position_recovered;
// 	frame->rotation = Quaterniond(rotation_recovered).conjugate();	//rotation from frame 1 back to no rotation
// 	frame->rotation.normalize();


// 	t_postprocess = double(clock()-time) / CLOCKS_PER_SEC;

// 	cout<<"Time(s) recover pose  = "<<t_poserecover<<endl;
// 	cout<<"Time(s) postprocess   = "<<t_postprocess<<endl;

// 	return newMs;

// }

// vector<Measurement::Ptr>
// SFMPipeline::optimizeCamPose_gv(	Frame::Ptr							&frame,
// 									const vector<Measurement::Ptr>		&ms,
// 									const double 						errorThresh,
// 									const int							iter,
// 									const int							maxOctave)
// {
// 	cout<<"(opengv) optimize pose"<<endl;

// 	clock_t				time;
// 	double				t_optimization;

// 	time = clock();
// 	Camera &camera 		= Camera::GetInstance();
// 	vector<Measurement::Ptr> inlierMs;
// 	unsigned char isInlier[ms.size()];
// 	int inlierCnt						= ms.size();
// 	bearingVectors_t 	bearingVecs;
// 	points_t 			mPoints;

// 	cout<<"optimize pose average error before = "<<computeMeanReprojectionError(ms)<<endl;

// 	Vector3d t		= frame->position;
// 	Matrix3d R		= frame->rotation.conjugate().toRotationMatrix();

// 	int i = 0;
// 	while(true){

// 		//update inlier
// 		inlierCnt = 0;
// 		for(unsigned int j = 0; j<ms.size(); j++){
// 			if(maxOctave !=-1 && frame->kpts[ms[j]->featureIdx].octave > maxOctave){
// 				isInlier[j] = 0;
// 				continue;
// 			}
// 			double error = computeReprojectionError(frame, ms[j]->landmark->pt, ms[j]->featureIdx);
// 			if(error<errorThresh){
// 				isInlier[j] = 1;
// 				inlierCnt++;
// 			}else{
// 				isInlier[j] = 0;
// 			}
// 		}

// 		if(inlierCnt<=inlierMs.size()) break;

// 		inlierMs.clear();
// 		inlierMs.reserve(inlierCnt);
// 		for(unsigned int j = 0; j<ms.size(); j++){
// 			if(isInlier[j]){
// 				inlierMs.push_back(ms[j]);
// 			}
// 		}

// 		if(iter>0 && i>= iter){
// 			break;
// 		}else{
// 			i++;
// 		}

// 		cout<<"optimize pose using "<<inlierMs.size()<<" measurements"<<endl;

// 		bearingVecs.clear();
// 		mPoints.clear();
// 		bearingVecs.reserve(inlierCnt);
// 		mPoints.reserve(inlierCnt);
// 		for(unsigned int j = 0; j<ms.size(); j++){
// 			assert(ms[j]->frame.get() == frame.get());
// 			if(isInlier[j] == 1){
// 				bearingVecs.push_back(camera.getBearingVector(frame->kpts[ms[j]->featureIdx]));
// 				mPoints.push_back(ms[j]->landmark->pt);
// 			}
// 		}


// 		// non-linear optimization (using all correspondences)
// 		//create adaptor
// 		absolute_pose::CentralAbsoluteAdapter adapter(bearingVecs, mPoints);
// 		adapter.sett(t);
// 		adapter.setR(R);

// 		//cout<<"R"<<endl<<adapter.getR()<<endl;
// 		//cout<<"t"<<endl<<adapter.gett()<<endl;

// 		transformation_t transform = absolute_pose::optimize_nonlinear(adapter);

// 		t			= transform.col(3);
// 		R			= transform.block(0,0,3,3);

// 		//cout<<transform<<endl;
// 		//cout<<"R"<<endl<<adapter.getR()<<endl;
// 		//cout<<"t"<<endl<<adapter.gett()<<endl;

// 		frame->position = t;
// 		frame->rotation = Quaterniond(R).conjugate();
// 		frame->rotation.normalize();

// 	}
// 	cout<<"optimize pose inliers = "<<inlierMs.size()<<endl;
// 	cout<<"optimize pose average error after = "<<computeMeanReprojectionError(inlierMs)<<endl;

// 	t_optimization = double(clock()-time) / CLOCKS_PER_SEC;
// 	cout<<"Time(s) optimization  = "<<t_optimization<<endl;

// 	return inlierMs;

// }

//precondition: input frame must have descriptors pre calculated & must not already be in the data
bool SFMPipeline::positionAndAddFrame(	Frame::Ptr				&frame)
{

	int measuresBefore = data.countMeasurements();
	vector<Measurement::Ptr> newMs = positionFrame(frame);

	if(newMs.empty()){
		return false;
	}else{
		bundleAdjustmentLocal(newMs);
		data.addFrame(frame);
		for(vector<Measurement::Ptr>::iterator it = newMs.begin(); it!=newMs.end(); ++it){
			data.addMeasurement(*it);
		}
	}

	int measuresAfter = data.countMeasurements();
	assert(measuresAfter - measuresBefore == newMs.size());
	cout<<"\t new measurements = "<<measuresAfter<<" ("<<newMs.size()<<" new)"<<endl;

	return true;
}

bool SFMPipeline::positionAndAddFrame(	const int 				imgIdx){
	cout<<"(opencv) position and add ["<<imgIdx<<"]"<<imgNames[imgIdx]<<endl;
	assert(!data.getFrame(imgIdx));
	Frame::Ptr frame = make_shared<Frame>(imgIdx, imgRoot, imgNames[imgIdx]);
	frame->init();
	return positionAndAddFrame(frame);
}

void SFMPipeline::reprojectionErrorCheck(
								const vector<Point3f> 	&pts3D,
								const vector<Point2f> 	&pts2D,
								const Matx34d			&P,
								float					&meanError,
								Mat						&isGood){

	assert(pts2D.size() == pts3D.size());
	Mat Pmat = Mat(P);
	Mat R(3,3,CV_64F);
	R.at<double>(0,0) = Pmat.at<double>(0,0);
	R.at<double>(0,1) = Pmat.at<double>(0,1);
	R.at<double>(0,2) = Pmat.at<double>(0,2);
	R.at<double>(1,0) = Pmat.at<double>(1,0);
	R.at<double>(1,1) = Pmat.at<double>(1,1);
	R.at<double>(1,2) = Pmat.at<double>(1,2);
	R.at<double>(2,0) = Pmat.at<double>(2,0);
	R.at<double>(2,1) = Pmat.at<double>(2,1);
	R.at<double>(2,2) = Pmat.at<double>(2,2);
	Mat rvec;
	Rodrigues(R,rvec);
	Mat t(1,3,CV_64F);
	t.at<double>(0)   = Pmat.at<double>(0,3);
	t.at<double>(1)   = Pmat.at<double>(1,3);
	t.at<double>(2)   = Pmat.at<double>(2,3);

	vector<Point2f> reprojected;
	Mat &camMat				= Camera::GetInstance().camMat;
	Mat &distortionMat 		= Camera::GetInstance().distortionMat;
	projectPoints(pts3D, rvec, t, camMat, distortionMat, reprojected);
	assert(reprojected.size() == pts3D.size());

	meanError = 0.0f;
	Mat errors(1,reprojected.size(),CV_32FC1);

	for(int j=0; j<reprojected.size(); j++){
		float error = (float) norm((reprojected[j]-pts2D[j]));	//distance between 2 points
		errors.at<float>(j) = error;
		meanError += error;
	}

	meanError = meanError/reprojected.size();

	float errorThresh 	= meanError*REPROJERROR_THRESH;

	isGood = (errors < errorThresh);


}
void SFMPipeline::triangulate(	const vector<Point2f> 	&pts2D1,
								const vector<Point2f> 	&pts2D2,
								const Matx34d			&P1,
								const Matx34d			&P2,
								vector<Point3f> 		&pts3D,
								Mat						&isGood)
{
	pts3D.clear();
	Mat pts4D; //will be of type cv_32f
	Mat pts1Mat = Mat(pts2D1).clone();		//clone to prevent modify the original input, though they were const
	Mat pts2Mat = Mat(pts2D2).clone();		//clone to prevent modify the original input, though they were const
	pts1Mat = pts1Mat.reshape(1, pts1Mat.rows);
	pts2Mat = pts2Mat.reshape(1, pts2Mat.rows);
	double f = Camera::GetInstance().getCamFocal();
	Point2d pp = Camera::GetInstance().getCamPrinciple();
	//standadize points by camera mat
	pts1Mat.col(0) = (pts1Mat.col(0) - pp.x) / f;
	pts2Mat.col(0) = (pts2Mat.col(0) - pp.x) / f;
	pts1Mat.col(1) = (pts1Mat.col(1) - pp.y) / f;
	pts2Mat.col(1) = (pts2Mat.col(1) - pp.y) / f;
	pts1Mat = pts1Mat.t();
	pts2Mat = pts2Mat.t();

	triangulatePoints(P1, P2, pts1Mat, pts2Mat, pts4D);
	convertPointsFromHomogeneous(pts4D.t(),pts3D);

	Mat isGood1, isGood2;

	//reprojectionErrorCheck
	float meanError1, meanError2;
	reprojectionErrorCheck(pts3D, pts2D1,P1,meanError1,isGood1);
	if(meanError1 > MEAN_ERROR_THRESH){
		isGood1 = Mat::zeros(1, pts3D.size(), CV_8UC1);
	}
	reprojectionErrorCheck(pts3D, pts2D2,P2,meanError2,isGood2);
	if(meanError2 > MEAN_ERROR_THRESH){
		isGood2 = Mat::zeros(1, pts3D.size(), CV_8UC1);
	}
	isGood = isGood1 & isGood2;

	//chieralityCheck
	float minDist = MIN_DIST_TO_CAM;
	float maxDist = MAX_DIST_TO_CAM;
	cheiralityCheck(pts3D,P1,minDist, maxDist, isGood1);
	cheiralityCheck(pts3D,P2,minDist, maxDist, isGood2);
	isGood = isGood & isGood1 & isGood2;

	int cnt = countNonZero(isGood);
	cout<<cnt<<"/"<<pts3D.size()<<" good triangulation"<<endl;

}
void SFMPipeline::cheiralityCheck(	const vector<Point3f> 	&pts3D,
									const Matx34d			&P,
									const float				&minDist,
									const float				&maxDist,
									Mat						&isGood){



	Mat pts4DMat; //4 channels type 29
	convertPointsToHomogeneous(pts3D, pts4DMat);
	pts4DMat = pts4DMat.reshape(1);	//convert to 1 channel, type 5, efficiency O(1)
	pts4DMat = pts4DMat.t();

	Mat PMat;
	Mat(P).convertTo(PMat,pts4DMat.type());
	pts4DMat = PMat*pts4DMat;	//bring the 3D points to camera's coordinate

	//check points' z-axis, mask points too near, too far and behind camera
	isGood = (pts4DMat.row(2) > minDist);
	isGood = (pts4DMat.row(2) < maxDist) & isGood;

	//cout<<"nonzero = "<<countNonZero(isGood)<<endl;

}

/*
void SFMPipeline::matchFeatures(const Mat &decs1, const Mat &decs2, vector<DMatch> &matches){
	matches.clear();
	vector<vector<DMatch> > matchesBidirection;
	double ratio = MATCH_RATIO;

	BFMatcher matcher(NORM_HAMMING,true); //true means cross check ie. bimatch

	//bimatch
	matcher.knnMatch(decs1,decs2,matchesBidirection,1);	//12 need closest 2 matches for ratio test later

	for(int i=0; i<matchesBidirection.size(); i++){
		if(matchesBidirection[i].size()>0){
		//if(matchesBidirection[i][0].distance/matchesBidirection[i][1].distance< ratio){
			matches.push_back(matchesBidirection[i][0]);
		//}
		}
	}

}
*/

void SFMPipeline::matchFrames(	const Frame::Ptr				&frame1,
								const Frame::Ptr				&frame2,
								std::vector<cv::DMatch> 		&matches)
{
	assert(frame1 && frame2);
	//cout<<"matching from ["<<frame1->imgIdx<<"]"<<frame1->imgName<<" to ["<<frame2->imgIdx<<"]"<<frame2->imgName<<endl;

	vector<KeyPoint> &kpts1 = frame1->kpts;
	vector<KeyPoint> &kpts2 = frame2->kpts;
	Mat 			 &decs1	= frame1->decs;
	Mat 			 &decs2	= frame2->decs;
	matches.clear();

	vector<Measurement::Ptr> ms1 = data.getMeasurements(frame1);
	vector<Measurement::Ptr> ms2 = data.getMeasurements(frame2);

	if(ms1.empty() && ms2.empty()){
		//free bruteforce match
		matchFeatures(decs1, decs2, matches);
	}else if(!ms1.empty() && !ms2.empty()){
		//find common measures
		for(vector<Measurement::Ptr>::iterator it = ms1.begin(); it != ms1.end(); ++it){
			LandMark::Ptr &lm 	= (*it)->landmark;
			Measurement::Ptr m2	= data.getMeasurement(lm,frame2);
			if(m2){
				matches.push_back(DMatch((*it)->featureIdx, m2->featureIdx, 0));
			}
		}

		/*
		//add measures
		Mat mask12, mask21;
		epipolarConstrainedMatchMask(frame1->imgIdx, frame2->imgIdx, mask12);
		epipolarConstrainedMatchMask(frame2->imgIdx, frame1->imgIdx, mask21);

		if(!(mask12.empty() || mask21.empty())){
			//match features
			vector<DMatch> matchesAdditional;
			matchFeatures(frame1->decs, frame2->decs, matchesAdditional, mask12, mask21);
			matches.insert( matches.end(), matchesAdditional.begin(), matchesAdditional.end());
		}*/


	}else{
		//find match to measured features
		int r,c;
		if(ms1.empty()){
			//cout<<"["<<frame2->imgIdx<<"]"<<frame2->imgName<<" was measured"<<endl;
			r = frame1->decs.rows;
			c = frame2->decs.rows;
		}else{
			//cout<<"["<<frame1->imgIdx<<"]"<<frame1->imgName<<" was measured"<<endl;
			r = frame2->decs.rows;
			c = frame1->decs.rows;
		}
		vector<Measurement::Ptr> &ms = (ms1.empty())?ms2:ms1;	//no copy
		Mat mask(r, c, CV_8UC1, Scalar(0));	//mask has size querydecsRow x traindecsRow
											//only mask.at<uchar>(i,j)!=0 will be considered for matching
		vector<int> measuredKptsIdxs;
		measuredKptsIdxs.reserve(ms.size());
		for(vector<Measurement::Ptr>::iterator it = ms.begin(); it != ms.end(); ++it){
			measuredKptsIdxs.push_back((*it)->featureIdx);
		}
		//sort indices in ascending order for memory access efficiency
		sort(measuredKptsIdxs.begin(), measuredKptsIdxs.end());

		unsigned char *maskData = mask.data;
		for(unsigned int i=0; i<r; i++){
			int offset = i*c;
			for(vector<int>::iterator it = measuredKptsIdxs.begin(); it!=measuredKptsIdxs.end(); ++it){
				maskData[offset+(*it)] = 1;
			}
		}
		if(ms2.empty()){
			mask = mask.t();
		}
		//cout<<mask<<endl;
		matchFeatures(decs1, decs2, matches, mask);
	}
}

void SFMPipeline::matchFeatures(const Mat &decs1, const Mat &decs2, vector<DMatch> &matches, const Mat &mask12, const Mat &mask21)
{
	matches.clear();

	clock_t	time = clock();

	if(mask12.empty() && mask21.empty()){
		cout<<"match type: all to all"<<endl;
		BFMatcher matcher(NORM_HAMMING,false);	//(NORM_L2,false);
												//from opencv 3.0 documentation:
												//One of NORM_L1, NORM_L2, NORM_HAMMING, NORM_HAMMING2. L1 and L2 norms are preferable choices for SIFT and SURF descriptors, NORM_HAMMING should be used with ORB, BRISK and BRIEF, NORM_HAMMING2 should be used with ORB when WTA_K==3 or 4 (see ORB::ORB constructor description)

		//matcher.match(decs1,decs2,matches);

		vector<vector<DMatch> > matches12;
		vector<vector<DMatch> > matches21;
		vector<DMatch> 			matches12RatioTested, matches21RatioTested;
		double ratio 			= MATCH_RATIO;

		//bimatch then ratio
		matcher.knnMatch(decs1,decs2,matches12,2);	//12 need closest 2 matches for ratio test later
		matcher.knnMatch(decs2,decs1,matches21,1);	//21 only need to find closest match
		vector<vector<DMatch> > matchesBidirection;
		for(int i=0; i<matches12.size(); i++){
			for(int j=0; j<matches21.size(); j++){
				if(	matches12[i][0].trainIdx == matches21[j][0].queryIdx &&
					matches12[i][0].queryIdx == matches21[j][0].trainIdx){
					matchesBidirection.push_back(matches12[i]);
					break;
				}
			}
		}
		for(int i=0; i<matchesBidirection.size(); i++){
			if(matchesBidirection[i][0].distance/matchesBidirection[i][1].distance< ratio){
				matches.push_back(matchesBidirection[i][0]);
			}
		}


		/*
		//ratio then bimatch
		vector<vector<DMatch> > matches2;
		matcher.knnMatch(decs1,decs2,matches12,2);
		for(int i=0; i<matches12.size(); i++){
			if(matches12[i][0].distance/matches12[i][1].distance< ratio){
				matches12RatioTested.push_back(matches12[i][0]);
			}
		}
		matcher.knnMatch(decs2,decs1,matches21,2);
		for(int i=0; i<matches21.size(); i++){
			if(matches21[i][0].distance/matches21[i][1].distance< ratio){
				matches21RatioTested.push_back(matches21[i][0]);
			}
		}
		for(int i=0; i<matches12RatioTested.size(); i++){
			for(int j=0; j<matches21RatioTested.size(); j++){
				if(matches12RatioTested[i].queryIdx == matches21RatioTested[j].trainIdx &&
				   matches12RatioTested[i].trainIdx == matches21RatioTested[j].queryIdx){
					matches.push_back(matches12RatioTested[i]);
					break;
				}
			}
		}
		*/
	}else if (mask21.empty()){
		cout<<"match type: 1 side masked"<<endl;
		BFMatcher matcher(NORM_HAMMING,false);	//for masked match, cannot use cross check
		//do cross checking in the hard way
		vector<DMatch> matches12;
		vector<DMatch> matches21;
		matcher.match(decs1,decs2,matches12,mask12);
		Mat mask_t = mask12.t();
		matcher.match(decs2,decs1,matches21,mask_t);

		for(vector<DMatch>::iterator it= matches12.begin(); it!=matches12.end(); ++it){
			for(vector<DMatch>::iterator jt= matches21.begin(); jt!=matches21.end(); ++jt){
				if(	(*it).trainIdx == (*jt).queryIdx &&
					(*it).queryIdx == (*jt).trainIdx){
					matches.push_back((*it));
					break;
				}
			}
		}
	}else{
		cout<<"match type: 2 sides masked"<<endl;
		BFMatcher matcher(NORM_HAMMING,false);	//for masked match, cannot use cross check
		//do cross checking in the hard way
		vector<DMatch> matches12;
		vector<DMatch> matches21;
		matcher.match(decs1,decs2,matches12,mask12);
		matcher.match(decs2,decs1,matches21,mask21);

		cout<<"matches12 size ="<<matches12.size()<<endl;
		cout<<"matches21 size ="<<matches21.size()<<endl;

		for(vector<DMatch>::iterator it= matches12.begin(); it!=matches12.end(); ++it){
			for(vector<DMatch>::iterator jt= matches21.begin(); jt!=matches21.end(); ++jt){
				if(	(*it).trainIdx == (*jt).queryIdx &&
					(*it).queryIdx == (*jt).trainIdx){
					matches.push_back((*it));
					break;
				}
			}
		}
	}
	double time_s = double(clock()-time) / CLOCKS_PER_SEC;
	cout<<"found "<<matches.size()<<" matches in "<<time_s<<" s"<<endl;

}

// void SFMPipeline::matchByBOW(	const Frame::Ptr 	&frame1,
// 								const Frame::Ptr 	&frame2,
// 								vector<DMatch> 		&matches,
// 								const double 		testRatio,
// 								const int			distThresh,
// 								const bool 			onlyMeasured)
// {
// 	matches.clear();

// 	clock_t				time 	= clock();

//     const DBoW3::FeatureVector &featVec1 			= data.computeFeatureVecs(frame1);
//     const DBoW3::FeatureVector &featVec2			= (data.getFrame(frame2->imgIdx)!=nullptr)? data.getFeatureVecs(frame2) : data.computeFeatureVecs(frame2);

//     // We perform the matching over features that belong to the same vocabulary node (at a certain level=4)
//     DBoW3::FeatureVector::const_iterator it1 		= featVec1.begin();
//     DBoW3::FeatureVector::const_iterator it2 		= featVec2.begin();
//     DBoW3::FeatureVector::const_iterator it1_end 	= featVec1.end();
//     DBoW3::FeatureVector::const_iterator it2_end 	= featVec2.end();

//     int kpts1Cnt = frame1->kpts.size();
//     vector<DMatch*> kpt2match(kpts1Cnt,nullptr);
//     //unsigned char matched[kpts1Cnt] = {0};
//     //map<int, DMatch>	idx12match;
//     //map<int, DMatch>	idx22match;

//     while(it2 != it2_end && it1 != it1_end)
//     {
//         if(it2->first == it1->first)
//         {
//         	//same feature vector node
//             const vector<unsigned int> kptIdxs2 = it2->second;
//             const vector<unsigned int> kptIdxs1 = it1->second;

//             for(size_t iKF=0; iKF<kptIdxs2.size(); iKF++)
//             {
//             	//for all node features in frame 2
//                 const unsigned int idx2 = kptIdxs2[iKF];
//                 if(onlyMeasured && frame2->measured[idx2]==0){
//                 	//feature in frame 2 does not have a measure, skip
// 					continue;
// 				}
//                 const cv::Mat &dKF= frame2->decs.row(idx2);

//                 int bestDist1=256;
//                 int bestIdxF =-1 ;
//                 int bestDist2=256;

//                 for(size_t iF=0; iF<kptIdxs1.size(); iF++)
//                 {
//                 	//for all node features in frame 1
//                     const unsigned int idx1 = kptIdxs1[iF];
//                     //if(matched[idx1]!=0) continue;
//                     const cv::Mat &dF = frame1->decs.row(idx1);
//                     //compute descriptor distance
//                     const int dist =  DetectorExtractor::descriptorDistance(dKF,dF);

//                     if(dist<bestDist1)
//                     {
//                         bestDist2=bestDist1;
//                         bestDist1=dist;
//                         bestIdxF=idx1;
//                     }
//                     else if(dist<bestDist2)
//                     {
//                         bestDist2=dist;
//                     }
//                 }

//                 if(bestDist1<distThresh && static_cast<double>(bestDist1)<testRatio*static_cast<double>(bestDist2)){
//                 	/*DMatch match(bestIdxF, idx2, bestDist1);
//                 	map<int, DMatch>::iterator itMatch1 = idx12match.find(match.queryIdx);
// 					map<int, DMatch>::iterator itMatch2 = idx22match.find(match.trainIdx);
// 					if(itMatch1 == idx12match.end() && itMatch2 == idx22match.end()){
// 						idx12match[match.queryIdx] 		= match;
// 						idx22match[match.trainIdx]		= match;
// 					}else if(itMatch1 != idx12match.end() && itMatch2 != idx22match.end()){
// 						assert(false);
// 						//the two should not be previously matched
// 						assert(itMatch1->second.trainIdx !=	match.trainIdx);
// 						assert(itMatch2->second.queryIdx != match.queryIdx);
// 						if(itMatch1->second.distance > bestDist1 && itMatch2->second.distance > bestDist1){
// 							idx12match.erase(itMatch2->second.queryIdx);
// 							idx22match.erase(itMatch1->second.trainIdx);
// 							itMatch1->second 			= match;
// 							itMatch2->second			= match;
// 						}
// 					}else if(itMatch1 != idx12match.end()){
// 						assert(itMatch1->second.trainIdx != match.trainIdx);
// 						if(itMatch1->second.distance > bestDist1){
// 							idx22match.erase(itMatch1->second.trainIdx);
// 							itMatch1->second			= match;
// 							idx22match[match.trainIdx] 	= match;
// 						}
// 					}else{
// 						assert(false);
// 						assert(itMatch2->second.queryIdx != match.queryIdx);
// 						if(itMatch2->second.distance > bestDist1){
// 							idx12match.erase(itMatch2->second.queryIdx);
// 							itMatch2->second			= match;
// 							idx12match[match.queryIdx] 	= match;
// 						}
// 					}*/

// 					//matched[bestIdxF] = bestDist1;
// 					//matches.push_back(match);

//                 	DMatch *prevMatch = kpt2match[bestIdxF];
// 					if(prevMatch!=nullptr){
// 						if(prevMatch->distance > bestDist1){
// 							delete prevMatch;
// 							prevMatch = nullptr;
// 							kpt2match[bestIdxF] = new DMatch(bestIdxF, idx2, bestDist1);
// 						}
// 					}else{
// 						kpt2match[bestIdxF] = new DMatch(bestIdxF, idx2, bestDist1);
// 					}
//                 }

//             }

//             it2++;
//             it1++;
//         }
//         else if(it2->first < it1->first)
//         {
//             it2 = featVec2.lower_bound(it1->first);
//         }
//         else
//         {
//             it1 = featVec1.lower_bound(it2->first);
//         }
//     }

// /*   //sanity check
//     assert(idx12match.size() == idx22match.size());
//     for(map<int, DMatch>::iterator itMatch1 = idx12match.begin(); itMatch1 != idx12match.end(); ++itMatch1 ){
//     	map<int, DMatch>::iterator itMatch2 = idx22match.find(itMatch1->second.trainIdx);
//     	assert(itMatch2!=idx22match.end());
//     	assert(itMatch2->second.trainIdx == itMatch1->second.trainIdx);
//     	assert(itMatch2->second.queryIdx == itMatch1->second.queryIdx);
//     	assert(itMatch2->second.distance == itMatch1->second.distance);
//     }
//     for(map<int, DMatch>::iterator itMatch2 = idx22match.begin(); itMatch2 != idx22match.end(); ++itMatch2 ){
// 		map<int, DMatch>::iterator itMatch1 = idx12match.find(itMatch2->second.queryIdx);
// 		assert(itMatch1!=idx12match.end());
// 		assert(itMatch2->second.trainIdx == itMatch1->second.trainIdx);
// 		assert(itMatch2->second.queryIdx == itMatch1->second.queryIdx);
// 		assert(itMatch2->second.distance == itMatch1->second.distance);
// 	}
//     for(map<int, DMatch>::iterator itMatch1 = idx12match.begin(); itMatch1 != idx12match.end(); ++itMatch1 ){
//     	matches.push_back(itMatch1->second);
//     }
// */
//     for(unsigned int i=0; i<kpts1Cnt; i++){
//     	DMatch *match = kpt2match[i];
//     	if(match!=nullptr){
//     		matches.push_back(*match);
//     		delete match;
//     		match = nullptr;
//     	}
//     }

// 	double t_matching 			= double(clock()-time) / CLOCKS_PER_SEC;
// 	cout<<"Time(s) bow matching   = "<<t_matching<< endl;

// }

void SFMPipeline::matchByProjection(		const Frame::Ptr 				&frame1,
											const Frame::Ptr 				&frame2,	//reference
											const int						radius,
											std::vector<cv::DMatch> 		&matches,
											const bool						excludeAlreadyMeasured)
{

	assert(data.getFrame(frame2->imgIdx));
	float mfNNratio = 0.8;

	Camera &camera = Camera::GetInstance();
	vector<unsigned char> matched;
	if(excludeAlreadyMeasured){
		matched = frame1->measured;
	}else{
		matched = vector<unsigned char>(frame1->kpts.size(), 0);
	}


	//for all measurements from frame 2
	vector<Measurement::Ptr> ms	= data.getMeasurements(frame2);
	for(vector<Measurement::Ptr>::iterator it = ms.begin(); it!=ms.end(); ++it){
		//get the corresponding landmark
		const LandMark::Ptr &lm = (*it)->landmark;
		//project landmark to frame1
		Camera::ProjectionStatus s;
		Vector2d pos = camera.project(frame1,lm->pt, s);
		//skip if landmark is behind camera plane z=1
		if(s == Camera::ProjectionStatus::Behind) continue;
		//skip if out of image given search radius
		if(camera.outofbound(pos, radius)) continue;
		//get feature search range around pos
		int xMin				= pos[0]-radius;
		int xMax				= pos[0]+radius;
		int yMin				= pos[1]-radius;
		int yMax				= pos[1]+radius;
		if(xMin<0) xMin = 0;
		if(yMin<0) yMin = 0;
		if(xMax>=camera.w) xMax = camera.w-1;
		if(yMax>=camera.h) yMax = camera.h-1;
		int feIdxStart			= frame1->kptLUT[yMin];
		if(feIdxStart >= frame1->decs.rows) continue;
		int feIdxEnd			= frame1->kptLUT[yMax];
		if(feIdxEnd >= frame1->decs.rows) feIdxEnd = frame1->decs.rows-1;
		if(feIdxEnd<feIdxStart) continue;
		//get frame2 feature
		const int &kptIdx2		= (*it)->featureIdx;
		const Mat &dec2			= frame2->decs.row(kptIdx2);
		//find best 2 matches
		int bestDist1=256;
		int bestIdxF =-1 ;
		int bestDist2=256;
		for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
			//skip if already matched
			if(matched[j]!=0) continue;
			//skip if not within the horizontal bound
			Point2f &pt	= frame1->kpts[j].pt;
			if(pt.x<xMin || pt.x>xMax) continue;
			//get frame1 feature
			const Mat &dec1		= frame1->decs.row(j);
			const int dist 		=  DetectorExtractor::descriptorDistance(dec1,dec2);
			if(dist<bestDist1)
			{
				bestDist2=bestDist1;
				bestDist1=dist;
				bestIdxF=j;
			}
			else if(dist<bestDist2)
			{
				bestDist2=dist;
			}
		}
		if(bestDist1<=128){
			if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
			{
				DMatch match(bestIdxF, kptIdx2, bestDist1);
				matched[bestIdxF] = 1;
				matches.push_back(match);
			}
		}
	}

}

vector<Measurement::Ptr>
SFMPipeline::getMoreMeasuresByProjection(	const Frame::Ptr 				&frame1,			//frame which you need more measures
											const Frame::Ptr 				&frame2,			//reference frame
											const vector<Measurement::Ptr> 	&existingMeasures,	//frame1 measures
											const int						searchThresh,
											const double					testRatio,
											const int						distThresh,
											const int						maxMatches,
											const bool						fixThresh)
{
	clock_t				time;
	double				t_preprocess;
	double				t_matching;
	vector<Measurement::Ptr> newMs;

	//preprocess
	time = clock();

	vector<Measurement::Ptr> 	ms2;
	vector<LandMark::Ptr> 		lms2;
	if(frame2){
		assert(data.getFrame(frame2->imgIdx));
		assert(data.getFrame(frame2->imgIdx).get() == frame2.get());
		ms2 = data.getMeasurements(frame2);
		assert(!(ms2.empty()));
		lms2.reserve(ms2.size());
		for(vector<Measurement::Ptr>::const_iterator it=ms2.begin(); it!=ms2.end(); ++it){
			lms2.push_back((*it)->landmark);
		}
		assert(ms2.size() == lms2.size());
	}else{
		if(maxMatches>0){
			data.shuffleLandMarks();
		}
	}

	const vector<LandMark::Ptr> &lmks = (frame2) ? lms2: data.getLandMarks();

	set<LandMark::Ptr, Data::LandMarkPtrCompare> measuredLms;
	for(vector<Measurement::Ptr>::const_iterator it = existingMeasures.begin(); it!=existingMeasures.end(); ++it){
		measuredLms.insert((*it)->landmark);
	}
	assert(measuredLms.size() == existingMeasures.size());
	t_preprocess = double(clock()-time) / CLOCKS_PER_SEC;


	//find new measures
	time = clock();
	//maps to keep track of the matches found and for update matches by distance comparison
	map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare> lm2match;
	map<int, LandMark::Ptr>	fe2lmk;
	Camera &camera 		= Camera::GetInstance();
	int N_lmk			= lmks.size();

	for(unsigned int i = 0; i<N_lmk; i++){
		assert(!(lmks[i]->deleted));
		//skip if landmark already measured
		if(!(measuredLms.empty()) && measuredLms.find(lmks[i])!=measuredLms.end()) continue;
		//project landmark to frame1
		Camera::ProjectionStatus s;
		Vector2d pos = camera.project(frame1,lmks[i]->pt, s);
		//skip if landmark is behind camera plane z=1
		if(s == Camera::ProjectionStatus::Behind) continue;

		DMatch bestMatch;

		if(testRatio<1){
			
			int bestQuery=-1 ;
			int bestTrain=-1 ;
			int bestFrame=-1 ;
			int bestDist1=256;
			int bestDist2=256;

			//get the landmark's recent measured kpts and decs
			const list<KeyPoint> 	&recentKpts = lmks[i]->recentKpts;
			const list<Mat> 		&recentDecs = lmks[i]->recentDecs;
			//try recent if it's not empty
			if(!frame2 && !recentKpts.empty()){
				list<KeyPoint>::const_iterator 	jtKpt 		= recentKpts.begin();
				list<KeyPoint>::const_iterator 	jtKpt_end 	= recentKpts.end();
				list<Mat>::const_iterator 		jtDec 		= recentDecs.begin();
				list<Mat>::const_iterator 		jtDec_end 	= recentDecs.end();
				while(jtKpt!=jtKpt_end){
					float searchRadius 		= (fixThresh) ? searchThresh : searchThresh*(jtKpt->octave+1);
					//skip if out of image given search radius
					if(camera.outofbound(pos, searchRadius)){
						++jtKpt;
						++jtDec;
						continue;
					} 

					//get feature search range around pos
					int xMin				= pos[0]-searchRadius;
					int xMax				= pos[0]+searchRadius;
					int yMin				= pos[1]-searchRadius;
					int yMax				= pos[1]+searchRadius;
					if(xMin<0) xMin = 0;
					if(yMin<0) yMin = 0;
					if(xMax>=camera.w) xMax = camera.w-1;
					if(yMax>=camera.h) yMax = camera.h-1;
					int feIdxStart			= frame1->kptLUT[yMin];
					if(feIdxStart >= frame1->decs.rows){
						++jtKpt;
						++jtDec;
						continue;
					} 
					int feIdxEnd			= frame1->kptLUT[yMax];
					if(feIdxEnd >= frame1->decs.rows) feIdxEnd = frame1->decs.rows-1;
					if(feIdxEnd<feIdxStart) {
						++jtKpt;
						++jtDec;
						continue;
					}

					for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
						//skip if already measured. can happen when input frame was already added and measured to some landmark
						if(frame1->measured[j]) continue;
						//skip if not within the horizontal bound
						Point2f &pt	= frame1->kpts[j].pt;
						if(pt.x<xMin || pt.x>xMax) continue;
						//get frame1 feature
						const Mat &dec1		= frame1->decs.row(j);

						const int dist 		=  DetectorExtractor::descriptorDistance(dec1,*jtDec);
						if(dist<bestDist1)
						{
							bestDist2=bestDist1;
							bestDist1=dist;
							bestQuery=j;
						}
						else if(dist<bestDist2)
						{
							bestDist2=dist;
						}
					}

					++jtKpt;
					++jtDec;
				}

				if(bestDist1<distThresh && static_cast<float>(bestDist1)<testRatio*static_cast<float>(bestDist2)){
					bestMatch = DMatch(bestQuery, bestTrain, bestFrame, bestDist1);
				}
			}

			if(bestMatch.queryIdx == -1){
				// no match found
				//find best match, using frame2 or all frames if no frame2
				const vector<Measurement::Ptr> &ms = (frame2) ? vector<Measurement::Ptr>(1,ms2[i]) : data.getMeasurements(lmks[i]);

				for(vector<Measurement::Ptr>::const_iterator kt = ms.begin(); kt!=ms.end(); ++kt){
					Frame::Ptr &otherFrame	= (*kt)->frame;
					const int otherKptIdx	= (*kt)->featureIdx;
					const Mat &otherDec		= (*kt)->frame->decs.row(otherKptIdx);
					float searchRadius 		= (fixThresh) ? searchThresh : searchThresh*(otherFrame->kpts[otherKptIdx].octave+1);
					//skip if out of image given search radius
					if(camera.outofbound(pos, searchRadius)) continue;
					//get feature search range around pos
					int xMin				= pos[0]-searchRadius;
					int xMax				= pos[0]+searchRadius;
					int yMin				= pos[1]-searchRadius;
					int yMax				= pos[1]+searchRadius;
					if(xMin<0) xMin = 0;
					if(yMin<0) yMin = 0;
					if(xMax>=camera.w) xMax = camera.w-1;
					if(yMax>=camera.h) yMax = camera.h-1;
					int feIdxStart			= frame1->kptLUT[yMin];
					if(feIdxStart >= frame1->decs.rows) continue;
					int feIdxEnd			= frame1->kptLUT[yMax];
					if(feIdxEnd >= frame1->decs.rows) feIdxEnd = frame1->decs.rows-1;
					if(feIdxEnd<feIdxStart) continue;

					for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
						//skip if already measured. can happen when input frame was already added and measured to some landmark
						if(frame1->measured[j]) continue;
						//skip if not within the horizontal bound
						Point2f &pt	= frame1->kpts[j].pt;
						if(pt.x<xMin || pt.x>xMax) continue;
						//get frame1 feature
						const Mat &dec1		= frame1->decs.row(j);

						const int dist 		=  DetectorExtractor::descriptorDistance(dec1,otherDec);
						if(dist<bestDist1)
						{
							bestDist2=bestDist1;
							bestDist1=dist;
							bestQuery=j;
							bestTrain=otherKptIdx;
							bestFrame=otherFrame->imgIdx;
						}
						else if(dist<bestDist2)
						{
							bestDist2=dist;
						}

					}
				}

				if(bestDist1<distThresh && static_cast<float>(bestDist1)<testRatio*static_cast<float>(bestDist2)){
					bestMatch = DMatch(bestQuery, bestTrain, bestFrame, bestDist1);
				}
			}

			if(bestMatch.queryIdx == -1){
				// no match found
				continue;
			}

		}else{

			float searchRadius 		= searchThresh;
			//skip if out of image given search radius
			if(camera.outofbound(pos, searchRadius)) continue;
			//get feature search range around pos
			int xMin				= pos[0]-searchRadius;
			int xMax				= pos[0]+searchRadius;
			int yMin				= pos[1]-searchRadius;
			int yMax				= pos[1]+searchRadius;
			if(xMin<0) xMin = 0;
			if(yMin<0) yMin = 0;
			if(xMax>=camera.w) xMax = camera.w-1;
			if(yMax>=camera.h) yMax = camera.h-1;
			int feIdxStart			= frame1->kptLUT[yMin];
			if(feIdxStart >= frame1->decs.rows) continue;
			int feIdxEnd			= frame1->kptLUT[yMax];
			if(feIdxEnd >= frame1->decs.rows) feIdxEnd = frame1->decs.rows-1;
			if(feIdxEnd<feIdxStart) continue;

			int 	bestQuery=-1 ;
			float 	bestDist1=searchRadius+3;	//3 is arbitrary, as long as initial distance is larger than search radius
			float 	bestDist2=bestDist1;
			for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
				//skip if already measured. can happen when input frame was already added and measured to some landmark
				if(frame1->measured[j]) continue;
				//skip if not within the horizontal bound
				Point2f &pt	= frame1->kpts[j].pt;
				if(pt.x<xMin || pt.x>xMax) continue;
				Vector2d pt_vec(pt.x, pt.y);
				const float dist 		=  (pos-pt_vec).norm(); //dist is now the point to point distance
				if(dist<bestDist1)
				{
					bestDist2=bestDist1;
					bestDist1=dist;
					bestQuery=j;
				}
				else if(dist<bestDist2)
				{
					bestDist2=dist;
				}
			}

			if(bestDist1<searchRadius+3){
				bestMatch = DMatch(bestQuery, 0, bestDist1);
			}else{
				// no match found
				continue;
			}

		}

		//compare with stored previous matches and resolve conflict
		map<int, LandMark::Ptr>::iterator ft = fe2lmk.find(bestMatch.queryIdx);
		if(ft!=fe2lmk.end()){
			//feature was measured to another landmark
			map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mt = lm2match.find(ft->second);
			map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mtNew = lm2match.find(lmks[i]);
			assert(mt!=lm2match.end());
			assert(mtNew==lm2match.end());
			assert(mt->second.queryIdx == bestMatch.queryIdx);
			if(bestMatch.distance < mt->second.distance){
				//the new landmark has not been measured to any other feature
				lm2match.erase(mt);		//warning: this makes mt invalid
				lm2match[lmks[i]] = bestMatch;
				ft->second = lmks[i];
			}
		}else{
			//feature was not measured
			map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mtNew = lm2match.find(lmks[i]);
			assert(mtNew==lm2match.end());
			fe2lmk[bestMatch.queryIdx] = lmks[i];
			lm2match[lmks[i]] = bestMatch;
		}

		if(maxMatches>0 && lm2match.size()>=maxMatches){
			//have enough matches, ignore subsequent landmarks
			break;
		}
	}

	//assertion check
	for(map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator jt = lm2match.begin(); jt != lm2match.end(); ++jt){
		map<int, LandMark::Ptr>::iterator ft = fe2lmk.find(jt->second.queryIdx);
		assert(ft!=fe2lmk.end());
		assert(ft->second.get() == jt->first.get());
	}
	for(map<int, LandMark::Ptr>::iterator jt=fe2lmk.begin(); jt!=fe2lmk.end(); ++jt){
		map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator mt = lm2match.find(jt->second);
		assert(mt!=lm2match.end());
		assert(mt->second.queryIdx == jt->first);
	}

	for(map<LandMark::Ptr, DMatch, Data::LandMarkPtrCompare>::iterator it = lm2match.begin(); it!=lm2match.end(); ++it){
		newMs.push_back(make_shared<Measurement>(frame1,it->second.queryIdx, it->first));
	}
	t_matching = double(clock()-time) / CLOCKS_PER_SEC;
	cout<<"found "<<newMs.size()<<" new measures"<<endl;
	cout<<"Time(s) preprocess    = "<<t_preprocess<<endl;
	cout<<"Time(s) proj matching = "<<t_matching<<endl;

	return newMs;
}

vector<Measurement::Ptr>
SFMPipeline::getMeasuresByProjection(	const Frame::Ptr 					&frame,
										const vector<Measurement::Ptr>		&otherFramesMs,		//these measures measure to different landmark
										const vector<Measurement::Ptr> 		&existingMeasures,	//frame existing measures
										const int							searchThresh,
										const double						testRatio,
										const int							distThresh,
										const bool							fixThresh)
{

	set<LandMark::Ptr, Data::LandMarkPtrCompare> measuredLms;
	for(vector<Measurement::Ptr>::const_iterator it = existingMeasures.begin(); it!=existingMeasures.end(); ++it){
		measuredLms.insert((*it)->landmark);
	}
	assert(measuredLms.size() == existingMeasures.size());

	vector<Measurement::Ptr> newMs;
	Camera &camera = Camera::GetInstance();
	const int kptCnt = frame->decs.rows;
	vector<DMatch*> kpt2match(kptCnt,nullptr);
	for(unsigned int i=0; i<otherFramesMs.size(); i++){
		const Frame::Ptr &otherFrame 	= otherFramesMs[i]->frame;
		const int otherKptIdx			= otherFramesMs[i]->featureIdx;
		//skip if the otherKpt does not have a descriptor. it can happen when the was done using optical flow
		if(otherKptIdx >= otherFrame->decs.rows) continue;
		const LandMark::Ptr &lm			= otherFramesMs[i]->landmark;
		//skip if landmark already measured
		if(!(measuredLms.empty()) && measuredLms.find(lm)!=measuredLms.end()) continue;

		if(testRatio<1){
			const Mat &otherDec				= otherFrame->decs.row(otherKptIdx);
			float searchRadius 				= (fixThresh) ? searchThresh : searchThresh*(otherFrame->kpts[otherKptIdx].octave+1);

			assert(!(lm->deleted));
			//project landmark to frame
			Camera::ProjectionStatus s;
			Vector2d pos = camera.project(frame,lm->pt, s);
			//skip if landmark is behind camera plane z=1
			if(s == Camera::ProjectionStatus::Behind) continue;
			//skip if out of image given search radius
			if(camera.outofbound(pos, searchRadius)) continue;
			//get feature search range around pos
			int xMin				= pos[0]-searchRadius;
			int xMax				= pos[0]+searchRadius;
			int yMin				= pos[1]-searchRadius;
			int yMax				= pos[1]+searchRadius;
			if(xMin<0) xMin = 0;
			if(yMin<0) yMin = 0;
			if(xMax>=camera.w) xMax = camera.w-1;
			if(yMax>=camera.h) yMax = camera.h-1;
			int feIdxStart			= frame->kptLUT[yMin];
			if(feIdxStart >= frame->decs.rows) continue;
			int feIdxEnd			= frame->kptLUT[yMax];
			if(feIdxEnd >= frame->decs.rows) feIdxEnd = frame->decs.rows-1;
			if(feIdxEnd<feIdxStart) continue;

			//find best match
			int bestQuery=-1 ;
			int bestDist1=256;
			int bestDist2=256;
			for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
				//skip if already measured. can happen when input frame was already added and measured to some landmark
				if(frame->measured[j]) continue;
				//skip if not within the horizontal bound
				Point2f &pt	= frame->kpts[j].pt;
				if(pt.x<xMin || pt.x>xMax) continue;
				//get frame feature
				const Mat &dec		= frame->decs.row(j);
				const int dist 		=  DetectorExtractor::descriptorDistance(dec,otherDec);
				if(dist<bestDist1)
				{
					bestDist2=bestDist1;
					bestDist1=dist;
					bestQuery=j;
				}
				else if(dist<bestDist2)
				{
					bestDist2=dist;
				}

			}

			if(bestDist1<distThresh && static_cast<float>(bestDist1)<testRatio*static_cast<float>(bestDist2)){
				DMatch *prevMatch = kpt2match[bestQuery];
				if(prevMatch == nullptr){
					kpt2match[bestQuery] = new DMatch(bestQuery, i, bestDist1);	//the trainIdx index to the otherFramesMs
				}else{
					if(prevMatch->distance > bestDist1){
						delete prevMatch;
						prevMatch = nullptr;
						kpt2match[bestQuery] = new DMatch(bestQuery, i, bestDist1); //the trainIdx index to the otherFramesMs
					}
				}
			}
		}else{
			float searchRadius 				= searchThresh;

			assert(!(lm->deleted));
			//project landmark to frame
			Camera::ProjectionStatus s;
			Vector2d pos = camera.project(frame,lm->pt, s);
			//skip if landmark is behind camera plane z=1
			if(s == Camera::ProjectionStatus::Behind) continue;
			//skip if out of image given search radius
			if(camera.outofbound(pos, searchRadius)) continue;
			//get feature search range around pos
			int xMin				= pos[0]-searchRadius;
			int xMax				= pos[0]+searchRadius;
			int yMin				= pos[1]-searchRadius;
			int yMax				= pos[1]+searchRadius;
			if(xMin<0) xMin = 0;
			if(yMin<0) yMin = 0;
			if(xMax>=camera.w) xMax = camera.w-1;
			if(yMax>=camera.h) yMax = camera.h-1;
			int feIdxStart			= frame->kptLUT[yMin];
			if(feIdxStart >= frame->decs.rows) continue;
			int feIdxEnd			= frame->kptLUT[yMax];
			if(feIdxEnd >= frame->decs.rows) feIdxEnd = frame->decs.rows-1;
			if(feIdxEnd<feIdxStart) continue;

			//find best match
			int		bestQuery=-1 ;
			float 	bestDist1=searchRadius+3;	//3 is arbitrary, as long as initial distance is larger than search radius
			float 	bestDist2=bestDist1;
			for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
				//skip if already measured. can happen when input frame was already added and measured to some landmark
				if(frame->measured[j]) continue;
				//skip if not within the horizontal bound
				Point2f &pt	= frame->kpts[j].pt;
				if(pt.x<xMin || pt.x>xMax) continue;

				Vector2d pt_vec(pt.x, pt.y);
				const float dist 		=  (pos-pt_vec).norm(); //dist is now the point to point distance
				if(dist<bestDist1)
				{
					bestDist2=bestDist1;
					bestDist1=dist;
					bestQuery=j;
				}
				else if(dist<bestDist2)
				{
					bestDist2=dist;
				}

			}

			if(bestQuery!=-1){
				DMatch *prevMatch = kpt2match[bestQuery];
				if(prevMatch == nullptr){
					kpt2match[bestQuery] = new DMatch(bestQuery, i, bestDist1);	//the trainIdx index to the otherFramesMs
				}else{
					if(prevMatch->distance > bestDist1){
						delete prevMatch;
						prevMatch = nullptr;
						kpt2match[bestQuery] = new DMatch(bestQuery, i, bestDist1); //the trainIdx index to the otherFramesMs
					}
				}
			}
		}
	}

	for(unsigned int i=0; i<kptCnt; i++){
		if(kpt2match[i]!=nullptr){
			newMs.push_back(make_shared<Measurement>(frame, kpt2match[i]->queryIdx, otherFramesMs[kpt2match[i]->trainIdx]->landmark));
			delete kpt2match[i];
			kpt2match[i] = nullptr;
		}
	}

	return newMs;

}


void SFMPipeline::bundleAdjustment(){
	BAHandler BA;
	//BA.adjustBundle();
	BA.adjustBundle_ceres_nocopy();
	printDebug();
}

void SFMPipeline::bundleAdjustmentLocal(){
	BAHandler BA;
	BA.adjustBundle_ceres_local_nocopy();
	printDebug();
}

void SFMPipeline::bundleAdjustmentLocal(vector<Measurement::Ptr> &ms){
	BAHandler BA;
	BA.adjustBundle_ceres_local_nocopy(ms);
	printDebug();
}

// void SFMPipeline::bundleAdjustmentLocalFixPoints(vector<Measurement::Ptr> &ms){
// 	BAHandler BA;
// 	BA.adjustBundle_ceres_local_fixPoints_nocopy(ms);

// }

double SFMPipeline::computeMeanReprojectionError(const std::vector<Measurement::Ptr> &in_ms){
	float meanError = 0;
	int	cnt = 0;
	const vector<Measurement::Ptr> &ms = in_ms.empty() ? data.getMeasurements() : in_ms;
	bool valid;
	for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){
		if((*it)->deleted) continue;
		double error = computeReprojectionError((*it)->frame, (*it)->landmark->pt, (*it)->featureIdx);
		meanError+=error;
		cnt ++;
	}
	meanError/=cnt;

	return meanError;
}

double SFMPipeline::computeReprojectionError(	const Frame::Ptr 			&frame,
												const Vector3d 				&pt3D,
												const	int					featureIdx)
{
	Camera::ProjectionStatus s;
	Vector2d pt_proj		= Camera::GetInstance().project(frame,pt3D, s);
	if(s==Camera::ProjectionStatus::Behind){
		cout<<"points behind camera"<<endl;
		return REPROJERROR_THRESH+1;
	}
	const Point2f	&pt		= frame->kpts[featureIdx].pt;

	/*const double &kptSize	= frame->kpts[featureIdx].size;
	double kptStd			= 0.8*kptSize/12.0;
	Matrix2d inverseCov 	= Matrix2d::Identity()/(kptStd*kptStd);
	Vector2d diff			= pt_proj - Vector2d(pt.x, pt.y);
	double result 			= diff.dot(inverseCov*diff);*/

	double result 			= (pt_proj - Vector2d(pt.x, pt.y)).norm();

	return result;

}

void SFMPipeline::pruneHighErrorMeasurements(){

	int	msCntBefore 	= data.countMeasurements();
	int lmkCntBefore	= data.countLandMarks();

	const vector<Measurement::Ptr> &ms = data.getMeasurements();
	for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it != ms.end(); ++it){
		if((*it)->deleted) continue;
		double error = computeReprojectionError((*it)->frame, (*it)->landmark->pt, (*it)->featureIdx);
		if(error>REPROJERROR_THRESH/4){
			(*it)->clear();
		}
	}

	data.deleteTrashes();

	int	msCnt 			= data.countMeasurements();
	int lmkCnt			= data.countLandMarks();

	cout<<"High Reproj Measures removed = "<<(msCntBefore-msCnt)<<" landmarks deleted = "<<(lmkCntBefore - lmkCnt)<<endl;

}





void SFMPipeline::printDebug(){
	float error = computeMeanReprojectionError();
	cout<<"mean reprojection error = "<<error<<endl;
	//cout<<Camera::GetInstance().camMat<<endl;
}

void SFMPipeline::saveProject(			const string 				&fname)
{
	ProjectIO::writeProject(fname,imgRoot, imgNames);
}

void SFMPipeline::loadProject(			const string				&fname)
{
	ProjectIO::readProject(fname,imgRoot, imgNames);
}


