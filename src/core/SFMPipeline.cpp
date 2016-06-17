/*
 * SFMModules.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <algorithm>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "SFMPipeline.h"
#include "BAHandler.h"
#include "Utils.h"
#include "ProjectIO.h"
#include "PlyIO.h"

using namespace cv;
using namespace std;

SFMPipeline::SFMPipeline(	const string			&root,
							const vector<string> 	&paths) {
	ptCloud.imgRoot = root;
	ptCloud.imgs 	= paths;

	/*
	double camMatArr[9] = { 499.09418996590045, 0.0, 				329.97466633492741,
						   0.0, 		493.39732262353107, 206.64286788669239,
						   0.0,			0.0,				1.0 };
	double distortCoeffArr[5] = { -9.4530758095419692e-02, 1.6386299906095591e-02,
								   -2.4033501462113189e-03, 3.8528268639875942e-04,
								   -3.6380704054147735e-02 };
	*/


	Mat CM,DM;
	if(ptCloud.imgs.size()>0){
		//new project
		//assume all images are from the same camera and orientation
		Mat tmp 	= imread(ptCloud.imgRoot+"/"+ptCloud.imgs[0],IMREAD_COLOR);
		double f	= (tmp.rows>tmp.cols)?tmp.rows:tmp.cols;
		//TODO:delete the line blow
		//f = 507.7488;
		double ppx	= tmp.cols/2.0;
		double ppy 	= tmp.rows/2.0;
		double camMatArr[9] = { f, 		0.0, 		ppx,
								0.0, 	f, 			ppy,
								0.0,	0.0,		1.0 };
		//assume no distorition
		double distortCoeffArr[5] = { 0, 0, 0, 0, 0 };

		CM = Mat(3, 3, CV_64F, camMatArr);
		DM = Mat(1, 5, CV_64F, distortCoeffArr);
	}else{
		//saved project
		CM = Mat(3, 3, CV_64F);
		DM = Mat(1, 5, CV_64F);

	}
	camMat = CM.clone();		//must copy the data else they will be destroyed after constructor
	distortionMat = DM.clone();	//must copy the data else they will be destroyed after constructor
}

SFMPipeline::~SFMPipeline() {
	// TODO Auto-generated destructor stub
}

double SFMPipeline::getCamFocal(){
	return camMat.at<double>(0,0);
}
Point2d SFMPipeline::getCamPrinciple(){
	return Point2d(camMat.at<double>(0,2),camMat.at<double>(1,2));
}

void getBestPairInRange(		const int start,
								const int end,
								int &imgIdx1,
								int &imgIdx2)
{
	imgIdx1 = -1;
	imgIdx2 = -1;

	vector<KeyPoint> 	kpts1,	kpts2;
	vector<Point2f> 	pts1,	pts2;
	Mat 				decs1, 	decs2;
	vector<DMatch> 		matches;
	bool 				successful;
}

void SFMPipeline::getNextPair(	int &imgIdx1, 	//from
								int &imgIdx2)	//to
{
	imgIdx1 = -1;
	imgIdx2 = -1;

	int lastAddedImgIdx 	= -2;
	if(!ptCloud.camMats.empty()){
		int lastCamIdx 		= ptCloud.camMats.size()-1;
		lastAddedImgIdx 	= ptCloud.camMat2img[lastCamIdx];
		cout<<"last added img idx = "<<lastAddedImgIdx<<endl;
	}
	//XXX:warning, when comparing a negative number to unsigned number, must convert unsigned to signed
	if(lastAddedImgIdx>= ((int) (ptCloud.imgs.size())-1)){
		cout<<"No more images to add"<<endl;
		return;
	}

	vector<KeyPoint> 	kpts1,	kpts2;
	vector<Point2f> 	pts1,	pts2;
	Mat 				decs1, 	decs2;
	vector<DMatch> 		matches;
	bool 				successful;

	if(ptCloud.camMats.empty()){
		for(int anchorIdx=0; anchorIdx<ptCloud.imgs.size()-1; anchorIdx++){
			for(int tryIdx=anchorIdx+1; tryIdx<ptCloud.imgs.size(); tryIdx++){
				cout<<" trying ["<<anchorIdx<<"]->["<<tryIdx<<"]"<<endl;
				//compute features
				getKptsAndDecs(anchorIdx,	kpts1,decs1);
				getKptsAndDecs(tryIdx,		kpts2,decs2);

				//do feature matching
				matchFeatures(decs1, decs2, matches);

				successful = checkMatchesBasePair(kpts1,kpts2,matches);
				if(successful){
					cout<<"total matches = "<<matches.size()<<endl;
					imgIdx1 = anchorIdx;	//from
					imgIdx2 = tryIdx;		//to
					return;
				}
			}
		}

	}else{
		assert(lastAddedImgIdx>=0 & lastAddedImgIdx<ptCloud.imgs.size()-1);
		int anchorIdx = lastAddedImgIdx;
		for(int tryIdx = lastAddedImgIdx+1; tryIdx < ptCloud.imgs.size(); tryIdx++){
			cout<<" trying ["<<tryIdx<<"]->["<<anchorIdx<<"]"<<endl;
			//get features
			getKptsAndDecs(tryIdx,		kpts1,decs1);
			getKptsAndDecs(anchorIdx,	kpts2,decs2);

			//match features
			matchFeatures(decs1, decs2, matches);

			successful = checkMatchesNextPair(tryIdx, anchorIdx, kpts1, kpts2, matches);

			if(successful){
				cout<<"total matches = "<<matches.size()<<endl;
				imgIdx1 = tryIdx;		//from
				imgIdx2 = anchorIdx;	//to
				return;
			}
		}
	}
	cout<<"no next pair found"<<endl;

}

void SFMPipeline::processBasePair(){

	int anchorIdx, tryIdx;
	getNextPair(anchorIdx, tryIdx);
	if(anchorIdx==-1 || tryIdx==-1){
		return;
	}
	vector<KeyPoint> 	kpts1,	kpts2;
	vector<Point2f> 	pts1,	pts2;
	Mat 				decs1, 	decs2;
	vector<DMatch> 		matches;
	bool 				successful;

	//compute features
	getKptsAndDecs(anchorIdx,	kpts1,decs1);
	getKptsAndDecs(tryIdx,		kpts2,decs2);

	//do feature matching
	matchFeatures(decs1, decs2, matches);

	reconstructBasePair(anchorIdx,tryIdx, kpts1, kpts2, decs1, decs2, matches);

}

void SFMPipeline::processNextPair(){

	int anchorIdx, tryIdx;
	getNextPair( tryIdx, anchorIdx);
	if(anchorIdx==-1 || tryIdx==-1){
		return;
	}
	vector<KeyPoint> 	kpts1,	kpts2;
	vector<Point2f> 	pts1,	pts2;
	Mat 				decs1, 	decs2;
	vector<DMatch> 		matches;
	bool 				successful;

	//get features
	getKptsAndDecs(tryIdx,		kpts1,decs1);
	getKptsAndDecs(anchorIdx,	kpts2,decs2);

	//match features
	matchFeatures(decs1, decs2, matches);

	reconstructNextPair(tryIdx, anchorIdx, kpts1, kpts2,  decs1, decs2, matches);
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

	//get camera intrinsics
	double f = getCamFocal();
	Point2d pp = getCamPrinciple();

	//prune by fundamental mat
	E = findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);
	nonzeroCnt = countNonZero(inliers);

	//check if have sufficient matches left
	if(nonzeroCnt<MIN_MATCHES){
		cout<<" too few matches for recover pose: "<<nonzeroCnt<<" target: "<<MIN_MATCHES<<endl;
		return false;
	}

	//prune by recover pose
	//inliers is both input and output.
	//input is from previous step.
	//output is to prune those failed cheiralityCheck.
	recoverPose(E, pts1, pts2, R, t, f, pp, inliers);
	nonzeroCnt = countNonZero(inliers);

	if(nonzeroCnt<MIN_TRIANGULATE){
		cout<<" too few points for triangulation: "<<nonzeroCnt<<" target: "<<MIN_TRIANGULATE<<endl;
		return false;
	}

	cout<<"matches will be triangulated = "<<nonzeroCnt<<endl;
	return true;
}

bool SFMPipeline::checkMatchesNextPair(		const int				imgIdx1,
											const int				imgIdx2,
											const vector<KeyPoint> 	&kpts1,
											const vector<KeyPoint> 	&kpts2,
											const vector<DMatch> 	&matches)
{
	if(ptCloud.imageIsUsed(imgIdx1) || !ptCloud.imageIsUsed(imgIdx2)){
		cout<<"MATCH CHECK FAILURE:first image must be not used and second image must be used"<<endl;
		return false;
	}

	vector<Point2f> 	pts1, pts2;
	vector<DMatch> 		prunedMatches, prunedMatches2, matchesHas3D, matchesNo3D;
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

	//get camera intrinsics
	double f = getCamFocal();
	Point2d pp = getCamPrinciple();

	//prune by fundamental mat
	E = findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);
	nonzeroCnt = countNonZero(inliers);

	//check if have sufficient matches left
	if(nonzeroCnt<MIN_MATCHES){
		cout<<" too few matches for recover pose: "<<nonzeroCnt<<" target: "<<MIN_MATCHES<<endl;
		return false;
	}

	//prune by recover pose
	//inliers is both input and output.
	//input is from previous step.
	//output is to prune those failed cheiralityCheck.
	recoverPose(E, pts1, pts2, R, t, f, pp, inliers);
	for(int i=0; i<inliers.rows; i++){
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches.push_back(matches[i]);
		}
	}

	separateMatches(imgIdx2, true, prunedMatches, matchesHas3D, matchesNo3D);

	//check if have sufficient matches left
	if(matchesHas3D.size()<MIN_3D4PNP){
		cout<<" too few matches for solve pnp: "<<matchesHas3D.size()<<" target: "<<MIN_3D4PNP<<endl;
		return false;
	}

	cout<<"matches already triangulated = "<<matchesHas3D.size()<<" matches new = "<<matchesNo3D.size()<<endl;
	return true;
}

bool SFMPipeline::checkMatchesAddNewMeasures(
										const vector<KeyPoint> 	&kpts1,
										const vector<KeyPoint> 	&kpts2,
										const vector<DMatch> 	&matches)
{
	vector<Point2f> 	pts1, pts2;
	Mat					inliers;

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

	//get camera intrinsics
	double f = getCamFocal();
	Point2d pp = getCamPrinciple();

	//prune by fundamental mat
	findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);
	int nonzeroCnt = countNonZero(inliers);

	if(nonzeroCnt<MIN_TRIANGULATE){
		cout<<" too few points for triangulation: "<<nonzeroCnt<<" target: "<<MIN_TRIANGULATE<<endl;
		return false;
	}

	return true;
}

//should only be called after checkMatchesBasePair
void SFMPipeline::reconstructBasePair(	const int				imgIdx1,	//this will have identity camera projection
										const int				imgIdx2,
										const vector<KeyPoint> 	&kpts1,
										const vector<KeyPoint> 	&kpts2,
										const Mat				&decs1,
										const Mat 				&decs2,
										const vector<DMatch> 	&matches)
{

	vector<Point2f> 	pts1, pts2;
	vector<int>			ptIdxs1, ptIdxs2;
	Mat 				E, R, rvec, t, inliers;
	Matx34d 			P1, P2;
	vector<DMatch> 		prunedMatches, prunedMatches2;
	vector<Point3f>		pts3D, pts3DGood;

	Utils::Matches2Points(kpts1,kpts2,matches,pts1,pts2);

	//get camera intrinsics
	double f = getCamFocal();
	Point2d pp = getCamPrinciple();
	E = findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);

	//inliers is both input and output.
	//input is from previous step.
	//output is to keep previous inliers that passed cheiralityCheck.
	recoverPose(E, pts1, pts2, R, t, f, pp, inliers);

	for(int i=0; i<inliers.rows; i++){
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches.push_back(matches[i]);
		}
	}

	//set projection mats
	P1 = 	Matx34d(1,0,0,0,
					0,1,0,0,
					0,0,1,0);
	P2 =	Matx34d(R.at<double>(0,0),	R.at<double>(0,1),	R.at<double>(0,2),	t.at<double>(0),
					R.at<double>(1,0),	R.at<double>(1,1),	R.at<double>(1,2),	t.at<double>(1),
					R.at<double>(2,0),	R.at<double>(2,1),	R.at<double>(2,2),	t.at<double>(2));

	Utils::Matches2Points(kpts1,kpts2,prunedMatches,pts1,pts2);

	//inliers do both cheiralityCheck and reprojectionErrorCheck
	triangulate(pts1,pts2,P1,P2,pts3D,inliers);
	assert(inliers.cols == prunedMatches.size());
	for(int i=0; i<inliers.cols; i++){	//type 0, size 1*n
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches2.push_back(prunedMatches[i]);
			pts3DGood.push_back(pts3D[i]);
		}
	}


	//update ptCloud -- 2D
	ptCloud.add2D(imgIdx1,kpts1,decs1);
	ptCloud.add2D(imgIdx2,kpts2,decs2);

	//update ptCloud -- 3D, for base pair, just add new points
	Utils::Matches2Indices(prunedMatches2, ptIdxs1, ptIdxs2);
	ptCloud.add3D(imgIdx1, imgIdx2, pts3DGood, ptIdxs1, ptIdxs2);

	//update ptCloud -- camMat
	ptCloud.addCamMat(imgIdx1,P1);
	ptCloud.addCamMat(imgIdx2,P2);

	int cloudSize = ptCloud.pt3Ds.size();

	cout<<"["<<imgIdx1<<"]"<<ptCloud.imgs[imgIdx1]<<" & "<<"["<<imgIdx2<<"]"<<ptCloud.imgs[imgIdx2];
	cout<<"\t cloud size = "<<cloudSize<<" ("<<cloudSize<<" new)"<<endl;

}

void SFMPipeline:: reconstructNextPair(const int				imgIdx1,	//this must be a new image
								const int				imgIdx2,	//this must be already used image
								const vector<KeyPoint> 	&kpts1,
								const vector<KeyPoint> 	&kpts2,
								const Mat				&decs1,
								const Mat 				&decs2,
								const vector<DMatch> 	&matches)
{
	assert(!(ptCloud.imageIsUsed(imgIdx1)) && ptCloud.imageIsUsed(imgIdx2));

	vector<Point2f> 	pts1,	pts2;
	vector<int> 		ptIdxs1,	ptIdxs2, pts3DIdxs, usedImgIdxs;
	vector<Point3f>		pts3D,	pts3DGood, pts3DAdd;
	vector<DMatch> 		prunedMatches, prunedMatches2, matchesHas3D, matchesNo3D, matchesAdd;
	Mat 				E, R, rvec, t, inliers;
	Matx34d 			P1, P2;
	vector<bool>		no3DMask;

	Utils::Matches2Points(kpts1,kpts2,matches,pts1,pts2);

	//get camera intrinsics
	double f = getCamFocal();
	Point2d pp = getCamPrinciple();
	E = findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);

	//inliers is both input and output.
	//input is from previous step.
	//output is to keep previous inliers that passed cheiralityCheck.
	recoverPose(E, pts1, pts2, R, t, f, pp, inliers);
	for(int i=0; i<inliers.rows; i++){
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches.push_back(matches[i]);
		}
	}

	Rodrigues(R,rvec); //for extrinsic guess in solvePnP

	//separate matches by checking if the matched point in anchor image already has a 3D point in cloud
	//the second input true is to specify using train idx ie. 2nd part of the match
	separateMatches(imgIdx2, true, prunedMatches, matchesHas3D, matchesNo3D);

	assert(matchesHas3D.size()>=MIN_3D4PNP);

	//get those 3D points
	Utils::Matches2Indices(matchesHas3D, ptIdxs1,ptIdxs2);
	ptCloud.get3DfromImage2D(imgIdx2,ptIdxs2,pts3D, pts3DIdxs);
	//get corresponding 2D points in the tryImage, we'll only use pts1
	Utils::Matches2Points(kpts1,kpts2,matchesHas3D,pts1,pts2);
	//solve PnP for the try image camera pose (this step is different from base pair)
	//solvePnP(pts3D,pts1,camMat,distortionMat,rvec,t);
	bool useExtrinsicGuess	= true;
	int iterationsCount		= PNP_MAX_ITERATION;
	float reprojectionError	= PNP_MAX_ERROR;
	double confidence		= PNP_CONFIDENCE;
	int flags				= SOLVEPNP_ITERATIVE;
	solvePnPRansac(pts3D,pts1,camMat,distortionMat,rvec,t,useExtrinsicGuess,iterationsCount,reprojectionError,confidence,inliers,flags);


	//get camera poses
	Rodrigues(rvec,R);
	P1 = 	Matx34d(R.at<double>(0,0),	R.at<double>(0,1),	R.at<double>(0,2),	t.at<double>(0),
					R.at<double>(1,0),	R.at<double>(1,1),	R.at<double>(1,2),	t.at<double>(1),
					R.at<double>(2,0),	R.at<double>(2,1),	R.at<double>(2,2),	t.at<double>(2));

	//update ptCloud -- 2D
	ptCloud.add2D(imgIdx1,kpts1,decs1);

	//update ptCloud -- camMat
	ptCloud.addCamMat(imgIdx1,P1);

	cout<<"new camera added"<<endl;

	//check with old views to update points already seen, prioritize update with imgIdx2
	addNewMeasures(imgIdx1, imgIdx2);
	ptCloud.getUsedImageIdxs(usedImgIdxs);
	for(int i=usedImgIdxs.size()-1; i>=0; i--){
		int usedImgIdx = usedImgIdxs[i];
		if(usedImgIdx != imgIdx1 && usedImgIdx != imgIdx2){
			addNewMeasures(imgIdx1, usedImgIdx);
		}
	}

	//get cameras
	ptCloud.getImageCamMat(imgIdx1,P1);
	ptCloud.getImageCamMat(imgIdx2,P2);
	//triangulate all prunedMatches, including those with 3D
	Utils::Matches2Points(kpts1,kpts2,prunedMatches,pts1,pts2);
	triangulate(pts1,pts2,P1,P2,pts3D,inliers);
	assert(prunedMatches.size() == pts3D.size() && pts3D.size() == inliers.cols);
	for(int i=0; i<inliers.cols; i++){	//type 0, size 1*n
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches2.push_back(prunedMatches[i]);
			pts3DGood.push_back(pts3D[i]);
		}
	}

	//get matches still having no 3D correspondences
	maskMatchesNo3D(imgIdx1, imgIdx2, prunedMatches2, no3DMask);
	assert(prunedMatches2.size() == no3DMask.size() && pts3DGood.size() == no3DMask.size());
	for(int i=0; i<no3DMask.size(); i++){
		if(no3DMask[i]){
			matchesAdd.push_back(prunedMatches2[i]);
			pts3DAdd.push_back(pts3DGood[i]);
		}
	}
	//update ptCloud -- 3D, add new points
	int cloudSizeOld = ptCloud.pt3Ds.size();
	Utils::Matches2Indices(matchesAdd, ptIdxs1, ptIdxs2);
	ptCloud.add3D(imgIdx1, imgIdx2, pts3DAdd, ptIdxs1, ptIdxs2);
	int cloudSizeNew = ptCloud.pt3Ds.size();

	cout<<"["<<imgIdx1<<"]"<<ptCloud.imgs[imgIdx1]<<" & "<<"["<<imgIdx2<<"]"<<ptCloud.imgs[imgIdx2];
	cout<<"\t cloud size = "<<cloudSizeNew<<" ("<<(cloudSizeNew-cloudSizeOld)<<" new)"<<endl;
}

void SFMPipeline:: addNewMeasures(	const int				imgIdx1,	//this is the newly used image (tryImg)
										const int				imgIdx2)	//this must be already used image (anchorImg)
{
	assert(ptCloud.imageIsUsed(imgIdx1) && ptCloud.imageIsUsed(imgIdx2));

	//declare variables
	vector<KeyPoint> 	kpts1,	kpts2;
	vector<Point2f> 	pts1,	pts2;
	Mat 				decs1, 	decs2;
	vector<DMatch> 		matches, prunedMatches, prunedMatches2, matchesToUpdate;
	bool 				successful;
	Mat 				inliers;
	Matx34d 			P1, P2;
	vector<Point3f>		pts3D, pts3DGood;
	vector<int>			pts3DIdxs, ptIdxs1,ptIdxs2;

	//get features
	getKptsAndDecs(imgIdx1,	kpts1,decs1);
	getKptsAndDecs(imgIdx2,	kpts2,decs2);

	//match features
	matchFeatures(decs1, decs2, matches);

	successful = checkMatchesAddNewMeasures(kpts1, kpts2, matches);
	if(!successful) return;

	Utils::Matches2Points(kpts1,kpts2,matches,pts1,pts2);

	//get camera intrinsics
	double f = getCamFocal();
	Point2d pp = getCamPrinciple();
	findEssentialMat(pts1, pts2, f, pp, RANSAC, 0.999, 1.0, inliers);
	for(int i=0; i<inliers.rows; i++){
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches.push_back(matches[i]);
		}
	}

	Utils::Matches2Points(kpts1,kpts2,prunedMatches,pts1,pts2);

	//get cameras
	ptCloud.getImageCamMat(imgIdx1,P1);
	ptCloud.getImageCamMat(imgIdx2,P2);

	//inliers do both cheiralityCheck and reprojectionErrorCheck
	triangulate(pts1,pts2,P1,P2,pts3D,inliers);
	assert(inliers.cols == prunedMatches.size());
	for(int i=0; i<inliers.cols; i++){	//type 0, size 1*n
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches2.push_back(prunedMatches[i]);
			pts3DGood.push_back(pts3D[i]);
		}
	}

	getMatchesHas3DSecondViewOnly(imgIdx1, imgIdx2, prunedMatches2, matchesToUpdate);
	Utils::Matches2Indices(matchesToUpdate, ptIdxs1,	ptIdxs2);

	ptCloud.get3DfromImage2D(imgIdx2,ptIdxs2, pts3D, pts3DIdxs);
	ptCloud.update3D(imgIdx1, pts3DIdxs, ptIdxs1);

	cout<<"added "<<matchesToUpdate.size()<<" measures from ["<<imgIdx1<<"]"<<ptCloud.imgs[imgIdx1]<<" to 3D points seen by "<<"["<<imgIdx2<<"]"<<ptCloud.imgs[imgIdx2]<<endl;

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
	double f = getCamFocal();
	Point2d pp = getCamPrinciple();
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
void SFMPipeline::removeNearAndFar3DPoints(){

	//list of boolean to mark whether 3d point should be removed
	//true to remove
	vector<bool> removeMask(ptCloud.pt3Ds.size(),false);

	//constants for cheiralityCheck
	float minDist = MIN_DIST_TO_CAM + (MAX_DIST_TO_CAM - MIN_DIST_TO_CAM)*0.1;
	float maxDist = MAX_DIST_TO_CAM - (MAX_DIST_TO_CAM - MIN_DIST_TO_CAM)*0.1;

	//do cheiralityCheck for all cameras
	map<int, int> img2camMat = ptCloud.img2camMat;
	for(map<int, int>::iterator i = img2camMat.begin(); i != img2camMat.end(); i++) {
		int imgIdx 	= (*i).first;
		int camIdx 	= (*i).second;
		vector<Point3f> pts3D;
		vector<int> 	pts3DIdxs;
		ptCloud.getAll3DfromImage2D(imgIdx,pts3D,pts3DIdxs);
		Mat isGood;
		cheiralityCheck(pts3D,ptCloud.camMats[camIdx],minDist, maxDist, isGood);
		assert(pts3DIdxs.size() == isGood.cols);
		for(int j=0; j<isGood.cols; j++){	//type 0, size 1*n
			unsigned int val = (unsigned int)isGood.at<uchar>(j);
			if(!val){
				removeMask[pts3DIdxs[j]] = true;
			}
		}
	}

	ptCloud.remove3Ds(removeMask);
}
void SFMPipeline::findEandPruneMatches(	const vector<Point2f> 	&pts1,
										const vector<Point2f> 	&pts2,
										const vector<DMatch> 	&matches,
										vector<DMatch> 			&prunedMatches,
										Mat 					&E){
	Mat inliers;
	prunedMatches.clear();
	E = findEssentialMat(pts1, pts2, getCamFocal(), getCamPrinciple(), RANSAC, 0.999, 1.0, inliers);
	//F = findFundamentalMat(pts1,pts2,CV_FM_RANSAC ,3,0.99,inliers);
	for (int i=0; i<inliers.rows; i++){	//type 0, size N*1
		unsigned int val = (unsigned int)inliers.at<uchar>(i);
		if(val){
			prunedMatches.push_back(matches[i]);
		}
	}
}
bool SFMPipeline::isGoodMatch(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, const double &thresh){

	double Hinlier = FindHomographyInliers(pts1,pts2);
	if(Hinlier<thresh){
		return true;
	}else{
		return false;
	}
}
double SFMPipeline::FindHomographyInliers(const vector<Point2f> &pts1, const vector<Point2f> &pts2){

	double minVal,maxVal;
	minMaxIdx(pts1,&minVal,&maxVal);
	vector<uchar> status;
	Mat H = findHomography(pts1,pts2,status,CV_RANSAC, 0.004 * maxVal); //threshold from Snavely07
	return countNonZero(status)*1.0/pts1.size(); //number of inliers
}

void SFMPipeline::getKptsAndDecs( 	const int 			imgIdx,
									vector<KeyPoint> 	&kpts,
									Mat					&decs){
	if(ptCloud.imageIsUsed(imgIdx)){
		ptCloud.getImageFeatures(imgIdx,kpts,decs);
	}else{
		assert(imgIdx>=0 && imgIdx<ptCloud.imgs.size());
		Mat img 	= imread(ptCloud.imgRoot+"/"+ptCloud.imgs[imgIdx],IMREAD_COLOR);
		Ptr<Feature2D> detector = ORB::create(IMG_KEYPOINTS,1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31);
		detector->detect(img, kpts);
		detector->compute(img, kpts, decs);
	}
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
void SFMPipeline::matchFeatures(const Mat &decs1, const Mat &decs2, vector<DMatch> &matches){
	matches.clear();
	vector<vector<DMatch> > matches12;
	vector<vector<DMatch> > matches21;
	vector<DMatch> 			matches12RatioTested, matches21RatioTested;
	double ratio = MATCH_RATIO;

	BFMatcher matcher(NORM_HAMMING,false);	//(NORM_L2,false);
											//from opencv 3.0 documentation:
											//One of NORM_L1, NORM_L2, NORM_HAMMING, NORM_HAMMING2. L1 and L2 norms are preferable choices for SIFT and SURF descriptors, NORM_HAMMING should be used with ORB, BRISK and BRIEF, NORM_HAMMING2 should be used with ORB when WTA_K==3 or 4 (see ORB::ORB constructor description)

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
}

void SFMPipeline::separateMatches(		const int				imageIdx,
										const int 				useTrainIdx,
										const vector<DMatch> 	&matches,
										vector<DMatch>			&matchesHas3D,
										vector<DMatch>			&matchesNo3D){


	vector<int> 	matchedPtIdxs1, matchedPtIdxs2;
	vector<bool> 	has3D;
	//get point indices from the match
	Utils::Matches2Indices(matches, matchedPtIdxs1,matchedPtIdxs2);

	//check
	assert(ptCloud.imageIsUsed(imageIdx));

	if(useTrainIdx){
		ptCloud.checkImage2Dfor3D(imageIdx, matchedPtIdxs2, has3D);
	}else{
		ptCloud.checkImage2Dfor3D(imageIdx, matchedPtIdxs1, has3D);
	}

	//separate the matches
	for(int i=0; i<has3D.size(); i++){
		if(has3D[i]){
			matchesHas3D.push_back(matches[i]);
		}else{
			matchesNo3D.push_back(matches[i]);
		}
	}
}

void SFMPipeline::getMatchesHas3DSecondViewOnly(
									const int				imgIdx1,
									const int 				imgIdx2,
									const vector<DMatch>	&matches,
									vector<DMatch>			&prunedMatches)
{
	//check
	assert(ptCloud.imageIsUsed(imgIdx1));
	assert(ptCloud.imageIsUsed(imgIdx2));

	prunedMatches.clear();
	vector<int> 	matchedPtIdxs1, matchedPtIdxs2;
	vector<bool> 	has3D1, has3D2;
	//get point indices from the match
	Utils::Matches2Indices(matches, matchedPtIdxs1,matchedPtIdxs2);
	ptCloud.checkImage2Dfor3D(imgIdx1, matchedPtIdxs1, has3D1);
	ptCloud.checkImage2Dfor3D(imgIdx2, matchedPtIdxs2, has3D2);
	for(int i=0; i<matches.size(); i++){
		if(!has3D1[i] && has3D2[i]){
			//keep only if img2 has a correspondence in the 3d cloud
			prunedMatches.push_back(matches[i]);
		}
	}
}

void SFMPipeline::maskMatchesNo3D(	const int				imgIdx1,
									const int 				imgIdx2,
									const vector<DMatch>	&matches,
									vector<bool>			&mask)
{
	//check
	assert(ptCloud.imageIsUsed(imgIdx1));
	assert(ptCloud.imageIsUsed(imgIdx2));

	mask.clear();
	mask.reserve(matches.size());
	vector<int> 	matchedPtIdxs1, matchedPtIdxs2;
	vector<bool> 	has3D1, has3D2;
	//get point indices from the match
	Utils::Matches2Indices(matches, matchedPtIdxs1,matchedPtIdxs2);
	ptCloud.checkImage2Dfor3D(imgIdx1, matchedPtIdxs1, has3D1);
	ptCloud.checkImage2Dfor3D(imgIdx2, matchedPtIdxs2, has3D2);
	for(int i=0; i<matches.size(); i++){
		mask.push_back(!has3D1[i] && !has3D2[i]);
	}
}

void SFMPipeline::bundleAdjustment(){
	BAHandler BA;
	BA.adjustBundle(ptCloud,camMat,distortionMat);
	//BA.adjustBundle_sba(ptCloud,camMat,distortionMat);
}

void SFMPipeline::computeMeanReprojectionError(){
	ptCloud.updateReprojectionErrors(camMat,distortionMat);
	float meanError;
	ptCloud.getMeanReprojectionError(meanError);
	cout<<"mean projection error = "<<meanError<<endl;
}

void SFMPipeline::pruneHighReprojectionErrorPoints(){

	int cloudSizeOld = ptCloud.pt3Ds.size();

	ptCloud.updateReprojectionErrors(camMat,distortionMat);
	float thresh = REPROJERROR_THRESH;
	ptCloud.removeHighError3D(thresh);

	int cloudSizeNew = ptCloud.pt3Ds.size();

	cout<<"High Reproj Error Points removed = "<<(cloudSizeOld-cloudSizeNew)<<endl;

}
void SFMPipeline::writePLY(	const string 			&root,
							const string			&fname)
{

	PlyIO::writePLY(root,fname,ptCloud);
}
void SFMPipeline::readPLY(		const string 			&path,
					vector<Point3f> 		&xyzs){
	xyzs.clear();

	ifstream infile(path.c_str());
	string line;
	int numVertex	= 0;
	while (getline(infile, line)){
		istringstream iss(line);
		string s;
		iss>>s;
		if(s.compare("element")==0){
			iss>>s;
			if(s.compare("vertex")==0){
				iss>>numVertex;
			}
		}
		if(s.compare("end_header")==0){
			break;
		}
	}

	xyzs.reserve(numVertex);
	for(int i=0; i<numVertex; i++){
		getline(infile, line);
		istringstream iss(line);
		float x,y,z;
		int r,g,b;
		iss>>x>>y>>z>>r>>g>>b;
		xyzs.push_back(Point3f(x,y,z));
	}
}

void SFMPipeline::printDebug(){
	ptCloud.updateReprojectionErrors(camMat,distortionMat);
	float error;
	ptCloud.getMeanReprojectionError(error);
	cout<<"mean reprojection error = "<<error<<endl;
}

void SFMPipeline::saveProject(			const string 				&fname)
{
	ProjectIO::writeProject(fname,camMat,distortionMat,ptCloud);
}

void SFMPipeline::loadProject(			const string				&fname)
{
	ProjectIO::readProject(fname,camMat,distortionMat,ptCloud);
}

void SFMPipeline::loadGPS(				const std::string			&fname)
{
	ProjectIO::readGPS(fname,ptCloud);
}

#include "ptam/KeyFrame.h"
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/vision.h>
#include <TooN/se3.h>
#include <TooN/so3.h>
#include <TooN/TooN.h>

void SFMPipeline::computeKeyFrame(		const int 					imgIdx,
										KeyFrame					&kf){

	//basic initialization of keyframe of an image
	Mat img 	= imread(ptCloud.imgRoot+"/"+ptCloud.imgs[imgIdx],IMREAD_COLOR);
	cvtColor(img,img,cv::COLOR_BGR2GRAY);
	CVD::Image<CVD::byte > im;
	im.resize(CVD::ImageRef(img.cols, img.rows));
	CVD::BasicImage<CVD::byte> img_tmp(img.ptr(), CVD::ImageRef(img.cols, img.rows));
	CVD::copy(img_tmp, im);
	kf.MakeKeyFrame_Lite(im);

	//if the camera projection matrix is known, add it to keyframe
	if(ptCloud.img2camMat.find(imgIdx) != ptCloud.img2camMat.end()){
		int camIdx = ptCloud.img2camMat[imgIdx];
		Mat r, t;
		ptCloud.getCamRvecAndT(camIdx, r, t);
		double t_data[3] = {t.at<double>(0), t.at<double>(1), t.at<double>(2)};
		double r_data[3] = {r.at<double>(0), r.at<double>(1), r.at<double>(2)};
		TooN::Vector <3,double,TooN::Reference> toon_vecT(t_data);
		TooN::Vector <3,double,TooN::Reference> toon_vecR(r_data);
		TooN::SO3<double> toon_so3R(toon_vecR);
		kf.se3CfromW = SE3<double>(toon_so3R,toon_vecT);

	}
}
