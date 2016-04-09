/*
 * PtCloud.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include "PtCloud.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
using namespace std;
using namespace cv;
PtCloud::PtCloud() {
	// TODO Auto-generated constructor stub

}

PtCloud::~PtCloud() {
	// TODO Auto-generated destructor stub
}
void PtCloud::clear(){
	imgRoot = "";
	imgs.clear();
	pt3Ds.clear();
	img2pt2Ds.clear();
	img2camMat.clear();
	camMats.clear();
}
bool PtCloud::imageIsUsed(	const int			imgIdx){
	return (img2camMat.find(imgIdx) != img2camMat.end());
}
void PtCloud::add2D(	const int 				imgIdx,
						const vector<KeyPoint> 	&kpts,
						const Mat 				&decs){

	//checks
	assert(kpts.size() == decs.rows);
	assert(img2pt2Ds.find(imgIdx)==img2pt2Ds.end());	//2d must not exist for this image


	//populate data
	img2pt2Ds[imgIdx] = vector<Pt2D>();
	img2pt2Ds[imgIdx].reserve(kpts.size());
	for(int i=0; i<kpts.size(); i++){
		Pt2D pt2D;
		pt2D.pt 			= kpts[i].pt;
		pt2D.dec			= decs.row(i);
		pt2D.img_idx 		= imgIdx;
		pt2D.pt3D_idx 		= -1;
		img2pt2Ds[imgIdx].push_back(pt2D);
	}
}

void PtCloud::add3D(	const int 				imgIdx1,
						const int				imgIdx2,
						const vector<Point3f> 	&xyzs,
						const vector<int> 		&img2Didxs1,
						const vector<int> 		&img2Didxs2){

	//checks
	assert(xyzs.size() == img2Didxs1.size() && img2Didxs1.size() == img2Didxs2.size());
	assert(imgIdx1 != imgIdx2);
	assert(img2pt2Ds.find(imgIdx1)!=img2pt2Ds.end());	//already has 2d
	assert(img2pt2Ds.find(imgIdx2)!=img2pt2Ds.end());	//already has 2d

	//populate data
	//NOTE: it must be reference(&) to original data
	vector<Pt2D>& pt2Ds1 = img2pt2Ds[imgIdx1];
	vector<Pt2D>& pt2Ds2 = img2pt2Ds[imgIdx2];
	for(int i=0; i<xyzs.size(); i++){
		//add 3D point
		Pt3D pt3D;
		pt3D.pt				= xyzs[i];
		pt3D.img2ptIdx[imgIdx1] = img2Didxs1[i];
		pt3D.img2error[imgIdx1] = 0.0f;
		pt3D.img2ptIdx[imgIdx2] = img2Didxs2[i];
		pt3D.img2error[imgIdx2] = 0.0f;
		pt3Ds.push_back(pt3D);

		//update corresponding 2D points
		int pt2D_idx1 		= img2Didxs1[i];
		int pt2D_idx2 		= img2Didxs2[i];
		assert(pt2Ds1[pt2D_idx1].img_idx == imgIdx1 && pt2Ds1[pt2D_idx1].pt3D_idx == -1);
		assert(pt2Ds2[pt2D_idx2].img_idx == imgIdx2 && pt2Ds2[pt2D_idx2].pt3D_idx == -1);
		pt2Ds1[pt2D_idx1].pt3D_idx = pt3Ds.size()-1;
		pt2Ds2[pt2D_idx2].pt3D_idx = pt3Ds.size()-1;
	}
}

void PtCloud::remove3Ds(	const vector<bool> 	&removeMask){
	assert(removeMask.size() == pt3Ds.size());
	vector<Pt3D>::iterator it = pt3Ds.begin();
	int i 				= 0;
	int removed 		= 0;

	while (it!= pt3Ds.end())
	{
		if(removeMask[i]){
			//before erase, clear pt2D's 3d reference
			const map<int,int>& img2ptIdx = (*it).img2ptIdx;
			//NOTE: you must use const_iterator to iterate through const data
			for(map<int, int>::const_iterator j = img2ptIdx.begin(); j != img2ptIdx.end(); j++) {
				int imgIdx 	= (*j).first;
				int imgPtIdx= (*j).second;
				img2pt2Ds[imgIdx][imgPtIdx].pt3D_idx = -1;
			}
			it = pt3Ds.erase(it);
			removed++;
		}else{
			//due to some 3d points being removed, idxs change, need update
			int pt3Didx = it - pt3Ds.begin();
			const map<int,int>& img2ptIdx = (*it).img2ptIdx;
			//NOTE: you must use const_iterator to iterate through const data
			for(map<int, int>::const_iterator j = img2ptIdx.begin(); j != img2ptIdx.end(); j++) {
				int imgIdx 	= (*j).first;
				int imgPtIdx= (*j).second;
				img2pt2Ds[imgIdx][imgPtIdx].pt3D_idx = pt3Didx;
			}
			++it;
		}
		i++;
	}
	assert(removed == (removeMask.size() - pt3Ds.size()));

}

void PtCloud::addCamMat(	int			imgIdx,
							cv::Matx34d	&camMat){
	assert(img2camMat.find(imgIdx) == img2camMat.end());
	camMats.push_back(camMat);
	img2camMat[imgIdx] = camMats.size()-1;
	assert(camMat2img.find(camMats.size()-1) == camMat2img.end());
	camMat2img[camMats.size()-1] = imgIdx;
}

void PtCloud::update3D(	const int 			imgIdx,
						const vector<int> 	&pt3Didxs,
						const vector<int> 	&img2Didxs){

	//checks
	assert(pt3Didxs.size() == img2Didxs.size());
	assert(img2pt2Ds.find(imgIdx)!=img2pt2Ds.end());	//already has 2d

	//update data
	for(int i=0; i<pt3Didxs.size(); i++){
		//update 3d point's pt2D_idx
		int pt2D_idx	= img2Didxs[i];
		int pt3D_idx	= pt3Didxs[i];
		if(pt3Ds[pt3D_idx].img2ptIdx.find(imgIdx) != pt3Ds[pt3D_idx].img2ptIdx.end()){
			//this occurs when adding a correspondence from a picture that already has another point corresponding to this 3D point
			//TODO: how shall we decide which one to keep? now it is first come first serve
			continue;
		}
		pt3Ds[pt3D_idx].img2ptIdx[imgIdx] = pt2D_idx;
		pt3Ds[pt3D_idx].img2error[imgIdx] = 0.0f;

		//update corresponding 2D points
		assert(img2pt2Ds[imgIdx][pt2D_idx].img_idx == imgIdx);
		assert(img2pt2Ds[imgIdx][pt2D_idx].pt3D_idx == -1);
		img2pt2Ds[imgIdx][pt2D_idx].pt3D_idx = pt3D_idx;
	}
}

void PtCloud::getImageFeatures(	const int 			imgIdx,
								vector<KeyPoint> 	&kpts,
								Mat					&decs){
	assert(img2pt2Ds.find(imgIdx) != img2pt2Ds.end());
	kpts.clear();
	vector<Mat> decList;
	kpts.reserve(img2pt2Ds[imgIdx].size());
	decs.reserve(img2pt2Ds[imgIdx].size());
	for(int i=0; i<img2pt2Ds[imgIdx].size(); i++){
		KeyPoint pt = KeyPoint(img2pt2Ds[imgIdx][i].pt, 0);	//wrap point2f in keypoint, size=0
		Mat dec		= img2pt2Ds[imgIdx][i].dec;
		kpts.push_back(pt);
		decList.push_back(dec);
	}
	vconcat( decList, decs);
}
void PtCloud::getImageCamMat(	int 					imgIdx,
								cv::Matx34d				&camMat){

	assert(img2camMat.find(imgIdx) != img2camMat.end());
	camMat = camMats[img2camMat[imgIdx]];
}
void PtCloud::getUsedImageIdxs(vector<int>			&imgIdxs){
	imgIdxs.clear();
	for(map<int, int>::iterator i = img2camMat.begin(); i != img2camMat.end(); i++) {
		imgIdxs.push_back((*i).first);
	}
}
void PtCloud::checkImage2Dfor3D(			const int 			imgIdx,
											const vector<int>	img2Didxs,
											vector<bool> 		&has3D)
{
	assert(img2pt2Ds.find(imgIdx) != img2pt2Ds.end());
	has3D.clear();
	has3D.resize(img2Didxs.size(),false);

	for(int i=0; i<img2Didxs.size(); i++){
		int pt2D_idx	= img2Didxs[i];
		int pt3D_idx	= img2pt2Ds[imgIdx][pt2D_idx].pt3D_idx;
		if(pt3D_idx != -1){
			has3D[i] = true;
		}
	}
}
void PtCloud::get3DfromImage2D(	const int 			imgIdx,
								const vector<int>	img2Didxs,
								vector<Point3f>		&pts3D,
								vector<int>			&pts3DIdxs)
{
	assert(img2pt2Ds.find(imgIdx) != img2pt2Ds.end());
	pts3D.clear();
	pts3DIdxs.clear();
	pts3D.reserve(img2Didxs.size());
	pts3DIdxs.reserve(img2Didxs.size());
	for(int i=0; i<img2Didxs.size(); i++){
		int pt2D_idx	= img2Didxs[i];
		int pt3D_idx	= img2pt2Ds[imgIdx][pt2D_idx].pt3D_idx;
		assert(pt3D_idx != -1);
		pts3D.push_back(pt3Ds[pt3D_idx].pt);
		pts3DIdxs.push_back(pt3D_idx);
	}
}
void PtCloud::getAll3DfromImage2D(	const int 			imgIdx,
									vector<Point3f>		&pts3D,
									vector<int>			&pts3DIdxs)		//this indexs the class variable pt3Ds
{

	assert(img2pt2Ds.find(imgIdx) != img2pt2Ds.end());
	pts3D.clear();
	pts3DIdxs.clear();
	pts3D.reserve(img2pt2Ds[imgIdx].size()/2);	//estimated size, for efficiency
	pts3DIdxs.reserve(img2pt2Ds[imgIdx].size()/2);

	for(int i=0; i<img2pt2Ds[imgIdx].size(); i++){
		int pt3D_idx	= img2pt2Ds[imgIdx][i].pt3D_idx;
		if(pt3D_idx != -1){
			pts3D.push_back(pt3Ds[pt3D_idx].pt);
			pts3DIdxs.push_back(pt3D_idx);
		}
	}
}

void PtCloud::getXYZs( 			vector<Point3f>		&xyzs){
	xyzs.clear();
	xyzs.reserve(pt3Ds.size());
	for(int i=0; i<pt3Ds.size(); i++){
		xyzs.push_back(pt3Ds[i].pt);
	}
}

void PtCloud::getAverageDecs( 	vector<Mat> 		&decs){
	decs.clear();
	decs.reserve(pt3Ds.size());
	for(int i=0; i<pt3Ds.size(); i++){
		Mat avgDec;
		getAverageDecAtPt3DIdx(i,avgDec);
		decs.push_back(avgDec);
	}
}

void PtCloud::getAverageDecAtPt3DIdx(	const int		idx,
										Mat				&averageDec){
	averageDec = Mat();
	const map<int,int>& img2ptIdx = pt3Ds[idx].img2ptIdx;
	//NOTE: you must use const iterator to iterate through const data
	for(map<int, int>::const_iterator i = img2ptIdx.begin(); i != img2ptIdx.end(); i++) {
		int imgIdx 	= (*i).first;
		int imgPtIdx= (*i).second;
		if(averageDec.rows == 0 && averageDec.cols == 0){
			averageDec = img2pt2Ds[imgIdx][imgPtIdx].dec;
		}else{
			averageDec+= img2pt2Ds[imgIdx][imgPtIdx].dec;
		}
	}
	assert(averageDec.rows != 0 && averageDec.cols != 0);
	averageDec/=img2ptIdx.size();
}

void PtCloud::get2DsHave3D(		vector<Point2f> 	&xys,
								vector<int>			&imgIdxs,
								vector<int>			&pt3DIdxs){
	xys.clear();
	imgIdxs.clear();
	pt3DIdxs.clear();

	//for efficiency
	int roughSizeEstimate = pt3Ds.size()*3;
	xys.reserve(roughSizeEstimate);
	imgIdxs.reserve(roughSizeEstimate);
	pt3DIdxs.reserve(roughSizeEstimate);

	for(map<int, vector<Pt2D> >::iterator i = img2pt2Ds.begin(); i!=img2pt2Ds.end(); i++){
		int imgIdx 					= (*i).first;
		const vector<Pt2D>& pt2Ds 	= (*i).second;	//use reference to avoid data copying, use constant to guard against modification
		for(int j=0; j<pt2Ds.size(); j++){
			if(pt2Ds[j].pt3D_idx!=-1){
				xys.push_back(		pt2Ds[j].pt);
				imgIdxs.push_back(	pt2Ds[j].img_idx);
				pt3DIdxs.push_back( pt2Ds[j].pt3D_idx);
			}
		}
	}
}

void PtCloud::getCamRvecsAndTs( vector<Mat> 		&rvecs,
								vector<Mat> 		&ts)
{
	rvecs.clear();
	ts.clear();
	rvecs.reserve(camMats.size());
	ts.reserve(camMats.size());
	for(int i=0; i<camMats.size(); i++){
		Mat cam 	= Mat(camMats[i]);
		Mat R(3,3,CV_64F);
		R.at<double>(0,0) = cam.at<double>(0,0);
		R.at<double>(0,1) = cam.at<double>(0,1);
		R.at<double>(0,2) = cam.at<double>(0,2);
		R.at<double>(1,0) = cam.at<double>(1,0);
		R.at<double>(1,1) = cam.at<double>(1,1);
		R.at<double>(1,2) = cam.at<double>(1,2);
		R.at<double>(2,0) = cam.at<double>(2,0);
		R.at<double>(2,1) = cam.at<double>(2,1);
		R.at<double>(2,2) = cam.at<double>(2,2);
		Mat rvec;
		Rodrigues(R,rvec);
		Mat t(1,3,CV_64F);
		t.at<double>(0)   = cam.at<double>(0,3);
		t.at<double>(1)   = cam.at<double>(1,3);
		t.at<double>(2)   = cam.at<double>(2,3);
		rvecs.push_back(rvec);
		ts.push_back(t);
	}
}

void PtCloud::updateReprojectionErrors(	const Mat		&camIntrinsicMat,
										const Mat		&camDistortionMat)
{

	int N = pt3Ds.size();
	int M = camMats.size();

	vector<vector<Point3f> >cam2pt3Ds(M,vector<Point3f>());
	vector<vector<Point2f> >cam2pt2Ds(M,vector<Point2f>());
	vector<vector<int> > 	cam2pt3Didxs(M,vector<int>());

	for(int i=0; i<N; i++){
		const map<int, int>& img2ptIdx = pt3Ds[i].img2ptIdx;
		const Point3f& xyz		= pt3Ds[i].pt;
		for(map<int, int>::const_iterator j = img2ptIdx.begin(); j != img2ptIdx.end(); j++) {
			int imgIdx 			= (*j).first;
			int camIdx 			= img2camMat[imgIdx];
			int pt2DIdx			= (*j).second;
			const Point2f& xy	= img2pt2Ds[imgIdx][pt2DIdx].pt;
			cam2pt3Ds[camIdx].push_back(xyz);
			cam2pt2Ds[camIdx].push_back(xy);
			cam2pt3Didxs[camIdx].push_back(i);

		}
	}
	vector<Mat> rvecs,ts;
	getCamRvecsAndTs(rvecs,ts);

	//update reprojection errors
	for(int i=0; i<M; i++){
		vector<Point2f> reprojected;
		if(cam2pt3Ds[i].empty()){
			continue;
		}
		projectPoints(cam2pt3Ds[i], rvecs[i], ts[i], camIntrinsicMat, camDistortionMat, reprojected);
		assert(reprojected.size() == cam2pt2Ds[i].size());
		assert(camMat2img.find(i)!=camMat2img.end());
		int imgIdx = camMat2img[i];
		for(int j=0; j<reprojected.size(); j++){
			float reprojectError = (float) norm((reprojected[j]-cam2pt2Ds[i][j]));	//distance between 2 points
			pt3Ds[cam2pt3Didxs[i][j]].img2error[imgIdx] = reprojectError;
		}
	}
}

//precondition: updateReprojectionErrors was called before
void PtCloud::getMeanReprojectionError( 	float 	&meanError){
	int N = pt3Ds.size();
	int totalMeasures = 0;
	float totalError  = 0.0f;

	for(int i=0; i<N; i++){
		const map<int, float>& img2error = pt3Ds[i].img2error;
		for(map<int, float>::const_iterator j = img2error.begin(); j != img2error.end(); j++) {
			totalError += (*j).second;
			totalMeasures++;
		}
	}

	meanError = totalError/totalMeasures;
}

//precondition: updateReprojectionErrors was called before
void PtCloud::removeHighError3D(	const float thresh){
	float meanError;
	getMeanReprojectionError(meanError);
	float errorThresh = meanError*thresh;

	int N = pt3Ds.size();
	vector<bool> 	removeMask(N,false);

	for(int i=0; i<N; i++){
		const map<int, float>& img2error = pt3Ds[i].img2error;
		assert(img2error.size()>0); 	//every 3d point must has at least 1 error measure, else it should be removed
		float totalPointError = 0.0f;
		for(map<int, float>::const_iterator j = img2error.begin(); j != img2error.end(); j++) {
			totalPointError += (*j).second;
		}
		if(totalPointError/img2error.size() > errorThresh){
			removeMask[i] = true;
		}
	}
	remove3Ds(removeMask);
}
