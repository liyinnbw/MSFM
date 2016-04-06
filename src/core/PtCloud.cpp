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
	imgs.clear();
	pt3Ds.clear();
	pt2Ds.clear();
	img2pt2Ds.clear();
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
	img2pt2Ds[imgIdx] = vector<int>();
	img2pt2Ds[imgIdx].reserve(kpts.size());
	for(int i=0; i<kpts.size(); i++){
		Pt2D pt2D;
		pt2D.pt 			= kpts[i].pt;
		pt2D.dec			= decs.row(i);
		pt2D.img_idx 		= imgIdx;
		pt2D.pt3D_idx 		= -1;
		pt2Ds.push_back(pt2D);
		img2pt2Ds[imgIdx].push_back(pt2Ds.size()-1);
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
	vector<int> pt2Didxs1 = img2pt2Ds[imgIdx1];
	vector<int> pt2Didxs2 = img2pt2Ds[imgIdx2];
	for(int i=0; i<xyzs.size(); i++){
		//add 3D point
		int pt2D_idx1 		= pt2Didxs1[img2Didxs1[i]];
		int pt2D_idx2 		= pt2Didxs2[img2Didxs2[i]];
		Pt3D pt3D;
		pt3D.pt				= xyzs[i];
		pt3D.img2ptIdx[imgIdx1] = img2Didxs1[i];
		pt3D.img2error[imgIdx1] = 0.0f;
		pt3D.img2ptIdx[imgIdx2] = img2Didxs2[i];
		pt3D.img2error[imgIdx2] = 0.0f;
		pt3Ds.push_back(pt3D);

		//update corresponding 2D points
		assert(pt2Ds[pt2D_idx1].img_idx == imgIdx1 && pt2Ds[pt2D_idx1].pt3D_idx == -1);
		assert(pt2Ds[pt2D_idx2].img_idx == imgIdx2 && pt2Ds[pt2D_idx2].pt3D_idx == -1);
		pt2Ds[pt2D_idx1].pt3D_idx = pt3Ds.size()-1;
		pt2Ds[pt2D_idx2].pt3D_idx = pt3Ds.size()-1;
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
			map<int,int> img2ptIdx = (*it).img2ptIdx;
			for(map<int, int>::iterator j = img2ptIdx.begin(); j != img2ptIdx.end(); j++) {
				int imgIdx 	= (*j).first;
				int imgPtIdx= (*j).second;
				int pt2Didx = img2pt2Ds[imgIdx][imgPtIdx];
				pt2Ds[pt2Didx].pt3D_idx = -1;
			}
			it = pt3Ds.erase(it);
			removed++;
		}else{
			//due to some 3d points being removed, idxs change, need update
			int pt3Didx = it - pt3Ds.begin();
			map<int,int> img2ptIdx = (*it).img2ptIdx;
			for(map<int, int>::iterator j = img2ptIdx.begin(); j != img2ptIdx.end(); j++) {
				int imgIdx 	= (*j).first;
				int imgPtIdx= (*j).second;
				int pt2Didx = img2pt2Ds[imgIdx][imgPtIdx];
				pt2Ds[pt2Didx].pt3D_idx = pt3Didx;
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
}

void PtCloud::update3D(	int 			imgIdx,
						vector<int> 	&pt3Didxs,
						vector<int> 	&img2Didxs){

	//checks
	assert(pt3Didxs.size() == img2Didxs.size());
	assert(img2pt2Ds.find(imgIdx)!=img2pt2Ds.end());	//already has 2d

	//update data
	vector<int> pt2Didxs = img2pt2Ds[imgIdx];
	for(int i=0; i<pt3Didxs.size(); i++){
		//update 3d point's pt2D_idx
		int pt2D_idx	= pt2Didxs[img2Didxs[i]];
		int pt3D_idx	= pt3Didxs[i];
		assert(pt3Ds[pt3D_idx].img2ptIdx.find(imgIdx) == pt3Ds[pt3D_idx].img2ptIdx.end());
		pt3Ds[pt3D_idx].img2ptIdx[imgIdx] = img2Didxs[i];
		pt3Ds[pt3D_idx].img2error[imgIdx] = 0.0f;

		//update corresponding 2D points
		assert(pt2Ds[pt2D_idx].img_idx == imgIdx);
		assert(pt2Ds[pt2D_idx].pt3D_idx == -1);
		pt2Ds[pt2D_idx].pt3D_idx = pt3D_idx;
	}
}

void PtCloud::getImageFeatures(	const int 			imgIdx,
								vector<KeyPoint> 	&kpts,
								Mat					&decs){
	assert(img2pt2Ds.find(imgIdx) != img2pt2Ds.end());
	kpts.clear();
	vector<Mat> decList;
	vector<int> pt2Didxs = img2pt2Ds[imgIdx];
	kpts.reserve(pt2Didxs.size());
	decs.reserve(pt2Didxs.size());
	for(int i=0; i<pt2Didxs.size(); i++){
		int idx		= pt2Didxs[i];
		KeyPoint pt = KeyPoint(pt2Ds[idx].pt, 0);	//wrap point2f in keypoint, size=0
		Mat dec		= pt2Ds[idx].dec;
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

	vector<int> pt2Didxs = img2pt2Ds[imgIdx];
	for(int i=0; i<img2Didxs.size(); i++){
		int pt2D_idx	= pt2Didxs[img2Didxs[i]];
		int pt3D_idx	= pt2Ds[pt2D_idx].pt3D_idx;
		if(pt3D_idx != -1){
			has3D[i] = true;
		}
	}
}
void PtCloud::get3DfromImage2D(	const int 			imgIdx,
								const vector<int>	img2Didxs,		//CAUSION: this does not index class variable pt2Ds but class variable img2pt2Ds[imgIdx]
								vector<Point3f>		&pts3D,
								vector<int>			&pts3DIdxs)		//this indexs the class variable pt3Ds
{
	assert(img2pt2Ds.find(imgIdx) != img2pt2Ds.end());
	pts3D.clear();
	pts3DIdxs.clear();
	pts3D.reserve(img2Didxs.size());
	pts3DIdxs.reserve(img2Didxs.size());
	vector<int> pt2Didxs = img2pt2Ds[imgIdx];
	for(int i=0; i<img2Didxs.size(); i++){
		int pt2D_idx	= pt2Didxs[img2Didxs[i]];
		int pt3D_idx	= pt2Ds[pt2D_idx].pt3D_idx;
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
	vector<int> pt2Didxs = img2pt2Ds[imgIdx];
	pts3D.reserve(pt2Didxs.size()/2);	//estimated size, for efficiency
	pts3DIdxs.reserve(pt2Didxs.size()/2);

	for(int i=0; i<pt2Didxs.size(); i++){
		int pt2D_idx	= pt2Didxs[i];
		int pt3D_idx	= pt2Ds[pt2D_idx].pt3D_idx;
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
void PtCloud::get2DsHave3D(		vector<Point2f> 	&xys,
								vector<int>			&imgIdxs,
								vector<int>			&pt3DIdxs){
	xys.clear();
	imgIdxs.clear();
	pt3DIdxs.clear();

	//for efficiency
	int roughSizeEstimate = pt2Ds.size()/2;
	xys.reserve(roughSizeEstimate);
	imgIdxs.reserve(roughSizeEstimate);
	pt3DIdxs.reserve(roughSizeEstimate);

	for(int i=0; i<pt2Ds.size(); i++){
		if(pt2Ds[i].pt3D_idx!=-1){
			xys.push_back(		pt2Ds[i].pt);
			imgIdxs.push_back(	pt2Ds[i].img_idx);
			pt3DIdxs.push_back( pt2Ds[i].pt3D_idx);
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
