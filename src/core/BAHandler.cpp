//
//  BAHandler.cpp
//  3D_Reconstruction
//
//  Created by qiyue song on 19/4/14.
//  Copyright (c) 2014 qiyue.song. All rights reserved.
//
#define V3DLIB_ENABLE_SUITESPARSE
#include "BAHandler.h"

#include "ba/v3d_linear.h"
#include "ba/v3d_vrmlio.h"
#include "ba/v3d_metricbundle.h"
#include "ba/v3d_stereobundle.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cvsba/cvsba.h>

using namespace V3D;
using namespace std;
using namespace cv; 
using namespace cvsba;

namespace
{
	inline double
	showErrorStatistics(double const f0,
						StdDistortionFunction const& distortion,
						vector<CameraMatrix> const& cams,
						vector<Vector3d> const& Xs,
						vector<Vector2d> const& measurements,
						vector<int> const& correspondingView,
						vector<int> const& correspondingPoint)
	{
		int const K = measurements.size();
		
		double meanReprojectionError = 0.0;
		for (int k = 0; k < K; ++k)
		{
			int const i = correspondingView[k];
			int const j = correspondingPoint[k];
			Vector2d p = cams[i].projectPoint(distortion, Xs[j]);
			
			double reprojectionError = norm_L2(f0 * (p - measurements[k]));
			meanReprojectionError += reprojectionError;
		}
		//cout << "mean reprojection error (in pixels): " << meanReprojectionError/K << endl;

		return meanReprojectionError/K;
	}
}

void BAHandler::adjustBundle(	PtCloud &ptCloud,
								Mat& cam_matrix,
								Mat& distortion_coefficients){

	vector<Point2f> 	xys;
	vector<int>			imgIdxs;
	vector<int>			pt3DIdxs;
	double ppx,ppy;
	double 	meanErrorBefore, meanErrorAfter;
	ptCloud.get2DsHave3D(xys,imgIdxs,pt3DIdxs);

	int N = ptCloud.camMats.size(), M = ptCloud.pt3Ds.size(), K = xys.size();
	cout << "N (cams) = " << N << " M (points) = " << M << " K (measurements) = " << K << endl;

	StdDistortionFunction distortion;
	distortion.k1 = distortion_coefficients.at<double>(0,0);
	distortion.k2 = distortion_coefficients.at<double>(0,1);
	distortion.p1 = distortion_coefficients.at<double>(0,2);
	distortion.p2 = distortion_coefficients.at<double>(0,3);


	//convert camera intrinsics to BA datastructs
	Matrix3x3d KMat;
	makeIdentityMatrix(KMat);
	KMat[0][0] = cam_matrix.at<double>(0,0); //fx
	KMat[1][1] = cam_matrix.at<double>(1,1); //fy
	KMat[0][1] = cam_matrix.at<double>(0,1); //skew
	KMat[0][2] = cam_matrix.at<double>(0,2); //ppx
	KMat[1][2] = cam_matrix.at<double>(1,2); //ppy

	ppx = KMat[0][2];
	ppy = KMat[1][2];

	double const f0 = KMat[0][0];

	//cout << "Cam = "<<endl;
	//displayMatrix(KMat);

	Matrix3x3d Knorm = KMat;
	// Normalize the intrinsic to have unit focal length.
	scaleMatrixIP(1.0/f0, Knorm);
	Knorm[2][2] = 1.0;

	//convert 3D point cloud to BA datastructs
	vector<Vector3d > Xs(M);
	for (int i = 0; i < M; ++i)
	{
		Xs[i][0] = ptCloud.pt3Ds[i].pt.x;
		Xs[i][1] = ptCloud.pt3Ds[i].pt.y;
		Xs[i][2] = ptCloud.pt3Ds[i].pt.z;
	}


	//convert cameras to BA datastructs
	vector<CameraMatrix> cams(N);
	for (int i = 0; i < N; ++i)
	{
		int camId = i;
		Matrix3x3d R;
		Vector3d T;

		Matx34d& P = ptCloud.camMats[i];

		R[0][0] = P(0,0); R[0][1] = P(0,1); R[0][2] = P(0,2); T[0] = P(0,3);
		R[1][0] = P(1,0); R[1][1] = P(1,1); R[1][2] = P(1,2); T[1] = P(1,3);
		R[2][0] = P(2,0); R[2][1] = P(2,1); R[2][2] = P(2,2); T[2] = P(2,3);

		cams[i].setIntrinsic(Knorm);
		cams[i].setRotation(R);
		cams[i].setTranslation(T);
	}


	vector<Vector2d > measurements;
	vector<int> correspondingView; 		//corresponding camera mat idx
	vector<int> correspondingPoint;		//corresponding 3d point idx

	measurements.reserve(K);
	correspondingView.reserve(K);
	correspondingPoint.reserve(K);

	//convert 2D measurements to BA datastructs
	for (int i = 0; i < xys.size(); ++i){
		Vector3d p;
		p[0] = xys[i].x;
		p[1] = xys[i].y;
		p[2] = 1.0;
		scaleVectorIP(1.0/f0, p);
		measurements.push_back(Vector2d(p[0], p[1]));
		correspondingView.push_back(ptCloud.img2camMat[imgIdxs[i]]);
		correspondingPoint.push_back(pt3DIdxs[i]);
	}

	meanErrorBefore = showErrorStatistics(f0, distortion, cams, Xs, measurements, correspondingView, correspondingPoint);

	double const inlierThreshold = 2.0 / fabs(f0);

	Matrix3x3d K0 = cams[0].getIntrinsic();
	//cout << "K0 = "; displayMatrix(K0);

	bool good_adjustment = false;
	{
		ScopedBundleExtrinsicNormalizer extNorm(cams, Xs);
		ScopedBundleIntrinsicNormalizer intNorm(cams,measurements,correspondingView);
		CommonInternalsMetricBundleOptimizer opt(V3D::FULL_BUNDLE_FOCAL_LENGTH_PP, inlierThreshold, K0, distortion, cams, Xs,
												 measurements, correspondingView, correspondingPoint);

		opt.tau = 1e-3;
		opt.maxIterations = 50;
		opt.minimize();

		cout << "optimizer status = " << opt.status << endl;

		good_adjustment = (opt.status != 2);
	}

	//cout << "refined K = "; displayMatrix(K0);

	for (int i = 0; i < N; ++i) cams[i].setIntrinsic(K0);

	Matrix3x3d Knew = K0;
	scaleMatrixIP(f0, Knew);
	Knew[2][2] = 1.0;

	//cout << "Cam new = "<<endl;
	//displayMatrix(Knew);

	meanErrorAfter = showErrorStatistics(f0, distortion, cams, Xs, measurements, correspondingView, correspondingPoint);

	if(good_adjustment) {

		//extract 3D points
		for (unsigned int j = 0; j < Xs.size(); ++j)
		{
			ptCloud.pt3Ds[j].pt.x = Xs[j][0];
			ptCloud.pt3Ds[j].pt.y = Xs[j][1];
			ptCloud.pt3Ds[j].pt.z = Xs[j][2];
		}

		//extract adjusted cameras
		for (int i = 0; i < N; ++i)
		{
			Matrix3x3d R = cams[i].getRotation();
			Vector3d T = cams[i].getTranslation();

			Matx34d P;
			P(0,0) = R[0][0]; P(0,1) = R[0][1]; P(0,2) = R[0][2]; P(0,3) = T[0];
			P(1,0) = R[1][0]; P(1,1) = R[1][1]; P(1,2) = R[1][2]; P(1,3) = T[1];
			P(2,0) = R[2][0]; P(2,1) = R[2][1]; P(2,2) = R[2][2]; P(2,3) = T[2];

			ptCloud.camMats[i] = P;
		}

		cam_matrix.at<double>(0,0) = Knew[0][0];
		cam_matrix.at<double>(0,1) = Knew[0][1];
		cam_matrix.at<double>(0,2) = Knew[0][2];
		cam_matrix.at<double>(1,1) = Knew[1][1];
		cam_matrix.at<double>(1,2) = Knew[1][2];
	}

	cout<<"focal :"<<f0<<" -> "<<Knew[0][0]<<endl;
	cout<<"center:("<<ppx<<","<<ppy<<") -> ("<<Knew[0][2]<<","<<Knew[1][2]<<")"<<endl;
	cout<<"error :"<<meanErrorBefore<<" -> "<<meanErrorAfter<<endl;
}

void BAHandler::adjustBundle_sba(	PtCloud &ptCloud,
									Mat& cam_matrix,
									Mat& distortion_coefficients)
{

	float error_before;
	double f_before, ppx_before, ppy_before;
	ptCloud.getMeanReprojectionError(error_before);
	f_before 		= cam_matrix.at<double>(0,0);
	ppx_before 		= cam_matrix.at<double>(0,2);
	ppy_before		= cam_matrix.at<double>(1,2);

	int M = ptCloud.camMats.size();
	int N = ptCloud.pt3Ds.size();
	vector<Point3f> points;
	ptCloud.getXYZs(points);

	vector<Mat> rvecs, ts;
	ptCloud.getCamRvecsAndTs(rvecs,ts);

	vector<Mat> cameraMatrix(M,cam_matrix);				//each camera's camMat
	vector<Mat> distCoeffs(M,distortion_coefficients);	//each camera's distortion

	vector<vector<Point2f> > imagePoints(M,vector<Point2f>(N));
	vector<vector<int> >	 visibility(M,vector<int>(N,0));
	for(int i=0; i<N; i++){
		map<int, int> img_pt2D_idxs = ptCloud.pt3Ds[i].img2ptIdx;
		for(map<int, int>::iterator i = img_pt2D_idxs.begin(); i != img_pt2D_idxs.end(); i++) {
			int imgIdx 		= (*i).first;
			int camIdx 		= ptCloud.img2camMat[imgIdx];
			int pt2Didx 	= (*i).second;
			Point2f &pt2d	= ptCloud.img2pt2Ds[imgIdx][pt2Didx].pt;
			int	pt3Didx		= ptCloud.img2pt2Ds[imgIdx][pt2Didx].pt3D_idx;
			imagePoints[camIdx][pt3Didx] = pt2d;
			visibility [camIdx][pt3Didx] = 1;
		}
	}

	Sba sba;
	Sba::Params runParams( Sba::MOTIONSTRUCTURE,50, 1e-3, 5, 5, false);
	sba.setParams(runParams);
	double error = sba.run (  points, imagePoints, visibility, cameraMatrix, rvecs, ts,distCoeffs);

	//extract 3D points
	for (int i = 0; i < N; i++)
	{
		ptCloud.pt3Ds[i].pt.x = (float) points[i].x;
		ptCloud.pt3Ds[i].pt.y = (float) points[i].y;
		ptCloud.pt3Ds[i].pt.z = (float) points[i].z;
	}

	//extract adjusted cameras
	for (int i = 0; i < M; i++)
	{
		Mat R;
		Rodrigues(rvecs[i],R);

		Matx34d P =	Matx34d(R.at<double>(0,0),	R.at<double>(0,1),	R.at<double>(0,2),	ts[i].at<double>(0),
							R.at<double>(1,0),	R.at<double>(1,1),	R.at<double>(1,2),	ts[i].at<double>(1),
							R.at<double>(2,0),	R.at<double>(2,1),	R.at<double>(2,2),	ts[i].at<double>(2));

		ptCloud.camMats[i] = P;
	}

	cam_matrix 				= cameraMatrix[0];
	distortion_coefficients	= distCoeffs[0];


	float error_after;
	double f_after, ppx_after, ppy_after;
	ptCloud.updateReprojectionErrors(cam_matrix,distortion_coefficients);
	ptCloud.getMeanReprojectionError(error_after);
	f_after 		= cam_matrix.at<double>(0,0);
	ppx_after 		= cam_matrix.at<double>(0,2);
	ppy_after		= cam_matrix.at<double>(1,2);

	cout<<"focal :"<<f_before<<" -> "<<f_after<<endl;
	cout<<"center:("<<ppx_before<<","<<ppy_before<<") -> ("<<ppx_after<<","<<ppy_after<<")"<<endl;
	cout<<"error :"<<error_before<<" -> "<<error<<"(="<<error_after<<")"<<endl;

}
