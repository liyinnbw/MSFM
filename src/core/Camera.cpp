/*
* camera object containing camera parameters and operations
* TODO: consider moving to datastructs folder
*/

#include "Camera.h"
#include <vector>
#include <opencv2/calib3d/calib3d.hpp>	//for reprojection

using namespace cv;
// using namespace opengv;
using namespace Eigen;
using namespace std;

Camera& Camera::GetInstance(){
	static Camera instance;
	return instance;
}

Camera::Camera()
:w(0)
,h(0)
{
}

Camera::~Camera() {
}

Vector3d
Camera::getBearingVector(		const KeyPoint			&kpt)
{
	Point2d pp 	= getCamPrinciple();
	double f	= getCamFocal();
	Vector3d vec(kpt.pt.x-pp.x, kpt.pt.y-pp.y,f);
	//Vector3d vec(kpt.pt.x-pp.x, -(kpt.pt.y-pp.y),-f);
	vec.normalize();
	return vec;
}

Vector2d
Camera::project(				const Vector3d			&ptCam)		//3d point in camera coordinate
{
	//assert(ptCam[2]>=0.9);	//by right should be 1, but sometimes due to rounding, 1 is not obtainable
	Point2d pp 	= getCamPrinciple();
	double f	= getCamFocal();
	double mul 	= f/ptCam[2];

	return Vector2d(ptCam[0]*mul+pp.x,ptCam[1]*mul+pp.y);		//if using y down, z into, x right
	//return Vector2d(-ptCam[0]*mul+pp.x,ptCam[1]*mul+pp.y);	//if using y up, z out, x right
}
Vector2d
Camera::project(				const Frame::Ptr 		&frame,
								const Vector3d			&ptWorld,	//3d point in world coordinate
								ProjectionStatus		&status)
{

	/*
	//opencv method for comparision
	Matx34d Rt 			= frame->getCVTransform();
	Mat RtMat			= Mat(Rt);
	Mat R				= RtMat(Rect(0,0,3,3));
	Mat t				= RtMat(Rect(3,0,1,3));
	Mat rvec;
	Rodrigues(R,rvec);

	vector<Point3f> pts;
	pts.push_back(Point3f(ptWorld[0],ptWorld[1],ptWorld[2]));
	vector<Point2f> reprojects;
	Mat camMat = Camera::GetInstance().camMat;
	Mat distortionMat = Camera::GetInstance().distortionMat;
	projectPoints(pts, rvec, t, camMat, distortionMat, reprojects);
	Vector2d result_cv(reprojects[0].x, reprojects[0].y);
	status = ProjectionStatus::Success;
	*/


	//transform point to camera view space
	Vector3d pt = frame->rotation*(ptWorld-frame->position);

	//check status
	if(pt[2]<=1){
		status = ProjectionStatus::Behind;
		return Vector2d::Zero();
	}
	Vector2d result = project(pt);
	if(outofbound(result,0)){
		status = ProjectionStatus::OutOfBound;
	}else{
		status = ProjectionStatus::Success;
	}

	return result;
}

bool Camera::outofbound(		const Vector2d			&pt2D,
								const int				margin)
{
	return (pt2D[0]<-margin || pt2D[0]>=w+margin || pt2D[1]<-margin || pt2D[1]>=h+margin);
}

// void Camera::getBearingVectors(	const vector<KeyPoint> 	&kpts1,
// 								const vector<KeyPoint> 	&kpts2,
// 								const vector<DMatch> 	&matches,
// 								bearingVectors_t 		&bearings1,
// 								bearingVectors_t 		&bearings2)
// {
// 	bearings1.clear();
// 	bearings2.clear();

// 	Point2d pp 	= getCamPrinciple();
// 	double f	= getCamFocal();

// 	for(vector<DMatch>::const_iterator it = matches.begin(); it!=matches.end(); ++it){
// 		bearingVector_t vec1 (kpts1[it->queryIdx].pt.x-pp.x, kpts1[it->queryIdx].pt.y-pp.y, f);
// 		bearingVector_t vec2 (kpts2[it->trainIdx].pt.x-pp.x, kpts2[it->trainIdx].pt.y-pp.y, f);
// 		//bearingVector_t vec1 (kpts1[it->queryIdx].pt.x-pp.x, -(kpts1[it->queryIdx].pt.y-pp.y), -f);
// 		//bearingVector_t vec2 (kpts2[it->trainIdx].pt.x-pp.x, -(kpts2[it->trainIdx].pt.y-pp.y), -f);
// 		vec1.normalize();
// 		vec2.normalize();
// 		bearings1.push_back(vec1);
// 		bearings2.push_back(vec2);
// 	}
// }

double Camera::getCamFocal(){
	return camMat.at<double>(0,0);
}
double Camera::getFOVH(){
	double hHalf = h/2.0;
	double f = getCamFocal();
	return (atan (hHalf/f) * 180 / M_PI)*2;
}
double Camera::getFOVW(){
	double wHalf = w/2.0;
	double f = getCamFocal();
	return (atan (wHalf/f) * 180 / M_PI)*2;
}
Point2d Camera::getCamPrinciple(){
	return Point2d(camMat.at<double>(0,2),camMat.at<double>(1,2));
}
