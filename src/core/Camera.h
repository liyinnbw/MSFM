/*
* camera object containing camera parameters and operations
* TODO: consider moving to datastructs folder
*/

#ifndef SRC_CORE_CAMERA_H_
#define SRC_CORE_CAMERA_H_

#include <vector>

#include <opencv2/opencv.hpp>
// #include <opengv/types.hpp>
#include <Eigen/Eigen>

#include "datastructs/Frame.h"

class Camera {
public:
	enum ProjectionStatus{
		Success 	= 0,
		OutOfBound	= 1,
		Behind		= 2
	};

	static Camera& GetInstance();

	Eigen::Vector3d
	getBearingVector(		const cv::KeyPoint 					&kpt);

	Eigen::Vector2d
	project(				const Eigen::Vector3d				&pt3D);		//3d point in camera coordinate

	Eigen::Vector2d
	project(				const Frame::Ptr 					&frame,
							const Eigen::Vector3d				&ptWorld,	//3d point in world coordinate
							ProjectionStatus					&status);

	bool outofbound(		const Eigen::Vector2d				&pt2D,
							const int							margin);

	// void getBearingVectors(	const std::vector<cv::KeyPoint> 	&kpts1,
	// 						const std::vector<cv::KeyPoint> 	&kpts2,
	// 						const std::vector<cv::DMatch> 		&matches,
	// 						opengv::bearingVectors_t 			&bearings1,
	// 						opengv::bearingVectors_t 			&bearings2);

	double getCamFocal();
	double getFOVH();
	double getFOVW();
	cv::Point2d getCamPrinciple();

	cv::Mat camMat;
	cv::Mat distortionMat;
	int w;
	int h;

private:
	Camera();
	virtual ~Camera();
};

#endif /* SRC_CORE_CAMERA_H_ */
