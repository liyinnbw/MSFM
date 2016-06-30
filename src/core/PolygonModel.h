/*
 * PolygonModel.h
 *
 *  Created on: Jun 29, 2016
 *      Author: yoyo
 */

#ifndef POLYGONMODEL_H_
#define POLYGONMODEL_H_
#include <opencv2/core/core.hpp>

class PolygonModel {
public:
	PolygonModel();
	virtual ~PolygonModel();

	void reset();
	void getPolygons(						std::vector<cv::Point3f> 		&_verts,
											std::vector<cv::Point3i> 		&_faces);

	void getVisiblePolygons(				const cv::Matx34d 				&camMat,
											const cv::Mat 					&camIntrinsicsMat,
											const cv::Mat					&distortionMat,
											std::vector<cv::Point3f> 		&_verts,
											std::vector<cv::Point3i> 		&_faces);

	int intersect3DRayTriangle(				const int						faceIdx,
											const cv::Point3f				&rayStart,
											const cv::Point3f				&rayEnd,
											cv::Point3f						&intersection,
											cv::Point3f						&intersectionNormal);

	void getPointsIntersectingSurface(		const cv::Matx34d 				&camMat,
											const cv::Mat 					&camIntrinsicsMat,
											const cv::Mat					&distortionMat,
											const std::vector<cv::Point2f>	&pt2Ds,
											std::vector<cv::Point3f>		&pt3Ds,	//ray intersecting surface poses
											std::vector<cv::Point3f>		&norms,	//intersecting surface normals
											std::vector<bool>				&status);	//has intersection = 1

	void trimFaces(							const cv::Matx34d 				&camMat,
											const cv::Mat 					&camIntrinsicsMat,
											const cv::Mat					&distortionMat,
											std::vector<int>				&visibleFaceIdxs);

	std::vector<cv::Point3f> verts;
	std::vector<cv::Point3i> faces;

};

#endif /* POLYGONMODEL_H_ */
