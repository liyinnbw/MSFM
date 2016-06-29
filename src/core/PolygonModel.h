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
	void getPolygons(std::vector<cv::Point3f> &_verts, std::vector<cv::Point3i> &_faces);

	std::vector<cv::Point3f> verts;
	std::vector<cv::Point3i> faces;

};

#endif /* POLYGONMODEL_H_ */
