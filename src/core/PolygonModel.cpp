/*
 * PolygonModel.cpp
 *
 *  Created on: Jun 29, 2016
 *      Author: yoyo
 */

#include "PolygonModel.h"

PolygonModel::PolygonModel() {
	// TODO Auto-generated constructor stub

}

PolygonModel::~PolygonModel() {
	// TODO Auto-generated destructor stub
}

void PolygonModel::reset(){
	verts.clear();
	faces.clear();
}
void PolygonModel::getPolygons(std::vector<cv::Point3f> &_verts, std::vector<cv::Point3i> &_faces){
	_verts.clear();
	_faces.clear();
	_verts.reserve(verts.size());
	_faces.reserve(faces.size());
	copy(verts.begin(),verts.end(),back_inserter(_verts));
	copy(faces.begin(),faces.end(),back_inserter(_faces));
}
