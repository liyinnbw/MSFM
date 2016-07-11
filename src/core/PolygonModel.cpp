/*
 * PolygonModel.cpp
 *
 *  Created on: Jun 29, 2016
 *      Author: yoyo
 */

#include "PolygonModel.h"
#include "Utils.h"
#include <limits>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;
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
void PolygonModel::getVisiblePolygons(	const cv::Matx34d 				&camMat,
										const cv::Mat 					&camIntrinsicsMat,
										const cv::Mat					&distortionMat,
										std::vector<cv::Point3f>		&_verts,
										std::vector<cv::Point3i> 		&_faces)
{
	_verts.clear();
	_faces.clear();
	vector<int>	visibleFaceIdxs;
	trimFaces(camMat,camIntrinsicsMat,distortionMat,visibleFaceIdxs);
	_verts.reserve(verts.size());
	_faces.reserve(visibleFaceIdxs.size());
	copy(verts.begin(),verts.end(),back_inserter(_verts));
	for(int i=0; i<visibleFaceIdxs.size(); i++){
		_faces.push_back(faces[visibleFaceIdxs[i]]);
	}
}

void PolygonModel::projectPolygonToCamera(	const cv::Matx34d 				&camMat,
											const cv::Mat 					&camIntrinsicsMat,
											const cv::Mat					&distortionMat,
											std::vector<cv::Point2f>		&_verts,
											std::vector<cv::Point3i> 		&_faces)
{
	_verts.clear();
	_faces.clear();
	if(verts.empty()) return;
	vector<int>	visibleFaceIdxs;
	trimFaces(camMat,camIntrinsicsMat,distortionMat,visibleFaceIdxs);
	_verts.reserve(verts.size());
	_faces.reserve(visibleFaceIdxs.size());
	Mat R(3,3,CV_64F);
	R.at<double>(0,0) = camMat(0,0);
	R.at<double>(0,1) = camMat(0,1);
	R.at<double>(0,2) = camMat(0,2);
	R.at<double>(1,0) = camMat(1,0);
	R.at<double>(1,1) = camMat(1,1);
	R.at<double>(1,2) = camMat(1,2);
	R.at<double>(2,0) = camMat(2,0);
	R.at<double>(2,1) = camMat(2,1);
	R.at<double>(2,2) = camMat(2,2);
	Mat rvec;
	Rodrigues(R,rvec);
	Mat t(1,3,CV_64F);
	t.at<double>(0)   = camMat(0,3);
	t.at<double>(1)   = camMat(1,3);
	t.at<double>(2)   = camMat(2,3);
	projectPoints(verts, rvec, t, camIntrinsicsMat, distortionMat, _verts);
	for(int i=0; i<visibleFaceIdxs.size(); i++){
		_faces.push_back(faces[visibleFaceIdxs[i]]);
	}
}

//Reduce the number of faces need to be considered for a given camera
//XXX: only view frustrum and back face are checked. occlusion not checked.
void PolygonModel::trimFaces(		const cv::Matx34d 				&camMat,
									const cv::Mat 					&camIntrinsicsMat,
									const cv::Mat					&distortionMat,
									vector<int>						&faceIdxs)
{
	faceIdxs.clear();

	//get camera center world pos
	double TRx = camMat(0,3);
	double TRy = camMat(1,3);
	double TRz = camMat(2,3);

	double Ix0  = camMat(0,0);
	double Iy0  = camMat(0,1);
	double Iz0  = camMat(0,2);

	double Jx0  = camMat(1,0);
	double Jy0  = camMat(1,1);
	double Jz0  = camMat(1,2);

	double Kx0  = camMat(2,0);
	double Ky0  = camMat(2,1);
	double Kz0  = camMat(2,2);

	double Tx  = -TRx*Ix0 -TRy*Jx0 -TRz*Kx0;
	double Ty  = -TRx*Iy0 -TRy*Jy0 -TRz*Ky0;
	double Tz  = -TRx*Iz0 -TRy*Jz0 -TRz*Kz0;

	//find ray start
	Point3f camCenter = Point3f(Tx,Ty,Tz);

	//some reusable variables
	int imgW 	= camIntrinsicsMat.at<double>(0,2)*2;	//XXX: assumed camera principle is at image center
	int imgH	= camIntrinsicsMat.at<double>(1,2)*2;	//XXX: assumed camera principle is at image center
	Mat R(3,3,CV_64F);
	R.at<double>(0,0) = camMat(0,0);
	R.at<double>(0,1) = camMat(0,1);
	R.at<double>(0,2) = camMat(0,2);
	R.at<double>(1,0) = camMat(1,0);
	R.at<double>(1,1) = camMat(1,1);
	R.at<double>(1,2) = camMat(1,2);
	R.at<double>(2,0) = camMat(2,0);
	R.at<double>(2,1) = camMat(2,1);
	R.at<double>(2,2) = camMat(2,2);
	Mat rvec;
	Rodrigues(R,rvec);
	Mat t(1,3,CV_64F);
	t.at<double>(0)   = camMat(0,3);
	t.at<double>(1)   = camMat(1,3);
	t.at<double>(2)   = camMat(2,3);

	for(int i=0; i<faces.size(); i++){
		// get triangle edge vectors and plane normal
		int p0Idx			= faces[i].x;
		int p1Idx			= faces[i].y;
		int p2Idx			= faces[i].z;
		Mat e1 				= Mat(verts[p1Idx] - verts[p0Idx]);
		Mat e2 				= Mat(verts[p2Idx] - verts[p0Idx]);
		Mat fn				= e1.cross(e2);

		//backface culling
		Mat v 				= Mat(verts[p0Idx] - camCenter);

		if (v.dot(fn)>=0){
			continue;
		}

		//view frustum culling
		//put verts in camera coordinate
		Matx43d hom_verts(	verts[p0Idx].x,verts[p1Idx].x,verts[p2Idx].x,
							verts[p0Idx].y,verts[p1Idx].y,verts[p2Idx].y,
							verts[p0Idx].z,verts[p1Idx].z,verts[p2Idx].z,
							1.0,			1.0,			1.0);
		Matx33d pt_cc = camMat*hom_verts;

		//check if triangle is completely behind camera
		if(pt_cc(2,0)<=0 && pt_cc(2,1)<=0 && pt_cc(2,2)<=0){
			continue;
		}


		//project triangle vertices to 2d image
		//XXX: to take distortion mat into account, use the commented out opencv version
		Mat hom_2d = camIntrinsicsMat*Mat(pt_cc);
		hom_2d.row(0)/=hom_2d.row(2);
		hom_2d.row(1)/=hom_2d.row(2);

		//check if triangle is completely outside image using simple bounding box collision test. Not 100% filter
		//check x
		Mat xInBound = (hom_2d.row(0)>=0 & hom_2d.row(0)<imgW);
		if(countNonZero(xInBound) == 0){
			//no vert's x is in bound
			continue;
		}
		//check y
		Mat yInBound = (hom_2d.row(1)>=0 & hom_2d.row(1)<imgH);
		if(countNonZero(yInBound) == 0){
			//no vert's y is in bound
			continue;
		}


		/*//opencv version
		//project triangle vertices to 2d image
		vector<Point2f> pts2D;
		vector<Point3f> pts3D;
		pts3D.push_back(verts[p0Idx]);
		pts3D.push_back(verts[p1Idx]);
		pts3D.push_back(verts[p2Idx]);
		projectPoints(pts3D, rvec, t, camIntrinsicsMat, distortionMat, pts2D);


		//check if triangle is completely outside image using simple bounding box collision test. Not 100% filter
		//check x
		int cnt = 0;
		for(int j=0; j<pts2D.size(); j++){
			if(pts2D[j].x>=0 && pts2D[j].x<imgW){
				cnt++;
			}
		}
		if(cnt == 0 ) continue;
		cnt = 0;
		for(int j=0; j<pts2D.size(); j++){
			if(pts2D[j].y>=0 && pts2D[j].y<imgH){
				cnt++;
			}
		}
		if(cnt == 0 ) continue;
		*/

		faceIdxs.push_back(i);
	}

	cout<<"visible face cnt = "<<faceIdxs.size()<<"/"<<faces.size()<<endl;

}

//    Input:  a ray R, and a triangle T
//    Output: *I = intersection point (when it exists)
//    Return: -1 = triangle is degenerate (a segment or point)
//             0 =  disjoint (no intersect)
//             1 =  intersect in unique point I1
//             2 =  are in the same plane
int PolygonModel::intersect3DRayTriangle(		const int						faceIdx,
												const cv::Point3f				&rayStart,
												const cv::Point3f				&rayEnd,
												cv::Point3f						&intersection,
												cv::Point3f						&intersectionNormal)
{
	//no polygon faces
	if(faces.empty()){
		return -1;
	}

	// get triangle edge vectors and plane normal
	int p0Idx			= faces[faceIdx].x;
	int p1Idx			= faces[faceIdx].y;
	int p2Idx			= faces[faceIdx].z;
	Mat e1 				= Mat(verts[p1Idx] - verts[p0Idx]);
	Mat e2 				= Mat(verts[p2Idx] - verts[p0Idx]);
	Mat fn				= e1.cross(e2);

	// triangle is degenerate
	if (countNonZero(fn) == 0){
		return -1;
	}



	//check parallel
	Mat ray = Mat(rayEnd-rayStart);              // ray direction vector
	Mat w0 	= Mat(rayStart - verts[p0Idx]);
	float a = -fn.dot(w0);
	float b = fn.dot(ray);
	double parallelThresh = 0.0001;
	if (fabs(b) < parallelThresh) {  				// ray is  parallel to triangle plane
		if (a == 0)             				// ray lies in triangle plane
			return 2;
		else return 0;         					// ray disjoint from plane
	}

	// get intersect point of ray with triangle plane
	float r = a / b;
	if (r <= 0.0)                    				// ray goes away from triangle or triangle is back facing the ray
		return 0;                   			// => no intersect
												// for a segment, also test if (r > 1.0) => no intersect
	Mat pt = Mat(rayStart) + r * ray;           // intersect point of ray and plane


	// is intersection inside triangle?
	float    uu, uv, vv, wu, wv, D;
	uu 			= e1.dot(e1);
	uv 			= e1.dot(e2);
	vv 			= e2.dot(e2);
	Mat w 		= pt - Mat(verts[p0Idx]);
	wu 			= w.dot(e1);
	wv 			= w.dot(e2);
	D 			= uv * uv - uu * vv;

	// get and test parametric coords
	float s, t;
	s = (uv * wv - vv * wu) / D;
	if (s < 0.0 || s > 1.0)         // I is outside T
		return 0;
	t = (uv * wu - uu * wv) / D;
	if (t < 0.0 || (s + t) > 1.0)  // I is outside T
		return 0;

	intersection 		= Point3f(pt.at<float>(0),pt.at<float>(1),pt.at<float>(2));
	intersectionNormal 	= Point3f(fn.at<float>(0),fn.at<float>(1),fn.at<float>(2));
	//normalize normal
	intersectionNormal /= norm(intersectionNormal);
	return 1;                       // I is in T

}

void PolygonModel::getPointsIntersectingSurface(	const cv::Matx34d 				&camMat,
													const cv::Mat 					&camIntrinsicsMat,
													const cv::Mat					&distortionMat,
													const std::vector<cv::Point2f>	&pt2Ds,
													std::vector<cv::Point3f>		&pt3Ds,	//ray intersecting surface poses
													std::vector<cv::Point3f>		&norms,	//intersecting surface normals
													std::vector<bool>				&status)	//has intersection = 1
{
	status = vector<bool>(pt2Ds.size(), false); //initialize all status to fail

	//no polygon faces
	if(faces.empty()){
		return;
	}

	pt3Ds.clear();
	norms.clear();
	pt3Ds.reserve(pt2Ds.size());
	norms.reserve(pt2Ds.size());

	//get camera center world pos
	double TRx = camMat(0,3);
	double TRy = camMat(1,3);
	double TRz = camMat(2,3);

	double Ix0  = camMat(0,0);
	double Iy0  = camMat(0,1);
	double Iz0  = camMat(0,2);

	double Jx0  = camMat(1,0);
	double Jy0  = camMat(1,1);
	double Jz0  = camMat(1,2);

	double Kx0  = camMat(2,0);
	double Ky0  = camMat(2,1);
	double Kz0  = camMat(2,2);

	double Tx  = -TRx*Ix0 -TRy*Jx0 -TRz*Kx0;
	double Ty  = -TRx*Iy0 -TRy*Jy0 -TRz*Ky0;
	double Tz  = -TRx*Iz0 -TRy*Jz0 -TRz*Kz0;

	//find ray start
	Point3f camCenter = Point3f(Tx,Ty,Tz);

	//prepare reusable matrices
	Matx44d homoTransfMat(	camMat(0,0),camMat(0,1),camMat(0,2),camMat(0,3),
							camMat(1,0),camMat(1,1),camMat(1,2),camMat(1,3),
							camMat(2,0),camMat(2,1),camMat(2,2),camMat(2,3),
							0,0,0,1);

	Mat camProjectionInv = Mat(homoTransfMat).inv();
	Mat camIntrinsicsInv = camIntrinsicsMat.inv();

	//find ray ends
	vector<Point3f> rayEnds;
	rayEnds.reserve(pt2Ds.size());
	for(int i=0; i<pt2Ds.size(); i++){
		//TODO: undistort 2d point if distortion mat is not 0

		//put in camera coordinates
		Matx31d hom_pt(pt2Ds[i].x, pt2Ds[i].y, 1.0);
		Mat pt3DMat = camIntrinsicsInv*Mat(hom_pt);

		//put in world coordinates
		Matx41d hom_pt3d(pt3DMat.at<double>(0),pt3DMat.at<double>(1),pt3DMat.at<double>(2),1.0);
		Mat pt4DMat = camProjectionInv*Mat(hom_pt3d);
		Point3f rayEnd(pt4DMat.at<double>(0),pt4DMat.at<double>(1),pt4DMat.at<double>(2));

		//normalize ray
		Point3f ray = rayEnd - camCenter;
		ray /= norm(ray);
		rayEnd = camCenter+ray;

		rayEnds.push_back(rayEnd);
	}

	//trim faces
	vector<int> visibleFaceIdxs;
	trimFaces(camMat,camIntrinsicsMat,distortionMat, visibleFaceIdxs);


	//find intersections & normals
	for(int j=0; j<rayEnds.size(); j++){
		bool hasIntersection = false;
		Point3f nearestIntersection;
		Point3f nearestIntersectionNormal;
		float minDist = numeric_limits<float>::infinity();
		for(int i=0; i<visibleFaceIdxs.size(); i++){

			int faceIdx = visibleFaceIdxs[i];
			Point3f intersection;
			Point3f normal;
			if(intersect3DRayTriangle(faceIdx,camCenter,rayEnds[j],intersection, normal)==1){
				float dist = cv::norm(intersection- camCenter);
				assert(dist>=0);
				if(dist<minDist){
					minDist = dist;
					nearestIntersection 	= intersection;
					nearestIntersectionNormal = normal;
					hasIntersection = true;
				}
			}
		}

		pt3Ds.push_back(nearestIntersection);
		norms.push_back(nearestIntersectionNormal);
		status[j] = hasIntersection;
	}
}
