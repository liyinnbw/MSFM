/*
 * PtCloud.h
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#ifndef PTCLOUD_H_
#define PTCLOUD_H_


#include <vector>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <map>
#include <string>

struct Pt2D {
	cv::Point2f pt;				//xy
	cv::Mat 	dec;			//descriptor
	int			img_idx;		//img idx where it comes from
	int 		pt3D_idx;		//corresponding 3D point idx, -1 if not used
};

struct Pt3D {
	cv::Point3f 			pt;				//xyz
	cv::Point3f 			norm;			//normal vector in world coordinate
	std::map<int, int> 		img2ptIdx;		//corresponding image idx and idx of 2D point in that image
	std::map<int, float> 	img2error;		//corresponding image idx and repojection error on that image

};

class PtCloud {
public:
	PtCloud();
	virtual ~PtCloud();
	void clear();
	void add2D(					const int 							imgIdx,
								const std::vector<cv::KeyPoint> 	&kpts,
								const cv::Mat 						&decs );

	void add3D(					const int 					imgIdx1,
								const int 					imgIdx2,
								const std::vector<cv::Point3f> 	&xyzs,
								const std::vector<int> 		&img2Didxs1,
								const std::vector<int> 		&img2Didxs2);

	void remove3Ds(				const std::vector<bool> 	&removeMask);

	void addCamMat(				int							imgIdx,
								cv::Matx34d					&camMat);

	void update3D(				const int 					imgIdx,
								const std::vector<int> 		&pt3Didxs,
								const std::vector<int> 		&img2Didxs);

	void getImageFeatures(		const int 					imgIdx,
								std::vector<cv::KeyPoint>	&kpts,
								cv::Mat						&decs);

	void getImageCamMat(		int 						imgIdx,
								cv::Matx34d					&camMat);

	void getUsedImageIdxs(		std::vector<int>			&imgIdxs);

	void checkImage2Dfor3D(		const int 					imgIdx,
								const std::vector<int>		img2Didxs,
								std::vector<bool> 			&has3D);

	void get3DfromImage2D(		const int 					imgIdx,
								const std::vector<int>		img2Didxs,
								std::vector<cv::Point3f>	&pts3D,
								std::vector<int>			&pts3DIdxs);

	void getAll3DfromImage2D(	const int 					imgIdx,
								std::vector<cv::Point3f>	&pts3D,
								std::vector<int>			&pts3DIdxs);

	bool imageIsUsed(			const int					imgIdx);

	void getXYZs( 				std::vector<cv::Point3f>	&xyzs) const;

	void getPointNormals(		std::vector<cv::Point3f>	&norms) const;

	void getAverageDecs( 		std::vector<cv::Mat> 		&decs);

	void getAverageDecAtPt3DIdx(const int					idx,
								cv::Mat 					&averageDec);

	void get2DsHave3D(			std::vector<cv::Point2f> 	&xys,
								std::vector<int>			&imgIdxs,
								std::vector<int>			&pt3DIdxs);

	void getImageMeasurements(	const int					&imgIdx,
								std::vector<cv::Point2f>	&xys,
								std::vector<int>			&pt3DIdxs) const;

	void getCamRvecAndT(		const int					camIdx,
								cv::Mat						&rvec,
								cv::Mat 					&t)	const;

	void getCamRvecsAndTs( 		std::vector<cv::Mat> 		&rvecs,
								std::vector<cv::Mat> 		&ts);

	/*void updateReprojectionErrors(	const cv::Mat			&camIntrinsicMat,
									const cv::Mat			&camDistortionMat);

	void getMeanReprojectionError( 	float 					&meanError);*/

	void getMeanReprojectionError( 	const cv::Mat			&camIntrinsicMat,
									const cv::Mat			&camDistortionMat,
									float 					&meanError);

	void removeHighError3D(		const cv::Mat			&camIntrinsicMat,
								const cv::Mat			&camDistortionMat,
								const float 				thresh);

	void getOverlappingImgs(	const int 								baseImgIdx,
								std::map<int,std::vector<int> > 		&img2pt3Didxs);
	void getBestOverlappingImgs(const int 								baseImgIdx,
								std::map<int,std::vector<int> > 		&img2pt3Didxs);

	void getImgsSeeingPoints(	const std::vector<int> 					&pt3DIdxs,
								std::vector<std::vector<int> >			&pt2Imgs);

	void getMeasuresToPoints(	const std::vector<int> 								&pt3DIdxs,
								std::vector<std::vector<std::pair<int,int> > >		&pt3D2Measures,
								std::vector<std::vector<cv::Point2f > >				&pt3D2pt2Ds);

	void ApplyGlobalTransformation(const cv::Mat 				&transfMat);

	bool getImageIdxByCameraIdx(const int camIdx, int &imgIdx) const;

	bool getImageGPS(const int imgIdx, double &lat, double &lon) const;

	void removeCamera(const int camIdx);

	void removeCameras(const std::set<int> &camIdxs);

	void remove3DsHaveNoMeasurements();

	void removeCamerasSeeingNo3Ds();

	void removeMeasures(const std::vector<std::pair<int,int> > &measures);

	void removeRedundancy();

	void sanityCheck();	//check certain assertions the data structure must always pass

	bool hasPointNormal;

	//data
	std::string 						imgRoot;
	std::vector<std::string> 			imgs;		//all input image paths, including non-used ones
	std::vector<Pt3D> 					pt3Ds;		//3d points
	std::map<int, std::vector<Pt2D> >	img2pt2Ds;	//used image idx to 2D points detected in this image (including non-triangulated ones)
	std::vector<cv::Matx34d>			camMats;	//camera mats
	std::map<int,std::pair<double,double> >img2GPS;	//image latitude longitude info
	//data correspondences
	std::map<int, int>					img2camMat;	//used image idx to corresponding camera mat idx
	std::map<int, int>					camMat2img;	//camera mat idx to corresponding used image idx

};

#endif /* PTCLOUD_H_ */
