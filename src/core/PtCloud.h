/*
 * PtCloud.h
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#ifndef PTCLOUD_H_
#define PTCLOUD_H_


#include <vector>
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

	void update3D(				int 						imgIdx,
								std::vector<int> 			&pt3Didxs,
								std::vector<int> 			&img2Didxs);

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

	void getXYZs( 				std::vector<cv::Point3f>	&xyzs);

	void get2DsHave3D(			std::vector<cv::Point2f> 	&xys,
								std::vector<int>			&imgIdxs,
								std::vector<int>			&pt3DIdxs);

	void getCamRvecsAndTs( 		std::vector<cv::Mat> 		&rvecs,
								std::vector<cv::Mat> 		&ts);
	//data
	std::string 						imgRoot;
	std::vector<std::string> 			imgs;		//all input image paths, including non-used ones
	std::vector<Pt3D> 					pt3Ds;		//3d points
	std::vector<Pt2D>					pt2Ds;		//2d points from all used images (including non-triangulated ones)
	std::vector<cv::Matx34d>			camMats;	//camera mats
	//data correspondences
	std::map<int, std::vector<int> >	img2pt2Ds;	//used image idx to 2D point idxs detected in this image (including non-triangulated ones)
	std::map<int, int>					img2camMat;	//used image idx to corresponding camera mat
};

#endif /* PTCLOUD_H_ */
