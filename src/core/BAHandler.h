//
//  BAHandler.h
//  3D_Reconstruction
//
//  Created by qiyue song on 19/4/14.
//  Copyright (c) 2014 qiyue.song. All rights reserved.
//


#include <vector>
#include <opencv2/core/core.hpp>
#include <map>
#include "PtCloud.h"
class BAHandler {
public:
	void adjustBundle(		PtCloud &ptCloud,
							cv::Mat& cam_matrix,
							cv::Mat& distortion_coefficients);

	void adjustBundle_sba(	PtCloud &ptCloud,
							cv::Mat& cam_matrix,
							cv::Mat& distortion_coefficients);

};
