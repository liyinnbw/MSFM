/*
 * Utils.h
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PtCloud.h"

class Utils {
public:
	Utils();
	virtual ~Utils();
	static int extractInt(std::string s);
	static void KeyPoints2Points(	const std::vector<cv::KeyPoint> &kpts,
									std::vector<cv::Point2f> &pts);

	static void Matches2Points(		const std::vector<cv::KeyPoint> &kpts1,
									const std::vector<cv::KeyPoint> &kpts2,
									const std::vector<cv::DMatch> &matches,
									std::vector<cv::Point2f> &pts1,
									std::vector<cv::Point2f> &pts2);

	static void Matches2Indices(	const std::vector<cv::DMatch> &matches,
									std::vector<int> &idxs1,
									std::vector<int> &idxs2);


};

#endif /* UTILS_H_ */
