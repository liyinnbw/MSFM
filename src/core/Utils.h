/*
* stateless static utility functions
*/

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

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

	static void Matches2Points(		const std::vector<cv::Point2f> &kpts1,
									const std::vector<cv::Point2f> &kpts2,
									const std::vector<cv::DMatch> &matches,
									std::vector<cv::Point2f> &pts1,
									std::vector<cv::Point2f> &pts2);

	static void Matches2Indices(	const std::vector<cv::DMatch> &matches,
									std::vector<int> &idxs1,
									std::vector<int> &idxs2);

	static void transformPoints(		const cv::Mat 				&transfMat,
										std::vector<cv::Point3f>	&xyzs);

	static void getTimeStampAsString(std::string &tstmp);

	static int getRandomInt(const int Min, const int Max);

	static bool endsWith(const std::string &s, const std::string &suffix);

	static void readPaths(	const std::string 				&root,
							std::vector<std::string> 		&paths);
};

#endif
