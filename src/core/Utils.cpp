/*
 * Utils.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include "Utils.h"
#include <iostream>

using namespace std;
using namespace cv;
Utils::Utils() {
	// TODO Auto-generated constructor stub

}

Utils::~Utils() {
	// TODO Auto-generated destructor stub
}

int Utils::extractInt(string s){
	int num=0;
	for(int i=0;i<s.length();i++){
		if(s.at(i)>='0' && s.at(i)<='9'){
			num=num*10+(s.at(i)-'0');
		}
	}
	return num;
}
void Utils::KeyPoints2Points(const vector<KeyPoint> &kpts, vector<Point2f> &pts){
	pts.clear();
	pts.reserve(kpts.size());
	for(int i=0; i<kpts.size(); i++){
		pts.push_back(kpts[i].pt);
	}

}
void Utils::Matches2Points(const std::vector<cv::KeyPoint> &kpts1,
					const std::vector<cv::KeyPoint> &kpts2,
					const std::vector<cv::DMatch> &matches,
					std::vector<cv::Point2f> &pts1,
					std::vector<cv::Point2f> &pts2){
	pts1.clear();
	pts2.clear();
	pts1.reserve(matches.size());
	pts2.reserve(matches.size());
	for(int i=0; i<matches.size(); i++){
		pts1.push_back(kpts1[matches[i].queryIdx].pt);
		pts2.push_back(kpts2[matches[i].trainIdx].pt);
	}
}
void Utils::Matches2Indices(	const std::vector<cv::DMatch> &matches,
								std::vector<int> &idxs1,
								std::vector<int> &idxs2){
	idxs1.clear();
	idxs2.clear();
	idxs1.reserve(matches.size());
	idxs2.reserve(matches.size());
	for(int i=0; i<matches.size();i++){
		idxs1.push_back(matches[i].queryIdx);
		idxs2.push_back(matches[i].trainIdx);
	}
}

