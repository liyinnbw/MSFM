/*
* stateless static utility functions
*/

#include "Utils.h"
#include <iostream>
#include <dirent.h>
#include <cstring>
#include <cstdio>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;
Utils::Utils() {
	srand (time(NULL));
}

Utils::~Utils() {
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

void Utils::Matches2Points(	const std::vector<cv::Point2f> &kpts1,
							const std::vector<cv::Point2f> &kpts2,
							const std::vector<cv::DMatch> &matches,
							std::vector<cv::Point2f> &pts1,
							std::vector<cv::Point2f> &pts2){
	pts1.clear();
	pts2.clear();
	pts1.reserve(matches.size());
	pts2.reserve(matches.size());
	for(int i=0; i<matches.size(); i++){
		pts1.push_back(kpts1[matches[i].queryIdx]);
		pts2.push_back(kpts2[matches[i].trainIdx]);
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

void Utils::transformPoints(		const Mat 			&transfMat,
									vector<Point3f>		&xyzs)
{
	Mat pts4DMat; //4 channels type 29
	convertPointsToHomogeneous(xyzs, pts4DMat);
	pts4DMat = pts4DMat.reshape(1);		//convert to 1 channel, type 5, efficiency O(1)
	pts4DMat = pts4DMat.t();
	Mat tmpMat;
	transfMat.convertTo(tmpMat,pts4DMat.type());
	pts4DMat = tmpMat*pts4DMat;			//transform 3D points, results in 3*N mat
	pts4DMat = pts4DMat.t();
	pts4DMat = pts4DMat.reshape(3);		//convert to 3 channels so that you can copy to vector of points
	pts4DMat.copyTo(xyzs);
}

void Utils::getTimeStampAsString(std::string &tstmp){
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	time (&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer,80,"%d-%m-%Y-%I-%M-%S",timeinfo);
	string str(buffer);
	tstmp = str;
}

int Utils::getRandomInt(const int Min, const int Max){
	return ((rand() % ((Max + 1) - Min)) + Min);
}

bool Utils::endsWith(const std::string &s, const std::string &suffix){
	return s.size() >= suffix.size() &&
	       s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void Utils::readPaths(	const string 				&root,
						vector<std::string> 		&paths){
	string rootPath = root;
	paths.clear();
	DIR *dir;
	struct dirent *ent;
	vector<string> imgExts;
	imgExts.push_back("JPG"); imgExts.push_back("jpg");
	imgExts.push_back("PNG"); imgExts.push_back("png");

	if ((dir = opendir (root.c_str())) != NULL) {
		vector<string> tmpPaths;
		while ((ent = readdir (dir)) != NULL) {
			if(strcmp(ent->d_name,".")==0 || strcmp(ent->d_name,"..")==0) continue;
			string fileName = string(ent->d_name);
			for(int i=0; i<imgExts.size(); i++){
				if(Utils::endsWith(fileName,imgExts[i])){
					tmpPaths.push_back(fileName);
				}
			}
		}
		//sort file name in ascending order
		while(!tmpPaths.empty()){
			vector<string>::iterator minFileName = tmpPaths.begin();
			for(vector<string>::iterator i= tmpPaths.begin();i!=tmpPaths.end();i++){
				if(Utils::extractInt(*i)< Utils::extractInt(*minFileName)) minFileName=i;
			}
			//paths.push_back(rootPath+"/"+*minFileName);
			paths.push_back(*minFileName);
			tmpPaths.erase(minFileName);
		}
		closedir(dir);
	} else {
	  /* could not open directory */
	  perror ("");
	}
}