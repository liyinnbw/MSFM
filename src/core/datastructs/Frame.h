/*
* Object representing an image keyframe
*/

#ifndef SRC_CORE_DATASTRUCTS_FRAME_H_
#define SRC_CORE_DATASTRUCTS_FRAME_H_

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <numeric>
#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#include "../DetectorExtractor.h"

class FrameID{
public:
	//singleton
	static FrameID& GetInstance(){
		static FrameID instance;
		return instance;
	}
	int idCounter;

private:
	FrameID()
	:idCounter(0)
	{

	}
	~FrameID(){}
};

class Frame {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//this is necessary to prevent crash when "new" class containing eigen object fields
	typedef std::shared_ptr<Frame> Ptr;
	// static bool KeyPointComparator(const cv::KeyPoint &i, const cv::KeyPoint &j)
	// {
	// 	if(i.pt.y==j.pt.y){return i.pt.x<j.pt.x;}
	// 	return i.pt.y<j.pt.y;
	// 	// if(i.pt.y<j.pt.y){
	// 	// 	return true;
	// 	// }else if(i.pt.y>j.pt.y){
	// 	// 	return false;
	// 	// }else{
	// 	// 	if(i.pt.x<j.pt.x){
	// 	// 		return true;
	// 	// 	}else{
	// 	// 		return false;
	// 	// 	}
	// 	// }

	// }

	//video frame constructer
	Frame(const cv::Mat &img, const long time)
	:imgIdx(0)
	,imgRoot("")
	,imgName("")
	,img(img.clone())		//input image must be in grayscale
	,fixed(false)
	,rotation(Eigen::Quaterniond::Identity())
	,position(Eigen::Vector3d::Zero())
	,measuresCnt(0)
	,algo(ORB)
	,time(time)
	{
		imgIdx = FrameID::GetInstance().idCounter++;
	}

	//picture frame constructuer
	Frame(int idx, std::string root, std::string name)
	:imgIdx(idx)
	,imgRoot(root)
	,imgName(name)
	,fixed(false)
	,rotation(Eigen::Quaterniond::Identity())
	,position(Eigen::Vector3d::Zero())
	,measuresCnt(0)
	,algo(ORB)
	,time(0)
	{
		if(imgIdx>=FrameID::GetInstance().idCounter){
			FrameID::GetInstance().idCounter=imgIdx+1;
		}
	}
	~Frame(){}

	//clear all feature data
	inline void clearFeatures(){
		kpts.clear();
		decs = cv::Mat();
		kptLUT.clear();
		measured.clear();
		measuresCnt = 0;
	}


	//overwrite all previous feature data, called to setup everything in one function
	inline void init()
	{
		cv::Mat tmpImg;
		if(img.empty()){
			tmpImg 	= cv::imread(imgRoot+"/"+imgName,cv::IMREAD_GRAYSCALE);
		}else{
			tmpImg	= img;
		}
		assert(!tmpImg.empty());

		DetectorExtractor &deAlgo = DetectorExtractor::GetInstance();
		if(algo == ORB){
			deAlgo.detectORB(tmpImg, kpts);
			deAlgo.computeORB(tmpImg, kpts, decs); //note kpts order and even count may change from previous line. 
		}
		if(algo == BRISK){
			deAlgo.detectBRISK(tmpImg, kpts);
			deAlgo.computeBRISK(tmpImg, kpts, decs); //note kpts order and even count may change from previous line.
		}

		sortFeatures();
		makeLUT(tmpImg.rows);
		resetMeasures();

	}

	//do more than just detect features
	inline void detectFeatures(){
		cv::Mat tmpImg;
		if(img.empty()){
			tmpImg 	= cv::imread(imgRoot+"/"+imgName,cv::IMREAD_GRAYSCALE);
		}else{
			tmpImg 	= img;
		}
		assert(!tmpImg.empty());

		DetectorExtractor &deAlgo = DetectorExtractor::GetInstance();
		if(algo == ORB){
			deAlgo.detectORB(tmpImg, kpts);
		}
		if(algo == BRISK){
			deAlgo.detectBRISK(tmpImg, kpts);
		}
		sortFeatures();
		makeLUT(tmpImg.rows);
		resetMeasures();
	}

	// sort feature points to allow faster search by (x,y)
	// sort both kpts and decs while keeping their correspondences
	inline void sortFeatures(){
		assert(kpts.size() == decs.rows);

		//fill array starting with value 0
		std::vector<size_t> idxs(kpts.size());
		std::iota(idxs.begin(), idxs.end(), 0); 

		//sort indices
		std::vector<cv::KeyPoint> &mkpts = kpts; //you cannot pass member variable directly to lambda function, this is one way to circumvent
		std::sort(idxs.begin(), idxs.end(), [&mkpts](size_t a, size_t b) {
			//sort indices by non-decreasing feature point's y value
			//if same y, sort by non-decreasing feature point's x value
			if(mkpts[a].pt.y==mkpts[b].pt.y){return mkpts[a].pt.x<mkpts[b].pt.x;}
			return mkpts[a].pt.y<mkpts[b].pt.y;
		});
		
		//copy data to temp containers. Is there a non-copying solution?
		std::vector<cv::KeyPoint> kpts_sorted;
		kpts_sorted.reserve(kpts.size());
		cv::Mat decs_sorted(decs.size(), decs.type());
		size_t newIdx=0;
		for(size_t const &i : idxs){
			kpts_sorted.push_back(kpts[i]);
			decs.row(i).copyTo(decs_sorted.row(newIdx)); 
			newIdx++;
		}

		//swap original
		kpts.swap(kpts_sorted);
		decs=decs_sorted; //the underlying data not copied.

		//std::sort(kpts.begin(),kpts.end(), KeyPointComparator);

	}

	//only create look up table, need to sanitize
	inline void makeLUT(int rows){

		//create row look up table for faster kpt search
		kptLUT.clear();
		kptLUT.reserve(rows);
		int v = 0;
		for(int currRow = 0; currRow<rows; currRow++){
			while(v<kpts.size() && currRow>kpts[v].pt.y){
				v++;
			}
			kptLUT.push_back(v);	//v may be bigger than kpts size
		}
		//for(int i=0; i<kptLUT.size(); i++){
		//	std::cout<<"["<<i<<"]"<<kptLUT[i]<<std::endl;
		//}
	}

	inline void resetMeasures(){
		measured 	= std::vector<unsigned char>(kpts.size(),0);
		measuresCnt = 0;
	}

	//only describe features, need to sanitize
	inline void describeFeatures()
	{
		cv::Mat tmpImg;
		if(img.empty()){
			tmpImg 	= cv::imread(imgRoot+"/"+imgName,cv::IMREAD_GRAYSCALE);
		}else{
			tmpImg 	= img;	//no data copy since we don't clone it
		}
		assert(!tmpImg.empty());
		DetectorExtractor &deAlgo = DetectorExtractor::GetInstance();
		if(algo == ORB){
			deAlgo.computeORB(tmpImg, kpts, decs);
		}
		if(algo == BRISK){
			deAlgo.computeBRISK(tmpImg, kpts, decs);
		}
	}

	//for conversion from opengv convention to opencv convention
	inline cv::Matx34d getCVTransform(){
		Eigen::Matrix3d R = rotation.toRotationMatrix();
		Eigen::Vector3d t = -R*position;
		cv::Matx34d P(	R(0,0),	R(0,1),	R(0,2),	t(0),
						R(1,0),	R(1,1),	R(1,2),	t(1),
						R(2,0),	R(2,1),	R(2,2),	t(2));
		return P;
	}

	inline std::string toString()
	{
		std::stringstream ss;
		ss<<"[FRAME]"<<imgIdx<<" "<<imgName;
		if(fixed){
			ss<<" fixed";
		}
		return ss.str();
	}


	int							imgIdx;		//image file idx
	const std::string			imgRoot;	//image file folder
	const std::string 			imgName;	//image file name
	bool						fixed;		//if true, the frame pose will not be changed during BA;

	//for video
	cv::Mat 					img;
	long						time;
	std::vector<int>			trackedKpts;//features that has been tracked, only managed by tracker loop and for UI display

	//feature related
	std::vector<cv::KeyPoint> 	kpts;		//features
	cv::Mat 					decs;		//descriptors
	std::vector<int> 			kptLUT;		//feature row lookup table, index equals to rows, value may exceed kpts size
	std::vector<unsigned char>	measured;	//0: unmeasured, 1: measured, incremented and decremented by measure con/destructor
	int							measuresCnt;//total number of measured features. when this goes to zero, frame can be deleted
											//incremented and decremented by measure constructors

	//for pose
	Eigen::Quaterniond			rotation;	//rotation back to world axis
	Eigen::Vector3d 			position;	//camera center in world


	enum {ORB,BRISK}			algo;
};

#endif /* SRC_CORE_DATASTRUCTS_FRAME_H_ */
