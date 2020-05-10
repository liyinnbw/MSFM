/*
 * An object represent a 3D point in scene
 */

#ifndef SRC_CORE_DATASTRUCTS_LANDMARK_H_
#define SRC_CORE_DATASTRUCTS_LANDMARK_H_

#include <vector>
#include <list>
#include <memory>
#include <string>
#include <sstream>
#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

class LandMark {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//this is necessary to prevent crash when "new" class containing eigen object fields
	typedef std::shared_ptr<LandMark> Ptr;
	LandMark()
	:pt(Eigen::Vector3d::Zero())
	,deleted(false)
	,measuresCnt(0)
	{
	}
	LandMark(Eigen::Vector3d &pt)
	:pt(pt)
	,deleted(false)
	,measuresCnt(0)
	{
	}
	~LandMark(){}

	inline std::string toString()
	{
		std::stringstream ss;
		ss<<"[LM]"<<pt.transpose();
		if(deleted){
			ss<<" deleted";
		}
		return ss.str();
	}

	inline bool addRecent(const cv::KeyPoint &kpt, const cv::Mat &dec, int max){
		bool maxReached = false;
		if(recentKpts.size()>max){
			recentKpts.resize(max-1);
			recentDecs.resize(max-1);
			maxReached = true;
		}
		recentKpts.push_front(kpt);
		recentDecs.push_front(dec);
		return maxReached;
	} 

	Eigen::Vector3d pt;
	bool 			deleted;
	int				measuresCnt;		//count number of measurements. when this goes to zero, landmark can be deleted
										//incremented and decremented by measure constructors
	std::list<cv::Mat> recentDecs;			//stores recently measured descriptors. these will be prioritized for use when tracking.
	std::list<cv::KeyPoint> recentKpts;		//stores recently measured kpts. these will be prioritized for use when tracking.

};

#endif /* SRC_CORE_DATASTRUCTS_LANDMARK_H_ */
