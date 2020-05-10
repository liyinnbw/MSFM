/*
 * An object representing a 2D-3D correspondence
 */

#ifndef SRC_CORE_DATASTRUCTS_MEASUREMENT_H_
#define SRC_CORE_DATASTRUCTS_MEASUREMENT_H_

#include <vector>
#include <memory>
#include <assert.h>

#include "Frame.h"
#include "LandMark.h"

class Measurement {
public:
	typedef std::shared_ptr<Measurement> Ptr;
	Measurement(Frame::Ptr frame, int featureIdx, LandMark::Ptr landmark)
	:frame(frame)
	,featureIdx(featureIdx)
	,landmark(landmark)
	,deleted(false)
	{
		assert(frame && landmark);
		assert(!frame->measured[featureIdx]);
		frame->measuresCnt++;
		frame->measured[featureIdx] += 1;
		landmark->measuresCnt++;
	}
	~Measurement(){
		clear();
	}

	//clear the smart pointer so that memory can be freed
	void clear(){
		if(deleted) return;
		if(frame){
			assert(frame->measuresCnt>0);
			assert(frame->measured[featureIdx]);
			frame->measuresCnt--;
			frame->measured[featureIdx] -= 1;
			frame.reset();
		}
		if(landmark){
			assert(landmark->measuresCnt>0);
			landmark->measuresCnt--;
			//delete landmark if no more measurement attached to it
			//else it will be a dangling landmark not useful
			if(landmark->measuresCnt ==0){
				landmark->deleted = true;
			}
			landmark.reset();
		}
		deleted = true;
	}

	inline std::string toString()
	{
		std::stringstream ss;
		ss<<"[MEAS]"<<"frame:"<<frame->imgIdx<<" feature:"<<featureIdx<<frame->kpts[featureIdx].pt<<" landmark:"<<landmark->pt.transpose();
		if(deleted){
			ss<<" deleted";
		}
		return ss.str();
	}

	Frame::Ptr 		frame;
	int				featureIdx;
	LandMark::Ptr	landmark;
	bool 			deleted;
};

#endif /* SRC_CORE_DATASTRUCTS_MEASUREMENT_H_ */
